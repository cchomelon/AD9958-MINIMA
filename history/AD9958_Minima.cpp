/*  DDS-AD9958 Firmware
    WARNING: This firmware is ONLY capatible with Arduino UNO R4 Minima
    Last update: 20-6-2024
    Author: Jack Cho (uxxitn3hx@mozmail.com)
    AD9958 Specifications (Stable output)
    Frequency   | 1..200 MHz / 1 kHz steps
    Amplitude   | -40..-10 dBm / 0.1 dBm steps
    Phase       | 0..359.9 deg / 0.1 deg steps

*/
#define DEBUG // Uncomment to enter Serial debug mode

#include "AD9958_Minima.h"

#define MAX_lenSerialCommand 110
char SerialBuffer[MAX_lenSerialCommand];

void setup() {
  pinMode(DDS_RESET, OUTPUT);
  pinMode(DDS_SDIO, OUTPUT);
  pinMode(DDS_SCLK, OUTPUT);
  pinMode(DDS_CSB, OUTPUT);
  digitalWrite(DDS_CSB, HIGH);
  pinMode(DDS_IOUP, OUTPUT);
  digitalWrite(DDS_IOUP, HIGH);

  pinMode(A5, INPUT);
  pinMode(A4, INPUT);
  pinMode(A3, INPUT);
  pinMode(A2, INPUT);
  pinMode(A1, INPUT);
  pinMode(A0, INPUT);

  // setPinMode(1, 2, OUTPUT);
  // setPinMode(1, 12, OUTPUT);
  // setPinMode(1, 9, OUTPUT);
  // setPinMode(1, 10, OUTPUT);
  // setPinHigh(1, 10);
  // setPinMode(3, 3, OUTPUT);
  // setPinHigh(3, 3);

  // *PFS_P102PFS_BY = 0x04;
  // *PFS_P112PFS_BY = 0x04;
  // *PFS_P109PFS_BY = 0x04;
  // *PFS_P110PFS_BY = 0x05;
  // *PFS_P303PFS_BY = 0x05;

  // setPinMode(0, 14, INPUT);
  // setPinMode(0, 1, INPUT);
  // setPinMode(0, 2, INPUT);
  // setPinMode(1, 1, INPUT);
  // setPinMode(1, 0, INPUT);

  // *PFS_P014PFS_BY = 0x00;
  // *PFS_P000PFS_BY = 0x00;
  // *PFS_P001PFS_BY = 0x00;
  // *PFS_P002PFS_BY = 0x00;
  // *PFS_P101PFS_BY = 0x00;
  // *PFS_P100PFS_BY = 0x00;

  DDS_Reset();
  DDS_Init();

  setFreq(0, Ch0);
  setFreq(0, Ch1);
  // setAmp(Ch0_AMPT, Ch0);
  // setAmp(Ch1_AMPT, Ch1);
  setDeg(Ch0_PHAS, Ch0);
  setDeg(Ch1_PHAS, Ch1);

  DDS_Update();

  // *ADC140_ADCER = 0x06;
  // *ADC140_ADCSR |= (0x01 << 15); 

#ifdef DEBUG
  Serial.begin(115200); // Ready for software control via COM
#endif
}

void loop() {
    pinScan();

#ifdef DEBUG
  readSerialCommand();
#endif
}

static inline uint32_t freqToFTW(double _freq) {
  // Frequency Tuning Word
  // FTW = (freq_out / f_ref) * 2^32
  return (uint32_t)(_freq * 8589934.592);
}

static inline uint16_t DegToPOW(double _deg) {
  // Phase Offset Word
  // POW = (P_out / 360) * 2^14
  return (uint16_t)(_deg * 45.51111111111111);
}

static inline uint32_t dBmToASF(double _dBm, double _freq) {
  // Amplitude Scale Factor
  // Last calibrated at 25-06-24 with CALIBRATION.ipynb
  // The following relationship might deviate on different DDS.
  // ASF = exp[ (P + 46.307) / 8.7355 ]
  // return (uint32_t)(pow(2.71828182846, ((_dBm + 68.975) / 8.684)));
  // ASF = exp[ 0.1371 * dBm + 0.2659 * ln(freq) + 7.2040]
  return (uint32_t)pow(2.71828182846, C1 * _dBm + C2 * log(_freq) + C3);
}

void DDS_Switch() {
  if (!currentStatus) {
    CALIBRATION_setFreq(FTW[0], Ch0);
    CALIBRATION_setFreq(FTW[0], Ch1);
    CALIBRATION_setAmp(ASF[0], Ch0);
    CALIBRATION_setAmp(ASF[0], Ch1);

    DDS_Update();

    currentStatus = true;

#ifdef DEBUG
    Serial.println("Switch ON.");
#endif
  } else if (currentStatus) {
    CALIBRATION_setFreq(0, Ch0);
    CALIBRATION_setFreq(0, Ch1);
    // CALIBRATION_setAmp(ASF[0], Ch0);
    // CALIBRATION_setAmp(ASF[0], Ch1);

    DDS_Update();

    currentStatus = false;

#ifdef DEBUG
    Serial.println("Switch OFF.");
#endif
  }
}

void setFreq(double freq, uint8_t channel) {
  unsigned long start = micros();
  uint8_t strBuffer[4];

  // double _C = pow(2,32) / REFCLK_FREQ;
  uint32_t FTW = freqToFTW(freq);

  // Frequency clamping to maximum FTW 2^32-1
  // if (FTW > 4294967295) {
  //     return;
  // }

  strBuffer[0] = (uint8_t)((0xFF000000 & FTW) >> 24);
  strBuffer[1] = (uint8_t)((0x00FF0000 & FTW) >> 16);
  strBuffer[2] = (uint8_t)((0x0000FF00 & FTW) >> 8);
  strBuffer[3] = (uint8_t)(0x000000FF & FTW);

  unsigned long mid = micros();

  setChannel(channel);
  sdioWriteReg(FCR, strBuffer, 4);
  DDS_Update();

  unsigned long end = micros();

#ifdef DEBUG
  Serial.print(F("setFreq data preparation duration: "));
  Serial.println(mid - start);
  Serial.print(F("setFreq writing/updating duration: "));
  Serial.println(end - mid);
  Serial.print(F("setFreq total duration: "));
  Serial.println(end - start);
  Serial.println(FTW);
#endif
}

void setDeg(double deg, uint8_t channel) {
  uint8_t strBuffer[2];

  uint16_t POW = DegToPOW(deg);

  // if (POW > 16383) {
  //     return;
  // }

  strBuffer[0] = (uint8_t)((POW & 0x3F00) >> 8);
  strBuffer[1] = (uint8_t)(POW & 0xFF);

  setChannel(channel);
  sdioWriteReg(PCR, strBuffer, 2);
  DDS_Update();
}

void setAmp(double dBm, uint8_t channel) {
  unsigned long start = micros();
  uint8_t strBuffer[3];
  uint32_t ASF = dBmToASF(dBm, currentFreq);
  unsigned long calculate = micros();
  uint8_t EMSB = (((ASF & 0x300) >> 8 | 0x10));
  // int EMSB = (ASF & 0x300) >> 8;
  // EMSB |= 0x10;
  unsigned long mid = micros();
  strBuffer[0] = (uint8_t)(0x00);
  strBuffer[1] = (uint8_t)(EMSB);
  strBuffer[2] = (uint8_t)(ASF & 0xFF);

  setChannel(channel);
  sdioWriteReg(ACR, strBuffer, 3);
  DDS_Update();
  unsigned long end = micros();
#ifdef DEBUG
  Serial.print(F("setAmp data Calculate duration: "));
  Serial.println(calculate - start);
  Serial.print(F("setAmp data preparation duration: "));
  Serial.println(mid - start);
  Serial.print(F("setAmp total duration: "));
  Serial.println(end - start);
  Serial.println(ASF);
#endif
}

void CALIBRATION_setAmp(uint32_t CALIBRATION_ASF, uint8_t channel) {
#ifdef DEBUG
  unsigned long start = micros();
#endif

  uint8_t strBuffer[3];
  uint8_t EMSB = (((CALIBRATION_ASF & 0x300) >> 8 | 0x10));

  strBuffer[0] = (uint8_t)(0x00);
  strBuffer[1] = (uint8_t)(EMSB);
  strBuffer[2] = (uint8_t)(CALIBRATION_ASF & 0xFF);

  setChannel(channel);
  sdioWriteReg(ACR, strBuffer, 3);
  DDS_Update();

#ifdef DEBUG
  unsigned long end = micros();
  Serial.print("CAL_AMP duration: ");
  Serial.println(end - start);
#endif
}

void CALIBRATION_setFreq(uint32_t CALIBRATION_FTW, uint8_t channel) {
  uint8_t strBuffer[4];

  strBuffer[0] = (uint8_t)((0xFF000000 & CALIBRATION_FTW) >> 24);
  strBuffer[1] = (uint8_t)((0x00FF0000 & CALIBRATION_FTW) >> 16);
  strBuffer[2] = (uint8_t)((0x0000FF00 & CALIBRATION_FTW) >> 8);
  strBuffer[3] = (uint8_t)(0x000000FF & CALIBRATION_FTW);

  setChannel(channel);
  sdioWriteReg(FCR, strBuffer, 4);
  DDS_Update();
}

/*  Analog input triggers  */

void triggerFreq() {
  static bool lastState = LOW;
  bool currentState = HIGH;

  if (currentState && !lastState) {
#ifdef DEBUG
    Serial.print(F("A5 triggered."));
#endif

    triggerTime = micros();
    ++frequencyCount;
    setTime = micros();
    frequencyCount %= 4;
    // setFreq(arrayFrequency[frequencyCount], selectedChannel);
    CALIBRATION_setFreq(FTW[frequencyCount], selectedChannel);

#ifdef DEBUG
    Serial.print(F("Set Frequency: "));
    Serial.print(F("["));
    Serial.print(frequencyCount);
    Serial.print(F("] "));
    Serial.println(arrayFrequency[frequencyCount]);
    Serial.print(F("Time from trigger to setFreq: "));
    Serial.print(setTime - triggerTime);
    Serial.println(F(" microseconds"));
#endif
    // delay(1000000);
    currentState = LOW;
  }
  lastState = currentState;
}

void triggerAmp() {
  static bool lastState = LOW;
  bool currentState = HIGH;

  if (currentState && !lastState) {
#ifdef DEBUG
    Serial.print(F("A4 triggered."));
#endif

    ++amplitudeCount;
    amplitudeCount %= 4;
    // setAmp(arrayAmplitude[amplitudeCount], selectedChannel);
    CALIBRATION_setAmp(ASF[amplitudeCount], selectedChannel);

#ifdef DEBUG
    Serial.print(F("Set Amplitude: "));
    Serial.print(F("["));
    Serial.print(amplitudeCount);
    Serial.print(F("] "));
    Serial.println(arrayAmplitude[amplitudeCount]);
#endif
    // delay(500000);
    currentState = LOW;
  }
  lastState = currentState;
}

void triggerChannel() {
  static bool lastState = LOW;
  bool currentState = HIGH;

  if (currentState && !lastState) {
#ifdef DEBUG
    Serial.println(F("A3 triggered."));
#endif

    ++channelCount;

    channelCount %= 2;
    selectedChannel = (channelCount == 0) ? Ch0 : Ch1;

#ifdef DEBUG
    Serial.print(F("Set Channel: "));
    Serial.print(F("["));
    Serial.print(channelCount);
    Serial.print(F("] "));
    Serial.println(channelCount);
#endif
    currentState = LOW;
  }
  lastState = currentState;
}

void resetIndex() {
  static bool lastState = LOW;
  bool currentState = HIGH;

  if (currentState && !lastState) {
    channelCount = 0;
    frequencyCount = 0;
    amplitudeCount = 0;
    setFreq(arrayFrequency[0], selectedChannel); // Reset to default frequency
    setAmp(arrayAmplitude[0], selectedChannel);  // Reset to default amplitude

#ifdef DEBUG
    Serial.println(F("All profiles reset to default"));
#endif
    currentState = LOW;
  }
  lastState = currentState;
}

inline void pulse(uint8_t pin) {
  digitalWrite(pin, HIGH);
  // __asm__ __volatile__("nop\n\t");
  digitalWrite(pin, LOW);
}

float readVoltage(uint8_t pin) {return analogRead(pin) * 0.00488758553;}

void pinScan() {
  triggeredPin = isPinTriggered();

  if (!isPinOccupied) {
    switch (triggeredPin) {
    case A5:
      isPinOccupied = true;
      triggerFreq();
      break;
    case A4:
      isPinOccupied = true;
      triggerAmp();
      break;
    case A3:
      isPinOccupied = true;
      triggerChannel();
      break;
    case A2:
      isPinOccupied = true;
      resetIndex();
      break;
    case A0:
      isPinOccupied = true;
      DDS_Switch();
      break;
    default:
      break;
    }
  }
}

uint8_t isPinTriggered() {
  if (readVoltage(A5) > TRIGGER_THRESHOLD) {
    return A5;
  }
  if (readVoltage(A4) > TRIGGER_THRESHOLD) {
    return A4;
  }
  if (readVoltage(A3) > TRIGGER_THRESHOLD) {
    return A3;
  }
  if (readVoltage(A2) > TRIGGER_THRESHOLD) {
    return A2;
  }
  if (readVoltage(A0) > TRIGGER_THRESHOLD) {
    return A0;
  }
  isPinOccupied = false;
  return 0;
}

/*  DDS Behaviour   */

void DDS_Init() {
  uint8_t initCSR[] = {0xF0};
  sdioWriteReg(CSR, initCSR, 1);

  uint8_t initFR1[] = {0xD0, 0x00, 0x00};
  sdioWriteReg(FR1, initFR1, 3);

  uint8_t initFR2[] = {0x00, 0x00};
  sdioWriteReg(FR2, initFR2, 2);

  uint8_t initCFR[] = {0x00, 0x03, 0x00};
  sdioWriteReg(CFR, initCFR, 3);

  // pulse(DIG_RESET);

  // uint8_t initManual[] = {0x00, 0x17, 0xFF};
  // sdioWriteReg(ACR, initManual, 3);
}

inline void setChannel(uint8_t channel) {
  // setPinLow(1, 10); // CSB - 12; P110
  *PFS_P110PFS_BY = 0x04;
  fastShiftOut(CSR);
  fastShiftOut(channel);
  *PFS_P110PFS_BY = 0x05;
  // setPinHigh(1, 10);
}

void DDS_Reset() {
  // setPinHigh(1, 2); // RESET - 5; P102
  // setPinLow(1, 2);

  *PFS_P102PFS_BY = 0x05;
  *PFS_P102PFS_BY = 0x04;
}

inline void DDS_Update() {
  // setPinHigh(3, 3); // IOUP - 9; P303
  // setPinLow(3, 3);

  *PFS_P303PFS_BY = 0x05;
  *PFS_P303PFS_BY = 0x04;
}

/*  SDIO Behaviour   */

void sdioWriteReg(uint8_t reg, uint8_t *strBuffer, int size) {
  // setPinLow(1, 10); // CSB - 12; P110
  *PFS_P110PFS_BY = 0x04;
  fastShiftOut(reg);
  for (int i = 0; i < size; ++i) {
    fastShiftOut(strBuffer[i]);
  }
  *PFS_P110PFS_BY = 0x05;
  // setPinHigh(1, 10);
}

bool inRange(int32_t val, int32_t minimum, int32_t maximum) {
// return ((minimum <= val) && (val <= maximum));
#ifdef DEBUG
  return true;
#endif
}

inline void fastShiftOut(uint8_t val) {
  uint8_t i;

  for (i = 0; i < 8; ++i) {
    // MSB first: check the (7-i)th bit
    if (val & (1 << (7 - i))) {
      // setPinHigh(1, 12);
      *PFS_P112PFS_BY = 0x05;
    } else {
      // setPinLow(1, 12);
      *PFS_P112PFS_BY = 0x04;
    }
    // Clock pulse
    *PFS_P109PFS_BY = 0x05;
    *PFS_P109PFS_BY = 0x04;
    // setPinHigh(1, 9);
    // setPinLow(1, 9);
  }
}

/*  Serial Command  */
#ifdef DEBUG

const char HELP_STRING[] PROGMEM =
    "F - Set Frequency in MHz (0.1 - 200)\n"
    "A - Set Amplitude in dBm (-68 - 0)\n"
    "C - Set Channel\n"
    "T - Trigger pin (2, 3, 4, 7)\n"
    "U - Directly update ASF (ONLY FOR CALIBRATION)"
    "; - Commands Separator"
    "\n"
    "Example:\n"
    "C0;F100;A-12\n"
    "Set Frequency to 100 MHz, and Output Power to -12 dBm in Channel 0.\n"
    "Any number of commands in any order is allowed.";

void readSerialCommand() {
  if (!Serial.available())
    return;

  int counter = Serial.readBytesUntil('\n', SerialBuffer, 110);
  if (counter == 0)
    return;
  SerialBuffer[counter] = '\0';

  int32_t value = 0;
  char command;

  GParser data(SerialBuffer, ';');
  int commandsCounter = data.split();

  for (int i = 0; i < commandsCounter; i++) {
    sscanf(data[i], "%c%ld", &command, &value);
    switch (command) {
    case 'C':
      if (value == 0) {
        currentChannel = 0;
        serialChannel = Ch0;
        Serial.print(F("Set Channel: "));
        Serial.println(currentChannel);
      } else if (value == 1) {
        currentChannel = 1;
        serialChannel = Ch1;
        Serial.print(F("Set Channel: "));
        Serial.println(currentChannel);
      } else {
        Serial.println("Incorrect Channel (0 / 1)");
      };
      break;

    case 'F': // Frequency
      if (currentChannel == -1) {
        Serial.println("The input channel is not selected.");
      }
      if (inRange(value, 0.1, 225)) {
        serialFrequency = (double)value;
        setFreq(serialFrequency, serialChannel);

        Serial.print(F("Set Freq: "));
        Serial.print(value);
        Serial.println(F(" MHz."));
      } else
        Serial.println("Frequency is OUT OF RANGE (" + String(0.1) + " - " +
                       String(225) + ")");
      break;

    case 'A': // Amplitude, dBm
      if (inRange(value, MIN_AMPL, MAX_AMPL)) {
        serialAmplitude = (double)value;
        setAmp(serialAmplitude, serialChannel);

        Serial.print(F("Set Amplitude: "));
        Serial.print(value);
        Serial.println(F(" dBm."));
      } else
        Serial.println("Power is OUT OF RANGE (-" + String(MAX_AMPL) + " - " +
                       String(MIN_AMPL) + ")");
      break;

    case 'T': { // Pin
      // setPinHigh(1, 04);
      uint8_t val = 0;
      unsigned long start = micros();
      *PFS_P103PFS_BY = 0x05;
      val = readPin(1, 03);
      *PFS_P103PFS_BY = 0x04;
      unsigned long end = micros();
      Serial.println(val);  
      Serial.println(end - start);

      start = micros();
      digitalWrite(4, HIGH);
      val = readPin(1, 03);
      digitalWrite(4, LOW);
      end = micros();
      Serial.println(val);
      Serial.println(end - start);
      break;
    }
    case 'U':
      CALIBRATION_ASF = (uint32_t)value;
      CALIBRATION_setAmp(CALIBRATION_ASF, serialChannel);
      break;

    case 'X':
      DDS_Switch();
      break;

    case 'R':
      Serial.println(readVoltage(A5));
      Serial.println(readVoltage(A4));
      Serial.println(readVoltage(A3));
      Serial.println(readVoltage(A2));
      Serial.println(readVoltage(A1));
      Serial.println(readVoltage(A0));
      Serial.println(triggeredPin);
      break;

    default:
      Serial.print(F("Unknown command:"));
      Serial.println(command);
      Serial.println((const __FlashStringHelper *)HELP_STRING);
    } // switch
  } // for
}

#endif
