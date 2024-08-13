/*  DDS-AD9958 Firmware
    WARNING: This firmware is ONLY capatible with Arduino UNO R4 Minima
    Last update: 20-6-2024
    Author: Jack Cho (uxxitn3hx@mozmail.com)
    AD9958 Specifications (Stable output)
    Frequency   | 1..200 MHz / 1 kHz steps
    Amplitude   | -40..-10 dBm / 0.1 dBm steps
    Phase       | 0..359.9 deg / 0.1 deg steps

*/
#define MASTER
// #define SLAVE

#include "AD9958_Minima.h"

void setup() {
  pinMode(DDS_RESET, OUTPUT);
  pinMode(DDS_SDIO, OUTPUT);
  pinMode(DDS_SCLK, OUTPUT);
  pinMode(DDS_CSB, OUTPUT);
  pinMode(DDS_IOUP, OUTPUT);
  digitalWrite(DDS_CSB, HIGH);
  digitalWrite(DDS_IOUP, HIGH);

  pinMode(A2, INPUT);
  pinMode(A0, INPUT);

  DDS_Reset();
  DDS_Init();
  CALIBRATION_setFreq(0, Ch0);
  CALIBRATION_setFreq(0, Ch1);
  DDS_Update();

  EEPROM.get(0, arraySize);
  allocateArrays(arraySize);
  readProfile();

  CALIBRATION_setAmp(profileASF[0], profileChannel[0]);
  CALIBRATION_setFreq(profileFTW[0], profileChannel[0]);

#ifdef MASTER
  Serial.begin(115200); // Ready for software control via COM
  Wire.begin();
#endif

#ifdef SLAVE
  Wire.begin(localSlaveADR);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
#endif
}

void loop() {
    pinScan();

#ifdef MASTER
  readSerialCommand();
#endif
}

static inline uint32_t freqToFTW(double _freq) {
  // Frequency Tuning Word
  // FTW = (freq_out / f_ref) * 2^32
  return static_cast<uint32_t> (_freq * 8589934.592);
}

static inline uint16_t DegToPOW(double _deg) {
  // Phase Offset Word
  // POW = (P_out / 360) * 2^14
  return static_cast<uint16_t> (_deg * 45.51111111111111);
}

static inline uint32_t dBmToASF(double _dBm, double _freq) {
  // Amplitude Scale Factor
  // Last calibrated at 25-06-24 with CALIBRATION.ipynb
  // The following relationship might deviate on different DDS.
  // ASF = exp[ (P + 46.307) / 8.7355 ]
  // return (uint32_t)(pow(2.71828182846, ((_dBm + 68.975) / 8.684)));
  return static_cast<uint32_t> (exp((_dBm + 68.975) / 8.684));
  // ASF = exp[ 0.1371 * dBm + 0.2659 * ln(freq) + 7.2040]
  // return static_cast<uint32_t> (exp((_dBm / C1) - (C3 / C1) - ((C2 * log(_freq))/ C1)));
  
}

void DDS_Switch() {
  if (!currentStatus) {
    // CALIBRATION_setFreq(profileFTW[0], Ch0);
    // CALIBRATION_setFreq(profileFTW[0], Ch1);
    // CALIBRATION_setAmp(profileASF[0], Ch0);
    // CALIBRATION_setAmp(profileASF[0], Ch1);

    DDS_Update();
    preloadProfile();

    currentStatus = true;

#ifdef MASTER
    Serial.println("Switch ON.");
#endif
  } else if (currentStatus) {
    CALIBRATION_setFreq(0, Ch0);
    CALIBRATION_setFreq(0, Ch1);
    // CALIBRATION_setAmp(ASF[0], Ch0);
    // CALIBRATION_setAmp(ASF[0], Ch1);

    DDS_Update();

    triggerCount = 0;
    CALIBRATION_setAmp(profileASF[triggerCount], profileChannel[triggerCount]);
    CALIBRATION_setFreq(profileFTW[triggerCount], profileChannel[triggerCount]);
    currentStatus = false;

#ifdef MASTER
    Serial.println("Switch OFF.");
#endif
  }
}

void setFreq(double freq, uint8_t channel) {
  unsigned long start = micros();
  uint8_t strBuffer[4];
  // double _C = pow(2,32) / REFCLK_FREQ;
  uint32_t FTW = freqToFTW(freq);


  strBuffer[0] = static_cast<uint8_t> ((0xFF000000 & FTW) >> 24);
  strBuffer[1] = static_cast<uint8_t> ((0x00FF0000 & FTW) >> 16);
  strBuffer[2] = static_cast<uint8_t> ((0x0000FF00 & FTW) >> 8);
  strBuffer[3] = static_cast<uint8_t> (0x000000FF & FTW);

  unsigned long mid = micros();

  setChannel(channel);
  sdioWriteReg(FCR, strBuffer, 4);
  DDS_Update();
  currentFreq = freq;
  unsigned long end = micros();

#ifdef MASTER
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

  strBuffer[0] = static_cast<uint8_t> ((POW & 0x3F00) >> 8);
  strBuffer[1] = static_cast<uint8_t> (POW & 0xFF);

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
  unsigned long mid = micros();
  strBuffer[0] = static_cast<uint8_t> (0x00);
  strBuffer[1] = static_cast<uint8_t> (EMSB);
  strBuffer[2] = static_cast<uint8_t> (ASF & 0xFF);

  setChannel(channel);
  sdioWriteReg(ACR, strBuffer, 3);
  DDS_Update();
  unsigned long end = micros();
#ifdef MASTER
  Serial.print(F("setAmp data Calculate duration: "));
  Serial.println(calculate - start);
  Serial.print(F("setAmp data preparation duration: "));
  Serial.println(mid - start);
  Serial.print(F("setAmp total duration: "));
  Serial.println(end - start);
  Serial.println(ASF);
#endif
}

void CALIBRATION_setAmp(uint32_t CALIBRATION_ASF, uint8_t channelNum) {
#ifdef MASTER
  unsigned long start = micros();
#endif
  uint8_t channel = (channelNum == 0) ? Ch0 : Ch1;
  uint8_t strBuffer[3];
  uint8_t EMSB = (((CALIBRATION_ASF & 0x300) >> 8 | 0x10));

  strBuffer[0] = (uint8_t) (0x00);
  strBuffer[1] = (uint8_t) (EMSB);
  strBuffer[2] = (uint8_t) (CALIBRATION_ASF & 0xFF);

  setChannel(channel);
  sdioWriteReg(ACR, strBuffer, 3);
  // DDS_Update();

#ifdef MASTER
  unsigned long end = micros();
  Serial.print("CAL_AMP duration: ");
  Serial.println(end - start);
#endif
}

void CALIBRATION_setFreq(uint32_t CALIBRATION_FTW, uint8_t channelNum) {
  uint8_t strBuffer[4];
  uint8_t channel = (channelNum == 0) ? Ch0 : Ch1;

  strBuffer[0] = (uint8_t) ((0xFF000000 & CALIBRATION_FTW) >> 24);
  strBuffer[1] = (uint8_t) ((0x00FF0000 & CALIBRATION_FTW) >> 16);
  strBuffer[2] = (uint8_t) ((0x0000FF00 & CALIBRATION_FTW) >> 8);
  strBuffer[3] = (uint8_t) (0x000000FF & CALIBRATION_FTW);

  setChannel(channel);
  sdioWriteReg(FCR, strBuffer, 4);
  // DDS_Update();
}

/*  Analog input triggers  */

void triggerEvent() {
  static bool lastState = LOW;
  bool currentState = HIGH;

  if (currentState && !lastState) {
#ifdef MASTER
    Serial.print(F("A5 triggered."));
#endif

    ++triggerCount;
    triggerCount %= arraySize;
    DDS_Update();
    preloadProfile();
    // CALIBRATION_setFreq(profileFTW[triggerCount], profileChannel[triggerCount]);
    // CALIBRATION_setAmp(profileASF[triggerCount], profileChannel[triggerCount]);

#ifdef MASTER
    Serial.print(F("Set for: "));
    Serial.print(F("["));
    Serial.print(triggerCount);
    Serial.print(F("] "));
    Serial.print(profileFTW[triggerCount]);
    Serial.print(",");
    Serial.print(profileASF[triggerCount]);
    Serial.print(",");
    Serial.println(profileChannel[triggerCount]);
#endif
    currentState = LOW;
  }
  lastState = currentState;
}

void preloadProfile() {
  static uint8_t nextProfile = ((triggerCount + 1) % arraySize);
  CALIBRATION_setFreq(profileFTW[nextProfile], profileChannel[nextProfile]);
  CALIBRATION_setAmp(profileASF[nextProfile], profileChannel[nextProfile]);
}

float readVoltage(uint8_t pin) {return ((analogRead(pin) * 5) >> 10);}

void pinScan() {
  triggeredPin = isPinTriggered();

  if (!isPinOccupied) {
    if (triggeredPin == A2) {
      isPinOccupied = true;
      triggerEvent();
      return;
    }
    if (triggeredPin == A0) {
      isPinOccupied = true;
      DDS_Switch();
      return;
    }
  }
}

uint8_t isPinTriggered() {
  if (readVoltage(A2) > TRIGGER_THRESHOLD) {return A2;}
  if (readVoltage(A0) > TRIGGER_THRESHOLD) {return A0;}
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

void setChannel(uint8_t channel) {
  // setPinLow(1, 10); // CSB - 12; P110
  *PFS_P110PFS_BY = 0x04;
  fastShiftOut(CSR);
  fastShiftOut(channel);
  *PFS_P110PFS_BY = 0x05;
  // setPinHigh(1, 10);
}

inline void DDS_Reset() {
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

void readProfile() {
  uint8_t address = 1;
  uint8_t i;

  for (i = 0; i < arraySize; ++i) {
    EEPROM.get(address, profileFTW[i]);
    address += sizeof(uint32_t);
  }

  for (i = 0; i < arraySize; ++i) {
    EEPROM.get(address, profileASF[i]);
    address += sizeof(uint32_t);
  }

  for (i = 0; i < arraySize; ++i) {
    EEPROM.get(address, profileChannel[i]);
    address += sizeof(uint8_t);
  }
}

void writeProfile() {
  uint8_t address = 1;
  uint8_t i;

  EEPROM.update(0, arraySize);

  for (i = 0; i < arraySize; ++i) {
    EEPROM.put(address, profileFTW[i]);
    address += sizeof(uint32_t);
  }

  for (i = 0; i < arraySize; ++i) {
    EEPROM.put(address, profileASF[i]);
    address += sizeof(uint32_t);
  }

  for (i = 0; i < arraySize; ++i) {
    EEPROM.put(address, profileChannel[i]);
    address += sizeof(uint8_t);
  }
}

void allocateArrays(uint8_t _arraySize) {
  profileFTW = std::make_unique<uint32_t[]>(_arraySize);
  profileASF = std::make_unique<uint32_t[]>(_arraySize);
  profileChannel = std::make_unique<uint8_t[]>(_arraySize);

  for (uint8_t i = 0; i < _arraySize; ++i) {
    profileFTW[i] = 0;
    profileASF[i] = 0;
    profileChannel[i] = 0;
  }
}

bool inRange(int32_t val, int32_t minimum, int32_t maximum) {
// return ((minimum <= val) && (val <= maximum));
  return true;
}

inline void fastShiftOut(uint8_t val) {
  uint8_t i;

  for (i = 0; i < 8; ++i) {
    // MSBFIRST
    if (val & (1 << (7 - i))) {
      *PFS_P112PFS_BY = 0x05;
    } else {
      *PFS_P112PFS_BY = 0x04;
    }
    // Clock pulse
    *PFS_P109PFS_BY = 0x05;
    *PFS_P109PFS_BY = 0x04;
  }
}

/*  Master-Slave Device Manipulation  */
#ifdef MASTER
void handleSlaves(uint8_t deviceId) {
  switch(deviceId) {
    case 0:
      currentSlave = 0x00;
      isLocal = true;
      Serial.println("Selected Device: Local (D0)");
      break;
    case 1:
      currentSlave = 0x08;
      isLocal = false;
      Serial.println("Selected Device: Slave 1 (D1)");
      break;
    default:
      Serial.println("Unknown Device ID");
      return;
  }
}

void sendToSlaves(const char* command, uint8_t length) {
  Wire.beginTransmission(currentSlave);
  Wire.write(command, length);
  Wire.endTransmission();

  Serial.print("Sent ");
  Serial.print(command);
  Serial.print(" to device at ");
  Serial.println(currentSlave);
}

void requestFromSlaves() {
  unsigned long start = millis();
  const uint8_t maxRequestSize = 110;
  uint8_t bytesRead = 0;
  Wire.requestFrom(currentSlave, maxRequestSize);
  slaveResponse = "";

  while (Wire.available() == 0) {
    if (millis() - start >= 40) {
      Serial.println("Error: Timeout waiting from slave.");
      return;
    }
  }

  while (Wire.available() && bytesRead < maxRequestSize) {
    char c = Wire.read();
    slaveResponse += c;
    ++bytesRead;
    if (c == '\n') {
      break;
    }
  }
  Serial.println("Response from slave - " + slaveResponse);
}
#endif

/*  Serial Command  */
#ifdef MASTER

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
  if (!Serial.available()) {return;}

  int counter = Serial.readBytesUntil('\n', SerialBuffer, 110);
  if (counter == 0) {return;}
  SerialBuffer[counter] = '\0';

  GParser data(SerialBuffer, ';');
  int commandsCounter = data.split();

  for (int i = 0; i < commandsCounter; ++i) {
    char command = data[i][0];
    uint8_t index = data[i][1] - '0';
    double value = atof(&data[i][2]);

    switch (command) {

    case 'D':
      handleSlaves(value);
      break;

    case 'M': // Update FTW[]
      if (isLocal) {
        if (index < arraySize) {
          profileFTW[index] = static_cast<uint32_t> (value);
          Serial.println("Updated FTW[" + String(index) + "] = " + String(value)); 
        } else {
          Serial.println("Index out of range of array. Please allocate correct size.");
        }
      } else {
        sendToSlaves(data[i], strlen(data[i]));
        requestFromSlaves();
      }
      break;

    case 'N': // Update ASF[]
      if (isLocal) {
        if (index < arraySize) {
          profileASF[index] = static_cast<uint32_t> (value);
          Serial.println("Updated ASF[" + String(index) + "] = " + String(value));
        } else {
          Serial.println("Index out of range of array. Please allocate correct size and input again.");
      } 
      } else {
        sendToSlaves(data[i], strlen(data[i]));
        requestFromSlaves();
      }
      break;

    case 'B': // Update profileChannel[]
      if (isLocal) {
        if (index < arraySize) {
        profileChannel[index] = static_cast<uint8_t> (value);
        Serial.println("Updated Channel[" + String(index) + "] = " + String(value));
      } else {
        Serial.println("Index out of range of array. Please allocate correct size and input again.");
      } 
      } else {
        sendToSlaves(data[i], strlen(data[i]));
        requestFromSlaves();
      }
      break;

    case 'V':
      if (isLocal) {
        if (value > 0 && value <= 10) {
        arraySize = static_cast<uint8_t> (value);
        allocateArrays(arraySize);
        readProfile();
        Serial.println("Set arraySize to " + String(arraySize));
      } else {
        Serial.println("The size of array should be <= 10.");
      }
      } else {
        sendToSlaves(data[i], strlen(data[i]));
        requestFromSlaves();
      }
      break;

    case 'Y':
      Serial.println(arraySize);
      Serial.println(String(profileFTW[0]) + "," + String(profileFTW[1]) + "," + String(profileFTW[2]));
      Serial.println(String(profileASF[0]) + "," + String(profileASF[1]) + "," + String(profileASF[2]));
      Serial.println(String(profileChannel[0]) + "," + String(profileChannel[1]) + "," + String(profileChannel[2]));
      break;

    case 'W':
      if (isLocal) {
        writeProfile();
      } else {
        sendToSlaves(data[i], strlen(data[i]));
      }
      break;   
    
    case 'C': // Channel
      if (isLocal) {
        if (value == 0 || value == 1) {
          serialChannel = (value == 0) ? Ch0 : Ch1;
          Serial.print(F("Set Channel: "));
          Serial.println(serialChannel);
        } else {
          Serial.println("Incorrect Channel.");
        };
      } else {
        sendToSlaves(data[i], strlen(data[i]));
        requestFromSlaves();
      }
      break;

    case 'F': // Frequency
      if (isLocal) {
        if (serialChannel == -1) {
        Serial.println("The input channel is not selected.");
        }
        if (inRange(value, MIN_FREQ, MAX_FREQ)) {
        serialFrequency = static_cast<double> (value);
        setFreq(serialFrequency, serialChannel);

        Serial.print(F("Set Freq: "));
        Serial.print(value);
        Serial.println(F(" MHz."));
      } else {
        Serial.println("Frequency is OUT OF RANGE (0.1 - 225)");
      }
      } else {
        sendToSlaves(data[i], strlen(data[i]));
        requestFromSlaves();
      }
      break;

    case 'A': // Amplitude, dBm
      if (isLocal) {
        if (inRange(value, MIN_AMPL, MAX_AMPL)) {
        serialAmplitude = static_cast<double> (value);
        setAmp(serialAmplitude, serialChannel);

        Serial.print(F("Set Amplitude: "));
        Serial.print(value);
        Serial.println(F(" dBm."));
      } else
        Serial.println("Power is OUT OF RANGE (-" + String(MAX_AMPL) + " - " + String(MIN_AMPL) + ")");
      } else {
        sendToSlaves(data[i], strlen(data[i]));
        requestFromSlaves();
      }
      break;

    case 'U':
      if (isLocal) {
        CALIBRATION_setAmp(static_cast<uint32_t>(value), serialChannel);
      } else {
        sendToSlaves(data[i], strlen(data[i]));
      }
      break;

    case 'I':
      if (isLocal) {
        CALIBRATION_setFreq(static_cast<uint32_t>(value), serialChannel);
      }

    case 'X':
      if (isLocal) {
        DDS_Switch();
      } else {
        sendToSlaves(data[i], strlen(data[i]));
        requestFromSlaves();
      }
      break;

    case 'R':
      if (isLocal) {
        readProfile();
      } else {
        sendToSlaves(data[i], strlen(data[i]));
        requestFromSlaves();
      }
      break;

    case 'S':
      scanI2CDevices();
      break;

    default:
      Serial.print(F("Unknown command:"));
      Serial.println(command);
      Serial.println((const __FlashStringHelper *)HELP_STRING);
    } // switch
  } // for
}

void scanI2CDevices() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning for I2C devices...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);

      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}
#endif



#ifdef SLAVE
void requestEvent() {
  slaveResponse += "\n";
  uint8_t bytesToSend = slaveResponse.length();

  if (bytesToSend > I2C_BUFFER_SIZE) {
    bytesToSend = I2C_BUFFER_SIZE;
  }

  Wire.write(slaveResponse.c_str(), bytesToSend);
  slaveResponse = "";
}

void receiveEvent(int count) {
  if (count > 0) {
    if (count >= I2C_BUFFER_SIZE) {
      count = I2C_BUFFER_SIZE - 1;
    }

    Wire.readBytesUntil('\n', I2CBuffer, count);
    I2CBuffer[count] = '\0';

    GParser data(I2CBuffer, ';');
    int commandsCounter = data.split();

    slaveResponse = "D[" + String(localSlaveADR, HEX) + "]: ";
    for (int i = 0; i < commandsCounter; ++i) {
      char command = data[i][0];
      uint8_t index = data[i][1] - '0';
      uint32_t value = atof(&data[i][2]);

      switch(command) {
        case 'M': // Update FTW[]
          if (index < arraySize) {
            profileFTW[index] = value;
            slaveResponse += "Updated FTW[" + String(index) + "] = " + String(value);
          } else {
            slaveResponse += "Index out of range of array. Please allocate correct size.";
          }
          break;

        case 'N': // Update ASF[]
          if (index < arraySize) {
            profileASF[index] = value;
            slaveResponse += "Updated ASF[" + String(index) + "] = " + String(value);
          } else {
            slaveResponse += "Index out of range of array. Please allocate correct size and input again.";
          }
          break;

        case 'B': // Update profileChannel[]
          if (index < arraySize) {
            profileChannel[index] = value;
            slaveResponse += "Updated Channel[" + String(index) + "] = " + String(value);
          } else {
            slaveResponse += "Index out of range of array. Please allocate correct size and input again.";
          }
          break;

        case 'V':
          if (value > 0 && value <= 10) {
            arraySize = static_cast<uint8_t> (value);
            allocateArrays(arraySize);
            readProfile();
            slaveResponse += "Set arraySize to " + String(arraySize);
          } else {
            slaveResponse += "The size of array should be <= 10.";
          }
          break;

        case 'W':
          writeProfile();
          break;

        case 'C':
          if (value == 0 || value == 1) {
            serialChannel = value;

            slaveResponse += "Set Channel: " + String(serialChannel);
          } else {
            slaveResponse += "Incorrect Channel. ";
          }
          break;

        case 'F': // Frequency
          if (serialChannel == -1) {
            Serial.println("The input channel is not selected.");
          }
          if (inRange(value, MIN_FREQ, MAX_FREQ)) {
            serialFrequency = (double)value;
            setFreq(serialFrequency, serialChannel);

            slaveResponse += "Set Freq: " + String(value) + " MHz.";
          } else
            slaveResponse += "Frequency is OUT OF RANGE (0.1 - 225 MHz).";
          break;

        case 'A': // Amplitude, dBm
          if (inRange(value, MIN_AMPL, MAX_AMPL)) {
            serialAmplitude = (double)value;
            setAmp(serialAmplitude, serialChannel);

            slaveResponse += "Set Amplitude: " + String(value) + " dBm.";
          } else
            slaveResponse += "Amplitude is OUT OF RANGE. Please check datasheet.";
          break;

        case 'U':
          CALIBRATION_ASF = (uint32_t)value;
          CALIBRATION_setAmp(CALIBRATION_ASF, serialChannel);
          break;

        case 'X':
          DDS_Switch();
          slaveResponse += "Switched ON.";
          break;

        case 'R':
          Serial.println(readVoltage(A5));
          Serial.println(readVoltage(A0));
          Serial.println(triggeredPin);
          break;

        default:
          slaveResponse += "Unknown command.";
          break;
      } // switch
    }
  }
}
#endif
