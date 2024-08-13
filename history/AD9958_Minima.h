#define DEBUG_H

#include "PORTS.h"
#include <GParser.h>
#include <Wire.h>

/*  Pin Definition  */

#define DDS_RESET 5 // Reset pin / RST
#define DDS_IOUP 9  // IO Update pin / SDIO
#define DDS_SDIO 10 // SDAT pin / SDIO
#define DDS_SCLK 11 // SCLK pin
#define DDS_CSB 12  // Chip Select pin / CSB

// #define FREQ_DIG_IN 2 // Profile select pins. Digital Input.
// #define AMPT_DIG_IN 3
// #define CHAN_DIG_IN 4
// #define DIG_RESET 7 // Profile reset pin. Digital Input.

#define MIN_FREQ 100000    // 100 kHz
#define MAX_FREQ 225000000 // 225 MHz

#define MIN_AMPL -40 // dBm
#define MAX_AMPL -10
#define MIN_ASF 1
#define MAX_ASF 1023 // 2^10 - 1

#define C1 0.1371
#define C2 0.2659
#define C3 7.2040
#define TRIGGER_THRESHOLD 4.0

/*      PORT MACRO      */

// #define R_PORT ((volatile R_PORT_Type *)0x40040000U)
// #define R_PFS  ((volatile R_PFS_Type *)0x40040900U)

#define setPinMode(port, pin, mode)                                            \
  do {                                                                         \
    if (mode == OUTPUT) {                                                      \
      R_PORT##port->PDR |= bit(pin);                                           \
    } else {                                                                   \
      R_PORT##port->PDR &= ~bit(pin);                                          \
    }                                                                          \
  } while (0)

#define setPinHigh(port, pin)                                                  \
  do {                                                                         \
    R_PORT##port->POSR = bit(pin);                                             \
  } while (0)

#define setPinLow(port, pin)                                                   \
  do {                                                                         \
    R_PORT##port->PORR = bit(pin);                                             \
  } while (0)

#define readPin(port, pin) (R_PORT##port->PIDR & bit(pin))

#define conversionADC() (*ADC140_ADCSR |= (1 << 15))
#define conversionWait() while (!(*ADC140_ADCSR & (1 << 15)))

/*      Declarations        */

volatile unsigned long triggerTime = 0;
volatile unsigned long setTime = 0;

const double REFCLK_FREQ = 25000000;  // 25 MHz reference clock
const double SYSCLK_FREQ = 500000000; // 500 MHz system clock

const uint8_t CSR = 0x00; // Channel select register
const uint8_t FR1 = 0x01; // Function register 1
const uint8_t FR2 = 0x02; // Function register 2
const uint8_t CFR = 0x03; // Channel function register
const uint8_t FCR = 0x04; // Frequency control register
const uint8_t PCR = 0x05; // Phase control register
const uint8_t ACR = 0x06; // Amplitude control register

const uint8_t Ch0 = 0x70;
const uint8_t Ch1 = 0xB0;

volatile bool currentStatus = false;
volatile bool isPinOccupied = false;
volatile uint8_t triggeredPin = 0;
volatile uint8_t selectedChannel = Ch1;
volatile uint8_t frequencyCount = 0;
volatile uint8_t amplitudeCount = 0;
volatile uint8_t channelCount = 0;

double serialFrequency;
double serialAmplitude;
uint8_t serialChannel;
int currentChannel = 1;

// Define own steps; Also change for the mod number in triggerFreq(),
// triggerAmp().
const double arrayFrequency[] = {80, 120, 80, 120};
const double arrayAmplitude[] = {-30, -20, -30, -18};

// Fast look-up table
const uint32_t FTW[] = {1030792192, 687194816, 1030792192, 687194816};

const uint32_t ASF[] = {706, 281, 706, 281};

// static bool bufferInUse = false;

// Default output when start
double Ch0_FREQ = 80.0; // MHz
double Ch1_FREQ = 80.0;
double Ch0_PHAS = 0; // degree
double Ch1_PHAS = 0;
double Ch0_AMPT = -12.0; // dBm
double Ch1_AMPT = -12.0;

double currentFreq;
double curretnAmp;

uint32_t CALIBRATION_ASF = 1;
void CALIBRATION_setAmp(uint32_t CALIBRATION_ASF, uint8_t channel);
void CALIBRATION_setFreq(uint32_t CALIBRATION_FTW, uint8_t channel);

void pinScan();
void DDS_Switch();
void DDS_Init();
void setFreq(double freq, uint8_t channel);
void setAmp(double dBm, uint8_t channel);
void setDeg(double deg, uint8_t channel);
void sdioWriteReg(uint8_t reg, uint8_t *strBuffer, int size);
inline void DDS_Reset();
inline void DDS_Update();
inline void setChannel(uint8_t channel);
inline void pulse(uint8_t pin);
inline void fastShiftOut(uint8_t val) __attribute__((always_inline));

void triggerFreq();
void triggerAmp();
void triggerChannel();
void resetIndex();
uint8_t isPinTriggered();
float readVoltage(uint8_t pin);
uint16_t fastVoltageRead(uint8_t pin);

#ifdef DEBUG_H
void readSerialCommand();
#endif
