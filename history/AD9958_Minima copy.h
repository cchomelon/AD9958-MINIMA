#define MASTER_H
// #define SLAVE_H

#include "PORTS.h"
#include <GParser.h>
#include <Wire.h>
#include <EEPROM.h>
#include <memory>

/*  Pin Definition  */

#define DDS_RESET 5 // Reset pin / RST
#define DDS_IOUP 9  // IO Update pin / SDIO
#define DDS_SDIO 10 // SDAT pin / SDIO
#define DDS_SCLK 11 // SCLK pin
#define DDS_CSB 12  // Chip Select pin / CSB

constexpr uint32_t MIN_FREQ = 100000;    // 100 kHz
constexpr uint32_t MAX_FREQ = 225000000; // 225 MHz

constexpr uint32_t MIN_AMPL = -40; // dBm
constexpr uint32_t MAX_AMPL = -10;
constexpr uint32_t MIN_ASF = 1;
constexpr uint32_t MAX_ASF = 1023; // 2^10 - 1

double C1 = 0.1371;
double C2 = 0.2659;
double C3 = 7.2040;
constexpr float TRIGGER_THRESHOLD = 4.0;

/*      PORT MACRO      */

// #define R_PORT ((volatile R_PORT_Type *)0x40040000U)
// #define R_PFS  ((volatile R_PFS_Type *)0x40040900U)

#define readPin(port, pin) (R_PORT##port->PIDR & bit(pin))

uint8_t currentSlave = 0x00;

#ifdef SLAVE
#define localSlaveADR 0x08
#endif

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
volatile uint8_t triggerCount = 0;

double serialFrequency;
double serialAmplitude;
uint8_t serialChannel;
int currentChannel = 1;

// Define own steps;
uint8_t arraySize = 10;
// Fast look-up table
std::unique_ptr<uint32_t[]> profileFTW;
std::unique_ptr<uint32_t[]> profileASF;
std::unique_ptr<uint8_t[]> profileChannel;
void readProfile();
void writeProfile();
void preloadProfile();
void allocateArrays(uint8_t _arraySize);
bool isLocal = true;

// static bool bufferInUse = false;

// Default output when start
// double Ch0_FREQ = 80.0; // MHz
// double Ch1_FREQ = 80.0;
// double Ch0_PHAS = 0; // degree
// double Ch1_PHAS = 0;
// double Ch0_AMPT = -12.0; // dBm
// double Ch1_AMPT = -12.0;

double currentFreq;
double curretnAmp;
double currentPhase;

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
void DDS_Reset() __attribute__((always_inline));
void DDS_Update() __attribute__((always_inline));
void setChannel(uint8_t channel);
void fastShiftOut(uint8_t val) __attribute__((always_inline));

void triggerEvent();
uint8_t isPinTriggered();
float readVoltage(uint8_t pin);

#ifdef MASTER_H
void readSerialCommand();
char SerialBuffer[110];
char I2CBuffer[110];
String slaveResponse = "";
void handleSlaves(uint8_t deviceId);
void sendToSlaves(const char* command, uint8_t length);
void requestFromSlaves();
void scanI2CDevices();
#endif

#ifdef SLAVE_H
constexpr uint8_t I2C_BUFFER_SIZE = 110;
char I2CBuffer[I2C_BUFFER_SIZE];
String slaveResponse = "";
void receiveEvent(int count);
void requestEvent();
#endif
