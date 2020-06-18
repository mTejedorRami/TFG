#ifndef PPG_H
#define PPG_H


//uint8_t MAX30101_checkForBeat(int32_t sample);

/*******/

float MAX30101_medirTemperaturaPPG(void);
void MAX30101_setupPPG(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange);
void MAX30101_setPulseAmplitudeRed(uint8_t amplitude);
void MAX30101_setPulseAmplitudeGreen(uint8_t amplitude);
void MAX30101_setPulseAmplitudeIR(uint8_t amplitude);
void MAX30101_enableDIETEMPRDY(void);
void MAX30101_disableDIETEMPRDY(void);
double MAX30101_readTemperature(void);

uint32_t MAX30101_getIR(void);
//uint8_t MAX30101_checkForBeat(int32_t sample);
uint8_t MAX30101_checkForBeat(int32_t sample);
uint8_t MAX30101_checkForBP(int32_t sample, uint32_t extremosPPG[6]);

//Sigiente estructura aquí para hacerla accesible desde main para pruebas
//The MAX30105 stores up to 32 samples on the IC
//This is additional local storage to the microcontroller
#define STORAGE_SIZE 32 //Each long is 4 bytes so limit this to fit on your micro
struct Record
{
  uint32_t red[STORAGE_SIZE];
  uint32_t IR[STORAGE_SIZE];
  uint32_t green[STORAGE_SIZE];
  uint16_t head; //uint8_t head;
  //uint8_t tail;
} sense; //This is our circular buffer of readings from the sensor


#endif
