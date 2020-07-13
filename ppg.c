/*
 * Código procedente de librería: https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library
 * Traducido a lenguaje C y adaptado a MAX30101
 * Modificado: función "check", defines
 */
#include "main.h"
#include "ppg.h"
#include "adc.h"
#include "i2c.h"
#include <string.h>

int16_t IR_AC_Max = 20;
int16_t IR_AC_Min = -20;

int16_t IR_AC_Signal_Current = 0;
int16_t IR_AC_Signal_Previous;
int16_t IR_AC_Signal_min = 0;
int16_t IR_AC_Signal_max = 0;
int16_t IR_Average_Estimated;
uint32_t IR_Time_Previous=0;
uint32_t IR_Time_Current=0;

uint32_t i = 1;
//1 pendiente positiva 0 negativa
uint8_t pendiente = 0;
uint8_t pendienteAnterior = 0;

uint8_t picoDicroDetectado = 0;
uint8_t picoMaxDetectado = 0;
uint8_t j = 0;
uint8_t k = 0;
uint8_t p = 0;
uint16_t picoMaximo[50];
uint16_t picoMinimo[50];
uint16_t picoMaximoValor[50];
uint16_t picoMinimoValor[50];
uint16_t picoDicrotico[50];

int16_t positiveEdge = 0;
int16_t negativeEdge = 0;
int32_t ir_avg_reg = 0;

int16_t cbuf[32];
uint8_t offset = 0;

static const uint16_t FIRCoeffs[12] = {172, 321, 579, 927, 1360, 1858, 2390, 2916, 3391, 3768, 4012, 4096};


//  Integer multiplier
int32_t mul16(int16_t x, int16_t y)
{
  return((long)x * (long)y);
}

//  Average DC Estimator
int16_t MAX30101_averageDCEstimator(int32_t *p, uint16_t x)
{
  *p += ((((long) x << 15) - *p) >> 4);  //Es una especie de filtro paso bajo con banda muy estrecha. Cambiando el 4 puedes decidir de cuántas muestras hace la media. Si subes el valor da una continua con menos rizado, aunque tarda más en estabilizarse
  return (*p >> 15);
}

//  Low Pass FIR Filter
int16_t MAX30101_lowPassFIRFilter(int16_t din)
{
  cbuf[offset] = din;

  int32_t z = mul16(FIRCoeffs[11], cbuf[(offset - 11) & 0x1F]);

  for (uint8_t i = 0 ; i < 11 ; i++)
  {
    z += mul16(FIRCoeffs[i], cbuf[(offset - i) & 0x1F] + cbuf[(offset - 22 + i) & 0x1F]);
  }

  offset++;
  offset %= 32; //Wrap condition

  return(z >> 15);
}

uint8_t MAX30101_checkForBP(int32_t sample, uint32_t extremosPPG[3])
{
	uint8_t bpDetected = 0;
	IR_Time_Previous= IR_Time_Current;
	IR_Time_Current= HAL_GetTick();

	IR_AC_Signal_Previous = IR_AC_Signal_Current;
	pendienteAnterior = pendiente;

	IR_Average_Estimated = MAX30101_averageDCEstimator(&ir_avg_reg, sample);
	IR_AC_Signal_Current = sample - IR_Average_Estimated;

	if(IR_AC_Signal_Current>=(IR_AC_Signal_Previous )){
		pendiente = 1;
	}
	else{
		pendiente = 0;
	}

	//Acotar máximos y minimos
	if((i>110) & ( i<150)){
		if(IR_AC_Signal_Current > IR_AC_Signal_max){
			IR_AC_Signal_max = IR_AC_Signal_Current;
		}

		if(IR_AC_Signal_Current < IR_AC_Signal_min){
			IR_AC_Signal_min = IR_AC_Signal_Current;
		}
	}
	int16_t IR_AC_Signal_Incremento = IR_AC_Signal_max - IR_AC_Signal_min;

	if(i>150){
		//Encontrar máximo
		if((pendienteAnterior == 1) & (pendiente == 0) &
		 (IR_AC_Signal_Previous >=(IR_AC_Signal_max - IR_AC_Signal_Incremento*0.30))
		){
			if((picoMaxDetectado == 1) & (i-picoMaximo[j-1]>20) &(i>200)){
				//Si detecta dos máximos seguidos vuelve a calibrar
				i=110;
			}
			else{
				if(((i-picoMaximo[j-1]) < 10) &( picoMaximoValor[j-1]< IR_AC_Signal_Previous)&(j>0)){
					j--;

				}
				picoMaximo[j]= i;
				picoMaximoValor[j]= IR_AC_Signal_Previous;
				j++;
				IR_AC_Signal_max = IR_AC_Signal_Previous;
				extremosPPG[0]=IR_Time_Previous;
				picoMaxDetectado = 1;
			}
		}
		//He encontrado un punto dicrótico
		if((pendienteAnterior == 1) & (pendiente == 0) &(IR_AC_Signal_Current <=(IR_AC_Signal_max - IR_AC_Signal_Incremento*0.25))&
			(IR_AC_Signal_Current >=(IR_AC_Signal_min + IR_AC_Signal_Incremento*0.1))&
			(picoDicroDetectado  == 0) & (picoMaxDetectado == 0)){
				picoDicrotico[p]= i;
				p++;
				picoDicroDetectado  = 1;
				extremosPPG[2]=IR_Time_Previous;
				bpDetected = 1;
		}

		if((pendienteAnterior == 0) &( pendiente == 1) ){
			//He encontrado un mínimo
			if((IR_AC_Signal_Previous <=(IR_AC_Signal_min + IR_AC_Signal_Incremento*0.25))
			){
				//Si detecta dos mínimos seguidos vuelve a calibrar
				if((picoMaxDetectado == 0) & (i>200) & (i-picoMinimo[k-1]>20)){
					i=110;
				}
				else if((i-picoMinimo[k-1])<5){

				}
				else{
					picoMinimo[k]= i;
					picoMinimoValor[k]= IR_AC_Signal_Previous;
					k++;
					picoDicroDetectado  = 0;
					IR_AC_Signal_min = IR_AC_Signal_Previous;
					extremosPPG[1]=IR_Time_Previous;
					picoMaxDetectado = 0;
				}
			}
		}
	}
i++;
return bpDetected;
}




//  Heart Rate Monitor functions takes a sample value and the sample number
//  Returns true if a beat is detected
//  A running average of four samples is recommended for display on the screen.
//uint8_t MAX30101_checkForBeat(int32_t sample)
uint8_t MAX30101_checkForBeat(int32_t sample)
{
	uint8_t beatDetected = 0;
  //  Save current state
  IR_AC_Signal_Previous = IR_AC_Signal_Current;

  //  Process next data sample
  IR_Average_Estimated = MAX30101_averageDCEstimator(&ir_avg_reg, sample);
  IR_AC_Signal_Current = MAX30101_lowPassFIRFilter(sample - IR_Average_Estimated);

  //  Detect positive zero crossing (rising edge)
  if ((IR_AC_Signal_Previous < 0) & (IR_AC_Signal_Current >= 0))
  {
    IR_AC_Max = IR_AC_Signal_max; //Adjust our AC max and min
    IR_AC_Min = IR_AC_Signal_min;

    positiveEdge = 1;
    negativeEdge = 0;
    IR_AC_Signal_max = 0;

        if (((IR_AC_Max - IR_AC_Min) > 20)){
      //Heart beat!!!
      beatDetected = 1;
    }
  }

  //  Detect negative zero crossing (falling edge)
  if ((IR_AC_Signal_Previous > 0) & (IR_AC_Signal_Current <= 0))
  {
    positiveEdge = 0;
    negativeEdge = 1;
    IR_AC_Signal_min = 0;
  }

  //  Find Maximum value in positive cycle
  if (positiveEdge & (IR_AC_Signal_Current > IR_AC_Signal_Previous))
  {
    IR_AC_Signal_max = IR_AC_Signal_Current;
  }

  //  Find Minimum value in negative cycle
  if (negativeEdge & (IR_AC_Signal_Current < IR_AC_Signal_Previous))
  {
    IR_AC_Signal_min = IR_AC_Signal_Current;
  }


  /*extremosPPG[0] = IR_AC_Signal_max;
  extremosPPG[1] = IR_AC_Signal_min;*/

  return(beatDetected);
}

/*****/

// Status Registers
//static const uint8_t MAX30101_INTSTAT1 =		0x00;
static const uint8_t MAX30101_INTSTAT2 =		0x01;
//static const uint8_t MAX30101_INTENABLE1 =		0x02;
static const uint8_t MAX30101_INTENABLE2 =		0x03;

// FIFO Registers
static const uint8_t MAX30101_FIFOWRITEPTR = 	0x04;
static const uint8_t MAX30101_FIFOOVERFLOW = 	0x05;
static const uint8_t MAX30101_FIFOREADPTR = 	0x06;
static const uint8_t MAX30101_FIFODATA =		0x07;

// Configuration Registers
static const uint8_t MAX30101_FIFOCONFIG = 		0x08;
static const uint8_t MAX30101_MODECONFIG = 		0x09;
static const uint8_t MAX30101_PARTICLECONFIG = 	0x0A;    // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
static const uint8_t MAX30101_LED1_PULSEAMP = 	0x0C;
static const uint8_t MAX30101_LED2_PULSEAMP = 	0x0D;
static const uint8_t MAX30101_LED3_PULSEAMP = 	0x0E;
/// NO EN MAX30101 static const uint8_t MAX30101_LED_PROX_AMP = 	0x10;
static const uint8_t MAX30101_MULTILEDCONFIG1 = 0x11;
static const uint8_t MAX30101_MULTILEDCONFIG2 = 0x12;

// Die Temperature Registers
static const uint8_t MAX30101_DIETEMPINT = 		0x1F;
static const uint8_t MAX30101_DIETEMPFRAC = 	0x20;
static const uint8_t MAX30101_DIETEMPCONFIG = 	0x21;

// Proximity Function Registers
//static const uint8_t MAX30101_PROXINTTHRESH = 	0x30;

// Part ID Registers
//static const uint8_t MAX30101_REVISIONID = 		0xFE;
//static const uint8_t MAX30101_PARTID = 			0xFF;    // Should always be 0x15. Identical to MAX30102.

// MAX30101 Commands
// Interrupt configuration (pg 13, 14)
//static const uint8_t MAX30101_INT_A_FULL_MASK =		(uint8_t)~0b10000000;
//static const uint8_t MAX30101_INT_A_FULL_ENABLE = 	0x80;
//static const uint8_t MAX30101_INT_A_FULL_DISABLE = 	0x00;
//
//static const uint8_t MAX30101_INT_DATA_RDY_MASK = (uint8_t)~0b01000000;
//static const uint8_t MAX30101_INT_DATA_RDY_ENABLE =	0x40;
//static const uint8_t MAX30101_INT_DATA_RDY_DISABLE = 0x00;
//
//static const uint8_t MAX30101_INT_ALC_OVF_MASK = (uint8_t)~0b00100000;
//static const uint8_t MAX30101_INT_ALC_OVF_ENABLE = 	0x20;
//static const uint8_t MAX30101_INT_ALC_OVF_DISABLE = 0x00;
//
//static const uint8_t MAX30101_INT_PROX_INT_MASK = (uint8_t)~0b00010000;
//static const uint8_t MAX30101_INT_PROX_INT_ENABLE = 0x10;
//static const uint8_t MAX30101_INT_PROX_INT_DISABLE = 0x00;
//
static const uint8_t MAX30101_INT_DIE_TEMP_RDY_MASK = (uint8_t)~0b00000010;
static const uint8_t MAX30101_INT_DIE_TEMP_RDY_ENABLE = 0x02;
static const uint8_t MAX30101_INT_DIE_TEMP_RDY_DISABLE = 0x00;

static const uint8_t MAX30101_SAMPLEAVG_MASK =	(uint8_t)~0b11100000;
static const uint8_t MAX30101_SAMPLEAVG_1 = 	0x00;
static const uint8_t MAX30101_SAMPLEAVG_2 = 	0x20;
static const uint8_t MAX30101_SAMPLEAVG_4 = 	0x40;
static const uint8_t MAX30101_SAMPLEAVG_8 = 	0x60;
static const uint8_t MAX30101_SAMPLEAVG_16 = 	0x80;
static const uint8_t MAX30101_SAMPLEAVG_32 = 	0xA0;

static const uint8_t MAX30101_ROLLOVER_MASK = 	0xEF;
static const uint8_t MAX30101_ROLLOVER_ENABLE = 0x10;
//static const uint8_t MAX30101_ROLLOVER_DISABLE = 0x00;
//
//static const uint8_t MAX30101_A_FULL_MASK = 	0xF0;

// Mode configuration commands (page 19)
//static const uint8_t MAX30101_SHUTDOWN_MASK = 	0x7F;
//static const uint8_t MAX30101_SHUTDOWN = 		0x80;
//static const uint8_t MAX30101_WAKEUP = 			0x00;

static const uint8_t MAX30101_RESET_MASK = 		0xBF;
static const uint8_t MAX30101_RESET = 			0x40;

static const uint8_t MAX30101_MODE_MASK = 		0xF8;
static const uint8_t MAX30101_MODE_REDONLY = 	0x02;
static const uint8_t MAX30101_MODE_REDIRONLY = 	0x03;
static const uint8_t MAX30101_MODE_MULTILED = 	0x07;

// Particle sensing configuration commands (pgs 19-20)
static const uint8_t MAX30101_ADCRANGE_MASK = 	0x9F;
static const uint8_t MAX30101_ADCRANGE_2048 = 	0x00;
static const uint8_t MAX30101_ADCRANGE_4096 = 	0x20;
static const uint8_t MAX30101_ADCRANGE_8192 = 	0x40;
static const uint8_t MAX30101_ADCRANGE_16384 = 	0x60;

static const uint8_t MAX30101_SAMPLERATE_MASK = 0xE3;
static const uint8_t MAX30101_SAMPLERATE_50 = 	0x00;
static const uint8_t MAX30101_SAMPLERATE_100 = 	0x04;
static const uint8_t MAX30101_SAMPLERATE_200 = 	0x08;
static const uint8_t MAX30101_SAMPLERATE_400 = 	0x0C;
static const uint8_t MAX30101_SAMPLERATE_800 = 	0x10;
static const uint8_t MAX30101_SAMPLERATE_1000 = 0x14;
static const uint8_t MAX30101_SAMPLERATE_1600 = 0x18;
static const uint8_t MAX30101_SAMPLERATE_3200 = 0x1C;

static const uint8_t MAX30101_PULSEWIDTH_MASK = 0xFC;
static const uint8_t MAX30101_PULSEWIDTH_69 = 	0x00;
static const uint8_t MAX30101_PULSEWIDTH_118 = 	0x01;
static const uint8_t MAX30101_PULSEWIDTH_215 = 	0x02;
static const uint8_t MAX30101_PULSEWIDTH_411 = 	0x03;

//Multi-LED Mode configuration (pg 22)
static const uint8_t MAX30101_SLOT1_MASK = 		0xF8;
static const uint8_t MAX30101_SLOT2_MASK = 		0x8F;
static const uint8_t MAX30101_SLOT3_MASK = 		0xF8;
static const uint8_t MAX30101_SLOT4_MASK = 		0x8F;

//static const uint8_t SLOT_NONE = 				0x00;
static const uint8_t SLOT_RED_LED = 			0x01;
static const uint8_t SLOT_IR_LED = 				0x02;
static const uint8_t SLOT_GREEN_LED = 			0x03;
//static const uint8_t SLOT_NONE_PILOT = 			0x04;
//static const uint8_t SLOT_RED_PILOT =			0x05;
//static const uint8_t SLOT_IR_PILOT = 			0x06;
//static const uint8_t SLOT_GREEN_PILOT = 		0x07;
//
//static const uint8_t MAX_30105_EXPECTEDPARTID = 0x15;

////The MAX30101 stores up to 32 samples on the IC
////This is additional local storage to the microcontroller
//#define STORAGE_SIZE 32 //Each long is 4 bytes so limit this to fit on your micro
//struct Record
//{
//  uint32_t red[STORAGE_SIZE];
//  uint32_t IR[STORAGE_SIZE];
//  uint32_t green[STORAGE_SIZE];
//  uint8_t head;
//  uint8_t tail;
//} sense; //This is our circular buffer of readings from the sensor

//activeLEDs is the number of channels turned on, and can be 1 to 3. 2 is common for Red+IR.
uint8_t activeLEDs; //Gets set during setup. Allows check() to calculate how many bytes to read from FIFO

//Para pruebas printf, hacer local_data global
#define BUFFER_LENGTH 32
static uint8_t local_data[BUFFER_LENGTH * 9];  //Lectura de los LEDs


#define DIR_PPG 0XAE  //Dirección del AFE como esclavo I2C


uint8_t MAX30101_readRegister8(uint8_t address, uint8_t reg){
	uint8_t lect;
	HAL_I2C_Master_Transmit(&hi2c1, address, &reg, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, address, &lect, 1, 10);
	return lect;
}

void MAX30101_writeRegister8(uint8_t address, uint8_t reg, uint8_t value) {
	uint8_t bufferPPG[2];
	bufferPPG[0] = reg;		//Dirección del registro
	bufferPPG[1] = value;	//Contenido a enviar
	HAL_I2C_Master_Transmit(&hi2c1, address, &bufferPPG[0], 2, 10);
}


//Given a register, read it, mask it, and then set the thing
void MAX30101_bitMask(uint8_t reg, uint8_t mask, uint8_t thing)
{
  // Grab current register context
  uint8_t originalContents = MAX30101_readRegister8(DIR_PPG, reg);

  // Zero-out the portions of the register we're interested in
  originalContents = originalContents & mask;

  // Change contents
  MAX30101_writeRegister8(DIR_PPG, reg, originalContents | thing);
}

void MAX30101_softReset(void) {
	MAX30101_bitMask(MAX30101_MODECONFIG, MAX30101_RESET_MASK, MAX30101_RESET);

  // Poll for bit to clear, reset is then complete
  // Timeout after 100ms
  unsigned long startTime = HAL_GetTick();
  while ((HAL_GetTick() - startTime) < 100)
  {
    uint8_t response = MAX30101_readRegister8(DIR_PPG, MAX30101_MODECONFIG);
    if ((response & MAX30101_RESET) == 0) break; //We're done!
    HAL_Delay(1); //Let's not over burden the I2C bus
  }
}

//Set sample average (Table 3, Page 18)
void MAX30101_setFIFOAverage(uint8_t numberOfSamples) {
	MAX30101_bitMask(MAX30101_FIFOCONFIG, MAX30101_SAMPLEAVG_MASK, numberOfSamples);
}

//Enable roll over if FIFO over flows
void MAX30101_enableFIFORollover(void) {
	MAX30101_bitMask(MAX30101_FIFOCONFIG, MAX30101_ROLLOVER_MASK, MAX30101_ROLLOVER_ENABLE);
}

void MAX30101_setLEDMode(uint8_t mode) {
  // Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
  // See datasheet, page 19
	MAX30101_bitMask(MAX30101_MODECONFIG, MAX30101_MODE_MASK, mode);
}

void MAX30101_setADCRange(uint8_t adcRange) {
  // adcRange: one of MAX30101_ADCRANGE_2048, _4096, _8192, _16384
	MAX30101_bitMask(MAX30101_PARTICLECONFIG, MAX30101_ADCRANGE_MASK, adcRange);
}

void MAX30101_setSampleRate(uint8_t sampleRate) {
  // sampleRate: one of MAX30101_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
	MAX30101_bitMask(MAX30101_PARTICLECONFIG, MAX30101_SAMPLERATE_MASK, sampleRate);
}

void MAX30101_setPulseWidth(uint8_t pulseWidth) {
  // pulseWidth: one of MAX30101_PULSEWIDTH_69, _188, _215, _411
	MAX30101_bitMask(MAX30101_PARTICLECONFIG, MAX30101_PULSEWIDTH_MASK, pulseWidth);
}

// NOTE: Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
// See datasheet, page 21
void MAX30101_setPulseAmplitudeRed(uint8_t amplitude) {
  MAX30101_writeRegister8(DIR_PPG, MAX30101_LED1_PULSEAMP, amplitude);
}

void MAX30101_setPulseAmplitudeIR(uint8_t amplitude) {
  MAX30101_writeRegister8(DIR_PPG, MAX30101_LED2_PULSEAMP, amplitude);
}

void MAX30101_setPulseAmplitudeGreen(uint8_t amplitude) {
  MAX30101_writeRegister8(DIR_PPG, MAX30101_LED3_PULSEAMP, amplitude);
}

//Given a slot number assign a thing to it
//Devices are SLOT_RED_LED or SLOT_RED_PILOT (proximity)
//Assigning a SLOT_RED_LED will pulse LED
//Assigning a SLOT_RED_PILOT will ??
void MAX30101_enableSlot(uint8_t slotNumber, uint8_t device) {

//  uint8_t originalContents;

  switch (slotNumber) {
    case (1):
		MAX30101_bitMask(MAX30101_MULTILEDCONFIG1, MAX30101_SLOT1_MASK, device);
      break;
    case (2):
		MAX30101_bitMask(MAX30101_MULTILEDCONFIG1, MAX30101_SLOT2_MASK, device << 4);
      break;
    case (3):
		MAX30101_bitMask(MAX30101_MULTILEDCONFIG2, MAX30101_SLOT3_MASK, device);
      break;
    case (4):
		MAX30101_bitMask(MAX30101_MULTILEDCONFIG2, MAX30101_SLOT4_MASK, device << 4);
      break;
    default:
      //Shouldn't be here!
      break;
  }
}

//Clears all slot assignments
void MAX30101_disableSlots(void) {
  MAX30101_writeRegister8(DIR_PPG, MAX30101_MULTILEDCONFIG1, 0);
  MAX30101_writeRegister8(DIR_PPG, MAX30101_MULTILEDCONFIG2, 0);
}

//Resets all points to start in a known state
//Page 15 recommends clearing FIFO before beginning a read
void MAX30101_clearFIFO(void) {
  MAX30101_writeRegister8(DIR_PPG, MAX30101_FIFOWRITEPTR, 0);
  MAX30101_writeRegister8(DIR_PPG, MAX30101_FIFOOVERFLOW, 0);
  MAX30101_writeRegister8(DIR_PPG, MAX30101_FIFOREADPTR, 0);
}

//Setup the sensor
//The MAX30101 has many settings. By default we select:
// Sample Average = 4
// Mode = MultiLED
// ADC Range = 16384 (62.5pA per LSB)
// Sample rate = 50
//Use the default setup if you are just getting started with the MAX30101 sensor
void MAX30101_setupPPG(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange) {
	MAX30101_softReset(); //Reset all configuration, threshold, and data registers to POR values

  //FIFO Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //The chip will average multiple samples of same type together if you wish
  if (sampleAverage == 1) MAX30101_setFIFOAverage(MAX30101_SAMPLEAVG_1); //No averaging per FIFO record
  else if (sampleAverage == 2) MAX30101_setFIFOAverage(MAX30101_SAMPLEAVG_2);
  else if (sampleAverage == 4) MAX30101_setFIFOAverage(MAX30101_SAMPLEAVG_4);
  else if (sampleAverage == 8) MAX30101_setFIFOAverage(MAX30101_SAMPLEAVG_8);
  else if (sampleAverage == 16) MAX30101_setFIFOAverage(MAX30101_SAMPLEAVG_16);
  else if (sampleAverage == 32) MAX30101_setFIFOAverage(MAX30101_SAMPLEAVG_32);
  else MAX30101_setFIFOAverage(MAX30101_SAMPLEAVG_4);

  //setFIFOAlmostFull(2); //Set to 30 samples to trigger an 'Almost Full' interrupt
  MAX30101_enableFIFORollover(); //Allow FIFO to wrap/roll over
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Mode Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if (ledMode == 3) MAX30101_setLEDMode(MAX30101_MODE_MULTILED); //Watch all three LED channels
  else if (ledMode == 2) MAX30101_setLEDMode(MAX30101_MODE_REDIRONLY); //Red and IR
  else MAX30101_setLEDMode(MAX30101_MODE_REDONLY); //Red only
  activeLEDs = ledMode; //Used to control how many bytes to read from FIFO buffer
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Particle Sensing Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if(adcRange < 4096) MAX30101_setADCRange(MAX30101_ADCRANGE_2048); //7.81pA per LSB
  else if(adcRange < 8192) MAX30101_setADCRange(MAX30101_ADCRANGE_4096); //15.63pA per LSB
  else if(adcRange < 16384) MAX30101_setADCRange(MAX30101_ADCRANGE_8192); //31.25pA per LSB
  else if(adcRange == 16384) MAX30101_setADCRange(MAX30101_ADCRANGE_16384); //62.5pA per LSB
  else MAX30101_setADCRange(MAX30101_ADCRANGE_2048);

  if (sampleRate < 100) MAX30101_setSampleRate(MAX30101_SAMPLERATE_50); //Take 50 samples per second
  else if (sampleRate < 200) MAX30101_setSampleRate(MAX30101_SAMPLERATE_100);
  else if (sampleRate < 400) MAX30101_setSampleRate(MAX30101_SAMPLERATE_200);
  else if (sampleRate < 800) MAX30101_setSampleRate(MAX30101_SAMPLERATE_400);
  else if (sampleRate < 1000) MAX30101_setSampleRate(MAX30101_SAMPLERATE_800);
  else if (sampleRate < 1600) MAX30101_setSampleRate(MAX30101_SAMPLERATE_1000);
  else if (sampleRate < 3200) MAX30101_setSampleRate(MAX30101_SAMPLERATE_1600);
  else if (sampleRate == 3200) MAX30101_setSampleRate(MAX30101_SAMPLERATE_3200);
  else MAX30101_setSampleRate(MAX30101_SAMPLERATE_50);

  //The longer the pulse width the longer range of detection you'll have
  //At 69us and 0.4mA it's about 2 inches
  //At 411us and 0.4mA it's about 6 inches
  if (pulseWidth < 118) MAX30101_setPulseWidth(MAX30101_PULSEWIDTH_69); //Page 26, Gets us 15 bit resolution
  else if (pulseWidth < 215) MAX30101_setPulseWidth(MAX30101_PULSEWIDTH_118); //16 bit resolution
  else if (pulseWidth < 411) MAX30101_setPulseWidth(MAX30101_PULSEWIDTH_215); //17 bit resolution
  else if (pulseWidth == 411) MAX30101_setPulseWidth(MAX30101_PULSEWIDTH_411); //18 bit resolution
  else MAX30101_setPulseWidth(MAX30101_PULSEWIDTH_69);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //LED Pulse Amplitude Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //Default is 0x1F which gets us 6.4mA
  //powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
  //powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
  //powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
  //powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch

  MAX30101_setPulseAmplitudeRed(powerLevel);
  MAX30101_setPulseAmplitudeIR(powerLevel);
  MAX30101_setPulseAmplitudeGreen(powerLevel);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Multi-LED Mode Configuration, Enable the reading of the three LEDs
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  MAX30101_enableSlot(1, SLOT_RED_LED);
    if (ledMode > 1) MAX30101_enableSlot(2, SLOT_IR_LED);
    if (ledMode > 2) MAX30101_enableSlot(3, SLOT_GREEN_LED);
    //enableSlot(1, SLOT_RED_PILOT);
    //enableSlot(2, SLOT_IR_PILOT);
    //enableSlot(3, SLOT_GREEN_PILOT);

  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  MAX30101_clearFIFO(); //Reset the FIFO before we begin checking the sensor
}


//Read the FIFO Read Pointer
uint8_t MAX30101_getReadPointer(void) {
	uint8_t rdptr = MAX30101_readRegister8(DIR_PPG, MAX30101_FIFOREADPTR);
	rdptr = rdptr & 0x1F; //OJO solo son válidos los 5 bits menos sig.
  return rdptr;
}

//Read the FIFO Write Pointer
uint8_t MAX30101_getWritePointer(void) {
	uint8_t wrptr = MAX30101_readRegister8(DIR_PPG, MAX30101_FIFOWRITEPTR);
	wrptr = wrptr & 0x1F; //OJO solo son válidos los 5 bits menos sig.
  return wrptr;
}

//Polls the sensor for new data
//Call regularly
//If new data is available, it updates the head and tail in the main struct
//Returns number of new samples obtained

uint16_t MAX30101_check(void)
{
  //Read register FIDO_DATA in (3-byte * number of active LED) chunks
  //Until FIFO_RD_PTR = FIFO_WR_PTR

  uint8_t readPointer = MAX30101_getReadPointer();
  uint8_t writePointer = MAX30101_getWritePointer();

  int numberOfSamples = 0;

  //Do we have new data?
  if (readPointer != writePointer)
  {
    //Calculate the number of readings we need to get from sensor
    numberOfSamples = writePointer - readPointer;
    if (numberOfSamples < 0) numberOfSamples += 32; //Wrap condition

    //We now have the number of readings, now calc bytes to read
    //For this example we are just doing Red and IR (3 bytes each)
    int bytesLeftToRead = numberOfSamples * activeLEDs * 3;

    //Get ready to read a burst of data from the FIFO register
	uint8_t dirReg = MAX30101_FIFODATA;
	HAL_I2C_Master_Transmit(&hi2c1, DIR_PPG, &dirReg, 1, 10);


    for (int i=0; i<BUFFER_LENGTH * 9; i++ ) //rellenar todo el buffer con FF para ver cuales se han leido
    	local_data[i] = 0xFF;
//    for (int j=0; j < bytesLeftToRead; j++)
//    	HAL_I2C_Master_Receive(&hi2c1, DIR_PPG, &local_data[j], 1, 10);

    HAL_I2C_Master_Receive(&hi2c1, DIR_PPG, local_data, bytesLeftToRead, 10000);

    //printf("Leidos %d bytes. Muestras: %d WR: %d RD: %d\n", bytesLeftToRead, numberOfSamples, writePointer, readPointer);

    uint8_t index = 0;  //Posición dentro del array/buffer de lecturas de la FIFO

    uint32_t lect1 = 0;  //Lectura LED rojo
    uint32_t lect2 = 0;  //Lectura LED IR
    uint32_t lect3 = 0;  //Lectura LED verde
    uint8_t muestras;
    if (numberOfSamples <= 28)
    	muestras = numberOfSamples;
    else
    	muestras = 28; //¡¡¡¡¡¡¡LAS MUESTRAS 29 y siguientes siempre son incongruentes ??????. Suprimidas hasta averiguar la causa

    for (int k = 0; k < muestras; k++){
        sense.head++; //Advance the head of the storage struct
        sense.head %= STORAGE_SIZE; //Wrap condition

        /*
         * De los 3 bytes por LED, usando 18 bits de resolución, los que tienen datos son 2+8+8, justificado a la derecha, hay que ignorar los 6 bits de la izquierda del primer byte que llega (el más significativo)
         */

    	//3 bytes por LED
    	//LED1
		 lect1= local_data[index] & 0x03; //solo dos bits, 18 bits en total
		 lect1= lect1 << 16;  //Porque el primer byte que llega es el más significativo, el de más a la izqda
		 index++;
		 //lect1 = lect1 + local_data[incr] * 256;
		 lect1 = lect1 + (local_data[index] << 8);  //Segundo byte que llega
		 index++;
		 lect1 = lect1 + local_data[index];
		 index++;
	     sense.red[sense.head] = lect1; //Store this reading into the sense array
		 //LED2
		 if (activeLEDs > 1){
			 lect2= local_data[index] & 0x03; //solo dos bits, 18 bits en total
			 lect2= lect2 << 16;
			 index++;
			 //lect2 = lect2 + local_data[incr] * 256;
			 lect2 = lect2 + (local_data[index] << 8);
			 index++;
			 lect2 = lect2 + local_data[index];
			 index++;
			 sense.IR[sense.head] = lect2;
		 }
		 //LED3
		 if (activeLEDs > 2){
			 lect3= local_data[index] & 0x03; //solo dos bits, 18 bits en total
			 lect3= lect3 << 16;
			 index++;
			 //lect3 = lect3 + local_data[incr] * 256;
			 lect3 = lect3 + (local_data[index] << 8);
			 index++;
			 lect3 = lect3 + local_data[index];
			 index++;
			 sense.green[sense.head] = lect3;
		 }
//		 printf("%2d\t%6d\t%6d\t%6d\n", k,  lect1, lect2, lect3);
    }  //k

  } //End readPtr != writePtr

  return (numberOfSamples); //Let the world know how much new data we found
}

//Check for new data but give up after a certain amount of time
//Returns true if new data was found
//Returns false if new data was not found
uint8_t MAX30101_safeCheck(uint8_t maxTimeToCheck)
{
  uint32_t markTime = HAL_GetTick();

  while(1)
  {
	if((HAL_GetTick() - markTime) > maxTimeToCheck) return 0;

	if(MAX30101_check() >= 1) //We found new data!
	  return 1;

	HAL_Delay(1);
  }
}

//Report the most recent red value
uint32_t MAX30101_getRed(void)
{
  //Check the sensor for new data for 250ms
  if(MAX30101_safeCheck(250))
    return (sense.red[sense.head]);
  else
    return(0); //Sensor failed to find new data
}

//Report the most recent IR value
uint32_t MAX30101_getIR(void)
{
  //Check the sensor for new data for 250ms
  if(MAX30101_safeCheck(250)>0)
    return (sense.IR[sense.head]);
  else
    return(0); //Sensor failed to find new data
}

//Report the most recent Green value
uint32_t MAX30101_getGreen(void)
{
  //Check the sensor for new data for 250ms
  if(MAX30101_safeCheck(250))
    return (sense.green[sense.head]);
  else
    return(0); //Sensor failed to find new data
}

double MAX30101_readTemperature(void) {

  //DIE_TEMP_RDY interrupt must be enabled
  //See issue 19: https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library/issues/19

  // Step 1: Config die temperature register to take 1 temperature sample
	MAX30101_writeRegister8(DIR_PPG, MAX30101_DIETEMPCONFIG , 0x01);

  // Poll for bit to clear, reading is then complete
  // Timeout after 100ms
  unsigned long startTime = HAL_GetTick();
  while (HAL_GetTick() - startTime < 100)
  {
    //uint8_t response = readRegister8(_i2caddr, MAX30105_DIETEMPCONFIG); //Original way
    //if ((response & 0x01) == 0) break; //We're done!

	//Check to see if DIE_TEMP_RDY interrupt is set
	uint8_t response = MAX30101_readRegister8(DIR_PPG, MAX30101_INTSTAT2);
    if ((response & MAX30101_INT_DIE_TEMP_RDY_ENABLE) > 0) break; //We're done!
    HAL_Delay(1); //Let's not over burden the I2C bus
  }
  //TODO How do we want to fail? With what type of error?
  //? if(millis() - startTime >= 100) return(-999.0);

  // Step 2: Read die temperature register (integer)
  int8_t tempInt = MAX30101_readRegister8(DIR_PPG, MAX30101_DIETEMPINT);
  uint8_t tempFrac = MAX30101_readRegister8(DIR_PPG, MAX30101_DIETEMPFRAC); //Causes the clearing of the DIE_TEMP_RDY interrupt

  // Step 3: Calculate temperature (datasheet pg. 23)
  double temp = (double)tempInt + ((double)tempFrac * 0.0625);
  return(temp) ;
}
void MAX30101_enableDIETEMPRDY(void) {
	MAX30101_bitMask(MAX30101_INTENABLE2, MAX30101_INT_DIE_TEMP_RDY_MASK, MAX30101_INT_DIE_TEMP_RDY_ENABLE);
}
void MAX30101_disableDIETEMPRDY(void) {
	MAX30101_bitMask(MAX30101_INTENABLE2, MAX30101_INT_DIE_TEMP_RDY_MASK, MAX30101_INT_DIE_TEMP_RDY_DISABLE);
}
