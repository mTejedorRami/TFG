/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "ppg.h"
#include "fsm.h"
#include <time.h>
#include <math.h>
#include "ppg.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//Batería
#define V_REF 3.3  //Valor de referencia de tensión
#define ADC_MAX 4096.0
#define V_BAT_MIN 3.2 //Tensión mínima de la batería para que no se estropee
#define V_BAT_MAX 4.2 //Tensión máxima de la batería
#define batBaja 15.0  //Nivel definido como batería baja

//Pulso y  Presión arterial
#define TASA_PPG 800 //Número de muestras por segundo (siendo "muestra" los 3 bytes del conjunto de todos los LEDs)
#define ANCHO_PULSO_PPG 215  //Ancho de pulso para PPG
#define NUM_MUESTRAS_MEDIA_PPG 16 //Número de muestras con que se hace la media para obtener la señal de PPG
#define AMPLI_IR_PPG 0x1f //Amplitud del LED IR
#define MODO_LEDS_PPG 2  //Para decidir cuántos LEDs encender y cuáles de ellos
#define RANGO_ADC_PPG 8192  //Para decidir cuántos bits de resolución tendrá la lectura
#define AMPLI_ROJO_PPG 0x02  //Amplitud del LED rojo
#define AMPLI_VERDE_PPG 0X00   //Amplitud del LED verde
#define POT_LEDS_PPG 0x00 //Potencia de LEDs para PPG

#define RATE_SIZE 20
#define pulsoAlto 100
#define pulsoBajo 60

#define maxBpAlto 140
#define maxBpBajo 90
#define minBpAlto 90
#define minBpBajo 60

//Temperatura
#define DIR_TEMP 0X90
#define tempHipotermia 35
#define tempFiebre 37.2

//Estres
#define estresElevado 3289

//Alcohol
#define alcoholElevado 132.98
#define r0 1017.76

#define A_TRAMO1 10.375  //Primer parámetro de la ecuación del tramo 1 del alcohol
#define B_TRAMO1 87.032  //Segundo parámetro de la ecuación del tramo 1 del alcohol
#define C_TRAMO1 230.55  //Tercer parámetro de la ecuación del tramo 1 del alcohol

//y = -32548x5 + 102490x4 - 119134x3 + 63102x2 - 16426x + 2839,6
#define A_TRAMO2 32548  //Primer parámetro de la ecuación del tramo 2 del alcohol
#define B_TRAMO2 102490  //Segundo parámetro de la ecuación del tramo 2 del alcohol
#define C_TRAMO2 119134  //Tercer parámetro de la ecuación del tramo 2 del alcohol
#define D_TRAMO2 63102  //Cuarto parámetro de la ecuación del tramo 2 del alcohol
#define E_TRAMO2 16426  //Quinto parámetro de la ecuación del tramo 2 del alcohol
#define F_TRAMO2 2839.6  //Sexto parámetro de la ecuación del tramo 2 del alcohol

#define A_TRAMO3 6666.7  //Primer parámetro de la ecuación del tramo 3 del alcohol
#define B_TRAMO3 2666.7  //Segundo parámetro de la ecuación del tramo 3 del alcohol

//Alarma
#define tiempoAlarmaBateria 2000
#define tiempoAlarmaPresion 8000
#define tiempoAlarmaTemperatura 4000
#define tiempoAlarmaEstres 6000
#define tiempoAlarmaAlcohol 3000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

enum fsm_state_medidas{
	bateria,
	pulsoInicio,
	presion,
	alcohol,
	temperatura,
	estres,
	pulso,
};

enum fsm_state_alarma{
	off,
	bateriaAlarma,
	pulsoAlarma,
	presionAlarma,
	alcoholAlarma,
	temperaturaAlarma,
	estresAlarma,
};

uint32_t adc[3];
uint32_t timerMedidas = 0;
uint32_t timerAlarma = 0;

//Bateria
float nivel_bateria;

//Pulso
float pulsosMedia;
uint8_t countRates = 0 ;
int16_t rates[RATE_SIZE]; //Array of heart rates

//Presion
float maxBp;
float minBp;
uint32_t tiemposPPG[3];
float maxBPTiempo[20];
float minBPTiempo[20];
uint32_t ST[20];
uint32_t T1[20];

//Temperatura
float temp ;
uint32_t tiempoInicio =0;

//Estres
uint32_t estresMedido;

//Alcohol
uint32_t alcoholMax = 0;
float nivelAlcohol;

//Alarma
uint8_t avisoBat = 0;
uint8_t avisoPulso = 0;
uint8_t avisoPresion = 0;
uint8_t avisoEstresAlto = 0;
uint8_t avisoTemp = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 *
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */


  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  fsm_t* fsm_new_medidas ();
  fsm_t* fsm_new_alarma ();

  void inicializarPPG(uint8_t potenciaLED, uint8_t mediaMuestras, uint8_t modoLEDs, int tasaMuestreo, int anchoPulso, int rangoADC, uint8_t amplitudRojo, uint8_t amplitudIR, uint8_t amplitudVerde);

  //Se activan los dos reguladores
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

//Calentar sensor alcohol
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

  //Se configura el sensor de la fotoplestimografía
  inicializarPPG(POT_LEDS_PPG, NUM_MUESTRAS_MEDIA_PPG, MODO_LEDS_PPG, TASA_PPG, ANCHO_PULSO_PPG, RANGO_ADC_PPG, AMPLI_ROJO_PPG, AMPLI_IR_PPG, AMPLI_VERDE_PPG);

  fsm_t* fsm_medidas = fsm_new_medidas();
  fsm_t* fsm_alarma = fsm_new_alarma();

  tiempoInicio = HAL_GetTick();
  HAL_ADC_Start_DMA (&hadc1, adc,3);
	  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  fsm_fire (fsm_medidas);
	  fsm_fire (fsm_alarma);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C3
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */

void inicializarPPG(uint8_t potenciaLED, uint8_t mediaMuestras, uint8_t modoLEDs, int tasaMuestreo, int anchoPulso, int rangoADC, uint8_t amplitudRojo, uint8_t amplitudIR, uint8_t amplitudVerde){
	MAX30101_setupPPG(potenciaLED, mediaMuestras, modoLEDs, tasaMuestreo, anchoPulso, rangoADC);  //3 LEDs a 0x0C de potencia, 400 samples/second, 411 de ancho de pulso y 18 bits. Se bajó la potencia para adaptarlo a pieles claras
	MAX30101_setPulseAmplitudeRed(amplitudRojo); //Turn Red LED to low to indicate sensor is running
	MAX30101_setPulseAmplitudeIR(amplitudIR); //Turn off IR LED
	MAX30101_setPulseAmplitudeGreen(amplitudVerde); //Turn off Green LED
}


//Funciones de comprobación Medidas de parámetros

//Se comprueba si es la primera vez que se ejecuta el bucle
static int tiempoNoIniciado (fsm_t* this) {
	uint8_t ret= 0;
	if(timerMedidas == 0){
		ret = 1;
	}
	return ret;
}

//Se comprueba si no es la primera vez que se ejecuta el bucle
static int tiempoIniciado (fsm_t* this) {
	uint8_t ret= 0;
	if(timerMedidas != 0){
		ret = 1;
	}
	return ret;
}

//Se comprueba si se tiene que seguir midiendo el parámetro
static int timpoNoAcabado (fsm_t* this) {
	uint8_t ret= 0;
	if(timerMedidas > HAL_GetTick()){
		ret = 1;
	}
	return ret;
}

//Se comprueba si no se tiene que seguir midiendo el parámetro
static int finTiempo (fsm_t* this) {
	uint8_t ret= 0;
	if(timerMedidas <= HAL_GetTick()){
		ret = 1;
	}
	return ret;
}

//Se pasa al siguiente estado
static int siguienteEstado (fsm_t* this) {
	uint8_t ret= 1;
	return ret;
}

//Funciones de transición

//Se calcula el nivel de batería
void calculoBateria(fsm_t* this){
	float tension_bat = 0.0;

	tension_bat = 2*(((float)adc[0])/ADC_MAX)*V_REF;  //Porque los ADCs son de 12 bits (4096) que marcan los 3,3V (Vcc del micro, la referencia) y 2 porque el divisor hace que midas la mitad de la tensión Vin_bat
	nivel_bateria = (((tension_bat - V_BAT_MIN)/(V_BAT_MAX - V_BAT_MIN))*100);
	if(timerMedidas == 0){
		//Se inicializa el timer con el tiempo de medida del pulso
		timerMedidas = timerMedidas +20000;
	}
}

//Se mide el pulso
void medirPulso(fsm_t* this){
	static uint32_t lastBeat; //Time at which the last beat occurred
	float beatsPerMinute;
	uint32_t irValue;
	irValue = MAX30101_getIR();  //Provoca una nueva recogida de datos del led de IR

	//Si se ha obtenido un valor del sensor
	if( irValue != 0){
		//Se compruba si se ha producido un latido
		if ( MAX30101_checkForBeat(irValue) == 1){
			//Se ha detectado un latido
			int delta = HAL_GetTick() - lastBeat;
			lastBeat = HAL_GetTick();
			beatsPerMinute = 60 / (delta / 1000.0);
			//Si el caluculo del pulso se encuentra dentro de un rango posible
			if((beatsPerMinute < 255) &( beatsPerMinute > 40 )& (countRates< 20)){
			  rates[countRates] = (uint8_t)beatsPerMinute; //Se guarda el pulso en el array
			  countRates ++;
			}
		}
	}
}

//Se calcula el pulso la primera vez que se enciende el dispositivo
void calculoPulsoInicio(fsm_t* this){
	for (int i = 2;i<countRates;i++){
		pulsosMedia = pulsosMedia + rates[i];
	}
	pulsosMedia = pulsosMedia/(countRates-2);
	countRates = 0;
	//Se inicializa el timer con el tiempo de medida de la presión
	timerMedidas = timerMedidas + 20000;

}

//Se calcula el pulso
void calculoPulso(fsm_t* this){
	float pulsaciones = 0;
	for (int i = 2;i<countRates;i++){
		pulsaciones = pulsaciones + rates[i];
	}
	pulsaciones = pulsaciones/(countRates-2);
	float desviacionMedia = 0.0;
	for ( int i = 2; i<countRates; i++){
		desviacionMedia = desviacionMedia + fabs( rates[i] - pulsaciones);
	}
	desviacionMedia = desviacionMedia / (countRates -2);
	if ( desviacionMedia <= 15.0){
		pulsosMedia = pulsaciones;
	}
	else{
		pulsosMedia = 0;
	}
}

//Se mide la presión
void medirPresion(fsm_t* this){
	int32_t irValue;
	static uint8_t p;
	for(int i= 0; i<1000;i++){
		irValue = MAX30101_getIR();  //Provoca una nueva recogida de datos del led de IR
		if( irValue != 0){
			//Se recogen los datos necesarios para calcular la presión
			if ( MAX30101_checkForBP(irValue, tiemposPPG) == 1){
				ST[p]= tiemposPPG[1]-tiemposPPG[0];
				T1[p] = tiemposPPG[2] -tiemposPPG[1];
				maxBPTiempo[p] = 12.84 +0.78*((float)T1[p])/5 + 2.296*((float)ST[p])/5 ;
				minBPTiempo[p] = -5 +1.224*(float)T1[p]/5 + 0.534* (float)ST[p]/5 ;
				//Si el valor almacenado esta dentro de un rango posible se guarda
				if(((maxBPTiempo[p]>70.0 ) )&(minBPTiempo[p]> 40.0)){
					p++;

				}
			}
		}
	}
}

//Se calcula la presión
void calculoPresion(fsm_t* this){
	//Se hace vibrar la puslera para indiciar al usuario que debe empezar a soplar
	for(uint i = 0;i<8;i++){
		HAL_GPIO_TogglePin(GPIOA,  GPIO_PIN_8);
		HAL_Delay(100);
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

	float maxBpMedia = 0.0;
	float minBpMedia = 0.0;
	uint8_t i = 0;
	while(minBPTiempo[i]!=0 && i != 20){
		maxBpMedia= maxBpMedia + maxBPTiempo[i];
		minBpMedia = minBpMedia + minBPTiempo[i];
		i++;
	}
	maxBpMedia = maxBpMedia/ i;
	minBpMedia = minBpMedia / i;
	float desviacionMaxBpMedia = 0.0;
	float desviacionMinBpMedia = 0.0;
	i =0;
	while(minBPTiempo[i]!=0 && i != 20){
		desviacionMaxBpMedia = desviacionMaxBpMedia + fabs( maxBPTiempo[i] - maxBpMedia);
		desviacionMinBpMedia = desviacionMinBpMedia + fabs( minBPTiempo[i] - minBpMedia);
		i++;
	}
	desviacionMaxBpMedia = desviacionMaxBpMedia/i;
	desviacionMinBpMedia = desviacionMinBpMedia/i;
	i=0;
	uint8_t bpCountMax = 0;
	uint8_t bpCountMin = 0;
	while(minBPTiempo[i]!=0 && i != 20){
		if((maxBPTiempo[i] <=(maxBpMedia + desviacionMaxBpMedia )) & (maxBPTiempo[i] >=(maxBpMedia - desviacionMaxBpMedia))){
			maxBp = maxBp + maxBPTiempo[i];
			bpCountMax ++;
		}
		if((minBPTiempo[i] <=(minBpMedia + desviacionMinBpMedia )) & (minBPTiempo[i] >=(minBpMedia - desviacionMinBpMedia))){
			minBp = minBp + minBPTiempo[i];
			bpCountMin ++;
		}
		i++;
	}
	maxBp = maxBp / bpCountMax;
	minBp = minBp /bpCountMin;
	//Se inicializa el timer con el tiempo de medida del nivel de alcohol
	timerMedidas = timerMedidas + 20000;
}

//Se mide el nivel de alcohol
void medirAlcohol(fsm_t* this){
	HAL_ADC_Start_DMA (&hadc1, adc,3);
	uint16_t alcohol;
	alcohol = adc[1];
	//Se almacena el valor máximo
	if(alcohol>alcoholMax){
		alcoholMax = alcohol;
	}
}

//Se calcula el nivel de alcohol
void calculoAlcohol(fsm_t* this){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	float vrl = ((float)alcoholMax *3.3)/4096;
	float rs = ((5-vrl)/vrl)* 2026.04;
	float relacion_r = (rs/r0);
	if((relacion_r < 4) & (relacion_r > 1.1)){  //Tramo 1
			//Ecuación de la curva del tramo 1: y = 10.375x2 - 87.032x + 230.55;
		nivelAlcohol = (float)(A_TRAMO1*pow(relacion_r, 2) - B_TRAMO1*relacion_r + C_TRAMO1);

		}else if((relacion_r <= 1.1) & (relacion_r >= 0.2)){  //Tramo 2
			//Ecuación de la curva del tramo 2: y = 163204*x6 - 688610x5 + 1E+06x4 - 983296x3 + 441823x2 - 100218x + 10120;
			//ppm = (float)(A_TRAMO2*pow(relacion_r, 6) - B_TRAMO2*pow(relacion_r, 5) + C_TRAMO2*pow(relacion_r, 4) - D_TRAMO2*pow(relacion_r, 3) + E_TRAMO2*pow(relacion_r, 2) - F_TRAMO2*relacion_r + G_TRAMO2);
			//Ecuación de la curva del tramo 2: y = -32548x5 + 102490x4 - 119134x3 + 63102x2 - 16426x + 2839,6
			nivelAlcohol = (float)((-A_TRAMO2)*pow(relacion_r, 5) + B_TRAMO2*pow(relacion_r, 4) - C_TRAMO2*pow(relacion_r, 3) + D_TRAMO2*pow(relacion_r, 2) - E_TRAMO2*relacion_r + F_TRAMO2);
		}
		else if(relacion_r < 0.2){  //Tramo 3
			//Ecuación de la curva del tramo 3: y = -6666.7x + 2666.7;
			nivelAlcohol = (float)(-A_TRAMO3*relacion_r + B_TRAMO3);
		}else if(relacion_r > 4){
			nivelAlcohol = 0.0;
		}
}

//Se calcula la temperatura
void calculoTemp(fsm_t* this){
	//No se realiza la medida si el tiempo es menor de 9 minutos
	if((HAL_GetTick()- tiempoInicio) > 540000){
		uint8_t bufferTemp[2];  //Buffer de datos a enviar y leer por I2C
		bufferTemp[0] = 0x01; //Dirección de registro de configuración
		bufferTemp[1] = 0x00; //Contenido a enviar al registro de configuración

		if(HAL_I2C_IsDeviceReady(&hi2c3, DIR_TEMP, 2, 10) == HAL_OK){
			HAL_I2C_Master_Transmit(&hi2c3, DIR_TEMP, bufferTemp, 2, 10);
		}

		bufferTemp[0] = 0x00; //Dirección de registro de temperatura
		HAL_I2C_Master_Transmit(&hi2c3, DIR_TEMP, bufferTemp, 1, 10);
		HAL_I2C_Master_Receive(&hi2c3, DIR_TEMP, bufferTemp, 2, 10);  //Sobreescribe lo que hay en el array porque ya no se necesita y mete los datos leidos ahí
		temp = (float)bufferTemp[0] + ((float)bufferTemp[1])*0.996/255;
	}
}

//Se calcula el estrés
void calculoEstres(fsm_t* this){
	HAL_ADC_Start_DMA (&hadc1, adc,3);
	estresMedido = adc[2];
	//Se inicializa el timer con el tiempo de medida del pulso
	timerMedidas = timerMedidas +10000;
}

fsm_t* fsm_new_medidas ()
{
	static fsm_trans_t tt[] = {
		{ bateria,tiempoNoIniciado,pulsoInicio,calculoBateria},
		{ bateria,tiempoIniciado,temperatura,calculoBateria },
		{ pulsoInicio,timpoNoAcabado,pulsoInicio,medirPulso },
		{ pulsoInicio,finTiempo,presion,calculoPulsoInicio },
		{ presion,timpoNoAcabado,presion,medirPresion },
		{ presion,finTiempo,alcohol,calculoPresion },
		{ alcohol,timpoNoAcabado,alcohol,medirAlcohol },
		{ alcohol,finTiempo,temperatura,calculoAlcohol},
		{ temperatura,siguienteEstado,estres,calculoTemp },
		{ estres,siguienteEstado,pulso,calculoEstres },
		{ pulso,timpoNoAcabado,pulso,medirPulso },
		{ pulso,finTiempo,bateria,calculoPulso },
		{ -1, NULL, -1, NULL },
	};
	return fsm_new (tt);
}

//Funciones de comprobación Alarma

//Se comprueba si tiene que seguir vibrando la pulsera
static int timpoNoAcabadoAlarma (fsm_t* this) {
	uint8_t ret= 0;
	if(timerAlarma > HAL_GetTick()){
		ret = 1;
	}
	return ret;
}

//Se comprueba si no tiene que seguir vibrando la pulsera
static int finTiempoAlarma (fsm_t* this) {
	uint8_t ret= 0;
	if(timerAlarma <= HAL_GetTick()){
		ret = 1;
	}
	return ret;
}

//Se comprueba si el nivel de bateria es inferior al 15%
static int bateriaBaja (fsm_t* this) {
	uint8_t ret= 0;
	if(((nivel_bateria <= batBaja) & (nivel_bateria >0)) & (avisoBat == 0)){
		ret = 1;
	}
	return ret;
}

//Se comprueba si se detecta la presión arterial fuera de un rango saludable
static int presionAnomala (fsm_t* this) {
	uint8_t ret= 0;
	if((((maxBp <= maxBpBajo )& ( maxBp>0))||(maxBp >= maxBpAlto) || ((minBp <= minBpBajo) & (minBp>0))||(minBp >= minBpAlto)) & (avisoPresion == 0) ){
		ret = 1;
	}
	return ret;
}

//Se comprueba si se detecta la temperatura fuera de un rango saludable
static int temperaturaAnomala (fsm_t* this) {
	uint8_t ret= 0;
	if((((temp <= tempHipotermia) &( temp>0)) ||(temp >= tempFiebre)) & ( avisoTemp== 0)){
		ret = 1;
	}
	return ret;
}

//Se comprueba si se detecta el pulso fuera de un rango saludable
static int pulsoAnomalo (fsm_t* this) {
	uint8_t ret= 0;
	if((((pulsosMedia <= pulsoBajo) &( pulsosMedia>0) )||pulsosMedia >= pulsoAlto)& (avisoPulso == 0) ){
		ret = 1;
	}
	return ret;
}

//Se comprueba si se detecta el nivel de estrés elevado
static int estresAlto (fsm_t* this) {
	uint8_t ret= 0;
	if((estresMedido <= estresElevado) & (avisoEstresAlto == 0) & (estresMedido >0)){
		ret = 1;
	}
	return ret;
}

//Se comprueba si se detecta el nivel de alcohol elevado
static int alcoholAlto (fsm_t* this) {
	uint8_t ret= 0;
	if(nivelAlcohol >= alcoholElevado){
		ret = 1;
	}
	return ret;
}

//Funciones de transición Alarma

//Se inicializa el timer con la duración de la vibración para la batería
void inicioTimerBateria(fsm_t* this){
	timerAlarma = HAL_GetTick() + tiempoAlarmaBateria;
	avisoBat = 1;
}

//Se inicializa el timer con la duración de la vibración para la presión
void inicioTimerPresion(fsm_t* this){
	timerAlarma = HAL_GetTick() + tiempoAlarmaPresion;
	avisoPresion = 1;
}

//Se inicializa el timer con la duración de la vibración para la temperatura
void inicioTimerTemperatura(fsm_t* this){
	timerAlarma = HAL_GetTick() + tiempoAlarmaTemperatura;
	avisoTemp = 1;
}

//Se inicializa el timer con la duración de la vibración para el estrés
void inicioTimerEstres(fsm_t* this){
	timerAlarma = HAL_GetTick() + tiempoAlarmaEstres;
	avisoEstresAlto = 1;
}

//Se inicializa el timer con la duración de la vibración para el alcohol
void inicioTimerAlcohol(fsm_t* this){
	timerAlarma= HAL_GetTick() + tiempoAlarmaAlcohol;
}

//Se hace vibrar el motor
void aviso(fsm_t* this){
	HAL_GPIO_TogglePin(GPIOA,  GPIO_PIN_8);
}

//Se realiza el aviso por el pulso
void avisoPulsaciones(fsm_t* this){
	for(uint8_t i= 0; i< 25; i++){
		HAL_GPIO_TogglePin(GPIOA,  GPIO_PIN_8);
		HAL_Delay(10);
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); //Para asegurar que no se quede vibrando
	avisoPulso = 1;
}

//Se asegura que se para la vibración
void alarmaFin(fsm_t* this){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); //Para asegurar que no se quede vibrando
}


fsm_t* fsm_new_alarma ()
{
	static fsm_trans_t tt[] = {
		{ off,bateriaBaja,bateriaAlarma,inicioTimerBateria},
		{ bateriaAlarma,timpoNoAcabadoAlarma,bateriaAlarma,aviso },
		{ bateriaAlarma,finTiempoAlarma,off,alarmaFin},
		{ off,pulsoAnomalo,pulsoAlarma,avisoPulsaciones},
		{ pulsoAlarma,timpoNoAcabadoAlarma,pulsoAlarma,aviso},
		{ pulsoAlarma,finTiempoAlarma,off,alarmaFin},
		{ off,presionAnomala,presionAlarma,inicioTimerPresion },
		{ presionAlarma,timpoNoAcabadoAlarma,presionAlarma,aviso },
		{ presionAlarma,finTiempoAlarma,off,alarmaFin },
		{ off,temperaturaAnomala,temperaturaAlarma,inicioTimerTemperatura },
		{ temperaturaAlarma,timpoNoAcabadoAlarma,temperaturaAlarma,aviso},
		{ temperaturaAlarma,finTiempoAlarma,off,alarmaFin },
		{ off,estresAlto,estresAlarma,inicioTimerEstres },
		{ estresAlarma,timpoNoAcabadoAlarma,estresAlarma,aviso },
		{ estresAlarma,finTiempoAlarma,off,alarmaFin },
		{ off,alcoholAlto,alcoholAlarma,inicioTimerAlcohol },
		{ alcoholAlarma,timpoNoAcabadoAlarma,estresAlarma,aviso },
		{ alcoholAlarma,finTiempoAlarma,off,alarmaFin },
		{ -1, NULL, -1, NULL },
	};
	return fsm_new (tt);
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
