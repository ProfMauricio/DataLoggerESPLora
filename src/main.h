#ifndef __MAIN_H__
#define __MAIN_H__

#include <Arduino.h>
#include <RtcDS3231.h>
#include <DHT.h>
#include <Wire.h>
#include <SSD1306.h>
#include <SPI.h>
#include "IntegracaoDados.h"
#include "TransmissaoInfo.h"
#include "main.h"
#include <Adafruit_Sensor.h>
#include "controleOLED.h"

#define SERIAL_VIEW 10
#define NONE 0
#define _DEBUG SERIAL_VIEW

// tempo de amostragem em segundos
#define TEMPO_AMOSTRAGEM_VEL_VENTO 10

// TEMPO PARA ATIVAÇÃO DO MÓDULO HELTEC
#define TEMPO_ESPERA_HELTEC 10000


/**
 * Pino onde o pino de sinal do DHT22 está conectado
 * */
#define DHT_PIN 13

/**
 * Pino de sinal do pluviometro
 * 
 **/
#define PLUVI_PIN 36

/**₢
 * Pino que está conectado ao SQW do RTC
 * */
#define pinoRTCSquareWave 37


/**
 * Pino onde está conectado o anemômetro
 **/  
#define ANEMOMETRO_PIN  38


/** 
 * Buffer que guarda as medidas de umidade/temperatura lidas
 **/ 
DHT_Data bufferDHT[TAM_BUFFER];
volatile int contadorDHT = 0;

/**
 * Buffer que armazena as medidas de vento 
 **/ 
Vento_Data bufferVento[TAM_BUFFER];
volatile int contadorVento = 0;

/**
 * Buffer que guarda a quantidade de pulsos em determinado tempo
 **/ 
Pluvi_Data bufferPluviometro[TAM_BUFFER];
int contadorPluviometro = 0;
int pulsosPluvi = 0;
/**
 * String que armazena os instantes
 * */
String datetimeStr;

/**
 * Objeto que captura os dados do sensor DHT
 * */
DHT sensorHT(DHT_PIN, DHT22);



/**
 * Contador de interrupção por alarme
 * */
volatile uint64_t interruptCountAlarm = 0;

/**
 * Flag do alarme RTC apra captura de dados por alarme do DS3231
 **/
volatile bool interruptFlagAlarm = false;

/**
 * Flag de aviso do usuario que deseja forçar a realização de uma coleta
 **/
bool usuarioSolicitaLeitura = false;

/**
 * Flag que sinaliza que houve oscilação no pluviometro
 * */
volatile bool flagOscilacao = false;


/**
 * Contador de pulsos do pluviometro utilizado na interrupção para 
 * gravar dados
 * */
volatile int contadorPulsosPluviometro=0;



String bufferSerial;

/**
 * Função que verifica se existe algum alarme ativo do RTC 
 * */
bool AlarmeAtivo();

/**
 * Função de interrupção para captura do dado do pluviometro
 **/
void CapturarOscilacao(int reset);

/**
 * Rotina que captura a velocidade do vento com base nos pulsos enviados 
 * pelo equipamento durante um tempo T
 **/
double MedirVelocidadeVento(); 

/**
 * Rotina que converte o instante i em uma String
 * @i instante que será convertido
 **/
String DateTime2String( RtcDateTime i );

/*************************************************************************************
 * Mostra na serial os comandos disponveis
 *************************************************************************************/
void help();

/**
 * Rotina para remover da lista de parametros enviados pela serial os dados 
 **/
String obterParametroSerial(int nroParam );

/**
 * Rotina para tratar eventos de i2c entre microcontroladores
 **/
void receberEventoI2C(int qtos);


/**
 * Rotina chamada na interrupção do RTC
 * */
void   RotinaInterrupcaoAlarm();

/**
 * Evento de chegada de dados na serial
 * */
// void IRAM_ATTR serialEvent();

/**
 * Rotina para tratar os dados que chegam na serial do microcontrolador
 **/ 
void tratarEntradaSerial();

/**
 * Rotina de captura de dados sobre o vento
 * */
void CapturarEventoAnemometro();

/**
 * Rotina que calcula a velocidade do vento com base na quantidade de pulsos contados durante uma janela de
 * tempo
 **/
double MedirVelocidadeVento(); 



//#######################################################################################################




#endif