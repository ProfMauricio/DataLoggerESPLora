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
 * Função que verifica se existe algum alarme ativo do RTC 
 * */
bool AlarmeAtivo();

/**
 * Função de interrupção para captura do dado do pluviometro
 **/
void IRAM_ATTR CapturarOscilacao();

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