#ifndef __MAIN_H__
#define __MAIN_H__

/**
 * Pino onde o pino de sinal do DHT22 está conectado
 * */
#define DHT_PIN 38

/**
 * Pino de sinal do pluviometro
 * 
 **/
#define PLUVI_PIN 37

/**
 * pino que está conectado ao SQW do RTC
 * */
#define pinoRTCSquareWave  39


/**
 * Função que verifica se existe algum alarme ativo do RTC 
 * */
bool AlarmeAtivo();

/**
 * Função de interrupção para captura do dado do pluviometro
 **/
void IRAM_ATTR CapturarOscilacao();
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
 * Rotina chamada na interrupção do RTC
 * */
void IRAM_ATTR RotinaInterrupcaoAlarm();

/**
 * Evento de chegada de dados na serial
 * */
// void IRAM_ATTR serialEvent();

/**
 * Rotina para tratar os dados que chegam na serial do microcontrolador
 **/ 
void tratarEntradaSerial();



#endif