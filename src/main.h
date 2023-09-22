#ifndef __MAIN_H__
#define __MAIN_H__
#include "IntegracaoDados.h"
#include <DHT.h>
#include <Adafruit_Sensor.h>
/**₢
 * Pino que está conectado ao SQW do RTC
 * */
#define pinoRTCSquareWave 37

/**
 * Pino onde o pino de sinal do DHT22 está conectado
 * */
#define DHT_PIN 13

/** 
 * Salva o instante, a temperatura e 
 *  e umidade do DHT
 **/ 
DHT_Data DHTEnviar;

/**
 * Salva o instante e o pulso do pluviometro
 **/ 
Pluvi_Data PluviometroEnviar;

/**
 * Objeto que captura os dados do sensor DHT
 * */
DHT sensorDHT(DHT_PIN, DHT22);

String DateTime2String(RtcDateTime);

#endif