#ifndef __INTEGRACAODADOS_H__
#define __INTEGRACAODADOS_H__

#include <Arduino.h>

const uint8_t slaveAddress = 0x69;

/**
 * Tipo para trabalhar dados do DHT
 **/
typedef struct DHT_Data{
  String instante;
  float temperatura;
  float umidade;
}DHT_Data;


/**
 * Tipo para trabalhar com dados do pluviometro
 * */
typedef struct  Pluvi_Data {
  String instante;
  int pulsos;
} Pluvi_Datas;

/**
 * Tipo para trabalhar com dados do pluviometro
 * */
typedef struct  Vento_Data {
  String instante;
  double velocidade;
} Vento_Data;

/**
 * Tamanho do buffer utilizado para guardar medidas 
 * dos sensores 
 * */
#define TAM_BUFFER 10
/**
 * Velocidade de comunicação com o outro microcontrolador
 * */
#define VELOCIDADE_COMM 115200


const char startMsg = '@';
const char endMsg = '!';
const char separatorMsg = '*';
const char csvSeparator = ';';

#endif
