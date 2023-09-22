#ifndef __INTEGRACAODADOS_H__
#define __INTEGRACAODADOS_H__
#include <Arduino.h>
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

#endif