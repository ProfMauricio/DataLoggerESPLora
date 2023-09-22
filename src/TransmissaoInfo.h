#ifndef __TRANSMISSAOINFO_H__
#define __TRANSMISSAOINFO_H__
#include <Arduino.h>
#include "IntegracaoDados.h"

#define BAND 433E6 //433E6 || 868E6 || 915E6

const int LORA_SCK_PIN = 5;
const int LORA_MISO_PIN = 19;
const int LORA_MOSI_PIN = 27;
const int LORA_SS_PIN = 18;
const int LORA_RST_PIN = 15;
const int LORA_DI00_PIN = 26;

void enviarPluviometro(Pluvi_Data);
void enviarDHT(DHT_Data);
void TransmitirDados(String);
void enviarMensagemLoRa(String);
void IRAM_ATTR receberMensagemLoRA(int);
void IniciarLoRa();
#endif