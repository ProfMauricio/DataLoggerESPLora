#ifndef __TRANSMISSAOINFO_H__
#define __TRANSMISSAOINFO_H__

#include "IntegracaoDados.h"
#include <Arduino.h>

const int TIMEOUT = 10000;
const int MAX_TENTATIVAS=5;

#define BAND 433E6//Frequenciado radio -podemos utilizar ainda : 433E6, 868E6, 915E6
#define PABOOST true

// Pinos do lora (comunicação spi)
const int LORA_SCK_PIN = 5;
const int LORA_MISO_PIN = 19;
const int LORA_MOSI_PIN = 27;
const int LORA_SS_PIN = 18;
const int LORA_RST_PIN = 15;
const int LORA_DI00_PIN = 26;

typedef enum TMSG_MESTRE {MSG_OK, MSG_ERRO, MSG_FALHA, MSG_DESCONHECIDO}  TMSG_MESTRE;

/**
 * Rotina para transmissao de dados obtidos do DHT e que estão armazenado em um buffer
 * @bufferDHT Vetor de registros com os dados obtidos a cada minuto
 * */
bool enviarDHT(DHT_Data *bufferDHT);

/**
 * Rotina para transmissão de dados obtidos do Pluviometro (pulsos a cada quantidade de volume de chuva)
 * @bufferPluviometro Vetor de registro com os dados de chuva obtidos a cada minuto
 * */
bool enviarPluviometro(Pluvi_Data *bufferPluviometro);

/**
 * Rotina para inicio de atividades do escrava
 **/
bool avisarInicio();

/**
 * Rotina que verifica se o mestre enviou alguma mensagem
 **/ 
// String VerificarSerialMestre();

/**
 * Rotina para tratar dados que chegam do mestre
 * */
// TMSG_MESTRE TratarMsgMestre( String msg );

/**
 * Rotina para iniciar a porta serial de comunicaçao com o mestre
 **/ 
//bool iniciarPortaSerialMestre();

bool enviarDHT(DHT_Data *bufferDHT, int tam);
bool enviarPluviometro(Pluvi_Data *bufferPluviometro, int tam);
//bool TransmitirDadosVento(int controleVento, Vento_Data *bufferVento);
bool TransmitirDados(String buffer);
void IniciarLoRa();
void IRAM_ATTR receberMensagemLoRA( int tamanhoPacote );
void enviarMensagemLoRa(String msg) ;

#endif