#include "TransmissaoInfo.h"
#include "IntegracaoDados.h"
#include <Arduino.h>

void enviarDHT(DHT_Data *bufferDHT)
{
  int i;
  Serial.println("Envio de dados do DHT para Mestre");
  for (i = 0; i < TAM_BUFFER; i++)
  {
    Serial.print(startMsg);
    Serial.print("DHT");
    Serial.print(separatorMsg);
    Serial.print(bufferDHT[i].instante);
    Serial.print(separatorMsg);
    Serial.print(bufferDHT[i].temperatura);
    Serial.print(separatorMsg);
    Serial.print(bufferDHT[i].umidade);
    Serial.print(endMsg);
    delay(100);    
  }
}

void enviarPluviometro(Pluvi_Data *bufferPluviometro)
{
  int i;
  Serial.println("Envio de dados do Pluviometro para Mestre");
  for (i = 0; i < TAM_BUFFER; i++)
  {
    Serial.print(startMsg);
    Serial.print("PLU");
    Serial.print(separatorMsg);
    Serial.print(bufferPluviometro[i].instante);
    Serial.print(separatorMsg);
    Serial.print(bufferPluviometro[i].pulsos);
    Serial.print(endMsg);
    bufferPluviometro[i].pulsos=0;
    delay(100);    
  }
}  
