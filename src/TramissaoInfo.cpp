#include "TransmissaoInfo.h"
#include "IntegracaoDados.h"
#include <Arduino.h>
#include <LoRa.h>


// configuracoes de enderecos para o LoRa
byte BroadCastAddressLoRa = 0xFF;
byte RemoteAddressLoRa = 0x05;
byte MyAddressLoRa = 0x01;
byte contadorMsg = 0 ;
long instanteUltimoEnvio = 0;  // TimeStamp da ultima mensagem enviada
int intervalo = 3000;          // Intervalo entre verificacoes
String retorno;
volatile byte flag;

void IniciarLoRa()
{
  // Iniciamos a comunicação SPI
  SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_SS_PIN);
  LoRa.setPins(LORA_SS_PIN,LORA_RST_PIN,LORA_DI00_PIN);
  Serial.println(F("Iniciando conector LoRa"));
  
  while ( !LoRa.begin(BAND)  )
  { 
     Serial.print(F("."));
     delay(100);
  }

  Serial.println(F("Conexão LoRa iniciada "));
  LoRa.setSPI(SPI);
  LoRa.setSPIFrequency(4e6);
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(42.5E3);
  LoRa.setTimeout(1000);
  LoRa.crc();
  Serial.println(F("Ajustando Spreading Factor"));
  LoRa.onReceive(receberMensagemLoRA);
  LoRa.receive();

  
}
// ##################################################################################################

bool TransmitirDados( String buffer )
{
  unsigned long inicio, duracao=0;
  int tentativas = 0;
  String ret="";
  
  while( tentativas < MAX_TENTATIVAS )
  {
    Serial.println(F("Retransmitindo dados para receiver ")); 
    Serial.print(F("Msg : "));
    Serial.println(buffer);
    retorno= "";
    enviarMensagemLoRa(buffer);
    flag = 0;
    LoRa.receive();
    Serial.println(F("Transmitido"));
    inicio = millis();      
  
    Serial.println(F("Aguardando retorno"));  
    do
    {
      duracao = millis() - inicio;
      delay(100);
      Serial.print("?");
    }     
    while (duracao < TIMEOUT && !flag );

    if ( duracao >= TIMEOUT )
    {
      tentativas++;  
      Serial.print(F("FALHA POR TIMEOUT (tentativa "));
      Serial.print(tentativas);
      Serial.println(F(")"));
    }
    else
    {
      if ( flag == 1)
        return true;
      else
      {
          tentativas++;
          Serial.print(F("Tentativa "));
          Serial.print(tentativas);
          Serial.println(F(" falhou"));
      }
      
    }
  }
  return false;
}

// Funcao que envia uma mensagem LoRa
void enviarMensagemLoRa(String msg) 
{
  
  LoRa.beginPacket();                   // Inicia o pacote da mensagem
  LoRa.write(RemoteAddressLoRa);              // Adiciona o endereco de destino
  LoRa.write(MyAddressLoRa);             // Adiciona o endereco do remetente
  LoRa.write(contadorMsg);                 // Contador da mensagem
  LoRa.write(msg.length());        // Tamanho da mensagem em bytes
  LoRa.print(msg);                 // Vetor da mensagem 
  Serial.println("Montagem de msg finalizada - enviando");
  LoRa.endPacket();                     // Finaliza o pacote e envia
  Serial.println("Pacote finalizado e enviado");
  contadorMsg++;  
                           // Contador do numero de mensagnes enviadas
}


// ##################################################################################################

void IRAM_ATTR receberMensagemLoRA( int tamanhoPacote )
{
  
  if (tamanhoPacote == 0) 
    return;
  else
  {
    // received a packet
    Serial.println(F("[LoRa Mestre] Received packet "));
    byte destinatario = LoRa.read();
    byte remetente = LoRa.read();
    byte MsgId = LoRa.read();
    byte tamanhoMsg = LoRa.read();
    String msg= "";
    msg = LoRa.readStringUntil(endMsg);
    // como leitura da mensagem retira o caractere de endMsg, o 
    // tamanho é sempre diferente, acrescimo para igualar tamanhos
    if (msg.length()!= (int) tamanhoMsg - 1 )
    {
      Serial.print(F("Mensagem incompleta "));
      return;
    } 
    if ( destinatario != MyAddressLoRa && destinatario != BroadCastAddressLoRa )
    {
        Serial.println(F("Mensagem para outro dispositivo"));
        return;
    }
    // se não saiu até aqui é porque está tuod correto 
    // print RSSI of packet

      // Caso a mensagem seja para este dispositivo, imprime os detalhes
    Serial.println(F("########################################"));
    Serial.print(F("Recebido do dispositivo: 0x")); 
    Serial.println(String(remetente, HEX));
    Serial.print(F("Enviado para: 0x")); 
    Serial.println(String(destinatario, HEX));
    Serial.print(F("ID da mensagem: "));
    Serial.println(String(MsgId));
    Serial.print(F("Tamanho da mensagem: ")); 
    Serial.println(String(msg.length()));
    Serial.print(F("Mensagem: "));
    Serial.println(msg);
    // Serial.print(F("RSSI: "));
    // Serial.println(String(LoRa.packetRssi()));
    //Serial.print(F("Snr: ")); 
    //Serial.println(String(LoRa.packetSnr()));
    Serial.println(F("########################################"));
    Serial.println();
    Serial.flush();
    if ( msg == "@OK")
    {
      flag = 1;
      Serial.println(F("msg confirmada"));
    }
    else if ( msg == "@FAIL")
    {
      flag = 0;
      Serial.print(F("Pacote chegou incompleto ao destino"));
    }
    Serial.flush();
  } 
}


// ##################################################################################################
/*
bool avisarInicio()
{
  unsigned long int inicio, tempoDecorrido ;
  bool envioOk = false;
  String ret = "";
  int tentativas = 0;
  while ( !envioOk  && tentativas < MAX_TENTATIVAS )
  {
    serialMestre->print(startMsg);
    serialMestre->print("INIT");
    serialMestre->print(endMsg);
    inicio = millis();
    tempoDecorrido = millis() - inicio;
    ret = VerificarSerialMestre();
    // aguardando a chegada de dados
    while( ret == ""  && tempoDecorrido < TIMEOUT )
    {
      ret = VerificarSerialMestre();
      delay(10);
      Serial.print(".");
      tempoDecorrido = millis() - inicio;
    }
    // saiu por chegada de dados?
    if (tempoDecorrido < TIMEOUT)
    {
        if ( TratarMsgMestre( ret ) == MSG_OK )
        {
            Serial.println("Comunicacao entre Tiva e Heltec [OK] ");
            envioOk=true;
        }
        else
        {
          Serial.println("Comunicacao entre Tiva e Heltec [Falha] ");  
        }      
    }
    else 
      Serial.println("saida por timeout");
    tentativas++;
    Serial.print("Tentativa de aviso de inicio: ");
    Serial.println(tentativas);    
  }
  return envioOk;
}
*/
// ##################################################################################################

bool enviarDHT(DHT_Data *bufferDHT, int tam)
{
  int i;
  int tentativas=0 ; 
  
  String ret;
  bool envioOk = false;
  Serial.println("Envio de dados do DHT para Mestre");
  // preparando informação
  while ( !envioOk  && tentativas < MAX_TENTATIVAS )
  {
    for (i = 0; i < tam; i++)
    {
      ret = startMsg;
      ret = ret + "DHT";
      ret = ret + separatorMsg ;
      ret = ret + bufferDHT[i].instante;
      ret = ret + separatorMsg;
      ret = ret + bufferDHT[i].temperatura;
      ret = ret + separatorMsg;
      ret = ret + bufferDHT[i].umidade;
      ret = ret + endMsg;
      tentativas++;
      Serial.print("Msg montada :");
      Serial.println(ret);
      envioOk =  TransmitirDados(ret);        
    }

  }
  return envioOk; 
}


// ##################################################################################################

bool enviarPluviometro(Pluvi_Data *bufferPluviometro, int tam)
{
  int i;
  String msg;
  Serial.println("Envio de dados do Pluviometro para Mestre");
  for (i = 0; i < tam; i++)
  {
    msg = startMsg;
    msg = msg + "PLU";
    msg = msg + separatorMsg;
    msg = msg + bufferPluviometro[i].instante;
    msg = msg + separatorMsg;
    msg = msg + bufferPluviometro[i].pulsos;
    msg = msg + endMsg;
    bufferPluviometro[i].pulsos=0;
    TransmitirDados(msg);
    return true;
  }
}  
