#include <Arduino.h>
#include "TransmissaoInfo.h"
#include <LoRa.h>
byte BroadCastAddressLoRa = 0xFF;
byte RemoteAddressLoRa = 0x05;
byte MyAddressLoRa = 0x01;
volatile bool flag = false;
void TransmitirDados(String msg)
{
    flag = false;
    unsigned long inicio, duracao = 0;
    int tentativas = 0;
    while (tentativas < 5 && !flag)
    {
        Serial.println(F("Transmistir dados - check"));
        enviarMensagemLoRa(msg);
        LoRa.receive();
        inicio = millis();
        do
        {
            duracao = millis() - inicio;
            delay(100);
        } while (duracao < 8000 && !flag);
        if (duracao > 8000)
        {
            Serial.print("Quantidade de tentativas: ");
            Serial.println(tentativas + 1);
        }
        tentativas++;
    }
}

void IniciarLoRa()
{
    SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_SS_PIN);
    LoRa.setPins(LORA_SS_PIN, LORA_RST_PIN, LORA_DI00_PIN);
    if (!LoRa.begin(BAND))
        Serial.println(F("Não passou da etapa: conectar a frequência da banda"));
    LoRa.setSPI(SPI);
    LoRa.setSPIFrequency(4e6);
    LoRa.setSpreadingFactor(10);
    LoRa.setSignalBandwidth(42.5E3);
    LoRa.setTimeout(1000);
    LoRa.onReceive(receberMensagemLoRA);
    LoRa.receive();
}

void enviarMensagemLoRa(String msg)
{
    LoRa.beginPacket();
    LoRa.write(RemoteAddressLoRa);
    LoRa.write(MyAddressLoRa);
    LoRa.write(msg.length());
    LoRa.print(msg);
    Serial.println(F("Enviar Mensagem LoRa - check"));
    LoRa.endPacket();
}

void IRAM_ATTR receberMensagemLoRA(int tamanhoPacote)
{
    if (tamanhoPacote == 0)
        return;
    String msg = LoRa.readStringUntil('!');
    int inicio = 0;
    for (int i = 0; i < msg.length(); i++)
    {
        if (msg[i] == '@')
            inicio = i;
    }
    msg = msg.substring(inicio, msg.length());
    if (msg = "@OK")
        flag = true;
}

void enviarDHT(DHT_Data dht)
{
    String msg = "@DHT*" + dht.instante + "*" + dht.temperatura + "*" + dht.umidade + ";!";
    TransmitirDados(msg);
}

void enviarPluviometro(Pluvi_Data pluviometro)
{
    String msg = "@PLU*" + pluviometro.instante + "*" + pluviometro.pulsos + ";!";
    TransmitirDados(msg);
}