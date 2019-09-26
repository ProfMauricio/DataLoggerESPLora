#include <Arduino.h>
#include <RtcDS3231.h>
#include <Wire.h>
#include <SSD1306.h>
#include <SPI.h>
#include <SD.h>
#include "IntegracaoDados.h"
#include "TransmissaoInfo.h"
#include "main.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>


/** 
 * Buffer que guarda as medidas de umidade/temperatura lidas
 **/ 
DHT_Data bufferDHT[TAM_BUFFER];
volatile int contadorDHT = 0;

/**
 * Buffer que armazena as medidas de vento 
 **/ 
float bufferVento[TAM_BUFFER];
volatile int contadorVento = 0;

/**
 * Buffer que guarda a quantidade de pulsos em determi
 * do tempo
 **/ 
Pluvi_Data bufferPluviometro[TAM_BUFFER];
int contadorPluviometro = 0;

/**
 * String que armazena os instantes
 * */
String datetimeStr;

/**
 * Objeto que captura os dados do sensor DHT
 * */
DHT sensorHT(DHT_PIN, DHT22);

/**
 * Objeto String onde os dados recebidos são temporáriamente armazenados
 * */
String bufferSerial;

/**
 * Objeto do rtc para captura de instantes
 * */
RtcDS3231<TwoWire> rtc(Wire);

/**
 * Configurações do cartão de memória
 * */
const int chipSelect = 13;

/**
 * Contador de interrupção por alarme
 * */
volatile uint64_t interruptCountAlarm = 0;
volatile bool interruptFlagAlarm = false;

/**
 * Flag que sinaliza que houve oscilação no pluviometro
 * */
volatile bool flagOscilacao = false;

/**
 * Contador de pulsos do pluviometro utilizado na interrupção para 
 * gravar dados
 * */
volatile int contadorPulsosPluviometro=0;


/**
 * Rotina chamada na interrupção do RTC
 * */
void IRAM_ATTR RotinaInterrupcaoAlarm()
{
  interruptCountAlarm++;
  interruptFlagAlarm = true;
  Serial.println("e");
}

/**
 * Função que verifica se existe algum alarme ativo do RTC 
 * */
bool AlarmeAtivo()
{
  bool ativo = false;
  if ( interruptFlagAlarm )
  {
    interruptFlagAlarm = false;
    ativo = true;
    DS3231AlarmFlag flag = rtc.LatchAlarmsTriggeredFlags();
    if ( flag & DS3231AlarmFlag_Alarm2)
    {
      Serial.println("Alarme 2 ativado");
    }
  }
  return ativo;
}


void IRAM_ATTR CapturarOscilacao()
{
  //bufferPluviometro[contadorPluviometro].pulsos++;
  flagOscilacao = true;
  contadorPulsosPluviometro++;
}

/*************************************************************************************
 * Funçao para tratar os dados recebidos pela porta serial
 *************************************************************************************
void IRAM_ATTR serialEvent() {
  char inChar;
  //Serial.print("Analisando comando\n");
  while (Serial.available()) {
    // get the new byte:
    inChar = (char)Serial.read();
    // add it to the inputString:
    bufferSerial = bufferSerial + String(inChar);
    // Caractere @ inicia modo de prompt de comandos
    if ( (inChar == 10 ) || (inChar == 13) || (inChar == '#')) {
      tratarEntradaSerial();
    }
  }
} */

// -----------------------------------------------------------------------------------------------------------------
/*************************************************************************************
 * Mostra na serial os comandos disponveis
 *************************************************************************************/
void help()
{
  Serial.println(F("Help de comandos"));
  Serial.println(F("@time <hh> <mm> <ss>"));
  Serial.println(F("Exemplo-> @time 10 05 30"));
  Serial.println(F("@date <dia> <mes> <ano>"));
  Serial.println(F("Exemplo-> @date 20 01 2013 "));
  Serial.println(F("@eepz Pos_eeprom"));
  Serial.println(F("Exemplo-> @sidf 0 0 "));
  Serial.println(F("@resp -> Reseta ESP"));
  Serial.println(F("@now"));
  Serial.println(F("@exit"));
}


void tratarEntradaSerial()
{
  uint8_t par1, par2, par3;
  uint16_t ano;
  RtcDateTime instante;
  String tmpMsg;
  String instanteStr;

  // Parando timer para tratar informaçoes que estao sendo recebidas
  // -> não precisa.stop();

  // parando a interrupção
  //  MostrarInstanteAtual();


  // extraindo o tipo de comando enviado
  tmpMsg = bufferSerial.substring(0,5);
#if _DEBUG >= DEBUG_SERIAL
  Serial.println(tmpMsg);
#endif
  //testando se o usuario que ajustar o horario
  if (tmpMsg.equalsIgnoreCase("@time"))
  {
    instante = rtc.GetDateTime();
    tmpMsg = obterParametroSerial(1);
    par1 = (uint8_t) tmpMsg.toInt();
    tmpMsg = obterParametroSerial(2);
    par2 = tmpMsg.toInt();
    tmpMsg =  obterParametroSerial(3);
    par3 = (uint8_t) tmpMsg.toInt();
    RtcDateTime novoInstante;novoInstante =   RtcDateTime(instante.Year(),instante.Month(), instante.Day(),par1, par2, par3);
    rtc.SetDateTime(novoInstante);
    instante = rtc.GetDateTime();
    datetimeStr = DateTime2String(instante);
    Serial.println(tmpMsg);
  }
  else if (tmpMsg.equalsIgnoreCase("@date"))
  {
    instante = rtc.GetDateTime();
    tmpMsg = obterParametroSerial(1);
    par2 = tmpMsg.toInt();
    tmpMsg = obterParametroSerial(2);
    par3 = tmpMsg.toInt();
    tmpMsg = obterParametroSerial(3);
    ano = tmpMsg.toInt();
    RtcDateTime novoInstante = RtcDateTime(ano, par3, par2,instante.Hour(), instante.Minute(), instante.Second());
    rtc.SetDateTime(novoInstante);
    instante = rtc.GetDateTime();
    instanteStr = DateTime2String(instante);
    Serial.println(tmpMsg);
  }
  else if ( tmpMsg.equalsIgnoreCase("@hisl")) // aviso de ativação do ESP
  {
    Serial.println("Mensagem do mestre: " + tmpMsg.substring(6));    
  }
  else if ( tmpMsg.equalsIgnoreCase("@tirq")) // requisição de instante pelo ESP
  {
    instante = rtc.GetDateTime();
    instanteStr = DateTime2String(instante);
    Serial1.print(String(instanteStr));
  } 
  else if (tmpMsg.equalsIgnoreCase("@help"))
    help();

  // testando se o comando para sair do modo de comandos
  else if (tmpMsg.equalsIgnoreCase("@exit"))
  {
    Serial.print("Saindo do modo de comando");
  }
  else 
  {
    tmpMsg = bufferSerial.substring(0,4);
    if (tmpMsg.equalsIgnoreCase("@now"))
    {
      instante = rtc.GetDateTime();
      instanteStr = DateTime2String(instante);
      Serial.println(instanteStr);
    }
    else Serial.println(F("Comando nao reconhecido"));
  }
  //
  // zerando buffer a cada comando
  bufferSerial = "";
}

/*************************************************************************************
 * Funcao para obter os parametros passados pela serial
 *************************************************************************************/
String obterParametroSerial(int nroParam )
{
  String tmp="";
  tmp.reserve(5);
  switch (nroParam)
  {
  case 1:
    tmp = bufferSerial.substring(6,8);
#if _DEBUG >= DEBUG_SERIAL
    Serial.print("par 1 ->");
    Serial.println(tmp.toInt());
#endif
    break;
  case 2:
    tmp = bufferSerial.substring(9,11);
#if _DEBUG >= DEBUG_SERIAL
    Serial.print("par 2 ->");
    Serial.println(tmp.toInt());
#endif
    break;
  case 3:
    tmp = bufferSerial.substring(12,16);
#if _DEBUG >= DEBUG_SERIAL
    Serial.print("par 3 ->");
    Serial.println(tmp.toInt());
#endif
    break;
  }
  return tmp;
}


String DateTime2String( RtcDateTime i )
{
  char datetimeStr[20];
  sprintf(datetimeStr,"%02u/%02u/%04u %02u:%02u:%02u",            
           i.Day(), i.Month(), i.Year(), i.Hour(), i.Minute(), i.Second());
  return String(datetimeStr);
}

void setup() {
  int i = 0;
  // put your setup code here, to run once:
  delay(1000);
  // put your setup code here, to run once:
  Serial.begin(9600);

  while(!Serial);

  Serial.print("Compilado em: ");
  Serial.print(__DATE__);
  Serial.println(__TIME__);

  rtc.Begin();
  RtcDateTime compilador = RtcDateTime(__DATE__,__TIME__);

  if (!rtc.IsDateTimeValid())
  {
    if ( rtc.LastError() != 0 )
    {
    Serial.println("Erro de comunicação");
    Serial.println(rtc.LastError());
    }    
  }
// ajustes de pinos
  rtc.Enable32kHzPin(false);
  rtc.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmTwo);

  // Ajustando o alarme 2 para disparar a cada 1 minuto
  DS3231AlarmTwo alarme2(0,0,0,DS3231AlarmTwoControl_OncePerMinute);
  rtc.SetAlarmTwo(alarme2);
  rtc.LatchAlarmsTriggeredFlags();

  // ativando as interrupções 
  attachInterrupt(pinoRTCSquareWave, RotinaInterrupcaoAlarm, FALLING);
  attachInterrupt(PLUVI_PIN, CapturarOscilacao, FALLING);
    
  //rtc.SetDateTime(compilador);
  RtcDateTime Instante = rtc.GetDateTime();
  Serial.println(DateTime2String(Instante));
  delay(1000);

  while ( i == 0 )
  {
    if ( !SD.begin(chipSelect))
    {
     Serial.println(F("Falha na inicialização da biblioteca"));
      while(i<100){
        Serial.print(".");
        delay(100);
        i++;
      }
      i = 0;
      Serial.print("Nova tentativa de ativar cartão");
    }  
    else
    {
      Serial.println("Cartão inicializado.");
      i++;
    }
  }


}

void loop() {
// put your main code here, to run repeatedly:
  char inChar;
  //Serial.print("Analisando comando\n");
  while (Serial.available()) {
    // get the new byte:
    inChar = (char)Serial.read();
    // add it to the inputString:
    bufferSerial = bufferSerial + String(inChar);
    // Caractere @ inicia modo de prompt de comandos
    if ( (inChar == 10 ) || (inChar == 13) || (inChar == '#')) {
      tratarEntradaSerial();
    }
  }

  if ( AlarmeAtivo() )
  {
    RtcDateTime t = rtc.GetDateTime();
    Serial.print("Instante: ");
    bufferDHT[contadorDHT].instante = DateTime2String(t);
    Serial.println(bufferDHT[contadorDHT].instante);
    Serial.print("Umidade do ar (%): ");
    Serial.println(sensorHT.readHumidity());
    Serial.print("Temperatura (º): ");
    Serial.println(sensorHT.readTemperature()); 

    // lendo dados do DHT22
    bufferDHT[contadorDHT].umidade = sensorHT.readHumidity();
    bufferDHT[contadorDHT].temperatura = sensorHT.readTemperature();
    contadorDHT++;

    Serial.print("Contador atual : ");
    Serial.println(contadorDHT);
    if ( contadorDHT == TAM_BUFFER)
      enviarDHT(bufferDHT);

    Serial.print("Pluviometro - Instante -");
    Serial.println(String(bufferPluviometro[contadorPluviometro].instante));
    Serial.print("Pulsos: ");
    bufferPluviometro[contadorPluviometro].pulsos = contadorPulsosPluviometro;
    contadorPulsosPluviometro = 0;
    Serial.println(bufferPluviometro[contadorPluviometro].pulsos);
    contadorPluviometro++;
    if ( contadorPluviometro == TAM_BUFFER)
      enviarPluviometro(bufferPluviometro);
    
    contadorDHT = contadorDHT % TAM_BUFFER;
    contadorPluviometro = contadorPluviometro % TAM_BUFFER;
    contadorVento = contadorVento % TAM_BUFFER;
  }
  if ( flagOscilacao == true)
  {
    Serial.println("P");
    Serial.flush();
    flagOscilacao = false;
  }  
}