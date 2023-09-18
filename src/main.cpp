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

// Pinos do display (comunicação i2c)
const int DISPLAY_ADDRESS_PIN = 0x3c;
const int DISPLAY_SDA_PIN = 4;
const int DISPLAY_SCL_PIN = 15;
const int DISPLAY_RST_PIN = 16;

SSD1306 display(0x3c, DISPLAY_SDA_PIN, DISPLAY_SCL_PIN);
controleOLED meuOled(&display);
unsigned long inicio = millis();

/**
 * Objeto do rtc para captura de instantes
 * */
RtcDS3231<TwoWire> rtc(Wire1);

unsigned long int contadorPacotesEnviados = 0, contadorPacotesFalha = 0;

/**
 * Buffer que guarda as medidas de umidade/temperatura lidas
 **/
DHT_Data bufferDHT[TAM_BUFFER];
volatile int contadorDHT = 0;

/**
 * String que armazena os instantes
 * */
String datetimeStr;

/**
 * Objeto que captura os dados do sensor DHT
 * */
DHT sensorHT(DHT_PIN, DHT22);

/**
 * Contador de pulsos do pluviometro utilizado na interrupção para
 * gravar dados
 * */
volatile int contadorPulsosPluviometro = 0;

/**
 * Buffer que armazena as medidas de vento
 **/
Vento_Data bufferVento[TAM_BUFFER];
volatile int contadorVento = 0;

/**
 * Flag de aviso do usuario que deseja forçar a realização de uma coleta
 **/
bool usuarioSolicitaLeitura = false;

/**
 * Contador de interrupção por alarme
 * */
volatile uint64_t interruptCountAlarm = 0;

String bufferSerial;

/**
 * Flag do alarme RTC apra captura de dados por alarme do DS3231
 **/
volatile bool interruptFlagAlarm = false;

/**
 * Buffer que guarda a quantidade de pulsos em determinado tempo
 **/
Pluvi_Data bufferPluviometro[TAM_BUFFER];
int contadorPluviometro = 0;
// int pulsosPluvi = 0;

/**
 * Flag que sinaliza que houve oscilação no pluviometro
 * */
volatile bool flagOscilacao = false;

/*****************************************************************************************************
 *
 *          INICIO DAS FUNÇÕES
 *
 ******************************************************************************************************/

/**
 * Rotina chamada na interrupção do RTC
 * */
void IRAM_ATTR RotinaInterrupcaoAlarm()
{
  interruptCountAlarm++;
  interruptFlagAlarm = true;
}

/**
 * Função que verifica se existe algum alarme ativo do RTC
 * */
bool AlarmeAtivo()
{
  bool ativo = false;

  if (interruptFlagAlarm == true)
  {
    interruptFlagAlarm = false;

    DS3231AlarmFlag flag = rtc.LatchAlarmsTriggeredFlags();
    if (flag & DS3231AlarmFlag_Alarm2)
    {
      Serial.println("Alarme 2 ativado");
      ativo = true;
    }
  }
  else if (usuarioSolicitaLeitura == true)
  {
    usuarioSolicitaLeitura = false;
    return true;
  }
  return ativo;
}

// #######################################################################################################

unsigned long ultimoPluviometroSegundos = millis();

void IRAM_ATTR CapturarOscilacao()
{
  if (millis() - ultimoPluviometroSegundos > 2000)
  {
    bufferPluviometro[contadorPluviometro].pulsos++;
    flagOscilacao = true;
#if _DEBUG == SERIAL_VIEW
    Serial.print("#");
#endif
    ultimoPluviometroSegundos = millis();
  }
}

void IRAM_ATTR CapturarEventoAnemometro()
{
  contadorVento++;
#if _DEBUG == SERIAL_VIEW
  Serial.print("~");
#endif
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

double MedirVelocidadeVento()
{
  int leitura, ultimoEstado = LOW;
  int debounceDelay = 10;
  int ultimoInstanteDebounce = 0;
  int estadoPino = LOW;

  Serial.println("iniciando medida de vento..");
  contadorVento = 0;
  // habilitar interrupçao na porta conectada ao sensor
  // attachInterrupt(ANEMOMETRO_PIN, CapturarEventoAnemometro, LOW );
  unsigned long instanteInicio = millis();
  while (millis() - instanteInicio < (TEMPO_AMOSTRAGEM_VEL_VENTO * 1000))
  {

    leitura = digitalRead(ANEMOMETRO_PIN);

    if (leitura != ultimoEstado)
    {
      ultimoInstanteDebounce = millis();
    }
    if ((millis() - ultimoInstanteDebounce) > debounceDelay)
    {

      if (leitura != estadoPino)
      {
        estadoPino = leitura;
        if (estadoPino == HIGH)
          contadorVento++;
      }
    }
    ultimoEstado = leitura;
  }

  // encerra interrupçao na porta
  // detachInterrupt(ANEMOMETRO_PIN);
  Serial.println("fim da amostragem");
  Serial.print("Pulsos computados: ");
  Serial.println(contadorVento);
  // calcula velocidade com base nas configuraçoes do equipamento
  double pulsosPorRaio = 8.0;
  // tamanho do raio em metros
  double raio = 0.02;
  double deltaS = (double)contadorVento / pulsosPorRaio;
  Serial.print("DeltaS = ");
  Serial.println(deltaS);
  deltaS *= raio;
  // calculando a velocidade já em metros/segundo
  double velocidade = (2.0 * PI * deltaS) / TEMPO_AMOSTRAGEM_VEL_VENTO;
  Serial.print(F("Velocidade -> "));
  Serial.println(velocidade);

  return velocidade;
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
  tmpMsg = bufferSerial.substring(0, 5);
#if _DEBUG >= DEBUG_SERIAL
  Serial.println(tmpMsg);
#endif
  // testando se o usuario que ajustar o horario
  if (tmpMsg.equalsIgnoreCase("@time"))
  {
    instante = rtc.GetDateTime();
    tmpMsg = obterParametroSerial(1);
    par1 = (uint8_t)tmpMsg.toInt();
    tmpMsg = obterParametroSerial(2);
    par2 = tmpMsg.toInt();
    tmpMsg = obterParametroSerial(3);
    par3 = (uint8_t)tmpMsg.toInt();
    RtcDateTime novoInstante;
    novoInstante = RtcDateTime(instante.Year(), instante.Month(), instante.Day(), par1, par2, par3);
    rtc.SetDateTime(novoInstante);
    instante = rtc.GetDateTime();
    instanteStr = DateTime2String(instante);
    Serial.println(tmpMsg);
  }
  else if (tmpMsg.equalsIgnoreCase("@date"))
  {
    Serial.print(F("Ajustando data"));
    instante = rtc.GetDateTime();
    tmpMsg = obterParametroSerial(1);
    par2 = tmpMsg.toInt();
    tmpMsg = obterParametroSerial(2);
    par3 = tmpMsg.toInt();
    tmpMsg = obterParametroSerial(3);
    ano = tmpMsg.toInt();
    RtcDateTime novoInstante = RtcDateTime(ano, par3, par2, instante.Hour(), instante.Minute(), instante.Second());
    rtc.SetDateTime(novoInstante);
    instante = rtc.GetDateTime();
    instanteStr = DateTime2String(instante);
    Serial.println(tmpMsg);
  }
  else if (tmpMsg.equalsIgnoreCase("@dado")) // usuário solicitando coleta de dados
  {
    Serial.println(F("Ativando flag de captura"));
    usuarioSolicitaLeitura = true;
    Serial.print("valor de flag: ");
    Serial.println(interruptFlagAlarm);
  }
  else if (tmpMsg.equalsIgnoreCase("@tirq")) // requisição de instante pelo ESP
  {
    instante = rtc.GetDateTime();
    instanteStr = DateTime2String(instante);
    Serial1.print(instanteStr);
  }
  else if (tmpMsg.equalsIgnoreCase("@help"))
    help();

  // testando se o comando para sair do modo de comandos
  else if (tmpMsg.equalsIgnoreCase("@exit"))
  {
    Serial.print(F("Saindo do modo de comando"));
  }
  else
  {
    tmpMsg = bufferSerial.substring(0, 4);
    if (tmpMsg.equalsIgnoreCase("@now"))
    {
      instante = rtc.GetDateTime();
      instanteStr = DateTime2String(instante);
      Serial.println(instanteStr);
    }
    else if (tmpMsg.equalsIgnoreCase("@deb"))
    {
      Serial.println(F("Situacao do sistema"));
      instante = rtc.GetDateTime();
      instanteStr = DateTime2String(instante);
      Serial.println(instanteStr);
      Serial.print(F("Pulsos do pluviometro: "));
      Serial.println(contadorPulsosPluviometro);
      Serial.print("Umidade do ar (%): ");
      Serial.println(sensorHT.readHumidity());
      Serial.print("Temperatura (º): ");
      Serial.println(sensorHT.readTemperature());
    }
    else
      Serial.println(F("Comando nao reconhecido"));
  }
  //
  // zerando buffer a cada comando
  bufferSerial = "";
}

/*************************************************************************************
 * Funcao para obter os parametros passados pela serial
 *************************************************************************************/
String obterParametroSerial(int nroParam)
{
  String tmp = "";
  tmp.reserve(5);
  switch (nroParam)
  {
  case 1:
    tmp = bufferSerial.substring(6, 8);
#if _DEBUG >= DEBUG_SERIAL
    Serial.print("par 1 ->");
    Serial.println(tmp.toInt());
#endif
    break;
  case 2:
    tmp = bufferSerial.substring(9, 11);
#if _DEBUG >= DEBUG_SERIAL
    Serial.print("par 2 ->");
    Serial.println(tmp.toInt());
#endif
    break;
  case 3:
    tmp = bufferSerial.substring(12, 16);
#if _DEBUG >= DEBUG_SERIAL
    Serial.print("par 3 ->");
    Serial.println(tmp.toInt());
#endif
    break;
  }
  return tmp;
}

String DateTime2String(RtcDateTime i)
{
  char datetimeStr[20];
  sprintf(datetimeStr, "%02u/%02u/%04u %02u:%02u:%02u",
          i.Day(), i.Month(), i.Year(), i.Hour(), i.Minute(), i.Second());
  return String(datetimeStr);
}

void setup()
{
  String instante;
  // put your setup code here, to run once:
  delay(1000);
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(pinoRTCSquareWave, INPUT);
  // pinMode(PLUVI_PIN, INPUT );
  // pinMode(ANEMOMETRO_PIN, INPUT);

  while (!Serial)
    ;
  Wire1.begin(21, 22);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.println(F("Iniciando OLED"));
  pinMode(DISPLAY_RST_PIN, OUTPUT); // RST do oled
  pinMode(25, OUTPUT);
  digitalWrite(DISPLAY_RST_PIN, LOW); // resetao OLED
  delay(50);
  digitalWrite(DISPLAY_RST_PIN, HIGH); // enquanto o OLED estiver ligado, GPIO16 deve estar HIGH
  display.init();                      // inicializa o display
  display.flipScreenVertically();
  // display.setFont(ArialMT_Plain_12); //configura a fonte para um tamanho maior
  display.clear();

  Serial.print("Compilado em: ");
  Serial.print(__DATE__);
  Serial.print(F(" "));
  Serial.println(__TIME__);
  // Wire.begin();
  rtc.Begin();
  RtcDateTime compilador = RtcDateTime(__DATE__, __TIME__);
  instante = DateTime2String(compilador);

  if (!rtc.IsDateTimeValid())
  {
    Serial.printf("Tempo inválido");
    if (rtc.LastError() != 0)
    {
      Serial.println("Erro de comunicação");
      Serial.println(rtc.LastError());
    }
    else
    {
      // rtc.SetAgingOffset(compilador);
    }
  }
  Serial.print("Instante do relogio: ");
  instante = DateTime2String(rtc.GetDateTime());
  Serial.println(instante);

  

  // ajustes de pinos para interrupcao do rtc
  rtc.Enable32kHzPin(false);
  rtc.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmTwo);
  Serial.println(uint8_t(rtc.LastError()));
  // Ajustando o alarme 2 para disparar a cada 1 minuto


  DS3231AlarmTwo alarme2(0,0,0,DS3231AlarmTwoControl_OncePerMinute);
  Serial.println(uint8_t(rtc.LastError()));
  rtc.SetAlarmTwo(alarme2);
  Serial.println(uint8_t(rtc.LastError()));
  rtc.LatchAlarmsTriggeredFlags();
  Serial.println(uint8_t(rtc.LastError()));



  // ativando as interrupções
  attachInterrupt(digitalPinToInterrupt(pinoRTCSquareWave), RotinaInterrupcaoAlarm, FALLING);

  delay(1000);
    




  // Iniciando sistema de coleta de temperatura/umidade

  attachInterrupt(digitalPinToInterrupt(PLUVI_PIN), CapturarOscilacao, FALLING);

  Serial.println(F("Iniciando sensor DHT22"));
  sensorHT.begin();
  Serial.println("Umidade :" + String(sensorHT.readHumidity()));
  Serial.println("Temperatura: " + String(sensorHT.readTemperature()));

  Serial.println(F("Iniciando o sistema de coleta de dados "));
  Serial.println(F("Sistema de coleta de sensores"));

  Serial.println(F("Iniciando LoRa no client"));
  IniciarLoRa();
  Serial.println("LoRa iniciado");
  meuOled.modificarLinha(LORA_INFO, "LoRa started");
  meuOled.atualizarVisualizacao();
}

/**
 * Rotina principal de looP
 * */
void loop()
{
  String temp;
  double vel;
  meuOled.modificarLinha(USER_INFO, DateTime2String(rtc.GetDateTime()));
  meuOled.modificarLinha(BUFFER_INFO, "Buffer Size DHT: " + String(contadorDHT) + "   " + String(contadorPluviometro));
  meuOled.atualizarVisualizacao();
  unsigned long atual = millis();
  RtcDateTime ob;
  uint8_t diaAtual = ob.Day();
  // put your main code here, to run repeatedly:
  if (AlarmeAtivo() || atual - inicio > 3000)
  {

    // meuOled.modificarLinha(STATUS_INFO, "Reading sensors");
    // meuOled.atualizarVisualizacao();
    RtcDateTime t = rtc.GetDateTime();
    temp = DateTime2String(t);

    /*
    bufferVento[contadorVento].instante =  temp;
    //vel = MedirVelocidadeVento();
    vel = 10;
    Serial.print(F("Velocidade do vento: "));
    Serial.println( vel );

    bufferVento[contadorVento].velocidade =  vel;
    Serial.print("Instante: ");


    */
    Serial.println();
    Serial.println();
    bufferDHT[contadorDHT].instante = temp;
    Serial.println(bufferDHT[contadorDHT].instante);
    Serial.print("Contador atual DHT: ");
    Serial.println(contadorDHT);
    //  Serial.print("Contador atual Pluviometro: ");
    // Serial.println(contadorPluviometro);
    // Serial.println("#############");
    Serial.flush();

    // DHT22
    Serial.print("Umidade do ar (%): ");
    Serial.println(sensorHT.readHumidity());
    Serial.print("Temperatura (º): ");
    Serial.println(sensorHT.readTemperature());
    bufferDHT[contadorDHT].umidade = sensorHT.readHumidity();
    bufferDHT[contadorDHT].temperatura = sensorHT.readTemperature();
    contadorDHT++;

    /*
        // PLUVIOMETRO
        Serial.print(F("Pulso(s) pluviometro "));
        if(bufferPluviometro[contadorPluviometro].pulsos!=0){
        bufferPluviometro[contadorPluviometro].instante = temp;
        Serial.print(F("Pulsos: "));
        //bufferPluviometro[contadorPluviometro].pulsos = contadorPulsosPluviometro;
        Serial.println(bufferPluviometro[contadorPluviometro].pulsos);
        contadorPluviometro++;
        Serial.println("diferente(s) de zero. Dados coletados");
        }
        Serial.println("igual a zero. Dados nao coletados");
        if (TAM_BUFFER==contadorPluviometro)
          {
            if (enviarPluviometro(bufferPluviometro, contadorPluviometro))
            {
              // GravarDadosPluvi();
              Serial.println();
              Serial.println("Dados de pulsos do pluviometro enviados");
              contadorPluviometro = 0;

            }
        }
    */

    if (TAM_BUFFER <= contadorDHT)
    {
      meuOled.modificarLinha(LORA_INFO, "Preparing Packet");
      meuOled.atualizarVisualizacao();
      detachInterrupt(digitalPinToInterrupt(PLUVI_PIN));
      if (enviarDHT(bufferDHT, contadorDHT))
      {
        Serial.println();
        Serial.println("Dados de Umidade e Temperatura enviados"); //      GravarDadosDHT();
        meuOled.modificarLinha(LORA_INFO, "Send it (" + String(contadorPacotesEnviados) + ")");
        meuOled.atualizarVisualizacao();
        contadorDHT = 0;
      }

      if (enviarPluviometro(bufferPluviometro, contadorPluviometro))
      {
        // GravarDadosPluvi();
        Serial.println();
        Serial.println("Dados de pulsos do pluviometro enviados");
        contadorPluviometro = 0;
      }
    }
    attachInterrupt(digitalPinToInterrupt(PLUVI_PIN), CapturarOscilacao, FALLING);

    inicio = atual;

    /*
        contadorPluviometro = contadorPluviometro % TAM_BUFFER;
        contadorVento = contadorVento % TAM_BUFFER;
      contadorDHT = contadorDHT % TAM_BUFFER;
    */
  }

  if (flagOscilacao || ob.Day() != diaAtual)
  {
    flagOscilacao = false;
    RtcDateTime t = rtc.GetDateTime();
    temp = DateTime2String(t);

    if (ob.Day() != diaAtual)
    {

      contadorPluviometro++;
      bufferPluviometro[contadorPluviometro].pulsos = 0;
      bufferPluviometro[contadorPluviometro].instante = temp;
      diaAtual = ob.Day();
      detachInterrupt(digitalPinToInterrupt(PLUVI_PIN));
      if (enviarPluviometro(bufferPluviometro, contadorPluviometro))
        contadorPluviometro = 0;
      attachInterrupt(digitalPinToInterrupt(PLUVI_PIN), CapturarOscilacao, FALLING);
      return;
    }

    Serial.print("Contador atual Pluviometro: ");
    Serial.println(contadorPluviometro);
    Serial.println("#############");
    Serial.flush();
    Serial.print(F("Pulso(s) pluviometro "));
    bufferPluviometro[contadorPluviometro].instante = temp;
    contadorPluviometro++;

    if (TAM_BUFFER <= contadorPluviometro)
    {
      detachInterrupt(digitalPinToInterrupt(PLUVI_PIN));
      if (enviarPluviometro(bufferPluviometro, contadorPluviometro))
      {
        // GravarDadosPluvi();
        Serial.println();
        Serial.println("Dados de pulsos do pluviometro enviados");
        contadorPluviometro = 0;
      }
      attachInterrupt(digitalPinToInterrupt(PLUVI_PIN), CapturarOscilacao, FALLING);
    }
  }

  /*

  if ( flagOscilacao == true)
  {
 //   Serial.println("P");
 //   Serial.flush();
    //detachInterrupt(PLUVI_PIN);
    contadorPulsosPluviometro++;
    flagOscilacao = false;
    //attachInterrupt(PLUVI_PIN, CapturarOscilacao, FALLING );

  }





  if (Serial.available()> 0)
  {
    //meuOled.modificarLinha(STATUS_INFO, "Serial checking ...");
    //meuOled.atualizarVisualizacao();
    // get the new byte:
    bufferSerial = Serial.readStringUntil('\rw');
    tratarEntradaSerial();
  }


  //temp = VerificarSerialMestre();


  if (temp != "")
  {
  Serial.print("Mensagem do mestre : ");
  Serial.println(temp);
  //TratarMsgMestre(temp);
  }
  */

  // Serial.println("entrando em modo sleep");
  // esp_sleep_enable_ext0_wakeup(GPIO_NUM_37,HIGH);
  // esp_deep_sleep_start();
}