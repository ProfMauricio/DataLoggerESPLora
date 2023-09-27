#include <Arduino.h>
#include <RtcDS3231.h>
#include <Wire.h>
#include "main.h"
#include "TransmissaoInfo.h"
/** #########
 *  Variáveis usadas
 *  #########
 */
RtcDS3231<TwoWire> Rtc(Wire);

/** #########
 *  Funções usadas
 *  #########
 */

String DateTime2String(RtcDateTime i)
{
  char datetimeStr[20];
  sprintf(datetimeStr, "%02u/%02u/%04u %02u:%02u:%02u",
          i.Day(), i.Month(), i.Year(), i.Hour(), i.Minute(), i.Second());
  return String(datetimeStr);
}
void setup()
{
  Serial.begin(115200);
  pinMode(pinoRTCSquareWave, INPUT);
  Rtc.Begin();
  sensorDHT.begin();
  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeAlarmTwo);
  DS3231AlarmTwo alarm2(0, 0, 0, DS3231AlarmTwoControl_OncePerMinute);
  Rtc.SetAlarmTwo(alarm2);
  Rtc.LatchAlarmsTriggeredFlags();
  IniciarLoRa();
}

void loop()
{ 
  RtcDateTime i = Rtc.GetDateTime();
  String temp = DateTime2String(i);
  if (int(i.Second())!=0)
  {
    PluviometroEnviar.instante = temp;
    PluviometroEnviar.pulsos = 1;
    enviarPluviometro(PluviometroEnviar);
  }
  else{
    DHTEnviar.instante = temp;
    DHTEnviar.temperatura = sensorDHT.readTemperature();
    DHTEnviar.umidade = sensorDHT.readHumidity();
    enviarDHT(DHTEnviar);
    Rtc.LatchAlarmsTriggeredFlags();
  }
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_37, LOW);
  esp_deep_sleep_start();
}