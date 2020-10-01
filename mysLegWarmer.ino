/*
 Name:		                    mysLegWarmer
 Created:	                    24.09.2020
 Modified:                      30.09.2020
 Programming:                   Alex aka Sr.FatCat

MySensors LegWarmer
TODO: 
	- Увеличить время HEART_BIT_INTERVAL
	- Подтверждение параметров обратной посылкой
	- Подтверждение параметров при старте
	- Перевод на таймер передачи Сонара
	- Перевод на таймер ожидания DS18B20
	- Умная презентация
	- Снижать яркость светодиода RF по ночам


*/

//#define MY_DEBUG
#define MY_NODE_ID 14
#define MY_RADIO_RF24
#define MY_REPEATER_FEATURE
#define RF24_DATARATE 	   RF24_250KBPS   //RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
#define RF24_PA_LEVEL 	   RF24_PA_MAX    //Sensor PA Level == RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_HIGH=-6dBM, and RF24_PA_MAX=0dBm
#define RF24_PA_LEVEL_GW   RF24_PA_MAX  //Gateway PA Level, defaults to Sensor net PA Level.  Tune here if using an amplified nRF2401+ in your gateway.

#include <MyArduino.h>

#include <MySensors.h>

#define PIN_OUT A1
#define PIN_LED_RF A2
#define PIN_LED_STBY A3
#define PIN_SONAR 3
#define PIN_LED_1WIRE 8

#include <OneWire.h>
#include <DallasTemperature.h>

OneWire oneWire(PIN_LED_1WIRE);
DallasTemperature sensTemp(&oneWire);

#include "Sonar.h"
CSonar sonar(PIN_SONAR, 500);

#define PROJECT "LEG WARMER"
#define VERSION "1.0"

#define CHILD_WARMER_TEMPER 0
#define CHILD_SONAR_ACTIVE 1
#define CHILD_SONAR_TIMEOUT 2
#define CHILD_STANDBY 3
#define CHILD_TEMPER_MAIN 4
#define CHILD_TEMPER_STDBY 5
#define CHILD_TEMPER_GISTER 6

MyMessage warmerTemp(CHILD_WARMER_TEMPER, V_TEMP);
MyMessage sonarActive(CHILD_SONAR_ACTIVE, V_TRIPPED);

#define HEART_BIT_INTERVAL 30000L//60000L

bool isPresentOk = false;
uint32_t timeSend = 0;

float temperMain, temperStdby, gisterTemp;
bool isStandby = false;
bool isActive = false;

#define EEPROM_TEMPER_MAIN_ADDRESS ((void*)(EEPROM_LOCAL_CONFIG_ADDRESS))
#define EEPROM_TEMPER_STDBY_ADDRESS ((void*)(EEPROM_LOCAL_CONFIG_ADDRESS + sizeof(temperMain)))
#define EEPROM_SONAR_TIMEOUT_ADDRESS ((void*)(EEPROM_LOCAL_CONFIG_ADDRESS + sizeof(temperMain) + sizeof(temperStdby)))
#define EEPROM_TEMPER_GISTER_ADDRESS ((void*)(EEPROM_LOCAL_CONFIG_ADDRESS + sizeof(temperMain) + sizeof(temperStdby) + sizeof(uint32_t)))


void loadParam(uint32_t& sonarTimeout){
	hwReadConfigBlock((void *)&temperMain, EEPROM_TEMPER_MAIN_ADDRESS, sizeof(temperMain));
	hwReadConfigBlock((void *)&temperStdby, EEPROM_TEMPER_STDBY_ADDRESS, sizeof(temperStdby));
	hwReadConfigBlock((void *)&sonarTimeout, EEPROM_SONAR_TIMEOUT_ADDRESS, sizeof(sonarTimeout));
	hwReadConfigBlock((void *)&gisterTemp, EEPROM_TEMPER_GISTER_ADDRESS, sizeof(gisterTemp));
}

void before(){
	pinMode(PIN_OUT, OUTPUT);
	pinMode(PIN_LED_RF, OUTPUT);
	pinMode(PIN_LED_STBY, OUTPUT);

	digitalWrite(PIN_LED_RF, 1);
}

void presentation(){
	isPresentOk = sendSketchInfo(PROJECT, VERSION, true);
	isPresentOk &= present(CHILD_WARMER_TEMPER, S_TEMP, "Warmer temper.", true);
	isPresentOk &= present(CHILD_SONAR_ACTIVE, S_BINARY, "Sonar detect", true);
	isPresentOk &= present(CHILD_SONAR_TIMEOUT, S_CUSTOM, "Active timeout", true);
	isPresentOk &= present(CHILD_STANDBY, S_BINARY, "Standby regim", true);
	isPresentOk &= present(CHILD_TEMPER_MAIN, S_TEMP, "Main temper.", true);
	isPresentOk &= present(CHILD_TEMPER_STDBY, S_TEMP, "Standby temper.", true);
	isPresentOk &= present(CHILD_TEMPER_GISTER, S_TEMP, "Gister. for temper.", true);
}

void heartBeatSend(){
	if (millis() - timeSend < HEART_BIT_INTERVAL) return;
	bool res = sendHeartbeat(true);
	if (res){
		timeSend = millis();
	}
}

void blinkStdbyLed(){
	DEF_TMENEGMENT;

	if (digitalRead(PIN_OUT)){
		digitalWrite(PIN_LED_STBY, /*!isActive && */isStandby);
	}
	else {
		bool ledState = (digitalRead(PIN_LED_STBY) == LOW);
		if (isActive && isStandby) {
			if (!ledState && t - prev_t > 200) {
				digitalWrite(PIN_LED_STBY , LOW);
				PASS_TMENEGMENT;
			}
			else if (ledState && t - prev_t > 500) {
				digitalWrite(PIN_LED_STBY , HIGH);
				PASS_TMENEGMENT;
			}
		}
		else if (isActive){
			if (!ledState && t - prev_t > 20) {
				digitalWrite(PIN_LED_STBY , LOW);
				PASS_TMENEGMENT;
			}
			else if (ledState && t - prev_t > 3000) {
				digitalWrite(PIN_LED_STBY , HIGH);
				PASS_TMENEGMENT;
			}
		}
		else if (isStandby){
			digitalWrite(PIN_LED_STBY , HIGH);
		}
		else {
			digitalWrite(PIN_LED_STBY , LOW);
		}
	}
}

void blinkRFLed(){
	DEF_TMENEGMENT;
	if (millis() - timeSend > HEART_BIT_INTERVAL || !_transportSM.uplinkOk) {
		digitalWrite(PIN_LED_RF, HIGH);
	}
	else {
		bool ledState = (digitalRead(PIN_LED_RF) == LOW);
		if (!ledState && t - prev_t > 20) {
			digitalWrite(PIN_LED_RF, LOW);
			PASS_TMENEGMENT;
		}
		else if (ledState && t - prev_t > 3000)	{
			digitalWrite(PIN_LED_RF, HIGH);
			PASS_TMENEGMENT;
		}
	}
}

void activeSender(){
	const bool act = sonarActive.getBool();

	if (act != isActive) {
		CORE_DEBUG(PSTR("Sonar active is %i\n"), isActive );
		if (send(sonarActive.set(isActive), true)){
			timeSend = millis();
		}
		else {
			sonarActive.set(act);
		}
	}
}

void thermoStat(){
	static uint32_t timeForReadTemper = 0;
	const uint32_t t = millis();

	if (timeForReadTemper == 0) {
		sensTemp.requestTemperatures();
		timeForReadTemper = t + sensTemp.millisToWaitForConversion(sensTemp.getResolution());
	}	
	else if (timeForReadTemper < t) {
		timeForReadTemper = 0;
		float tempC = sensTemp.getTempCByIndex(0);
		if(tempC != DEVICE_DISCONNECTED_C) {
			float tc = warmerTemp.getFloat();
			if (abs(tempC-tc) >= 0.5){
				CORE_DEBUG(PSTR("Temperature is: %i.%i°C\n"), int(tempC), FRACT100(tempC) );
				if (send(warmerTemp.set(tempC, 2), true)){
					timeSend = t;
				}
				else {
					warmerTemp.set(tc, 2);
				}
			}
			/*---------------------------мясо---------------------------*/
			if (isActive || isStandby){
				float setC = ( isActive ? temperMain : temperStdby );
				if (tempC >= setC + gisterTemp) digitalWrite(PIN_OUT, LOW);
				if (tempC <= setC - gisterTemp) digitalWrite(PIN_OUT, HIGH);
			}
			else {
				digitalWrite(PIN_OUT, LOW);
			}
			/*---------------------------------------------------------*/
		} 
		else {
			CORE_DEBUG(PSTR("Error: Could not read temperature data\n"));
		}
	}

}

void setup(){
	digitalWrite(PIN_LED_RF, !_transportSM.uplinkOk);
	CORE_DEBUG(PSTR("********** %s *******\n"), PROJECT);
	CORE_DEBUG(PSTR("********** present = %i \n"), isPresentOk);
	sensTemp.begin();
	sensTemp.setWaitForConversion(false);
	
	uint32_t sonarTimeout;
	loadParam(sonarTimeout);
	sonar.attachVar(&isActive, 450, sonarTimeout);
	warmerTemp.set(0.0, 2);
	
	CORE_DEBUG(PSTR("********** Load param MT:%i.%i ST:%i.%i GT:%i.%i TO:%ldms\n"), int(temperMain), FRACT100(temperMain), int(temperStdby), FRACT100(temperStdby), 
		int(gisterTemp), FRACT100(gisterTemp), sonarTimeout);
	
	bool okReq;
	for(int i=0; i<5; i++){
		if (okReq = request(CHILD_STANDBY, V_STATUS)){
			CORE_DEBUG(PSTR("********** Request param CHILD_STANDBY is OK\n"));
			break;
		}
		wait(100);
	}
	if (!okReq) CORE_DEBUG(PSTR("**********ERR!!! Request param CHILD_STANDBY\n"));
}

void loop(){
#ifdef MY_DEBUG
    uint16_t dist;
    if (sonar.get(dist)) CORE_DEBUG(PSTR("Active %i, dist: %imm\n"), isActive, dist);
#endif	
    sonar.update();
	activeSender();

	thermoStat();
	heartBeatSend();
	blinkRFLed();
	blinkStdbyLed();
}

void receive(const MyMessage &message){
	if (message.getSensor() == CHILD_SONAR_TIMEOUT && message.getType()==V_VAR1){
		uint32_t tm = strtoul(message.getString(), 0, 10);
		sonar.setVarTimeout(tm);
		//CORE_DEBUG(PSTR(">>>>> Receive for %ld ==>"), tm);
		hwWriteConfigBlock((void *)&tm, EEPROM_SONAR_TIMEOUT_ADDRESS, sizeof(tm));
		CORE_DEBUG(PSTR(">>>>> Receive for SONAR_TIMEOUT => %ld\n"), tm);
	}

	if (message.getSensor() == CHILD_STANDBY && message.getType()==V_STATUS){
		isStandby = message.getBool();
		CORE_DEBUG(PSTR(">>>>> Receive for CHILD_STANDBY => %i\n"), isStandby);
	}

	if (message.getSensor() == CHILD_TEMPER_MAIN && message.getType()==V_TEMP){
		temperMain = message.getFloat();
		hwWriteConfigBlock((void *)&temperMain, EEPROM_TEMPER_MAIN_ADDRESS, sizeof(temperMain));
		CORE_DEBUG(PSTR(">>>>> Receive for CHILD_TEMPER_MAIN => %i.%i\n"), int(temperMain), FRACT100(temperMain));
	}

	if (message.getSensor() == CHILD_TEMPER_STDBY && message.getType()==V_TEMP){
		temperStdby = message.getFloat();
		hwWriteConfigBlock((void *)&temperStdby, EEPROM_TEMPER_STDBY_ADDRESS, sizeof(temperStdby));
		CORE_DEBUG(PSTR(">>>>> Receive for CHILD_TEMPER_STDBY =>%i.%i\n"), int(temperStdby), FRACT100(temperStdby));
	}

	if (message.getSensor() == CHILD_TEMPER_GISTER && message.getType()==V_TEMP){
		gisterTemp = message.getFloat();
		hwWriteConfigBlock((void *)&gisterTemp, EEPROM_TEMPER_GISTER_ADDRESS, sizeof(gisterTemp));
		CORE_DEBUG(PSTR(">>>>> Receive for CHILD_TEMPER_GISTER => %i.%i\n"), int(gisterTemp), FRACT100(gisterTemp));
	}
}


