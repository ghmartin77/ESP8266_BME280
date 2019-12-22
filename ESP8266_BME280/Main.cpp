extern "C" {
#include <user_interface.h>
}
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include "defines.h"
#include "Log.h"

#define VERSION 			"1.0"
#define STATE_DEEPSLEEPMODE	0x01
#define STATE_SENDDATA		0x02
#define STATE_SAFEMODE		0x03

// BME280 Spec
#define TEMP_MIN -40.0f
#define TEMP_MAX 85.0f
#define HUM_MIN 0.0f
#define HUM_MAX 100.0f

char myhostname[16] = { 0 };

WiFiClient mqttWiFiClient;
PubSubClient mqtt(mqttWiFiClient);

boolean safeMode = false;
boolean bmeSensorAvailable;
boolean wifiOn = false;
Adafruit_BME280 bme;

unsigned long startedAtMs;

ADC_MODE(ADC_VCC);

struct {
	int magicNumber;
	int state;
	int counter;
	// -----------------------
	int vcc;
	int temp;
	int hum;
} rtcData;

void mqttReconnect() {
	int retries = 3;

	while (--retries >= 0 && !mqtt.connected()) {
		Logger.debug("Attempting MQTT connection...");

		if (mqtt.connect(myhostname)) {
			mqttWiFiClient.setSync(true);
			Logger.debug("Connected");
		} else {
			Logger.debug("Failed, rc=%n", mqtt.state());
			Logger.debug("Trying again in 2 seconds");
			delay(2000);
		}
	}
}

void publish(const char* topic, const char* payload, boolean retained) {
	int retries = 3;

	while (--retries >= 0) {
		if (!mqtt.connected()) {
			mqttReconnect();
		}
		if (mqtt.publish(topic, payload, retained))
			return;

		yield();
	}
}

void setupOTA() {
	ArduinoOTA.setPort(8266);
	ArduinoOTA.setHostname(myhostname);
	ArduinoOTA.onStart([]() {
		Serial.println("OTA Transfer started");
	});
	ArduinoOTA.onEnd([]() {
		Serial.println("DONE");
	});

	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		Serial.printf(".", (progress / (total / 100)));
	});
	ArduinoOTA.onError([](ota_error_t error) {
		Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR)
		Serial.println("Auth Failed");
		else
		if (error == OTA_BEGIN_ERROR)
		Serial.println("Begin Failed");
		else
		if (error == OTA_CONNECT_ERROR)
		Serial.println("Connect Failed");
		else
		if (error == OTA_RECEIVE_ERROR)
		Serial.println("Receive Failed");
		else
		if (error == OTA_END_ERROR)
		Serial.println("End Failed");
	});
	ArduinoOTA.begin();
}

void connectToWifi() {
	WiFi.hostname(myhostname);
	WiFi.mode(WIFI_STA);

	IPAddress ip, gateway, subnet;
	ip.fromString(IP_ADDR);
	gateway.fromString(IP_GATEWAY);
	subnet.fromString(IP_SUBNET);

	WiFi.config(ip, gateway, subnet);
	WiFi.begin(WIFI_SSID, WIFI_PASSWD);

	Logger.info("Connecting to WiFi...");

	unsigned long startWiFiConnect = millis();

	while (WiFi.status() != WL_CONNECTED) {
		delay(50);

		if (!safeMode
				&& (millis() - startWiFiConnect)
						> WIFI_CONNECT_TIMEOUT_SECS * 1e3) {
			Logger.info(
					"WiFi connect exceeded timeout of %d seconds. Will retry in %d seconds. Going to deep sleep...",
					WIFI_CONNECT_TIMEOUT_SECS, WIFI_RECONNECT_WAITTIME_SECS);
			ESP.deepSleep(WIFI_RECONNECT_WAITTIME_SECS * 1e6, WAKE_RFCAL);
			return;
		}
	}

	wifiOn = true;

	Logger.info("...Connected!");
	Logger.debug("SSID: %s", WiFi.SSID().c_str());
	Logger.debug("IP: %s", WiFi.localIP().toString().c_str());
}

boolean setupSafeMode() {
	/////////////////////////////////////////////////
	// ATTENTION - Change this code with care !!!
	// If you break it, Safe Mode's broken and you
	// cannot flash any longer using OTA.
	// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	WiFi.forceSleepBegin();
	yield();

	digitalWrite(D7, LOW);
	pinMode(D7, INPUT_PULLUP);

	pinMode(D6, OUTPUT);
	digitalWrite(D6, LOW);

	pinMode(PIN_BME_GND, OUTPUT);
	digitalWrite(PIN_BME_GND, LOW);

	pinMode(PIN_BME_VCC, OUTPUT);
	digitalWrite(PIN_BME_VCC, HIGH);

	sprintf(myhostname, "TH_%06X", ESP.getChipId());

	Logger.begin();
	Logger.setLogLevel(LOG_LEVEL_INFO);
	Logger.info("\n\n\n\n\r");
	system_rtc_mem_read(64, &rtcData, sizeof(rtcData));

	if (rtcData.magicNumber != 0xCAFE) {
		Logger.debug("Powered up for the first time");
		rtcData.magicNumber = 0xCAFE;
		rtcData.state = STATE_DEEPSLEEPMODE;
		rtcData.counter = 0;

		rtcData.temp = -1111;
		rtcData.hum = -1111;
		rtcData.vcc = -1111;
	} else {
		Logger.debug("Waking up after reset");
		rtcData.counter++;
	}

	Logger.debug("Hostname: %s", myhostname);

	Logger.debug("RTC data: vcc:%d, hum:%d, temp:%d, counter:%d, state:%d",
			rtcData.vcc, rtcData.hum, rtcData.temp, rtcData.counter,
			rtcData.state);

	if (digitalRead(D7) == 1 && rtcData.state != STATE_SAFEMODE) {
		rtcData.state = STATE_SAFEMODE;
		system_rtc_mem_write(64, &rtcData, sizeof(rtcData));
		ESP.deepSleep(10, WAKE_RFCAL);
	}

	if (digitalRead(D7) == 0 && rtcData.state == STATE_SAFEMODE) {
		rtcData.state = STATE_DEEPSLEEPMODE;
	}

	safeMode = (rtcData.state == STATE_SAFEMODE);

	if (safeMode) {
		Logger.info("Entering *********** SAFE MODE ***********");
	}

	if ((rtcData.state == STATE_SENDDATA) || safeMode) {
		WiFi.forceSleepWake();
		wifi_set_sleep_type(MODEM_SLEEP_T);

		connectToWifi();
	}

	if (wifiOn) {
		setupOTA();
	}

	system_rtc_mem_write(64, &rtcData, sizeof(rtcData));

	return safeMode;
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// Safe Mode code ends here.
	/////////////////////////////////////////////////
}

void setup() {
	startedAtMs = millis();

	/////////////////////////////////////////////////
	// ATTENTION - Don't change the following block.
	// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	if (setupSafeMode()) {
		return;
	}
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	/////////////////////////////////////////////////

	Logger.info("Starting T/H Sensor %s", VERSION);

	Wire.begin(PIN_BME_SDA, PIN_BME_CLK);

	bmeSensorAvailable = bme.begin(0x76);
	if (bmeSensorAvailable) {
		Logger.debug("BME280 sensor found.");
		bme.setSampling(Adafruit_BME280::MODE_FORCED,
				Adafruit_BME280::SAMPLING_X1, // temperature
				Adafruit_BME280::SAMPLING_X1, // pressure
				Adafruit_BME280::SAMPLING_X1, // humidity
				Adafruit_BME280::FILTER_OFF);
	} else {
		Logger.error("*NO* BME280 sensor found. Check your wiring...");
	}

	if (wifiOn && rtcData.state == STATE_SENDDATA) {
		mqtt.setServer(MQTT_SERVER, 1883);
	}
}

boolean loopSafeMode() {
	/////////////////////////////////////////////////
	// ATTENTION - Change this code with care !!!
	// If you break it, Safe Mode's broken and you
	// cannot flash any longer using OTA.
	// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	if (wifiOn) {
		ArduinoOTA.handle();
		Logger.loop();
	}

	return safeMode;
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	// Safe Mode code ends here.
	/////////////////////////////////////////////////
}

void loop() {
	/////////////////////////////////////////////////
	// ATTENTION - Don't change the following block.
	// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
	if (loopSafeMode()) {
		return;
	}
	// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
	/////////////////////////////////////////////////

	if (wifiOn && rtcData.state == STATE_SENDDATA) {
		if (!mqtt.connected()) {
			mqttReconnect();
		}
		mqtt.loop();

		String s("home/");
		s += LOCATION;
		String tempTopic = s + "/temperature";
		String humTopic = s + "/humidity";
		String vccTopic = s + "/vcc";

		publish(tempTopic.c_str(), String(rtcData.temp / 10.0).c_str(), true);
		publish(humTopic.c_str(), String(rtcData.hum).c_str(), true);
		publish(vccTopic.c_str(), String(rtcData.vcc / 100.0).c_str(), true);

		Logger.debug("MQTT data published");
		mqttWiFiClient.flush();
		mqtt.disconnect();

		rtcData.counter = 0;
		rtcData.state = STATE_DEEPSLEEPMODE;
	}

	float temp = 22.2;
	float hum = 44;
	int readRetries = 5;
	bool validRead = false;

	while (bmeSensorAvailable && !validRead && --readRetries >= 0) {
		bme.takeForcedMeasurement();
		temp = bme.readTemperature();
		hum = bme.readHumidity();

		validRead = (temp >= TEMP_MIN && temp <= TEMP_MAX && hum >= HUM_MIN
				&& hum <= HUM_MAX);

		if (!validRead)
			delay(100);

		temp = temp + ADJUST_TEMPERATURE;
		hum = hum + ADJUST_HUMIDITY;
	}

	float vcc = wifiOn ? rtcData.vcc / 100.0 : ESP.getVcc() / 1000.0;

	Logger.debug("Temp: %f, Humidity: %f, VCC: %f", temp, hum, vcc);

	int vccCur = vcc * 100;
	int tempCur = temp * 10;
	int humCur = hum;

	rtcData.state = STATE_DEEPSLEEPMODE;
	int tempDiff = abs(tempCur - rtcData.temp);
	int humDiff = abs(humCur - rtcData.hum);

	if (tempDiff >= THRESHOLD_SEND_DATA_TEMPERATURE
			|| humDiff >= THRESHOLD_SEND_DATA_HUMIDITY
			|| rtcData.counter * INTERVAL_READ_SENSOR_SECS
					>= INTERVAL_SEND_DATA_SECS) {
		Logger.debug("Threshold exceeded, resetting to send data...");

		rtcData.vcc = vccCur;
		rtcData.temp = tempCur;
		rtcData.hum = humCur;

		rtcData.state = STATE_SENDDATA;
	}

	digitalWrite(PIN_BME_VCC, LOW);

	if (wifiOn) {
		WiFiClient::stopAll();

		WiFi.persistent(false);
		WiFi.disconnect(true);
	}

	system_rtc_mem_write(64, &rtcData, sizeof(rtcData));

	Logger.debug("Having been awake for %dms", millis() - startedAtMs);

	if (rtcData.state == STATE_SENDDATA) {
		ESP.deepSleep(10, WAKE_RFCAL);
	} else {
		Logger.debug("Snoozing now for %d secs... ****zzzzZZZZ****",
		INTERVAL_READ_SENSOR_SECS);
		ESP.deepSleep(INTERVAL_READ_SENSOR_SECS * 1e6, WAKE_RF_DISABLED);
	}
}
