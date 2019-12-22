/*
 * defines.h
 *
 *  Created on: 03.01.2019
 *      Author: ghmartin77
 */

#ifndef DEFINES_H_
#define DEFINES_H_

#define WIFI_SSID						"xyz"
#define WIFI_PASSWD						"*******"

#define WIFI_CONNECT_TIMEOUT_SECS		10
#define WIFI_RECONNECT_WAITTIME_SECS	5

#define IP_ADDR 						"192.168.0.77"
#define IP_GATEWAY 						"192.168.0.1"
#define IP_SUBNET 						"255.255.255.0"

#define MQTT_SERVER 					"192.168.0.14"

// see loop() for topics created from LOCATION
#define LOCATION						"your_room"

#define INTERVAL_READ_SENSOR_SECS		300
#define INTERVAL_SEND_DATA_SECS			3600

#define THRESHOLD_SEND_DATA_TEMPERATURE	10
#define THRESHOLD_SEND_DATA_HUMIDITY	2

#define ADJUST_TEMPERATURE				0.0
#define ADJUST_HUMIDITY					0.0

// Purple BME280
#define PIN_BME_SDA						D4
#define PIN_BME_CLK						D3
#define PIN_BME_VCC						D1
#define PIN_BME_GND						D2

// Blue BME280
//#define PIN_BME_SDA						D1
//#define PIN_BME_CLK						D2
//#define PIN_BME_VCC						D4
//#define PIN_BME_GND						D3

#endif /* DEFINES_H_ */
