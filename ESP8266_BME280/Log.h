#ifndef LOG_H_
#define LOG_H_

#define LOG_LEVEL_NONE 0
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_INFO 2
#define LOG_LEVEL_DEBUG 3
#define LOG_LEVEL_VERBOSE 4

#ifndef LOGLEVEL
//#define LOGLEVEL LOG_LEVEL_NONE
//#define LOGLEVEL LOG_LEVEL_ERROR
//#define LOGLEVEL LOG_LEVEL_INFO
#define LOGLEVEL LOG_LEVEL_DEBUG
//#define LOGLEVEL LOG_LEVEL_VERBOSE
#endif

#include <Arduino.h>
#include <inttypes.h>
#include <stdarg.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>

class Log {
public:
	Log();
	virtual ~Log();

	void begin();
	void begin(int level, long baud, boolean logServer);

	void setLogLevel(int level);

	void error(char* msg, ...);
	void info(char* msg, ...);
	void debug(char* msg, ...);
	void verbose(char* msg, ...);

	void loop();

protected:
	inline void println(String& msg) {
		Serial.println(msg);
		if (_logServer && _pServerClient && _pServerClient->connected())
			_pServerClient->println(msg);
	}

private:
	int _level = LOG_LEVEL_DEBUG;
	long _baud = 115200L;
	boolean _logServer = false;
	WiFiServer *_pServer = NULL;
	WiFiClient *_pServerClient = NULL;

	String bufferString;

#if LOGLEVEL > 0
	void print(const char *format, va_list args, String& dst);
#endif
};

extern Log Logger;

#endif /* LOG_H_ */
