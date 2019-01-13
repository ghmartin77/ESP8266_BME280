#include "Log.h"
#include <WString.h>

Log::Log() {
}

Log::~Log() {
	if (_pServer)
		_pServer->stop();
		delete _pServer;
	if (_pServerClient)
		_pServerClient->stop();
		delete _pServerClient;
}

void Log::begin() {
	begin(LOG_LEVEL_DEBUG, 115200L, true);
}

void Log::begin(int level, long baud, boolean logServer) {
	_level = constrain(level, LOG_LEVEL_NONE, LOG_LEVEL_VERBOSE);
	_baud = baud;
	_logServer = logServer;
	Serial.begin(_baud);
}

void Log::setLogLevel(int level) {
	_level = constrain(level, LOG_LEVEL_NONE, LOGLEVEL);
	if (level > LOGLEVEL) {
		error("Loglevel %d not possible", level);     // requested level
		error("Selected level %d instead", _level);   // selected/applied level
	}
}

void Log::error(char* msg, ...) {
	if (LOG_LEVEL_ERROR <= _level) {
		bufferString.remove(0);
		bufferString += "Error: ";
		va_list args;
		va_start(args, msg);
		print(msg, args, bufferString);
		println(bufferString);
	}
}

void Log::info(char* msg, ...) {
#if LOGLEVEL > 1
	if (LOG_LEVEL_INFO <= _level) {
		bufferString.remove(0);
		bufferString += "Info: ";
		va_list args;
		va_start(args, msg);
		print(msg, args, bufferString);
		println(bufferString);
	}
#endif
}

void Log::debug(char* msg, ...) {
#if LOGLEVEL > 2
	if (LOG_LEVEL_DEBUG <= _level) {
		bufferString.remove(0);
		bufferString += "Debug: ";
		va_list args;
		va_start(args, msg);
		print(msg, args, bufferString);
		println(bufferString);
	}
#endif
}

void Log::verbose(char* msg, ...) {
#if LOGLEVEL > 3
	if (LOG_LEVEL_VERBOSE <= _level)
	{
		bufferString.remove(0);
		bufferString += "Verbose: ";
		va_list args;
		va_start(args, msg);
		print(msg, args, bufferString);
		println(bufferString);
	}
#endif
}

void Log::print(const char *format, va_list args, String& dst) {
	for (; *format != 0; ++format) {
		if (*format == '%') {
			++format;
			if (*format == '\0')
				break;
			else if (*format == '%') {
				dst.concat(*format);
				continue;
			} else if (*format == 's') {
				register char *s = (char *) va_arg(args, int);
				dst.concat(s);
				continue;
			} else if (*format == 'd' || *format == 'i' || *format == 'c') {
				dst.concat(va_arg(args, int));
				continue;
			} else if (*format == 'f') {
				dst.concat(va_arg(args, double));
				continue;
			} else if (*format == 'l') {
				dst.concat(va_arg(args, long));
				continue;
			} else if (*format == 't') {
				if (va_arg(
						args, int) == 1) {
					dst.concat('T');
				} else {
					dst.concat('F');
				}
				continue;
			} else if (*format == 'T') {
				if (va_arg(
						args, int) == 1) {
					dst.concat("true");
				} else {
					dst.concat("false");
				}
				continue;
			}
		}
		dst.concat(*format);
	}
}

void Log::loop() {
	if (!_logServer)
		return;

	if (_logServer && !_pServer) {
		_pServer = new WiFiServer(8888);
		_pServerClient = new WiFiClient();
		_pServer->begin();
		_pServer->setNoDelay(true);
	}

	if (_pServer->hasClient()) {

		if (!_pServerClient || !_pServerClient->connected()) {
			if (_pServerClient)
				_pServerClient->stop();
			*_pServerClient = _pServer->available();
		}
		WiFiClient serverClient = _pServer->available();
		serverClient.stop();
	}
}

Log Logger;
