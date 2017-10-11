/*
 * Logger.cpp
 *
 * Copyright 2017 Christopher B. Liebman
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 *  Created on: May 31, 2017
 *      Author: liebman
 */

#include "Logger.h"

#ifndef __AVR_ATtinyX5__
#define DEBUG

#ifdef DEBUG

#undef dbprintf
#undef dbprint
#undef dbprintln
#undef dbflush

#define DBP_BUF_SIZE 256
extern unsigned int snprintf(char*, unsigned int, ...);
#define dbprintf(...)  {char dbp_buf[DBP_BUF_SIZE]; snprintf(dbp_buf, DBP_BUF_SIZE-1, __VA_ARGS__); Serial.print(dbp_buf);}
#define dbprintln(x)   Serial.println(x)
#define dbflush()      Serial.flush()
#endif

Logger::Logger()
{
	_host       = NULL;
	_port       = 0;
	_failed     = 0;
#ifdef ESP8266
	_client.setTimeout(1000); // 1 second connect timeout
#endif
}

void Logger::begin()
{
    begin(LOGGER_DEFAULT_BAUD);
}

void Logger::begin(long int baud)
{
    Serial.begin(baud);
}

void Logger::end()
{
#ifdef ESP8266
#ifdef USE_TCP
    _client.stop();
#endif
#endif
}

void Logger::setNetworkLogger(const char* host, uint16_t port)
{
    _host = host;
    _port = port;
}

void Logger::println(const char* message)
{
    snprintf(_buffer, LOGGER_BUFFER_SIZE-1, "%s\n", message);
    Serial.print(_buffer);
    Serial.flush();
#ifdef ESP8266
    send(_buffer);
#endif
}

extern int vsnprintf(char* buffer, size_t size, const char* fmt, va_list args);

void Logger::printf(const char* fmt, ...)
{
    va_list argp;
    va_start(argp, fmt);

    vsnprintf(_buffer, LOGGER_BUFFER_SIZE-1, fmt, argp);
    va_end(argp);

    Serial.print(_buffer);
    Serial.flush();
#ifdef ESP8266
    send(_buffer);
#endif
}

void Logger::flush()
{
    Serial.flush();
#ifdef ESP8266
    if (_client.connected())
    {
        _client.flush();
    }
#endif
}

#ifdef ESP8266
void Logger::send(const char* message)
{
    // if we are not configured for TCP then just return
    if (_host == NULL)
    {
        return;
    }

    IPAddress ip;

    if (!WiFi.isConnected())
    {
        dbprintln("Logger::log: not connected!");
        return;
    }

    if (!WiFi.hostByName(_host, ip))
    {
        dbprintf("Logger::log failed to resolve address for:%s\n", _host);
        return;
    }

#ifdef USE_TCP
    if (!_client.connected() && _failed < 3) // try 3 times at most to connect then give up.
    {
        if(!_client.connect(ip, _port) && _failed < 3)
        {
        	    _failed++;
            dbprintf("Logger::send failed to connect!\n");
            return;
        }
    }

    if (_client.write(message) != strlen(message))
    {
        dbprintf("Logger::write failed to write message: %s\n", message);
        return;
    }
    _client.flush();
#else
    _udp.beginPacket(ip, _port);
    _udp.write(message);
    if (!_udp.endPacket())
    {
        dbprintln("Logger::log failed to send packet!");
    }
#endif
}
#endif

Logger logger;
#endif
