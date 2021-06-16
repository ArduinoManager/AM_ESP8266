/*
 *
 * AMControlle libraries, samples sketches (“The Software”) and the related documentation (“The Documentation”) are supplied to you 
 * by the Author in consideration of your agreement to the following terms, and your use or installation of The Software and the use of The Documentation 
 * constitutes acceptance of these terms.  
 * If you do not agree with these terms, please do not use or install The Software.
 * The Author grants you a personal, non-exclusive license, under author's copyrights in this original software, to use The Software. 
 * Except as expressly stated in this notice, no other rights or licenses, express or implied, are granted by the Author, including but not limited to any 
 * patent rights that may be infringed by your derivative works or by other works in which The Software may be incorporated.
 * The Software and the Documentation are provided by the Author on an "AS IS" basis.  THE AUTHOR MAKES NO WARRANTIES, EXPRESS OR IMPLIED, INCLUDING WITHOUT 
 * LIMITATION THE IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, REGARDING THE SOFTWARE OR ITS USE AND OPERATION 
 * ALONE OR IN COMBINATION WITH YOUR PRODUCTS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE, 
 * REPRODUCTION AND MODIFICATION OF THE SOFTWARE AND OR OF THE DOCUMENTATION, HOWEVER CAUSED AND WHETHER UNDER THEORY OF CONTRACT, TORT (INCLUDING NEGLIGENCE), 
 * STRICT LIABILITY OR OTHERWISE, EVEN IF THE AUTHOR HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Author: Fabrizio Boco - fabboco@gmail.com
 *
 * All rights reserved
 *
 */

#include "AM_ESP8266.h"

#if defined(ALARMS_SUPPORT) && !defined(_VARIANT_ARDUINO_ZERO_)

#include <avr/eeprom.h>

#endif

#define TIMEOUT    4
#define CONNECT    5
#define CLOSED     6
#define LINKISNOT  7
#define DATA       8
#define PROMPT     9
#define SENDOK     10
#define SENDERROR  11
#define BUSY	   12

#if defined (_VARIANT_ARDUINO_ZERO_)
char *dtostrf (double val, signed char width, unsigned char prec, char *sout);
#endif 

#ifdef DEBUG
#define LEAP_YEAR(Y)     ( ((1970+Y)>0) && !((1970+Y)%4) && ( ((1970+Y)%100) || !((1970+Y)%400) ) )

static  const uint8_t monthDays[]={31,28,31,30,31,30,31,31,30,31,30,31}; // API starts months from 1, this array starts from 0
#endif 

#if defined(ALARMS_SUPPORT) && !defined(_VARIANT_ARDUINO_ZERO_)
AMController::AMController(
							HardwareSerial &s,
							void (*doWork)(void), 
							void (*doSync)(void),
							void (*processIncomingMessages)(char *variable, char *value),
							void (*processOutgoingMessages)(void),
							void (*processAlarms)(char *alarm),
							void (*deviceConnected)(void),
							void (*deviceDisconnected)(void)) : _serial(&s)
{
	_isConnected = false;
	_response = "";
	_messages = "";
	_isQueueEmpty = true;  
	_canWrite = false;  
	_doWork = doWork;
    _doSync = doSync;
    _processIncomingMessages = processIncomingMessages;
    _processOutgoingMessages = processOutgoingMessages;
    _processAlarms = processAlarms;
    _deviceConnected = deviceConnected;
    _deviceDisconnected = deviceDisconnected;
            
    _timeServerAddress = "81.94.123.17";   // New York City, NY NTP Server nist1-ny2.ustiming.org
    
    _startTime = 0;
    _lastAlarmCheck = 0;
    
	this->inizializeAlarms();
}
#endif 


AMController::AMController(
														HardwareSerial &s,
                            void (*doWork)(void), 
                            void (*doSync)(void),
                            void (*processIncomingMessages)(char *variable, char *value),
                            void (*processOutgoingMessages)(void),
                            void (*deviceConnected)(void),
							void (*deviceDisconnected)(void)) : _serial(&s)
{
	_isConnected = false;
	_response = "";
	_messages = "";
	_isQueueEmpty = true;    
    _doWork = doWork;
    _doSync = doSync;
    _processIncomingMessages = processIncomingMessages;
    _processOutgoingMessages = processOutgoingMessages;
    _deviceConnected = deviceConnected;
    _deviceDisconnected = deviceDisconnected;
    
#if defined(ALARMS_SUPPORT) && !defined(_VARIANT_ARDUINO_ZERO_)   
    _startTime = 0;
    _lastAlarmCheck = 0;

    _timeServerAddress = "81.94.123.17";   // New York City, NY NTP Server nist1-ny2.ustiming.org
#endif         
}   

bool AMController::begin(String ssid, String password, String address, byte port) {

//	String response="";

	Serial.println("Setting up...");

	_serial->println("AT+RST");
		
	_response = waitForData("ready",20000,SERIALDEBUG,"Init");
	if (_response.length()==0 || _response.indexOf("ERROR") != -1) {
    	Serial.println("Device disconnected?");
    	return false;
  	}
  	
	_response = "";

	_serial->println("ATE0");
	_response = waitForData("OK","ERROR",10000,SERIALDEBUG,"Init");
	if (_response.indexOf("ERROR") != -1) {
		Serial.println("Error setting ECHO");
    	return false;
	}
	
	_response = "";
	_serial->println("AT+CWMODE=3");
	_response = waitForData("OK","ERROR",10000,SERIALDEBUG,"Init");
	if (_response.indexOf("ERROR") != -1) {
		Serial.println("Error setting MODE");
    	return false;
	}
		
	_response = "";		
	_serial->println("AT+CWDHCP=2,1");
	waitForData("OK",10000,SERIALDEBUG,"Init");
	
	while(_serial->available()>0)
		_serial->read();
		
	_serial->print("AT+CWJAP=\"");
	_serial->print(ssid);
	_serial->print("\",\"");
	_serial->print(password);
	_serial->println("\"");
	_response = waitForData("OK", "FAIL", 20000, SERIALDEBUG,"Init");
  	if (_response.indexOf("FAIL") != -1) {
    	Serial.println("Error connecting to network");
    	return false;
  	}

	_response = "";
	_serial->print("AT+CIPSTA=\"");
	_serial->print(address);
	_serial->println("\"");
	_response = waitForData("OK", "ERROR", 20000, SERIALDEBUG,"Init");
  	if (_response.indexOf("ERROR") != -1) {
  		Serial.println("Error setting IP address");
		return false;
	}

#ifndef SOFTWARESERIAL
	_serial->println("AT+CIFSR");
	_response = waitForData("OK", "ERROR", 20000, SERIALDEBUG,"Init");
  	if (_response.indexOf("ERROR") != -1) {
  		Serial.println("Error checking IP address");
		return false;
	}

	Serial.println("-------------");
	Serial.println(_response);
	Serial.println("-------------");
#endif

	_serial->println("AT+CIPMUX=1");
	_response = waitForData("OK", "FAIL", 20000, SERIALDEBUG,"Init");
  	if (_response.indexOf("FAIL") != -1) {
		Serial.println("Error setting multiple connection");
		return false;
	}

	_serial->print("AT+CIPSERVER=1,");
	_serial->println(port);
	_response = waitForData("OK", "FAIL", 20000, SERIALDEBUG,"Init");
  	if (_response.indexOf("FAIL") != -1) {
		Serial.println("Error starting TCP Server");
		return false;
	}

#if defined(ALARMS_SUPPORT) && !defined(_VARIANT_ARDUINO_ZERO_)
	syncTime();
#endif

	_response = "";
	return true;
}
                          

void AMController::loop() {
	this->loop(25);
}

void AMController::loop(unsigned long _delay) {
  	
#if defined(ALARMS_SUPPORT) && !defined(_VARIANT_ARDUINO_ZERO_)
  	
 	if (_processAlarms != NULL) {
 	
		unsigned long now = _startTime + millis()/1000;

		if ( (now - _lastAlarmCheck) > ALARM_CHECK_INTERVAL) {
		
 			_lastAlarmCheck = now;
			checkAndFireAlarms();
		}
    }
    
#endif  	
  	
 	_doWork();
 	
	if (_isQueueEmpty)
		_currentStatus = this->receiveData(500, SERIALDEBUG,"Main");
		
	_isQueueEmpty = true;

	if (!_isConnected && _currentStatus == CONNECT) {
		
		_response = "";
		_messages = "";
		
		// Reads available data immediately after connection
		//_currentStatus = this->receiveData(10000, SERIALDEBUG,F("After connection"));
		
		_response = waitForData("Sync=Stop#", 20000, SERIALDEBUG,F("After connection"));
  		if (_response.indexOf("Sync=Stop#") == -1) {
			Serial.println("Error connecting");
			return;
		}
		
#ifdef DEBUGSERIAL		
		Serial.println("---------------");
		Serial.println("Data after connection");
		Serial.println(_response);
#endif
		
		int idx = _response.lastIndexOf("+IPD,");
    	if (idx != -1) {

			int commaIdx =  _response.indexOf(",",idx+5);
		
			if (commaIdx != -1) {
			
				int colonIdx =  _response.indexOf(":",commaIdx); 
			
				if (colonIdx != -1) {
				
					int len = _response.substring(commaIdx+1,colonIdx).toInt();				
				
					if ((unsigned int)colonIdx+1+len<=_response.length()) {
						_messages += _response.substring(idx, colonIdx+1+len);	
						_currentStatus = DATA;			        
        			}
        		
				}
			}
		}
		
		_isConnected = true;
		if (_deviceConnected != NULL)
			_deviceConnected();
	}

	if (_isConnected && (_currentStatus == CLOSED || _currentStatus == LINKISNOT)) {
	
		if (_deviceDisconnected != NULL)
			_deviceDisconnected();
		_isConnected = false;
		_canWrite = false;
			
		delay(1500);
		
		return;
	}
    
    if (!_isConnected)
    	return;
    	
	if (_currentStatus == TIMEOUT)
    	_response = "";

  	if (_currentStatus == DATA) {

#if SERIALDEBUG
    	Serial.println("DATA RECEIVED");
#endif    	
    	//Serial.println("DATA AVAILABLE");
	    handlingIncomingMessages(_messages);
    	_response = "";
	    _messages = "";
  	}
  	
  	if (!_canWrite)
  		return;
  	
  	this->_processOutgoingMessages();	
  	delay(_delay);
}

bool AMController::readVariable(String &data, String &variable, String &value) {

  unsigned int index_eq = -1;    /* = */
  unsigned int index_pound = -1; /* # */

  variable = "";
  value = "";

  //Serial.print("Working on Msg >>>"); Serial.print(data); Serial.println("<<<");

  index_eq = data.indexOf("=");
  index_pound = data.indexOf("#");

  //Serial.print("Idx = "); Serial.println(index_eq);
  //Serial.print("Idx # "); Serial.println(index_pound);

  variable = data.substring(0, index_eq);
  value = data.substring(index_eq + 1, index_pound);

  //Serial.print("Var: "); Serial.println(variable);
  //Serial.print("Val: "); Serial.println(value);

  if (index_pound < data.length() - 1) {
    data = data.substring(index_pound + 1);
    return true;
  }

  return false;
}

void AMController::writeMessage(const char *variable, int value) {
ar buffer[VARIABLELEN+VALUELEN+3];
    char vbuffer[VALUELEN];
            
    dtostrf(value, 0, 0, vbuffer);    
    snprintf(buffer,VARIABLELEN+VALUELEN+3, "%s=%s#", variable, vbuffer); 
        
    _serial->print("AT+CIPSEND=");
  	_serial->print(0);  // mux !!!!
  	_serial->print(",");
  	_serial->println(strlen(buffer));

  	_currentStatus = receiveData(500, SERIALDEBUG,"In write 0");

  	if (_currentStatus == PROMPT) {
    	_serial->print(buffer);
  	}
  	
  	if (_currentStatus == BUSY) {
  		delay(5);
    	return;
  	}

  	if (_currentStatus == DATA) {
		
#if SERIALDEBUG
    	Serial.println("DATA RECEIVED");
#endif    			
		handlingIncomingMessages(_messages);
		_response = "";
		_messages = "";
  	}

	if (_currentStatus == LINKISNOT || _currentStatus == CLOSED) {
		if (_deviceDisconnected != NULL)
			_deviceDisconnected();
		_isConnected = false;
		_canWrite = false;
				
		delay(1500);
		
		return;
  	}
  	
  	if (_currentStatus == BUSY) {
  		delay(5);
    	return;
  	}
  	
  	_currentStatus = receiveData(1000, SERIALDEBUG,"In write 1");

  	if (_currentStatus == SENDERROR) {
    	Serial.println("SEND ERROR");
    	Serial.println(buffer);
  	}

  	if (_currentStatus == DATA) {
#if SERIALDEBUG
    	Serial.println("DATA RECEIVED");
#endif    	  	
		//    _isQueueEmpty = false;
		handlingIncomingMessages(_messages);
		_response = "";
		_messages = "";
  	}

  	if (_currentStatus == LINKISNOT || _currentStatus == CLOSED) {
		if (_deviceDisconnected != NULL)
			_deviceDisconnected();
		_isConnected = false;
		_canWrite = false;
		
		delay(1500);
  	}
  	
  	if (_currentStatus == BUSY) {
  		delay(5);
    	return;
  	}
}

void AMController::writeMessage(const char *variable, float value) {

    char buffer[VARIABLELEN+VALUELEN+3];
    char vbuffer[VALUELEN];
            
    dtostrf(value, 0, 3, vbuffer);    
    snprintf(buffer,VARIABLELEN+VALUELEN+3, "%s=%s#", variable, vbuffer); 
        
    _serial->print("AT+CIPSEND=");
  	_serial->print(0);  // mux !!!!
  	_serial->print(",");
  	_serial->println(strlen(buffer));

  	_currentStatus = receiveData(500, SERIALDEBUG,"In write 0");

  	if (_currentStatus == PROMPT) {
    	_serial->print(buffer);
  	}
  	
  	if (_currentStatus == BUSY) {
  		delay(5);
    	return;
  	}

  	if (_currentStatus == DATA) {
		
#if SERIALDEBUG
    	Serial.println("DATA RECEIVED");
#endif    			
		handlingIncomingMessages(_messages);
		_response = "";
		_messages = "";
  	}

	if (_currentStatus == LINKISNOT || _currentStatus == CLOSED) {
		if (_deviceDisconnected != NULL)
			_deviceDisconnected();
		_isConnected = false;
		_canWrite = false;
				
		delay(1500);
		
		return;
  	}
  	
  	if (_currentStatus == BUSY) {
  		delay(5);
    	return;
  	}
  	
  	_currentStatus = receiveData(1000, SERIALDEBUG,"In write 1");

  	if (_currentStatus == SENDERROR) {
    	Serial.println("SEND ERROR");
    	Serial.println(buffer);
  	}

  	if (_currentStatus == DATA) {
#if SERIALDEBUG
    	Serial.println("DATA RECEIVED");
#endif    	  	
		//    _isQueueEmpty = false;
		handlingIncomingMessages(_messages);
		_response = "";
		_messages = "";
  	}

  	if (_currentStatus == LINKISNOT || _currentStatus == CLOSED) {
		if (_deviceDisconnected != NULL)
			_deviceDisconnected();
		_isConnected = false;
		_canWrite = false;
		
		delay(1500);
  	}
  	
  	if (_currentStatus == BUSY) {
  		delay(5);
    	return;
  	}
}

void AMController::writeTripleMessage(char *variable, float vX, float vY, float vZ) {

    char buffer[VARIABLELEN+VALUELEN+3];
    char vbufferAx[VALUELEN];
    char vbufferAy[VALUELEN];
    char vbufferAz[VALUELEN];
    
    dtostrf(vX, 0, 2, vbufferAx); 
    dtostrf(vY, 0, 2, vbufferAy); 
    dtostrf(vZ, 0, 2, vbufferAz);    
    snprintf(buffer,VARIABLELEN+VALUELEN+3, "%s=%s:%s:%s#", variable, vbufferAx,vbufferAy,vbufferAz); 
        
    //_client.write((const uint8_t *)buffer, strlen(buffer)*sizeof(char));    
}

void AMController::writeTxtMessage(const char *variable, const char *value) {
	
	writeTxtMessage((char *)variable, (char *)value);
}

void AMController::writeTxtMessage(char *variable, char *value) {
    char buffer[128];    
        
    snprintf(buffer,128, "%s=%s#", variable, value);
    
    _serial->print("AT+CIPSEND=");
  	_serial->print(0);  // mux !!!!
  	_serial->print(",");
  	_serial->println(strlen(buffer));

  	_currentStatus = receiveData(500, SERIALDEBUG,"In write txt 0");

  	if (_currentStatus == PROMPT) {
    	_serial->println(buffer);
  	}

  	if (_currentStatus == DATA) {
		
#if SERIALDEBUG
    	Serial.println("DATA RECEIVED");
#endif    	
		handlingIncomingMessages(_messages);
		_response = "";
		_messages = "";
  	}
  	
  	if (_currentStatus == LINKISNOT || _currentStatus == CLOSED) {
		if (_deviceDisconnected != NULL)
			_deviceDisconnected();
		_isConnected = false;
		_canWrite = false;
		
		delay(1500);
		
		return;
  	}

  	_currentStatus = this->receiveData(1000, SERIALDEBUG,"In write txt 2");

  	if (_currentStatus == SENDERROR) {
    	Serial.println("SEND ERROR");
    	Serial.println(buffer);
    	
    	delay(500);
    	return;
  	}

  	if (_currentStatus == DATA) {

#if SERIALDEBUG
    	Serial.println("DATA RECEIVED");
#endif    	
		//    _isQueueEmpty = false;
		handlingIncomingMessages(_messages);
		_response = "";
		_messages = "";
  	}

  	if (_currentStatus == LINKISNOT || _currentStatus == CLOSED) {
		if (_deviceDisconnected != NULL)
			_deviceDisconnected();
		_isConnected = false;
		_canWrite = false;
		
		delay(1500);
  	}
}

void AMController::log(const char *msg) {

	this->writeTxtMessage((char *)"$D$",msg);
}

void AMController::log(char *msg) {

	this->writeTxtMessage((char *)"$D$",msg);
}

void AMController::log(int msg)
{
	char buffer[11];
	itoa(msg, buffer, 10);
	
	this->writeTxtMessage((char *)"$D$",buffer);
}

void AMController::logLn(const char *msg) {

	this->writeTxtMessage((char *)"$DLN$",msg);
}

void AMController::logLn(char *msg) {

	this->writeTxtMessage((char *)"$DLN$",msg);
}

void AMController::logLn(int msg)
{
	char buffer[11];
	itoa(msg, buffer, 10);
	
	this->writeTxtMessage((char *)"$DLN$",buffer);
}

void AMController::logLn(long msg)
{
	char buffer[11];
	ltoa(msg, buffer, 10);
	
	this->writeTxtMessage((char *)"$DLN$",buffer);
}

void AMController::logLn(unsigned long msg) {

	char buffer[11];
	ltoa(msg, buffer, 10);
	
	this->writeTxtMessage((char *)"$DLN$",buffer);
}

void AMController::temporaryDigitalWrite(uint8_t pin, uint8_t value, unsigned long ms) {

	boolean previousValue = digitalRead(pin);

    digitalWrite(pin, value);
    delay(ms);
    digitalWrite(pin, previousValue);
}


// Time Management 

#if defined(ALARMS_SUPPORT) && !defined(_VARIANT_ARDUINO_ZERO_)

void AMController::setNTPServerAddress(String address) {

	_timeServerAddress = address;
}

void AMController::syncTime() {

	byte 	packetBuffer[48];

  	Serial.print("NTP Request Sent to address ");
  	Serial.println(_timeServerAddress);

  	// set all bytes in the buffer to 0
  	memset(packetBuffer, 0, 48); 
  	// Initialize values needed to form NTP request
  	// (see URL above for details on the packets)
  	packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  	packetBuffer[1] = 0;     // Stratum, or type of clock
  	packetBuffer[2] = 6;     // Polling Interval
  	packetBuffer[3] = 0xEC;  // Peer Clock Precision
  	// 8 bytes of zero for Root Delay & Root Dispersion
  	packetBuffer[12]  = 49; 
  	packetBuffer[13]  = 0x4E;
  	packetBuffer[14]  = 49;
  	packetBuffer[15]  = 52;
	
  	// all NTP fields have been given values, now
  	// you can send a packet requesting a timestamp: 		   
  
  	//_serial->println("AT+CIPSTART=1,\"UDP\",\"81.94.123.17\",123");
  	
  	_serial->print("AT+CIPSTART=1,\"UDP\",\"");
  	_serial->print(_timeServerAddress);
  	_serial->println("\",123");
  	
  	String resp="";
  	unsigned long startTime = millis();
  
	while(resp.indexOf("CONNECT")==-1 && startTime+millis()<25000) {
  		if (_serial->available() > 0) {
      		char c = _serial->read();
      		resp += c;
#if SERIALDEBUG      		
      		Serial.write(c);    
#endif       		  
    	}
   	}
  	if(resp.indexOf("CONNECT")==-1) {
  		Serial.println(F("Error connecting to NTP Server"));
  		return ;
  	}
  
  	_serial->println("AT+CIPSEND=1,48");
  
	startTime = millis();
  	while(resp.indexOf(">")==-1 && startTime+millis()<25000) {
  		if (_serial->available() > 0) {
      		char c = _serial->read();
      		resp += c;
#if SERIALDEBUG      		
      		Serial.write(c);      
#endif      		
    	}
   	}
  	if(resp.indexOf(">")==-1) {
  		Serial.println(F("Error connecting to NTP Server"));
  		_serial->println("AT+CIPCLOSE=1");
  	}
  	
  	_serial->write(packetBuffer, 48);
  	
  	startTime = millis();  	
  	while(resp.indexOf("+IPD,1,48:")==-1 && startTime+millis()<25000) {
  		if (_serial->available() > 0) {
      		char c = _serial->read();
      		resp += c;
#if SERIALDEBUG      		
      		Serial.write(c);      
#endif      		
    	}
   	}
  	if(resp.indexOf("+IPD,1,48:")==-1) {
  		Serial.println(F("No data received - Time not set!"));
  		_serial->println("AT+CIPCLOSE=1");
  		
  		return;
  	}
  	
  	int pos = 0;
  	memset(packetBuffer, 0, 48);

  	while ((pos < 48) && (startTime+millis()<25000)) {
    	if (_serial->available() > 0)
      	packetBuffer[pos++] = _serial->read();
  	}
	if (startTime+millis()>25000) {
  		Serial.println(F("Not enough data received - Time not set!"));
  		_serial->println("AT+CIPCLOSE=1");
  		
  		return;
  	}
  	
  	_serial->println("AT+CIPCLOSE=1");
  	
  	unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
  	unsigned long lowWord  = word(packetBuffer[42], packetBuffer[43]);
	// combine the four bytes (two words) into a long integer
  	// this is NTP time (seconds since Jan 1 1900):
  	unsigned long secsSince1900 = highWord << 16 | lowWord;
  	// Serial.print("Seconds since Jan 1 1900 = " );
  	// Serial.println(secsSince1900);

  	// now convert NTP time into everyday time:
  	// Serial.print("Unix time = ");
  	// Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
  	const unsigned long seventyYears = 2208988800UL;
  	// subtract seventy years:  	
  	_startTime = secsSince1900 - seventyYears;  
    	
    // subtract current millis to synch with time in Arduino
    _startTime -= millis()/1000;
    	
#ifdef DEBUG    	
  	Serial.println("-- Current Time: ");
  	this->printTime(_startTime);
  	Serial.println();
#endif  
	Serial.println("Time set");	
}

unsigned long AMController::now() {
	
	unsigned long now = _startTime + millis()/1000;
	
	return now;
}

#ifdef DEBUG


void AMController::breakTime(unsigned long time, int *seconds, int *minutes, int *hours, int *Wday, long *Year, int *Month, int *Day) {
  // break the given time_t into time components
  // this is a more compact version of the C library localtime function
  // note that year is offset from 1970 !!!

  uint16_t year;
  uint8_t month, monthLength;
  unsigned long days;

  *seconds = time % 60;
  time /= 60; // now it is minutes
  *minutes = time % 60;
  time /= 60; // now it is hours
  *hours = time % 24;
  time /= 24; // now it is days
  *Wday = ((time + 4) % 7) + 1;  // Sunday is day 1 

  year = 0;  
  days = 0;
  while((unsigned)(days += (LEAP_YEAR(year) ? 366 : 365)) <= time) {
    year++;
  }
  *Year = year+1970; // year is offset from 1970 

  days -= LEAP_YEAR(year) ? 366 : 365;
  time -= days; // now it is days in this year, starting at 0

  days=0;
  month=0;
  monthLength=0;
  for (month=0; month<12; month++) {
    if (month==1) { // february
      if (LEAP_YEAR(year)) {
        monthLength=29;
      } 
      else {
        monthLength=28;
      }
    } 
    else {
      monthLength = monthDays[month];
    }

    if (time >= monthLength) {
      time -= monthLength;
    } 
    else {
      break;
    }
  }
  *Month = month + 1;  // jan is month 1  
  *Day = time + 1;     // day of month
}



void AMController::printTime(unsigned long time) {

    int seconds;
   	int minutes;
   	int hours;
   	int Wday;
   	long Year;
   	int Month;
   	int Day;
		
	this->breakTime(time, &seconds, &minutes, &hours, &Wday, &Year, &Month, &Day);

	Serial.print(Day);
	Serial.print("/");
	Serial.print(Month);
	Serial.print("/");
	Serial.print(Year);
	Serial.print(" ");
	Serial.print(hours);
	Serial.print(":");
	Serial.print(minutes);
	Serial.print(":");
	Serial.print(seconds);
}
#endif

#if defined(ALARMS_SUPPORT) && !defined(_VARIANT_ARDUINO_ZERO_)

void AMController::createUpdateAlarm(char *id, unsigned long time, bool repeat) {

	char lid[12];
	
	lid[0] = 'A';
	strcpy(&lid[1],id);

	// Update

	for(int i=0; i<MAX_ALARMS; i++) {
		
		alarm a;
		
		eeprom_read_block((void*)&a, (void*)(i*sizeof(a)), sizeof(a));
		
		if (strcmp(a.id,lid) == 0) {
				a.time = time;
				a.repeat = repeat;
				
				eeprom_write_block((const void*)&a, (void*)(i*sizeof(a)), sizeof(a));
				
				return;
		}		
	}

	// Create

	for(int i=0; i<MAX_ALARMS; i++) {
	
		alarm a;
		
		eeprom_read_block((void*)&a, (void*)(i*sizeof(a)), sizeof(a));
	
		if(a.id[1]=='\0') {
		
			strcpy(a.id,lid);
			a.time = time;
			a.repeat = repeat;
		
			eeprom_write_block((const void*)&a, (void*)(i*sizeof(a)), sizeof(a));
			
			return;
		}
	}
}

void AMController::removeAlarm(char *id) {

	char lid[12];
	
	lid[0] = 'A';
	strcpy(&lid[1],id);

	for(int i=0; i<5; i++) {
	
		alarm a;
		
		eeprom_read_block((void*)&a, (void*)(i*sizeof(a)), sizeof(a));
	
		if(strcmp(a.id,lid) == 0) {
		
			a.id[1]='\0';
			a.time = 0;
	        a.repeat = 0;
			
			eeprom_write_block((const void*)&a, (void*)(i*sizeof(a)), sizeof(a));
		}
	}
}

void AMController::inizializeAlarms() {

	for(int i=0; i<MAX_ALARMS; i++) {
	
		alarm a;
		
		eeprom_read_block((void*)&a, (void*)(i*sizeof(a)), sizeof(a));
	
		if(a.id[0] != 'A') {
		
			a.id[0]='A';
			a.id[1]='\0';
			a.time=0;
			a.repeat = 0;
			
			eeprom_write_block((const void*)&a, (void*)(i*sizeof(a)), sizeof(a));
		}
	}
}

#ifdef DEBUG
void AMController::dumpAlarms() {

	Serial.println("\t----Dump Alarms -----"); 
	
	for(int i=0;i<MAX_ALARMS; i++) {

		alarm al;
					
		eeprom_read_block((void*)&al, (void*)(i*sizeof(al)), sizeof(al));

		Serial.print("\t");
    	Serial.print(al.id); 
    	Serial.print(" "); 
    	this->printTime(al.time);
    	Serial.print(" ");
    	Serial.println(al.repeat);
	}
}
#endif

void AMController::checkAndFireAlarms() {

	unsigned long now = _startTime + millis()/1000;
	
#ifdef DEBUG
	Serial.print("checkAndFireAlarms ");
	this->printTime(now);
	Serial.println();
	this->dumpAlarms();
#endif

	for(int i=0; i<MAX_ALARMS; i++) {

		alarm a;

		eeprom_read_block((void*)&a, (void*)(i*sizeof(a)), sizeof(a));

		if(a.id[1]!='\0' && a.time<now) {

#ifdef DEBUG
			Serial.println(a.id);
#endif				
			// First character of id is A and has to be removed
			_processAlarms(&a.id[1]);

			if(a.repeat) {
		
				a.time += 86400; // Scheduled again tomorrow
				
#ifdef DEBUG
				Serial.print("Alarm rescheduled at ");
				this->printTime(a.time);
				Serial.println();
#endif	            	
			}
			else {
				//     Alarm removed
			
				a.id[1]='\0';
				a.time = 0;
				a.repeat = 0;
			}

			eeprom_write_block((const void*)&a, (void*)(i*sizeof(a)), sizeof(a));
#ifdef DEBUG
			this->dumpAlarms();
#endif        		 
		}
	}
}

#endif
#endif

String AMController::waitForData(String keyword, unsigned long timeout, bool debug, String tag) {

  String data;
  char a;
  unsigned long start = millis();

  while (millis() - start < timeout) {
  
    while (_serial->available() > 0) {

      a = _serial->read();
      if (a == '\0')
        continue;

      data += a;
    }

    if (data.indexOf(keyword) != -1) {
    
      if (debug) {
      	Serial.print(tag);
        Serial.print(" waitForData >>>>");
        Serial.print(data);
        Serial.println("<<<<");
      }

      return data;
    }
  }
  
  if (debug) {
	Serial.print(tag);
  	Serial.println("----Time Out!----");
    Serial.print("waitForData >>>>");
    Serial.print(data);
    Serial.println("<<<<");
  }
  return data;
}

String AMController::waitForData(String keyword1, String keyword2, unsigned long timeout, bool debug, String tag) {

  String data;
  char a;
  unsigned long start = millis();

  while (millis() - start < timeout) {

    while (_serial->available() > 0) {

      a = _serial->read();
      if (a == '\0')
        continue;
      data += a;
    }
    if (data.indexOf(keyword1) != -1 || data.indexOf(keyword2) != -1) {
      if (debug) {
      	Serial.print(tag);
        Serial.print(" waitForData >>>>");
        Serial.print(data);
        Serial.println("<<<<");
      }
      return data;
    }
  }
  
  if (debug) {
	Serial.println("----Time Out!----");
	Serial.print(tag);
    Serial.print(" waitForData >>>>");
    Serial.print(data);
    Serial.println("<<<<");
  }
  return data;
}	

int AMController::receiveData(unsigned long timeout, bool debug, String tag) {
	char a;
  	unsigned long start = millis();
  	int idx;

	_response = "";

  	while (millis() - start < timeout) {

    	while (_serial->available() > 0) {

      		a = _serial->read();
      		if (a == '\0')
        		continue;
      		_response += a;
    	}

    	if (debug && _response != "" && _response != "\r\n" && _response != "\n" && _response != " ") {
    		Serial.print(tag);
      		Serial.print(" receivedData >>>>");
      		Serial.print(_response);
      		Serial.println("<<<<");
    	}

		if (_response.indexOf("CONNECT FA") != -1 || _response.indexOf("CONNECT FAIL") != -1) {
    	
//     		Serial.println("LINK FAILED AND STOPPED");
//     		while(1)
//     			;
	       	return CLOSED;
    	}
    	
    	if (_response.indexOf("CONNECT") != -1) {
      		return CONNECT;
    	}

		if (_response.indexOf("CLOSED") != -1) {
			return CLOSED;
		}

		if (_response.indexOf("busy") != -1 || _response.indexOf("s...") != -1) {
		  	return BUSY;
		}

	    if (_response.indexOf("link is not") != -1) {
      		return LINKISNOT;
    	}
    	
    	idx = _response.indexOf("+IPD,");
    	if (idx != -1) {

			int commaIdx =  _response.indexOf(",",idx+5);
		
			if (commaIdx != -1) {
			
				int colonIdx =  _response.indexOf(":",commaIdx); 
			
				if (colonIdx != -1) {
				
					int len = _response.substring(commaIdx+1,colonIdx).toInt();				
//					Serial.print("Len ");Serial.println(len);
				
					if ((unsigned int)colonIdx+1+len<=_response.length()) {
						_messages += _response.substring(idx, colonIdx+1+len);
				
// 						Serial.print("msg ");Serial.println(_messages);
//         				Serial.println("DATA");
        
        				return DATA;
        		}
			}
			
		}
      
    }

    if (_response.indexOf(">") != -1) {
      	return PROMPT;
    }

    if (_response.indexOf("SEND OK") != -1) {
      	return SENDOK;
    }

    if (_response.indexOf("ERROR") != -1) {
    
    	Serial.println("-- SEND ERROR ---");
    	Serial.print(">>");
    	Serial.print(_response);
    	Serial.println("<<");
    
      	return SENDERROR;
    }
  }

  if (debug && _response != "" && _response != "\r\n" && _response != "\n" && _response != " ") {
  
  	Serial.println("----Time Out!----");
  	Serial.print(tag);
    Serial.print(" receivedData >>>>");
    Serial.print(_response);
    Serial.println("<<<<");
  }

  return TIMEOUT;
}

void AMController::handlingIncomingMessages(String &response) {
  String messages;
  boolean otherIPDS = true;

  while (otherIPDS) {
  
    otherIPDS = extractReceivedMessages(response, messages);
    
#ifdef DEBUG    
    Serial.print(">>>"); Serial.print(messages); Serial.println("<<<");
#endif

    boolean moreMessages = true;
    String variable;
    String value;

    while (moreMessages) {

      	moreMessages = readVariable(messages, variable, value);
      
#ifdef DEBUG   
      	Serial.print("Variable "); Serial.println(variable);
      	Serial.print("Value    "); Serial.println(value);
#endif 

      	if (variable == "Sync") {
        	_doSync();
        	_canWrite = true;
      	}
      	else {

      		_processIncomingMessages((char *)variable.c_str(),(char *)value.c_str());
      	}
    }
  }
}

boolean AMController::extractReceivedMessages(String &data, String &msg) {

  int32_t len = -1;
  int8_t id = -1;
  int32_t index_PIPDcomma = -1;
  int32_t index_colon = -1; 
  int32_t index_comma = -1; 

  index_PIPDcomma = data.indexOf("+IPD,");

  if (index_PIPDcomma != -1) {

    index_colon = data.indexOf(':', index_PIPDcomma + 5);

    if (index_colon != -1) {

      index_comma = data.indexOf(',', index_PIPDcomma + 5);
      
      if (index_comma != -1 && index_comma < index_colon) {
        id = data.substring(index_PIPDcomma + 5, index_comma).toInt();
        if (id < 0 || id > 4) {
          return false;
        }
        len = data.substring(index_comma + 1, index_colon).toInt();
        if (len <= 0) {
          return false;
        }

      }
      else {

        len = data.substring(index_PIPDcomma + 5, index_colon).toInt();
        if (len <= 0) {
          return false;
        }

      }
    }

//     Serial.println(index_colon);
//     Serial.println(len);

    msg = data.substring(index_colon + 1, index_colon + 1 + len);
  }

  index_PIPDcomma = data.indexOf("+IPD,", index_colon + len);

  if (index_PIPDcomma != -1) {
    data = data.substring(index_colon + len);
    return true;
  }

  return false;
}

#if defined (_VARIANT_ARDUINO_ZERO_)

char *dtostrf (double val, signed char width, unsigned char prec, char *sout) {

  uint32_t iPart = (uint32_t)val;
  sprintf(sout, "%d", iPart);
    
  if (prec > 0) {
    uint8_t pos = strlen(sout);
    sout[pos++] = '.';
    uint32_t dPart = (uint32_t)((val - (double)iPart) * pow(10, prec));

    for (uint8_t i = (prec - 1); i > 0; i--) {
      size_t pow10 = pow(10, i);
      if (dPart < pow10) {
        sout[pos++] = '0';
      }
      else {
        sout[pos++] = '0' + dPart / pow10;
        dPart = dPart % pow10;
      }
    }

    sout[pos++] = '0' + dPart;
    sout[pos] = '\0';
  }
  
  return sout;
}

#endif