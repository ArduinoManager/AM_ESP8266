/*
*
 * AMController libraries, example scketches (“The Software”) and the related documentation (“The Documentation”) are supplied to you 
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
 * Version: 1.0.0
 *
 * All rights reserved
 *
 */

#ifndef AM_ESP8266_H
#define AM_ESP8266

#define ALARMS_SUPPORT    		// uncomment to enable support for Alarm Widget
//#define DEBUG           		// uncomment to enable debugging - You should not need it !
#define SERIALDEBUG		false	// true to debug serial communication 

#ifdef _VARIANT_ARDUINO_ZERO_
#undef ALARMS_SUPPORT
#endif

#include <Arduino.h>

#ifdef ALARMS_SUPPORT

#define MAX_ALARMS  			5			// Maximum number of Alarms Widgets
#define ALARM_CHECK_INTERVAL	10			// [s]

typedef struct  {
	char 			id[12];  // First character of id is always A
	unsigned long 	time;
	bool			repeat;
} alarm;

#endif

#define VARIABLELEN 14
#define VALUELEN 14

class AMController {

private:
HardwareSerial	*_serial;
bool 			_isConnected;
String 		_response = "";
String 		_messages = "";
bool 			_isQueueEmpty = true;
bool			_canWrite;
uint8_t 	_currentStatus;


#ifdef ALARMS_SUPPORT
char						_timerId[11];
unsigned long		_timerTime;
String	 				_timeServerAddress;  // NTP Server Address
unsigned long   _lastAlarmCheck;
unsigned long		_startTime;
#endif

/**
Pointer to the function where to put code in place of loop()
**/
void (*_doWork)(void); 

/*
Pointer to the function where Switches, Knobs and Leds are syncronized
*/
void (*_doSync)();

/*
Pointer to the function where incoming messages are processed
*
* variable
*
* value
*
*/
void (*_processIncomingMessages)(char *variable, char *value);

/*
Pointer to the function where outgoing messages are processed
*
*/
void (*_processOutgoingMessages)(void);

#ifdef ALARMS_SUPPORT
/*
Pointer to the function where alerts are processed
*
*/
void (*_processAlarms)(char *alarm);
#endif

/*
Pointer to the function called when a device connects to Arduino
*
*/
void (*_deviceConnected)(void);

/*
Pointer to the function called when a device disconnects from Arduino
*
*/
void (*_deviceDisconnected)(void);

bool readVariable(String &data, String &variable, String &value);

#ifdef ALARMS_SUPPORT

unsigned long sendNTPpacket(String &address);
void breakTime(unsigned long time, int *seconds, int *minutes, int *hours, int *Wday, long *Year, int *Month, int *Day);

void syncTime();

void inizializeAlarms();
void checkAndFireAlarms();
void createUpdateAlarm(char *id, unsigned long time, bool repeat);
void removeAlarm(char *id);
	
#endif

String waitForData(String keyword, unsigned long timeout, bool debug, String tag);
String waitForData(String keyword1, String keyword2, unsigned long timeout, bool debug, String tag);
int receiveData(unsigned long timeout, bool debug, String tag);
void handlingIncomingMessages(String &response);
boolean extractReceivedMessages(String &data, String &msg);

public:

#ifdef ALARMS_SUPPORT
	AMController(
							HardwareSerial &s,
							void (*doWork)(void), 
							void (*doSync)(void), 
							void (*processIncomingMessages)(char *variable, char *value),
							void (*processOutgoingMessages)(void),
							void (*processAlarms)(char *alarm),
							void (*deviceConnected)(void),
							void (*deviceDisconnected)(void));
#endif

	AMController(
							HardwareSerial &s,
							void (*doWork)(void), 
							void (*doSync)(void), 
							void (*processIncomingMessages)(char *variable, char *value),
							void (*processOutgoingMessages)(void),
							void (*deviceConnected)(void),
							void (*deviceDisconnected)(void));
				
	bool begin(String ssid, String password, String address,byte port);							  		  
	void loop();
	void loop(unsigned long delay);
	void writeMessage(const char *variable, float value);
	void writeMessage(char *variable, float value);
	void writeTripleMessage(char *variable, float vX, float vY, float vZ);
	void writeTxtMessage(const char *variable, const char *value);
	void writeTxtMessage(char *variable, char *value);
		
	void log(const char *msg);	
	void log(char *msg);
	void log(int msg);

	void logLn(const char *msg);
	void logLn(char *msg);
	void logLn(int msg);
	void logLn(long msg);
	void logLn(unsigned long msg);

	void temporaryDigitalWrite(uint8_t pin, uint8_t value, unsigned long ms);	
	
#ifdef ALARMS_SUPPORT 	
	void setNTPServerAddress(String address);
	unsigned long now();
#ifdef DEBUG
	void dumpAlarms();
  	void printTime(unsigned long time);
#endif 
#endif 

#ifdef SDLOGGEDATAGRAPH_SUPPORT

	void sdLogLabels(char *variable, char *label1);
	void sdLogLabels(char *variable, char *label1, char *label2);
	void sdLogLabels(char *variable, char *label1, char *label2, char *label3);
	void sdLogLabels(char *variable, char *label1, char *label2, char *label3, char *label4);
	void sdLogLabels(char *variable, char *label1, char *label2, char *label3, char *label4, char *label5);

	void sdLog(char *variable, unsigned long time, float v1);
	void sdLog(char *variable, unsigned long time, float v1, float v2);
	void sdLog(char *variable, unsigned long time, float v1, float v2, float v3);
	void sdLog(char *variable, unsigned long time, float v1, float v2, float v3, float v4);
	void sdLog(char *variable, unsigned long time, float v1, float v2, float v3, float v4, float v5);
	
	void sdSendLogData(char *variable);
	
	void sdPurgeLogData(char *variable);
	
#endif	

};
#endif

