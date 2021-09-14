/*
   Test Arduino Manager for iPad / iPhone / Mac
   A simple test program to show the Arduino Manager features.
   Author: Fabrizio Boco - fabboco@gmail.com
   Version: 1.1
   09/14/2021
   All rights reserved
*/

/*
   AMController libraries, example sketches (The Software) and the related documentation (The Documentation) are supplied to you
   by the Author in consideration of your agreement to the following terms, and your use or installation of The Software and the use of The Documentation
   constitutes acceptance of these terms.
   If you do not agree with these terms, please do not use or install The Software.
   The Author grants you a personal, non-exclusive license, under authors copyrights in this original software, to use The Software.
   Except as expressly stated in this notice, no other rights or licenses, express or implied, are granted by the Author, including but not limited to any
   patent rights that may be infringed by your derivative works or by other works in which The Software may be incorporated.
   The Software and the Documentation are provided by the Author on an AS IS basis.  THE AUTHOR MAKES NO WARRANTIES, EXPRESS OR IMPLIED, INCLUDING WITHOUT
   LIMITATION THE IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, REGARDING THE SOFTWARE OR ITS USE AND OPERATION
   ALONE OR IN COMBINATION WITH YOUR PRODUCTS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL OR CONSEQUENTIAL DAMAGES (INCLUDING,
   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE,
   REPRODUCTION AND MODIFICATION OF THE SOFTWARE AND OR OF THE DOCUMENTATION, HOWEVER CAUSED AND WHETHER UNDER THEORY OF CONTRACT, TORT (INCLUDING NEGLIGENCE),
   STRICT LIABILITY OR OTHERWISE, EVEN IF THE AUTHOR HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "AM_ESP8266.h"

#define CONNECTIONLED   13
#define YELLOWLED       2
#define REDLED          10
#define TEMPERATUREPIN  14

float   temperature;
uint8_t ledBrightness;

/*

   Prototypes of AMController’s callbacks


*/
void doWork();
void doSync(char *variable);
void processIncomingMessages(char *variable, char *value);
void processOutgoingMessages();
void processAlarms(char *variable);
void deviceConnected();
void deviceDisconnected();


char ssid[] = "Your SSID";
char pass[] = "Your Password";


#ifdef ALARMS_SUPPORT
AMController amController(Serial1, &doWork, &doSync, &processIncomingMessages, &processOutgoingMessages, &processAlarms, &deviceConnected, &deviceDisconnected);
#else
AMController amController(Serial1, &doWork, &doSync, &processIncomingMessages, &processOutgoingMessages, &deviceConnected, &deviceDisconnected);
#endif

void setup() {

  Serial.begin(9600);
  Serial1.begin(115200); // your esp's baud rate might be different

  if (!amController.begin(ssid, pass, "192.168.1.235", 80)) {
    Serial.println("Setup failed");
    while (true)
      ;
  }

  pinMode(TEMPERATUREPIN, INPUT);
  pinMode(CONNECTIONLED, OUTPUT);
  pinMode(YELLOWLED, OUTPUT);
  pinMode(REDLED, OUTPUT);

  digitalWrite(CONNECTIONLED, LOW);
  digitalWrite(YELLOWLED, LOW);

  ledBrightness = 125;
  analogWrite(REDLED, ledBrightness);

  Serial.println();
  Serial.println("Setup completed");
  Serial.println("Listening for connection...");
}

void loop() {
  amController.loop(50);
}

/**


  This function is called periodically and its equivalent to the standard loop() function

*/
void doWork() {
  //Serial.println("doWork");

  temperature = (3.25 * analogRead(TEMPERATUREPIN) * 100.0) / 1024.;
}

/**
  This function is called when the ios device connects and needs to initialize the position of switches and knobs
*/
void doSync () {
  //Serial.print("doSync ");

  amController.writeTxtMessage("Msg", "Hello, I am connected to ESP8266");
  amController.writeMessage("S1", digitalRead(YELLOWLED));
  amController.writeMessage("Knob1", (int) map(ledBrightness, 0, 255, 0, 1023));
}

/**
  This function is called when a new message is received from the iOS device
*/
void processIncomingMessages(char *variable, char *value) {

  //Serial.println("processIncomingMessages");

  amController.logLn("processIncomingMessages");

  if (strcmp(variable, "S1") == 0)
    digitalWrite(YELLOWLED, atoi(value));

  if (strcmp(variable, "Knob1") == 0) {

    Serial.println(atoi(value));
    ledBrightness = atoi(value);
    analogWrite(REDLED, (float)map(ledBrightness, 0, 1023, 0, 255));
  }
}

/**
  This function is called periodically and messages can be sent to the iOS device
*/
void processOutgoingMessages() {

  //Serial.println("processOutgoingMessages");

  amController.writeMessage("T", temperature);
  amController.writeMessage("Led13", digitalRead(YELLOWLED));
}

void processAlarms(char *variable) {
  Serial.print("fired Alarm: "); Serial.println(variable);
}

/**
  This function is called when the iOS device connects
*/
void deviceConnected () {
  Serial.println("Device connected");
  digitalWrite(CONNECTIONLED, HIGH);
}

/**
  This function is called when the iOS device disconnects
*/
void deviceDisconnected () {
  Serial.println("Device disconnected");
  digitalWrite(CONNECTIONLED, LOW);
}


