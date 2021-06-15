#include "AM_ESP8266.h"

#define CONNECTIONLED   13
#define YELLOWLED       2
#define REDLED          10
#define TEMPERATUREPIN  14

float   temperature;
uint8_t ledBrightness;

/*
 *
 * Prototypes of AMController’s callbacks
 *
 *
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
  AMController amController(Serial1, &doWork,&doSync,&processIncomingMessages,&processOutgoingMessages,&processAlarms, &deviceConnected,&deviceDisconnected);
#else
  AMController amController(Serial1, &doWork,&doSync,&processIncomingMessages,&processOutgoingMessages, &deviceConnected,&deviceDisconnected);
#endif

void setup() {
  
  Serial.begin(9600);
  Serial1.begin(115200); // your esp's baud rate might be different

  if (!amController.begin(ssid,pass, "192.168.1.235",80)) {
    Serial.println("Setup failed");
    while(true)
      ;
  }

  pinMode(TEMPERATUREPIN, INPUT);
  pinMode(CONNECTIONLED, OUTPUT);
  pinMode(YELLOWLED, OUTPUT);
  pinMode(REDLED,OUTPUT);

  digitalWrite(CONNECTIONLED, LOW);
  digitalWrite(YELLOWLED, LOW);
  
  ledBrightness = 125;
  analogWrite(REDLED,ledBrightness);  
  
  Serial.println();
  Serial.println("Setup completed");
  Serial.println("Listening for connection...");  
}

void loop() {
  amController.loop(50);
}

/**
*
*
* This function is called periodically and its equivalent to the standard loop() function
*
*/
void doWork() {
  //Serial.println("doWork");
  
  temperature = (3.25 * analogRead(TEMPERATUREPIN) * 100.0) / 1024.;
}

/**
*
*
* This function is called when the ios device connects and needs to initialize the position of switches and knobs
*
*/
void doSync () {
  //Serial.print("doSync "); 
  
  amController.writeTxtMessage("Msg","Hello, I am connected to ESP8266");
  amController.writeMessage("S1",digitalRead(YELLOWLED));      
  amController.writeMessage("Knob1",map(ledBrightness,0,255,0,1023));  
}

/**
*
*
* This function is called when a new message is received from the iOS device
*
*/
void processIncomingMessages(char *variable, char *value) {
  
  //Serial.println("processIncomingMessages");
  
  amController.logLn("processIncomingMessages");
  
  if (strcmp(variable,"S1") == 0)
    digitalWrite(YELLOWLED,atoi(value));
    
  if (strcmp(variable,"Knob1") == 0) {
    
    Serial.println(atoi(value));
    ledBrightness = atoi(value);
    analogWrite(REDLED,map(ledBrightness,0,1023,0,255));
  }  

}

/**
*
*
* This function is called periodically and messages can be sent to the iOS device
*
*/
void processOutgoingMessages() {
  
  //Serial.println("processOutgoingMessages");
  
  amController.writeMessage("T",temperature);  
  amController.writeMessage("Led13",digitalRead(YELLOWLED));
}

void processAlarms(char *variable) {
  
  Serial.print("fired Alarm: "); Serial.println(variable);
}

/**
*
*
* This function is called when the iOS device connects
*
*/
void deviceConnected () {  
  Serial.println("Device connected");
  digitalWrite(CONNECTIONLED,HIGH);
}

/**
*
*
* This function is called when the iOS device disconnects
*
*/
void deviceDisconnected () {
  Serial.println("Device disconnected");
  digitalWrite(CONNECTIONLED,LOW);
}



