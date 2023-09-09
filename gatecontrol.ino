/*Gate control and doorbell
 * Paul Bradley 2022
 * v2.0 2023
 * 
 * New for v2.0:
 * - Added gate 12v battery monitoring
 * - Changed board to an ESP32 Ethernet POE
 * 
 * Circuit:
 * Doorbell receiver LED connected to pin 35
 * Gate relay connected to pin 33
 * Gate status switch connected to pin 34
 * Battery circuit connected to pin 36.  Needs to be on ADC1 GPIOs 32-39
 * 
 * 680k ohm and 100k ohm resistors and a 100 nF capacitor at the ADC input
 * 
 */

#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <SimpleTimer.h>

// Network and server config
byte mac[] = { 0x00, 0xAA, 0xBB, 0xCC, 0xDA, 0x02 };
IPAddress ip(192,168,1,213);  // Only used if DHCP fails
const char* mqttServer = "192.168.1.128";  // MQTT server address
const int mqttPort = 1883;  // MQTT port
const char* clientName = "gate";
const char* userName = "mqttusername"; // MQTT username
const char* password = "password";  // MQTT pasword

// Global variables
bool doorbellAlreadyTriggered = false;
bool gateAlreadyTriggered = false;
bool gateLatchStatus = false;
bool inTransit = false;
int gateOldStatus = 1;
bool boot = true;

//Arduino pins
const int doorbellPin = 35;
const int gateRelayPin = 33;
const int gateStatusPin = 34;
const int batteryPin = 36;  // Needs to be on an ADC1 pin

void mqttReceive(char* topic, byte* payload, unsigned int length);
void mqttReconnect();  //  Connects or reconnects to the MQTT server
void getDoorbell();  //  Checks the status of the doorbell
void resetDoorbellTrigger();  //  Resets the doorbell trigger
void resetGateTrigger();  //  Resets the gate trigger
void reportGateStatus();  //  Checks the status of the gate
void checkIn();  //  Checks in with the MQTT server and reports the battery status
void syncGateState();  // Re-synchronizes the state of the gate.  Fixes state if it was interupted when changing states

//Ethernet and MQTT objects
EthernetClient ethClient;
PubSubClient mqttClient(ethClient);
SimpleTimer timer;

void setup() 
{
  Serial.begin(9600);
  
  pinMode(doorbellPin, INPUT);
  pinMode(gateRelayPin, OUTPUT);
  pinMode(gateStatusPin, INPUT);
  pinMode(batteryPin, INPUT);
  
  if (!Ethernet.begin(mac)) 
  {
    // if DHCP fails, start with a hard-coded address:
    Ethernet.begin(mac, ip);
  }
  
  delay(3000);  // Pause to allow ethernet to boot up

  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttReceive);

  timer.setInterval(300000, checkIn);  // Checks in with server every 5 minutes
  timer.setInterval(500, reportGateStatus);  // Checks and reports gate status if it changed every 500 ms
  timer.setInterval(200, getDoorbell);  // Checks doorbell status every 200ms
}

void loop() 
{
  if (!mqttClient.connected())
  {
    mqttReconnect();
  }
  
  mqttClient.loop();
  timer.run();
}

void mqttReconnect()
{
  int retries = 0;
  while (!mqttClient.connected())
  {
    if (retries < 15)
    {
        if (mqttClient.connect(clientName, userName, password))
          {
            Serial.println("Connection to MQTT server has been estabilished");
            if(boot)
            {
              mqttClient.publish("checkIn/GateControl", "Rebooted");
              boot = false;
            }
            if(!boot)
            {
              mqttClient.publish("checkIn/GateControl", "Reconnected");
            }
            mqttClient.subscribe("gate/commands");
          }
        else
          {
            retries++;
            Serial.print("Connection to MQTT server failed. Retrying. Attempt: ");\
            Serial.print(retries);
            Serial.println();
            delay(5000);
          }
    }
    if ( retries >= 15 )
    {
      Serial.println("Connection to MQTT server failed. Rebooting.");
      esp_restart(); // Reboot the ESP32
    }
  }
}

void mqttReceive(char* topic, byte* payload, unsigned int length)
{
  // Print the incoming message
  Serial.print("Message received [");
  String newTopic = topic;
  Serial.print(topic);
  Serial.print("]: ");
  payload[length] = '\0';
  String newPayload = String((char*)payload);
  Serial.print(newPayload);
  Serial.println();

  //Do what the message requests
  if ( newTopic == "gate/commands" )
  {
    if ( newPayload == "open" && gateAlreadyTriggered == false)
    {
      // Open the relay to make the gate open.
      digitalWrite(gateRelayPin, HIGH);
      gateAlreadyTriggered = true;  //  Locks out triggers until reset
      timer.setTimeout(2000, resetGateTrigger);  //  Resets the button in 2 seconds
      mqttClient.publish("gate/status", "opening", true);  // Announces that the gate is opening
      timer.setTimeout(30000, syncGateState);  // Re-syncs the status after 30 seconds
      inTransit = true;
    }
    if ( newPayload == "hold" )
    {
      digitalWrite(gateRelayPin, HIGH); //Open the gate and leave it open
      gateLatchStatus = true;
      mqttClient.publish("gate/status", "opening", true);  // Announces that the gate is opening
      timer.setTimeout(30000, syncGateState);  // Re-syncs the status after 30 seconds   
      inTransit = true;  
    }
    if ( newPayload == "close" )
    {
      digitalWrite(gateRelayPin, LOW);  //Release the button push
      mqttClient.publish("gate/status", "closing", true);  // Announces that the gate is closing
      gateLatchStatus = false;
      timer.setTimeout(30000, syncGateState);  // Re-syncs the status after 30 seconds
      inTransit = true;
    }
    if ( newPayload == "reboot" )
    {
      esp_restart(); // Reboot the ESP32
    }
  }
}

void reportGateStatus()
{
  if ( !inTransit )
  {
      int gateNewStatus = digitalRead(gateStatusPin);
    if ( gateNewStatus != gateOldStatus)
   {
      if ( gateNewStatus == 1)
      {
       mqttClient.publish("gate/status", "closed", true);
        Serial.println("Gate is closed");
        gateOldStatus = gateNewStatus;
     }
     if ( gateNewStatus == 0)
      {
       if ( gateLatchStatus )
       {
          mqttClient.publish("gate/status", "latched", true);
         Serial.println("Gate is latched");
        }
        else
        {
          mqttClient.publish("gate/status", "open", true);
          Serial.println("Gate is open");
       }

       gateOldStatus = gateNewStatus;      
      }
   }
  }

}

void getDoorbell()
{
  if(digitalRead(doorbellPin) == 1 && doorbellAlreadyTriggered == false)
  {
    mqttClient.publish("doorbell", "ringing");  //  Doorbell has been pushed
    Serial.println("Doorbell is ringing");
    doorbellAlreadyTriggered = true;  //  Prevents client from spamming server
    timer.setTimeout(30000, resetDoorbellTrigger);  //  Resets the trigger in 6 seconds
  }
}

void checkIn()
{
  char voltage[4];
  int batteryVRaw = 0;
  int batteryVRawSum = 0;
  float batteryVoltage = 0;

  for(int i = 0; i < 10; i++)  // Get 10 readings to minimize noise
  {
    batteryVRaw = analogRead(batteryPin);
    batteryVRawSum += batteryVRaw;
  }
  batteryVRaw = batteryVRawSum / 10;  // Calculates the average
  batteryVoltage = batteryVRaw * 0.00641 + 0.95;  // Calculates the voltage based on the ADC input
  
  dtostrf(batteryVoltage, 4, 1, voltage);  // Converts the float to a char
  mqttClient.publish("checkIn/GateControl/battery", voltage);
  mqttClient.publish("checkIn/GateControl", "OK");
}

void resetGateTrigger()
{
  gateAlreadyTriggered = false;
  digitalWrite(gateRelayPin, LOW);  // Releases the relay
}

void resetDoorbellTrigger()
{
  mqttClient.publish("doorbell", "waiting", true);
  doorbellAlreadyTriggered = false;
}

void syncGateState()
{
  inTransit = false;
  int gateNewStatus = digitalRead(gateStatusPin);
  if ( gateNewStatus == 1)
    {
      mqttClient.publish("gate/status", "closed", true);
      Serial.println("Gate is closed");
      gateOldStatus = gateNewStatus;
    }
    if ( gateNewStatus == 0)
    {
      if ( gateLatchStatus )
      {
        mqttClient.publish("gate/status", "latched", true);
        Serial.println("Gate is latched");
      }
      else
      {
        mqttClient.publish("gate/status", "open", true);
        Serial.println("Gate is open");
      }

      gateOldStatus = gateNewStatus;      
    }
}
