#include <SPI.h>
#include <LoRa.h>
#include "LowPower.h"

#define TEAM_NUMBER 1 // number of team

//LoR32u4II 868MHz or 915MHz (black board)
#define SCK     15
#define MISO    14
#define MOSI    16
#define SS      8
#define RST     4
#define DI0     7
#define BAND    8663E5  // 915E6
#define PABOOST true


// We implemented structs that get send over the LoRa network
// All structs will hold the team number to make sure we dont receive packages from other groups
// Both the gateway and end device will have the same structs so that, at receive time, they can get
// compared to each other and make sure its receiving the correct data

// Struct that will get send by the gateway that holds:
// - Team number
// - The time untill the next beacon arives
typedef struct
{
  uint8_t team_number;
  long beacon_time;
} Beacon;
Beacon beacon_packet; //Create beacon packet placeholder

// Struct that the gateway receives from the end device that holds:
// - Team number
// - Received beacons (For debug/potential expansion)
// - The measured voltage from the end device
typedef struct
{
  uint8_t team_number;
  uint8_t beacon_count;
  float voltage;
} Ack;
Ack ack_packet; //Create ack packet placeholder
int ack_packet_size = sizeof(ack_packet);

bool receive_state = false;
bool timed_out = false;

uint8_t ack_count = 0;
uint8_t beacon_send = 0;

unsigned long previousMillis = 0;
const long interval = 1000;

long rand_time = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial);
  
  // This gets used to get a random seed everytime because of the noise on the analog pin
  randomSeed(analogRead(0));
  
  // LoRa Setup
  LoRa.setPins(SS, RST, DI0);
  Serial.println("INFO: LoRa Receiver init succes!");
  if (!LoRa.begin(BAND, PABOOST)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  beacon_packet.team_number = TEAM_NUMBER;
  receive_state = false;
  delay(2000); // Will be 10 seconds on deployment
}

void loop() {
  // This checks if we are in receive state or not
  // After sending the beacon frame the gateway will wait for 1 second to receive data from the end device
  if (!receive_state) {
    
    rand_time = random(2, 10); // Get a random time between 2 and 10 seconds
    Serial.print("random time: ");
    Serial.println(rand_time);
    beacon_packet.beacon_time = rand_time;
    
    Serial.println("Sending");
    
    // Send the Beacon frame and update the amount of beacons send
    LoRa.beginPacket();
    LoRa.write((uint8_t *)&beacon_packet, sizeof(beacon_packet));
    LoRa.endPacket();
    beacon_send++;
    
    receive_state = true; //Update receive flag to start receiving for 1 second
    previousMillis = millis();
  } else {
    timed_out = false; // This flag will get used to check if 1 second has passed
    while (receive_state) { // As long as 1 second has not passed the gateway has to keep listening for a receive
      int packetSize = LoRa.parsePacket(); // Try to parse received packet
      if (packetSize == ack_packet_size) { // This checks if the size is the expected size, thus also filters out unwanted data
        uint8_t buf[ack_packet_size];
        int i = 0;
        // For as long as the end device is sending data, the gateway will save it in a buffer
        while (LoRa.available()) {
          buf[i] = LoRa.read(); // read buffer
          i++;
        }
        // After all the data has been received we cast it back to a struct so the gateway can read its data
        ack_packet = *(Ack*)buf;
        if (ack_packet.team_number == TEAM_NUMBER) { // Check for the right team number
          Serial.print("Voltage: ");
          Serial.println(ack_packet.voltage); // Get voltage from received struct
          ack_count++; // Update the number of received packages
          receive_state = false; // If a package got received we can start sending again
        }
      }
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        timed_out = true; // Update time out flag
        previousMillis = currentMillis;
        receive_state = false; // If one second passed start sending again but dont update anything else
      }
    }
    // Only if we did not time out we should wait the time given to the end device
    if(!timed_out){
      delay(rand_time * 1000);
    }
  }
  
  // After sending 20 beacons the gateway prints the number beacons send and received
  if (beacon_send == 20 && !receive_state) {
    Serial.print("Beacons send: ");
    Serial.println(beacon_send);
    Serial.print("Ack received: ");
    Serial.println(ack_count);
    delay(150); // Give time to print to serial monitor
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); //Put in ultra sleep mode
    //while (1);
  }
}
