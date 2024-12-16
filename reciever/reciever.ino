#include <SPI.h>
#include <LoRa.h>
#include "LowPower.h"


//LoR32u4II 868MHz or 915MHz (black board)
#define SCK     15
#define MISO    14
#define MOSI    16
#define SS      8
#define RST     4
#define DI0     7
#define BAND    8663E5  // 915E6
#define PABOOST true

#define TEAM_NUMBER 1 // number of team

typedef struct
{
  uint8_t team_number;
  long beacon_time;
} Beacon;
Beacon beacon_packet;

typedef struct
{
  uint8_t team_number;
  uint8_t beacon_count;
  float voltage;
} Ack;
Ack ack_packet;

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
  randomSeed(analogRead(0));
  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(BAND, PABOOST)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  beacon_packet.team_number = TEAM_NUMBER;
  receive_state = false;
  delay(2000);
}

void loop() {
  if (!receive_state) {
    rand_time = random(2, 10);
    Serial.print("random time: ");
    Serial.println(rand_time);
    beacon_packet.beacon_time = rand_time;
    Serial.println("Sending");
    LoRa.beginPacket();
    LoRa.write((uint8_t *)&beacon_packet, sizeof(beacon_packet));
    LoRa.endPacket();
    beacon_send++;
    receive_state = true;
    previousMillis = millis();
  } else {
    timed_out = false;
    while (receive_state) {
      int packetSize = LoRa.parsePacket(); // try to parse received packet
      if (packetSize == ack_packet_size) { //this checks if the size is the expected size, thus also filters out unwanted data
        uint8_t buf[ack_packet_size];
        int i = 0;
        while (LoRa.available()) {
          buf[i] = LoRa.read(); // read buffer
          i++;
        }
        ack_packet = *(Ack*)buf;
        if (ack_packet.team_number == TEAM_NUMBER) { // check for the right team number
          Serial.print("Voltage: ");
          Serial.println(ack_packet.voltage); // get voltage from received struct
          ack_count++;
          receive_state = false;
        }
      }
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        timed_out = true;
        previousMillis = currentMillis;
        receive_state = false;
      }
    }
    if(!timed_out){
      delay(rand_time * 1000);
    }
  }
  if (beacon_send == 20 && !receive_state) {
    Serial.print("Beacons send: ");
    Serial.println(beacon_send);
    Serial.print("Ack received: ");
    Serial.println(ack_count);
    delay(150); // give time to print to serial monitor
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); //Put in ultra sleep mode
    //while (1);
  }
}
