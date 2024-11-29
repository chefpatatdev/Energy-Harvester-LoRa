#include <SPI.h>
#include <LoRa.h>

//LoR32u4II 868MHz or 915MHz (black board)
#define SCK     15
#define MISO    14
#define MOSI    16
#define SS      8
#define RST     4
#define DI0     7
#define BAND    8663E5  // 915E6
#define PABOOST true 

#define TEAM_NUMBER 1

typedef struct 
{
    uint8_t team_number;
    long beacon_time;
} Beacon;
Beacon beacon_packet;

typedef struct 
{
    uint8_t team_number;
    int voltage;
} Ack;
Ack ack_packet;

int ack_packet_size = sizeof(ack_packet);

bool receive_state = false;

unsigned long previousMillis = 0; 
const long interval = 1000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  randomSeed(analogRead(0));
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND,PABOOST)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  beacon_packet.team_number = TEAM_NUMBER;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!receive_state){
    long rand_time = random(2,10);

    beacon_packet.beacon_time = rand_time;
    
    LoRa.beginPacket();
    LoRa.write((uint8_t *)&beacon_packet, sizeof(beacon_packet));
    LoRa.endPacket();
    receive_state = true;
  }
  else{
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      int packetSize = LoRa.parsePacket();
      if (packetSize == struct_size) { //this checks if the size is the expected size, thus also filters out unwanted data
        uint8_t buf[struct_size];
        int i = 0;
        while (LoRa.available()) {
          buf[i] = LoRa.read();
          //Serial.print((char)buf[i]);
          i++;
        }
      }
      receive_state = false;
  }
  
  
  delay(rand_time*1000);

}
