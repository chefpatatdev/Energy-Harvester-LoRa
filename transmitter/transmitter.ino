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

typedef struct 
{
    uint8_t team_number;
    long beacon_time;
} Beacon;
Beacon beacon_packet;

unsigned long next_beacon_timestamp = 0;
unsigned long recieved_beacon_timestamp = 0;


void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("LoRa Receiver");
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND,PABOOST )) {
    Serial.println("Starting LoRa failed!");
    while (1);
  } 
  LoRa.setSyncWord(0xF3);

}

void loop() {
    // try to parse packet
    int packet_size = LoRa.parsePacket();
    if (packet_size > 0){
    uint8_t buf[packet_size];
  
    int i = 0;
    while (LoRa.available()) {
      buf[i] = LoRa.read();
      i++;
    }    
    beacon_packet = *(Beacon*)buf;
    recieved_beacon_timestamp = millis();
    recieved_beacon_timestamp += beacon_packet.beacon_time*1000;
    Serial.print("team number: ");
    Serial.println(beacon_packet.team_number);
    Serial.print("beacon time: ");
    Serial.println(beacon_packet.beacon_time);    
    
    
  }

}
