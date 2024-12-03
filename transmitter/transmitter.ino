#include <SPI.h>
#include <LoRa.h>
#include <EEPROM.h>

#define LOG_LINES 20
#define TEAM_NUMBER 1

#define SCK     15
#define MISO    14
#define MOSI    16
#define SS      8
#define RST     4
#define DI0     7
#define BAND    8663E5
#define PABOOST true

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

long next_beacon_timestamp = 0;
long recieved_beacon_timestamp = 0;
uint8_t beacon_count = 0;
int beacon_packet_size = sizeof(Beacon);

int lastAddress = 0;


void setup() {
  Serial.begin(9600);

  while (!Serial);
  Serial.println("LoRa Receiver");
  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(BAND, PABOOST )) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  ack_packet.team_number = TEAM_NUMBER;
  ack_packet.beacon_count = beacon_count;
  ack_packet.voltage = 0.0;
  int sensorValue = analogRead(A0);

  EEPROM.get( 0, lastAddress ); //read the lastAddress where data was printed to and continue there
  if (lastAddress < 0 || lastAddress > EEPROM.length()) { //check if lastAddress is in valid range, address 0 is reserved for the lastAddress
    lastAddress = 0; //if no address is found, set address to 0x4 which is equal to sizeof(float) as start address
  }
}

void loop() {
  // try to parse packet
  int packet_size = LoRa.parsePacket();
  if (packet_size == beacon_packet_size) {
    uint8_t buf[packet_size];

    int i = 0;
    while (LoRa.available()) {
      buf[i] = LoRa.read();
      i++;
    }
    beacon_packet = *(Beacon*)buf;
    if (beacon_packet.team_number == TEAM_NUMBER) {
      recieved_beacon_timestamp = millis();
      Serial.print("team number: ");
      Serial.println(beacon_packet.team_number);
      Serial.print("beacon time: ");
      Serial.println(beacon_packet.beacon_time);
      float voltage = read_voltage();
      send_ack(voltage);
      log_data(voltage);
    }
  }
}

void send_ack(float voltage) {
  ack_packet.voltage = voltage;
  beacon_count++;
  Serial.println(beacon_count);
  ack_packet.beacon_count++;

  LoRa.beginPacket();
  LoRa.write((uint8_t *)&ack_packet, sizeof(ack_packet));
  LoRa.endPacket();
}

float read_voltage() {
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.0 / 1023.0);
  return voltage;
}

void print_values() { //print last 20 values
  float temp = 0;
  int currentAddress = lastAddress;

  for (int i = 0; i <= LOG_LINES; i++) {
    currentAddress = currentAddress - sizeof(float);
    if (currentAddress < sizeof(float)) { //loop back to top of EEPROM address space
      currentAddress = EEPROM.length() - sizeof(float);
    }
    EEPROM.get(currentAddress, temp);
    //Serial.print(currentAddress);
    Serial.print(temp);
    Serial.println(" °C");
    delay(10);
  }
}

void log_data(float voltage) {
  lastAddress += sizeof(float); //move 4 bytes up in address

  EEPROM.put(lastAddress, voltage);
  if (lastAddress >= EEPROM.length()) { //create circular buffer, loop back to address 0x04
    lastAddress = sizeof(float);
  }
}
