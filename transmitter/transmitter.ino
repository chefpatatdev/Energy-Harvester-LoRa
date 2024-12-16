//End device

#include <SPI.h>
#include <LoRa.h>
#include <EEPROM.h>
#include "Arduino_FreeRTOS.h"
//#include "LowPower.h" //does not work with FreeRTOS

#define TEAM_NUMBER 1 //number of team

//LoR32u4II 868MHz or 915MHz (black board)
#define SCK     15
#define MISO    14
#define MOSI    16
#define SS      8
#define RST     4
#define DI0     7
#define BAND    8663E5
#define PABOOST true

// We implemented structs that get send over the LoRa network
// All structs will hold the team number to make sure we dont receive packages from other groups
// Both the gateway and end device will have the same structs so that, at receive time, they can get
// compared to each other and make sure its receiving the correct data

// Struct that the end device receives from the gateway that holds:
// - Team number
// - The time untill the next beacon arives
typedef struct
{
  uint8_t team_number;
  long beacon_time;
} Beacon;
Beacon beacon_packet; //Create beacon packet placeholder

// Struct that will get send by the end device that holds:
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

long beacon_time = 0;
long recieved_beacon_timestamp = 0;
uint8_t beacon_count = 0;
int beacon_packet_size = sizeof(Beacon);

int lastAddress = 0;

// The end device will use tasks to coordinate receiving/sending packages over LoRa
// and processing a command from the user

void task_process_command( void *params);
void task_recieve_packet( void *params);

TaskHandle_t process_command_handle;

void setup() {
  Serial.begin(9600);

  //while (!Serial);
  
  // LoRa Setup
  LoRa.setPins(SS, RST, DI0);
  Serial.println("INFO: LoRa Receiver init succes!");
  if (!LoRa.begin(BAND, PABOOST )) {
    Serial.println("INFO: Starting LoRa failed!");
    while (1);
  }

  ack_packet.team_number = TEAM_NUMBER;
  ack_packet.beacon_count = beacon_count;
  ack_packet.voltage = 0.0;
  
  int sensorValue = analogRead(A0); // Get rid of the first voltage read as this holds invalid data

  lastAddress = 0; //sets address to 0x0

  // Create the tasks for receiving/sending and to process a command from the user
  BaseType_t  ret = xTaskCreate(task_receive_packet, "task_receive_packet", 256, NULL, 1, NULL);
  ret = xTaskCreate(task_process_command, "task_process_command", 256, NULL, 1, NULL);


  vTaskStartScheduler();
}

void loop() {
}

// Function that will send the voltage to the gateway as a response to receiving the beacon frame
void send_ack(float voltage) {
  ack_packet.voltage = voltage;
  beacon_count++; // Update the beacon count
  ack_packet.beacon_count++;

  LoRa.beginPacket();
  LoRa.write((uint8_t *)&ack_packet, sizeof(ack_packet));
  LoRa.endPacket();
}

// Function to read the sensor value from the analog pin and outputs the actual voltage
float read_voltage() {
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.0 / 1023.0);
  return voltage;
}

// Function to print all the saved voltages along with their timestamps
void print_all_values() {
  float temp_voltage = 0;
  long temp_timestamp = 0;
  int currentAddress = lastAddress;
  
  while (currentAddress > (sizeof(float) + sizeof(long))) { //print untill EEPROM address is back at 0 to print all values
    currentAddress = currentAddress - sizeof(long);
    EEPROM.get(currentAddress, temp_voltage);
    currentAddress = currentAddress - sizeof(long);
    EEPROM.get(currentAddress, temp_timestamp);
    Serial.print(temp_timestamp);
    Serial.print(": ");
    Serial.print(temp_voltage);
    Serial.println(" V");
    vTaskDelay( 1 / portTICK_PERIOD_MS ); 
  }
}

// Function to print the last voltage saved along with its timestamp
void print_last_value() {
  int currentAddress = lastAddress;
  float temp_voltage = 0;
  long temp_timestamp = 0;

  if (currentAddress > (sizeof(float) + sizeof(long))) { // Checks if there is data in EEPROM
    currentAddress = currentAddress - sizeof(long);
    EEPROM.get(currentAddress, temp_voltage);
    currentAddress = currentAddress - sizeof(long);
    EEPROM.get(currentAddress, temp_timestamp);
    Serial.print(temp_timestamp);
    Serial.print(": ");
    Serial.print(temp_voltage);
    Serial.println(" V");
    vTaskDelay( 1 / portTICK_PERIOD_MS ); 
  } else {
    Serial.println("EEPROM address is empty!");
  }

}

// Function used to save timestap and voltage into the EEPROM
void log_data(long timestamp, float voltage) {
  if ((lastAddress + sizeof(long) + sizeof(float)) >= EEPROM.length()) { //create circular buffer, loop back to address 0x00
    lastAddress = 0x0;
  }

  EEPROM.put(lastAddress, timestamp);
  lastAddress += sizeof(long); //move 4 bytes up in address
  EEPROM.put(lastAddress, voltage);
  lastAddress += sizeof(float); //move 4 bytes up in address
}

// Taks used to receive beacon frame and then send the voltage as a response to the gateway
void task_receive_packet( void *params) {
  while (1) {
    // try to parse packet
    int packet_size = LoRa.parsePacket(); //try to parse recieved packet
    if (packet_size == beacon_packet_size) { //check if the packet has the size of a beacon packet
      uint8_t buf[packet_size]; //create buffer

      int i = 0;
      while (LoRa.available()) { //fill buffer with packet data
        buf[i] = LoRa.read();
        i++;
      }
      
      beacon_packet = *(Beacon*)buf; //cast buffer back to struct
      if (beacon_packet.team_number == TEAM_NUMBER) { //check if the beacon packet is from our team
        recieved_beacon_timestamp = millis();
        beacon_time = beacon_packet.beacon_time;
        unsigned int sleep_time = int(float(beacon_time)*0.8);
  
        Serial.print("team number: ");
        Serial.println(beacon_packet.team_number);
        Serial.print("beacon time: ");
        Serial.println(beacon_time);
        
        float voltage = read_voltage();
        send_ack(voltage);
        log_data(recieved_beacon_timestamp, voltage);
        
        if (beacon_count == 20) {
          print_all_values();
          //delete task_process_command
          //vTaskDelete(process_command_handle);
          vTaskDelay( 150 / portTICK_PERIOD_MS ); 
          //LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); //deep sleep forever, does not work with FreeRTOS          
        }

        //dynamicSleep(sleep_time); //does not work with FreeRTOS
      }
    }
    vTaskDelay( 1 / portTICK_PERIOD_MS ); 
  }
  vTaskDelete(NULL);
}

// Task used to process a command from the user
// Depending on what the user wants to see they will type 1, 2 or 3
void task_process_command( void *params ) {
  while (1) {
    if (Serial.available() > 0) {
      String command_input = Serial.readStringUntil('\n');
      if (command_input.length() == 1) { // If the input is anything bigger then one char it will display an error
        char command_input_char = command_input[0];
        switch (command_input_char) {
          case '1': // Command 1 will display the last saved voltage with its timestamp
            Serial.println("Printing last value!");
            print_last_value();
            break;
          case '2': // Command 2 will display all the values saved in the EEPROM
            Serial.println("Printing all values!");
            print_all_values();
            break;
          case '3': // Command 3 will put the end device into sleep mode
            Serial.println("Entering sleep mode...");
            //enter deep sleep mode
            break;
          default:
            Serial.println("Invalid Input:"); // If the input is not 1, 2 or 3 it will display an error
            Serial.println("Use 1, 2, or 3 to start command");
            break;
        }
      }
      else {
        Serial.println("Invalid Input:");
        Serial.println("Use 1, 2, or 3 to start command");
      }
    }
    vTaskDelay( 100 / portTICK_PERIOD_MS ); 
  }
  vTaskDelete(NULL);
}

// Function to sleep dynamically for a specified duration (in seconds)
/*
void dynamicSleep(unsigned int sleepTimeSeconds) {

    if (sleepTimeSeconds < 0 || sleepTimeSeconds > 10) {
        return; // Invalid input, do nothing
    }
    // Convert seconds into milliseconds for precision
    unsigned long remainingTimeMs = sleepTimeSeconds * 1000;
    
    // Available sleep intervals in milliseconds
    const unsigned long sleepIntervals[] = {
        8000, // SLEEP_8S
        4000, // SLEEP_4S
        2000, // SLEEP_2S
        1000, // SLEEP_1S
        500,  // SLEEP_500MS
        250,  // SLEEP_250MS
        120,  // SLEEP_120MS
        60,   // SLEEP_60MS
        30,   // SLEEP_30MS
        15    // SLEEP_15MS
    };
    
    // Corresponding sleep durations
    const period_t sleepPeriods[] = {
        SLEEP_8S,
        SLEEP_4S,
        SLEEP_2S,
        SLEEP_1S,
        SLEEP_500MS,
        SLEEP_250MS,
        SLEEP_120MS,
        SLEEP_60MS,
        SLEEP_30MS,
        SLEEP_15MS
    };

    // Loop until the remaining time is 0
    while (remainingTimeMs > 0) {
        // Find the largest sleep interval less than or equal to the remaining time
        for (int i = 0; i < sizeof(sleepIntervals) / sizeof(sleepIntervals[0]); i++) {
            if (remainingTimeMs >= sleepIntervals[i]) {
                // Sleep for the determined interval
                //LowPower.powerDown(sleepPeriods[i], ADC_ON, BOD_OFF);
                //LowPower.idle(sleepPeriods[i], ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART1_OFF, TWI_OFF, USB_OFF); //9.7mA
                //LowPower.adcNoiseReduction(sleepPeriods[i], ADC_ON, TIMER2_ON); // 10.64mA // with timer 2 on 10.9
                //LowPower.powerStandby(sleepPeriods[i], ADC_OFF, BOD_OFF); // 5.8mA
                LowPower.powerSave(sleepPeriods[i], ADC_OFF, BOD_OFF, TIMER2_OFF); //5.40mA

                // Subtract the slept time from the remaining time
                remainingTimeMs -= sleepIntervals[i];
                break;
            }
        }
    }
}*/
