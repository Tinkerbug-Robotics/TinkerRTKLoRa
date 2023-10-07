/**
 * Firmware for RP2040 running on the rover to receive correction data directly from a base station 
 * through a LoRa network using the TinkerSend - LoRa radio (LLCC68 based). This firmware also reads 
 * data from TinkerCharge and makes it available to the ESP32 to display on a webpage.
 * This firmware sends RTCM correction data to the rover's the PX112X GNSS receiver, which
 * in turn computes an RTK solution for hte rover.
 *  * !!! Note this must be compiled with the Earle Philhower RP2040 board set !!!
 * Copyright Tinkerbug Robotics 2023
 * Provided under GNU GPL 3.0 License
 */

 // !!! Note this must be compiled with the Earle Philhower RP2040 board set !!!

#include "Arduino.h"
#include <RadioLib.h>
#include <Wire.h>
#include "SerialTransfer.h"
#include <SoftwareSerial.h>
#include "pico/stdlib.h"
#include <MAX17055_TR.h>
#include <SPI.h>
#include "programSkyTraq.h"

programSkyTraq program_skytraq;

// Serial connection to ESP32 radio (RX, TX)
SoftwareSerial swSerial(20, 3);

// Library and structure for transfering data to TinkerSend radio
SerialTransfer radioTransfer;
struct STRUCT 
{
    float voltage;
    float avg_voltage;
    float current;
    float avg_current;
    float battery_capacity;
    float battery_age;
    float cycle_counter;
    float SOC;
    float temperature;
} dataForTinkerSend;

// MAX17055 Battery Fuel Cell Gauge

// I2C pins
#define SDA 26
#define SCL 27

MAX17055 max17055;

// Timer to write SOC data on a specified periodic (ms)
unsigned long last_soc_time = 0;
int soc_periodic = 2000;

// RP2040 pins for LoRa radio
#define L_DIO1 6
#define L_TXEN 22
#define L_RXEN 21
#define L_SS   9
#define L_BUSY 2
#define L_RST  5
#define L_MISO 8
#define L_MOSI 11
#define L_SCK  10

// LoRa radio instance
Module* mod;
LLCC68* radio;

// Max line length
#define PACKET_LENGTH 250

// Max size of data received from GNSS receiver in any single burst
#define MAX_MSG_LENGTH 1024

// Create SPI instance for the LoRa radio pins
//arduino::MbedSPI SPI1(L_MISO, L_MOSI, L_SCK);

// Serial connection to GPS receiver's RXD2 pin that receives correction data
//UART Serial2(4, 5, 0, 0);

// Flag to indicate that a packet was received
volatile bool receivedFlag = false;

// Flags for reading and storing correction data
unsigned int msg_indx = 0;
static uint8_t rtcm_msg[MAX_MSG_LENGTH];
char last_byte = 0;
unsigned int msg_length = 99999;
bool in_message = false;

void setup() 
{
    Serial.begin(115200);
    //while (!Serial){}; // Pauses till serial starts. TODO: Remove if running apart from computer
    
    Serial.println("Starting LLCC68 Radio ...");
    
    // Start SPI for LoRa radio
    // Set SPI pins for the radio's SPI
    SPI1.setRX(L_MISO);
    SPI1.setTX(L_MOSI);
    SPI1.setCS(L_SS);
    SPI1.setSCK(L_SCK);
    SPI1.begin();

    // Serial connection to GPS receiver's RXD2 pin that receives correction data
    Serial2.setTX(4);
    Serial2.setRX(5);
    Serial2.begin(115200);

    // Radio serial connection
    swSerial.begin(9600);
    radioTransfer.begin(swSerial);

    // Set I2C pins for communicating with MAX17055
    Wire1.setSDA(SDA);
    Wire1.setSCL(SCL);
    Wire1.begin();

    // Configure MAX17055
    max17055.setResistSensor(0.01); 
    max17055.setCapacity(4400);
    max17055.setChargeTermination(44);
    max17055.setEmptyVoltage(3.3);

    last_soc_time = millis();
    
    // Configure and create instance of LoRa radio
    mod = new Module(L_SS,L_DIO1,L_RST,L_BUSY,SPI1);
    radio = new LLCC68(mod);
    
    // Setup LoRa radio pins
    pinMode(L_RST, OUTPUT);
    
    // LoRa initialization
    digitalWrite(L_RST, LOW);
    delay(100);
    digitalWrite(L_RST, HIGH);
    delay(100);
  
    // Carrier frequency:           915.0 MHz
    // Bandwidth:                   125.0 kHz
    // Spreading factor:            7
    // Coding rate:                 5
    // Sync word:                   0x1424 (private network)
    // Output power:                22 dBm
    // Current limit:               100 mA
    // Preamble length:             8 symbols
    // TCXO voltage:                2.4V
    // Use LDO regulator:           false
    int state = radio->begin(915.0, 125.0, 7, 5, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 22, 8, 0);
    
    if (state == RADIOLIB_ERR_NONE) 
    {
        Serial.println(F("Success!"));
    } 
    else 
    {
        Serial.print(F("Failed, code "));Serial.println(state);
        while (true);
    }

    // External RF switches controlled by (RX enable, TX enable)
    // radio->setRfSwitchPins(L_RXEN, L_TXEN);

    // Set the function that will be called
    // when new packet is received
    radio->setPacketReceivedAction(receivePacket);
  
    // Start listening for LoRa packets
    Serial.print(F("LLCC68 Starting to listen ... "));
    state = radio->startReceive();
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println(F("Listening!"));
    }
    else
    {
        Serial.print(F("Failed, code "));Serial.println(state);
        while (true);
    }

    // Start serial connections to send correction data to GNSS receiver
    Serial1.begin(115200);

    // Initialze library to program SkyTraq
    program_skytraq.init(Serial1);

    // GNSS input/output Serial is Serial1 using default 0,1 (TX, RX) pins
    // Loop through valid baud rates and determine the current setting
    // Set Serial1 to the detected baud rate, stop if a baud rate is not found
    // From NavSpark binary protocol. Search for "SkyTrq Application Note AN0037"
    // Currently available at: https://www.navsparkforum.com.tw/download/file.php?id=1162&sid=dc2418f065ec011e1b27cfa77bf22b19
    if(!autoSetBaudRate())
    {
        Serial.println("No valid baud rate found to talk to receiver, stopping");
        while(1);
    }
    
    delay(250);
  
    Serial.println("Setup complete");
}

void loop()
{
    // Read RTK corrected NMEA data output from GNSS receiver and print to USB serial
    readAndSendNMEAData();

    // Read RTCM correction data from LoRa radio and send to GNSS receiver
    // for it to use in making RTK corrections
    readAndSendRTCMData();

    unsigned long current_time = millis();
    // Periodically write SOC data to ESP32
    if (current_time > last_soc_time + soc_periodic || last_soc_time > current_time)
    { 
        //Serial.println(millis());
        readAndSendSOC();
        last_soc_time = current_time;
    }
}

// Read RTCM correction data from the LoRa radio on the base station and send it
// to the local PX1125R GNSS receiver so it can compute an RTK solution
void readAndSendRTCMData()
{
    unsigned long i = 0;
    bool data_read = false;
    char rtcm_data[2500];

    // Check if the flag is set
    if(receivedFlag)
    {
        // Reset flag
        receivedFlag = false;

        // Read RTK correction data packets from base station
        uint8_t rtk_data[PACKET_LENGTH];
        uint8_t state = radio->readData(rtk_data, PACKET_LENGTH);
        unsigned int msg_num;
        unsigned long i;
        
        if (state == RADIOLIB_ERR_NONE)
        {

            // Packet was successfully received
            int data_length = radio->getPacketLength(true);
            //Serial.print(in_message);Serial.print(" Receive packet of length: ");Serial.println(data_length);
  
            // Loop through received data, may contain more than one message
            for(i=0;i<data_length;i++)
            {
                // Read data from serial
                rtcm_data[i] = rtk_data[i];
                //i++;
                data_read = true;     
        
                // Ensure the char array is not overfilled
                if (i >= 2500)
                {
                    Serial.println("!!!More data than space in char array");
                    break;
                }
            }
            if (data_read)
            {
                //Serial.print(millis()/1000.0);Serial.print(" RTCM chars read: ");Serial.println(i);
                
                // Send RTCM data to GNSS receiver correction input
                Serial2.write(rtcm_data, i);
            }
        }
    }
}

// Function called when packets are received
void receivePacket(void) 
{
  // Packet received, set the flag
  receivedFlag = true;
}

// Read NMEA data from the GNSS receiver and send to USB serial
void readAndSendNMEAData()
{
    // Read latest GNSS data and send to parser
    while(Serial1.available() > 0)
    {
        char ch = Serial1.read();
        Serial.write(ch);
    }
}

// Read and send state of charge (SOC) data to ESP32
void readAndSendSOC()
{
    // Read data and pack into structure
    dataForTinkerSend.voltage = max17055.getInstantaneousVoltage();
    dataForTinkerSend.avg_voltage = max17055.getAvgVoltage();
    dataForTinkerSend.current = max17055.getInstantaneousCurrent();
    dataForTinkerSend.avg_current = max17055.getAvgCurrent();
    dataForTinkerSend.battery_capacity = max17055.getCalculatedCapacity();
    dataForTinkerSend.battery_age = max17055.getBatteryAge();
    dataForTinkerSend.cycle_counter = max17055.getChargeCycle();
    dataForTinkerSend.SOC = max17055.getSOC();
    dataForTinkerSend.temperature = max17055.getTemp();

//    Serial.println("");
//    Serial.print("Voltage: ");Serial.println(dataForTinkerSend.voltage);
//    Serial.print("Avg Voltage: ");Serial.println(dataForTinkerSend.avg_voltage);
//    Serial.print("Current: ");Serial.println(dataForTinkerSend.current);
//    Serial.print("Avg Current: ");Serial.println(dataForTinkerSend.avg_current);
//    Serial.print("Battery Capactity: ");Serial.println(dataForTinkerSend.battery_capacity);
//    Serial.print("Battery Age: ");Serial.println(dataForTinkerSend.battery_age);
//    Serial.print("Number of Cycles: ");Serial.println(dataForTinkerSend.cycle_counter);
//    Serial.print("SOC: ");Serial.println(dataForTinkerSend.SOC);
//    Serial.print("Temperature: ");Serial.println(dataForTinkerSend.temperature);
//    Serial.println("------------------------------------------------------------");
    
    uint16_t sendSize = 0;

    // Send data to TinkerSend radio using serial connection
    sendSize = radioTransfer.txObj(dataForTinkerSend,sendSize);
    radioTransfer.sendData(sendSize);
}

// Loop through valid baud rates for the GNSS receiver and determine the current setting
bool autoSetBaudRate()
{
    // Start serial connections to send correction data to GNSS receiver
    // This loop will detect the current baud rate of the GNSS receiver
    // by sending a message and determining which buad rate returns a valid
    // ACK message
    int valid_baud_rates[9] = {4800, 9600, 19200, 38400, 57600, 115200, 
                               230400, 460800, 921600};

    // Message to reset receiver to defaults
    uint8_t res_payload_length[]={0x00, 0x02};
    int res_payload_length_length = 2;
    uint8_t res_msg_id[]={0x04};
    int res_msg_id_length = 1;
    uint8_t res_msg_body[]={0x01};
    int res_msg_body_length = 1;

    // Loop through possible baud rates
    for (int i=0;i<9;i++)
    {
        // Open the serial connection to the receiver
        Serial1.begin(valid_baud_rates[i]);

        // Send a message to reset receiver to defaults
        if (program_skytraq.sendGenericMsg(res_msg_id,
                                           res_msg_id_length,
                                           res_payload_length,
                                           res_payload_length_length,
                                           res_msg_body,
                                           res_msg_body_length) == 1)
        {
            Serial.print("Found correct baud rate of ");
            Serial.print(valid_baud_rates[i]);
            Serial.println(" for GNSS receiver");
            return true;            
        }               
        else
        {
            Serial1.end();
        }
    }

    return false;
}
