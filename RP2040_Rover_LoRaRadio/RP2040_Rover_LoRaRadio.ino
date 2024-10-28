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
#include <SoftwareSerial.h>
#include "pico/stdlib.h"
#include <TinyGPSPlus.h>
#include <SPI.h>
#include "programSkyTraq.h"
#include <CRC.h>

programSkyTraq program_skytraq;

// Serial connection to RocketComputer ESP32 (RX, TX)
SoftwareSerial swSerial(12, 13);
bool msg_printed = false;


typedef struct  
{
    unsigned long time_stamp;
    uint8_t gnss_quality_indicator;
    float rtk_ratio;
    float rtk_age;
    float latitude;
    float longitude;
    float altitude;
    float RSSI;
} gnssData;

// GNSS parsing
uint8_t gnss_quality_indicator = 0;
float rtk_age = 0;
float rtk_ratio = 0;
float latitude = 0.0;
float longitude = 0.0;
float altitude = 0.0;
uint32_t last_read_time;
float RSSI = 0.0;

// GNSS receiver data is parsed by the TinyGPSPlus library
TinyGPSPlus gnss;
TinyGPSCustom gnss_quality(gnss, "GPGGA", 6);

// I2C pins
#define SDA 26
#define SCL 27

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
#define MAX_PACKET_LENGTH 256

// Max size of data received from GNSS receiver in any single burst
#define MAX_MSG_LENGTH 1024

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

    // Start serial connection to Skytraq receiver
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
        while(1) delay(1000);
    }
    
    delay(250);
    
    // Configure NMEA messages to send
    //  a0 a1 00 0f 64 02 01 00 00 00 01 00 00 00 00 00 00 00 00 66 0d 0a  
    Serial.println("Configure receiver to send specific NMEA messages");
    uint8_t msg_payload_length[]={0x00, 0x0f};
    int msg_payload_length_length = 2;
    uint8_t msg_msg_id[]={0x64};
    int msg_msg_id_length = 1;
    uint8_t msg_msg_body[]={0x02, 0x01, 0x00, 0x00, 0x00,
                            0x01, 0x00, 0x00, 0x00, 0x00, 
                            0x00, 0x00, 0x00, 0x00};
    int msg_msg_body_length = 14;
    program_skytraq.sendGenericMsg(msg_msg_id,
                                   msg_msg_id_length,
                                   msg_payload_length,
                                   msg_payload_length_length,
                                   msg_msg_body,
                                   msg_msg_body_length);
    delay(100);
    
    // Configure receiver to output position data at 10 Hz
    // a0 a1 00 03 0e 0a 00 04 0d 0a
    Serial.println("Configure receiver to send data at 10 Hz");
    uint8_t bin_payload_length[]={0x00, 0x03};
    int bin_payload_length_length = 2;
    uint8_t bin_msg_id[]={0x0E};
    int bin_msg_id_length = 1;
    uint8_t bin_msg_body[]={0x0A, 0X00}; // {0x0A, 0X00}; 10 Hz
    int bin_msg_body_length = 2;
    program_skytraq.sendGenericMsg(bin_msg_id,
                                   bin_msg_id_length,
                                   bin_payload_length,
                                   bin_payload_length_length,
                                   bin_msg_body,
                                   bin_msg_body_length);
    delay(100);

    // Rocket computer serial connection
    swSerial.begin(57600);
    
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
    // Spreading factor:            10
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

    Serial.println("Setup complete");
}

void loop()
{

    // Read RTCM correction data from LoRa radio and send to GNSS receiver
    // for it to use in making RTK corrections
    readAndSendRTCMData();

    unsigned long current_time = millis();

}

// Read RTCM correction data from the LoRa radio on the base station and send it
// to the local PX1125R GNSS receiver so it can compute an RTK solution
void readAndSendRTCMData()
{
    unsigned long i = 0;
    bool data_read = false;

    // Check if the flag is set
    if(receivedFlag)
    {
        // Reset flag
        receivedFlag = false;

        // Read RTK correction data packets from base station
        uint8_t rtk_data[MAX_PACKET_LENGTH];
        uint16_t data_length = radio->getPacketLength(true);
        int state = radio->readData(rtk_data, data_length);
        RSSI = radio->getRSSI();
        Serial.print("RSSI = ");Serial.println(radio->getRSSI());
        Serial.print("SNR = ");Serial.println(radio->getSNR());
        Serial.print("Received radio packet, state: ");Serial.println(state);
        
        if (state == RADIOLIB_ERR_NONE)
        {

            // Packet was successfully received
            //Serial.print(" Receive packet of length: ");Serial.println(data_length);

            // Don't send checksum to receiver
            uint16_t rtcm_data_length = data_length - 2;

            // Calculate checksum of the sent data, without checksum
            uint16_t calc_checksum = calcCRC16(rtk_data, rtcm_data_length);

            // Extract the checksum from the received data (last 2 bytes)
            uint16_t received_checksum = (rtk_data[data_length-2] << 8) 
                                        | rtk_data[data_length-1];

            // Compare the extracted checksum with the calculated one
            if (received_checksum == calc_checksum) 
            {

                // Send RTCM data to GNSS receiver correction input
                // Exclude checksum data 
                Serial2.write(rtk_data, data_length);

            }
            else
            {
                Serial.print("Checksum received: ");Serial.print(received_checksum);
                Serial.print(", does not match calculated checksum: ");
                Serial.print(calc_checksum);Serial.println(" Discarding data");
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
