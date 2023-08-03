/**
 * Firmware for RP2040 running on the rover to receive correction data directly from a base station 
 * through a LoRa network using the TinkerSend - LoRa radio (LLCC68 based).
 * This firmware sends RTCM correction data to the rover's the PX112X GNSS receiver, which
 * in turn computes an RTK solution for hte rover.
 * Copyright Tinkerbug Robotics 2023
 * Provided under GNU GPL 3.0 License
 */

// Radio library for working with radio
#include <RadioLib.h>

// Used for hardware serial on GPIO pins
#include <Arduino.h>

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
arduino::MbedSPI SPI1(L_MISO, L_MOSI, L_SCK);

// Serial connection to GPS receiver's RXD2 pin that receives correction data
UART Serial2(4, 5, 0, 0);

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
    SPI1.begin();
    
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
    Serial2.begin(115200);
  
    Serial.println("Setup complete");
}

void loop()
{
    // Read RTK corrected NMEA data output from GNSS receiver and print to USB serial
    readAndSendNMEAData();

    // Read RTCM correction data from LoRa radio and send to GNSS receiver
    // for it to use in making RTK corrections
    readAndSendRTCMData();
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
