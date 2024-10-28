/**********************************************************************
*
* Copyright (c) 2024 Tinkerbug Robotics
*
* This program is free software: you can redistribute it and/or modify it under the terms
* of the GNU General Public License as published by the Free Software Foundation, either
* version 3 of the License, or (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful, but WITHOUT ANY
* WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
* PARTICULAR PURPOSE. See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with this 
* program. If not, see <https://www.gnu.org/licenses/>.
* 
* Authors: 
* Christian Pedersen; tinkerbug@tinkerbugrobotics.com
* 
**********************************************************************/

/**
 * Firmware for RP2040 running on the base station which reads RTCM correction data from the 
 * GNSS receiver and sends it to the LoRa radio. This firmware also reads data from TinkerCharge 
 * and makes it available to the ESP32 to display on a webpage. 
 *
 * Note this must be compiled with the Earle Philhower RP2040 board set. This is
 * becuase it uses software serial to communicate to the ESP32. Hardware serial is
 * used for the more time critical GNSS communications.
 * Copyright Tinkerbug Robotics 2024
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
#include <CRC.h>

#include "programSkyTraq.h"

#include <Arduino.h>

// Configuration for pins is in User_Setup.h in the TFT_eSPI library folder
#include <TFT_eSPI.h>
#include <TinyGPSPlus.h>
#include <TouchScreenTR.h>
#include <math.h>
#include <FS.h>
#include <LittleFS.h>

#include "TBR_Logo.h"

#define DEBUG false

// Serial connection to ESP32 radio (RX, TX)
SoftwareSerial swSerial(20, 3);

programSkyTraq program_skytraq;

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
#define L_DIO2 23
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

// Max number of bytes in a packet
#define MAX_PACKET_LENGTH 250

// Save transmission state between loops
int transmission_state = RADIOLIB_ERR_NONE;

// A character array for one block of GNSS data
uint8_t rtcm_data[2500];

// Flag and counter for reading RTCM correction data
bool data_avail = false;
int data_length = 0;

// Flag to indicate that a packet was sent
volatile bool transmit_complete = false;

// Define a small font to use
#define AA_FONT_SMALL "NotoSansKannadaBold10"
#define AA_FONT_MED "NotoSansKannadaBold20"
#define AA_FONT_LARGE "NotoSansKannadaBold40"

// Font files and touch screen calibration are stored in Flash FS
#define FlashFS LittleFS

// Touchscreen pins
#define YP A2
#define XM A3
#define YM 24
#define XP 25

uint8_t screen_rotation = 1;

// Touchscreen calibration values
uint16_t TS_MINX;
uint16_t TS_MINY;
uint16_t TS_MAXX;
uint16_t TS_MAXY;

// Current tab index
uint8_t current_tab_index = 0;

// Reads touchscreen X,Y location and pressure
uint8_t num_touch_samples = 2;
TouchScreen ts = TouchScreen(XP, YP, XM, YM, num_touch_samples);

// TFT library instance
TFT_eSPI tft;

// TFT sprite
TFT_eSprite spr = TFT_eSprite(&tft); 
TFT_eSprite gps_time_spr = TFT_eSprite(&tft); 
TFT_eSprite pos_spr = TFT_eSprite(&tft); 
TFT_eSprite nmea_spr = TFT_eSprite(&tft); 

// Serial1 is the serial connection to GPS receiver on GPIO pins

// For the byte we read from the serial port
byte data = 0;

// Flag to indicate that a packet was sent
volatile bool transmitted_flag = false;

void setup() 
{

    Serial.begin(115200);
    //while (!Serial){};
    delay(2000);
    Serial.println("Starting ...");

    // GNSS serial connection Serial1 default pins of 0 and 1
    Serial1.begin(115200);

    // Initialize TFT display
    tft.begin();

    if (!LittleFS.begin()) 
    {
        Serial.println("Flash FS initialization failed!");
        while (1) yield();
    }
    Serial.println("Flash FS available!");

    bool file_missing = false;
    if (LittleFS.exists("/NotoSansKannadaBold20.vlw") == false) file_missing = true;
    if (LittleFS.exists("/NotoSansKannadaBold40.vlw") == false) file_missing = true;
  
    if (file_missing)
    {
        Serial.println("\nFont file missing in file system, upload fonts using the LittleFS upload tool (Tools->Pico LitleFS Data Upload)?");
        while(1)
            yield();
    }
    else
    {
        Serial.println("\nFonts found OK.");
    }

    // Specify the file path
    const char *filepath = "/ts_cal_data.txt";
  
    // Open the file for reading
    File file = LittleFS.open(filepath, "r");
    if (file) 
    {
        // Get the file size
        size_t fileSize = file.size();
      
        // Read the entire file into a dynamically allocated buffer
        char *buffer = (char *)malloc(fileSize + 1);  // +1 for null terminator
      
        if (buffer == NULL) 
        {
          Serial.println("Error allocating memory for the buffer.");
          file.close();
          return;
        }
      
        size_t bytesRead = file.read((uint8_t *)buffer, fileSize);
      
        // Null-terminate the buffer
        buffer[bytesRead] = '\0';
      
        // Close the file
        file.close();
      
        Serial.print("Read ");
        Serial.print(bytesRead);
        Serial.print(" bytes from LittleFS for touch screen calibration: ");
        Serial.println(buffer);
      
        // Allocate memory
        free(buffer);
    
        if (bytesRead > 10)
        {
            // Parse touchscreen calibration data
            char *ptr = NULL;
            ptr = strtok(buffer,",");
            TS_MINX = atoi(ptr);
            
            ptr = strtok(NULL, ",");
            TS_MINY = atoi(ptr);
            
            ptr = strtok(NULL, ",");
            TS_MAXX = atoi(ptr);
            
            ptr = strtok(NULL, ",");
            TS_MAXY = atoi(ptr);
        }
    }
    // If file does not exist, then run calibration and create file
    else
    {
        Serial.println(" => No such file, run calibration");
        file.close();
        calibrateTouchscreen();
    }

    current_tab_index = 0;

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

    delay(500);

    // Message to set mode to RTK base with survey in
    // Note this message is not documented in the NavSpark binary protocol 
    // documentation it was found by copying the message sent by the by the
    // SkyTraq GNSS viewer when configuring the receiver
    uint8_t payload_length[]={0x00, 0x25};
    int payload_length_length = 2;
    uint8_t msg_id[]={0x6A, 0x06};
    int msg_id_length = 2;
    uint8_t msg_body[]={0x01, 0x01, 0x00, 0x00, 0x00, 
                        0x3C, 0x00, 0x00, 0x00, 0x1E,
                        0x00, 0x00, 0x00, 0x00, 0x00,  
                        0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x00,
                        0x00, 0x00, 0x00, 0x00, 0x01};
    int msg_body_length = 35;
    program_skytraq.sendGenericMsg(msg_id,
                                   msg_id_length,
                                   payload_length,
                                   payload_length_length,
                                   msg_body,
                                   msg_body_length);
    delay(250);
    
    // Configure RTCM measurement data output (0x20)
    uint8_t rtcm_payload_length[]={0x00, 0x11};
    int rtcm_payload_length_length = 2;
    uint8_t rtcm_msg_id[]={0x20};
    int rtcm_msg_id_length = 1;
    uint8_t rtcm_msg_body[]={0x01, 0x00, 0x01, 0x01, 0x00,
                             0x01, 0x01, 0x01, 0x01, 0x0a, 
                             0x00, 0x0a, 0x0a, 0x00, 0x02, 
                             0x01};
    int rtcm_msg_body_length = 16;
    program_skytraq.sendGenericMsg(rtcm_msg_id,
                                   rtcm_msg_id_length,
                                   rtcm_payload_length,
                                   rtcm_payload_length_length,
                                   rtcm_msg_body,
                                   rtcm_msg_body_length);
    delay(250);

    // Start SPI for LoRa radio
    // Set SPI pins for the radio's SPI
    SPI1.setRX(L_MISO);
    SPI1.setTX(L_MOSI);
    SPI1.setCS(L_SS);
    SPI1.setSCK(L_SCK);
    SPI1.begin();

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

    Serial.print("LLCC68 initializing ... ");

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

    // Initialize LLCC68 with default settings
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
  
    // Set the function that will be called
    // when packet transmission is finished
    radio->setPacketSentAction(setFlag);

    // Transmit initial dummy data
    uint8_t byteArr[250] = {0x00};
    transmission_state = radio->startTransmit(byteArr,250);

    Serial.println("Setup Complete");

}

void loop()
{

    if (current_tab_index == 0)
    {
        homeTab();
    }
  
}

void homeTab()
{
    tft.init();
    screen_rotation = 1;
    tft.setRotation(screen_rotation);
    
    tft.fillScreen(TFT_WHITE);
    
    tft.setSwapBytes(true);

    tft.pushImage (205, 5, 105, 115, TBR_Logo);
    tft.fillRoundRect(290, 115, 20, 5, 5, TFT_WHITE);

    // Set the font colour and the background colour
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    
    // Set datum to top center
    tft.setTextDatum(TL_DATUM);

    // Load font
    tft.loadFont(AA_FONT_MED, LittleFS);
    
    tft.drawString("Voltage:", 3, 24);
    tft.drawString("Current:", 3, 44);
    tft.drawString("Battery:", 3, 64);
    tft.drawString("LoRa Bytes Sent:", 3, 147);

    tft.setTextColor(TFT_WHITE, TFT_PURPLE);

    // Scatter plot button
    tft.fillRoundRect(3, 183, 120, 25, 3, TFT_PURPLE);
    tft.drawString("Scatter Plot", 8, 187);

    // Satellite viewer button
    tft.fillRoundRect(125, 183, 110, 25, 3, TFT_PURPLE);
    tft.drawString("Sat Viewer", 131, 187);

    // Signals button
    tft.fillRoundRect(237, 183, 80, 25, 3, TFT_PURPLE);
    tft.drawString("Signals", 242, 187);

    // NMEA messages button
    tft.fillRoundRect(3, 210, 120, 25, 3, TFT_PURPLE);
    tft.drawString("NMEA Msgs", 7, 215);

    // Receiver button
    tft.fillRoundRect(125, 210, 110, 25, 3, TFT_PURPLE);
    tft.drawString("Settings", 140, 215);
    
    // Receiver settings
    tft.fillRoundRect(237, 210, 80, 25, 3, TFT_PURPLE);
    tft.drawString("Rcvr", 255, 215);

    // GPS time sprite setup
    gps_time_spr.unloadFont();
    gps_time_spr.setColorDepth(16);
    gps_time_spr.loadFont(AA_FONT_SMALL, LittleFS);
    gps_time_spr.setTextColor(TFT_BLACK, TFT_WHITE);

    // Break out when the tab index changes, otherwise update data on home tab
    current_tab_index = 0;
    while (current_tab_index == 0)
    {

        uint16_t read_data_length = 0;
        // Read serial buffer and store in a large array
        if (Serial1.available())
        {
            read_data_length = readSerialBuffer();
        }
    
        // If there is complete data to transmit and the previous transmission is 
        // complete, then break up the data into LoRa packets and transmit over radio
        if (data_avail && transmitted_flag)
        {
            decomponseAndSendData(read_data_length);
        }

        unsigned long current_time = millis();

        // Periodically write SOC data to ESP32
        if (current_time > last_soc_time + soc_periodic || last_soc_time > current_time)
        { 
            readAndSendSOC();
            last_soc_time = current_time;
        }
    }
    
    tft.unloadFont();
    gps_time_spr.unloadFont();
    pos_spr.unloadFont();

}

// Read serial buffer into a large array
uint16_t readSerialBuffer ()
{
    // Position through current packet
    int input_pos = 0;

    data_avail = false;

    // Serial character
    byte in_byte;

    // Last time data was read
    unsigned long last_read_time = 0;

    // Read a burst of GNSS data, the PX11XX receivers send RTCM
    // data in bursts of messages with hundreds of milliseconds
    // between each burst
    last_read_time = millis();

    // If serial data is still available or data was recently received
    // This loops through all data in a burst, bursts are 1000 milliseconds apart
    while (Serial1.available () || (millis() - last_read_time) < 10)
    {
        if (Serial1.available ())
        {
            // Read data from serial
            in_byte = Serial1.read();

            rtcm_data[input_pos] = in_byte;

            //Serial.println(in_byte, HEX);
            input_pos++;
            last_read_time = millis();
            data_avail = true;
        }
    }
    if (data_avail)
    {
        Serial.println("-----------------------------------------");
        Serial.print("Read new data: data_length = ");Serial.println(input_pos);
        return input_pos;
    }
    else
    {
        return 0;
    }
}

// Transmit available data over radio
void decomponseAndSendData (uint16_t read_data_length)
{

    uint8_t rtcm_data_to_send[MAX_PACKET_LENGTH] = {0};

    char last_rtcm_char = 0;

    unsigned int message_index = 0;

    // Loop over all data break into separate RTCM messages
    for (int i=0;i<read_data_length;i++)
    {
        // Read data into message to send
        rtcm_data_to_send[message_index] = rtcm_data[i];

        // End of data set
        if(i == read_data_length-1)
        {
            sendTransmission(rtcm_data_to_send, message_index+1);
            Serial.println();Serial.print("End of data buffer, sending ");Serial.println(message_index+1);
            message_index = 0;
        }
        // Max size of a Lora message
        else if (message_index == MAX_PACKET_LENGTH-3)
        {
            sendTransmission(rtcm_data_to_send, message_index+1);
            Serial.println();Serial.print("Filled LoRa buffer, sending ");Serial.println(message_index+1);
            message_index = 0;
        }
        // New message start
        else if (i>2 && (last_rtcm_char == 0XD3 && rtcm_data_to_send[message_index] == 0X00))
        {
            // Message ended two data points ago
            sendTransmission(rtcm_data_to_send, message_index-1);
            Serial.println();Serial.print("End of message found, sending ");Serial.println(message_index-1);

            // Start the next message
            rtcm_data_to_send[0] = 0XD3;
            rtcm_data_to_send[1] = 0X00;

            message_index = 2;
        }
        else
        {
            // Save previous value and increment message index
            last_rtcm_char = rtcm_data_to_send[message_index];
            message_index++;      
        }


    }
    
    // All data sent, no data available
    data_avail = false;
}

void sendTransmission(uint8_t data_to_send[MAX_PACKET_LENGTH], int data_length)
{

    // Block while waiting for last transmission to complete
    while(!transmitted_flag)
        delay(1);

    // Print results
    if (transmission_state == RADIOLIB_ERR_NONE) 
    {
        // Packet was successfully sent 
        if (DEBUG)
            Serial.println(F("transmission finished!"));
    } 
    else 
    {
        Serial.print(F("Transmission failure, code "));Serial.println(transmission_state);
    }
    
    // Clean up after transmission is finished
    // this will ensure transmitter is disabled,
    // RF switch is powered down etc.
    radio->finishTransmit();

    // Add checksum to the end of the message
    uint16_t checksum = calcCRC16(data_to_send,data_length);
    data_to_send[data_length] = (uint8_t)(checksum >> 8);
    data_to_send[data_length+1] = (uint8_t)(checksum & 0xFF);
    if (DEBUG)
    {
        Serial.println(data_to_send[data_length],HEX);
        Serial.println(data_to_send[data_length+1],HEX);
        Serial.print("Checksum = ");Serial.println(checksum);
    }

    data_length += 2;

    for (int i=0;i<data_length-2;i++)
    {
        Serial.print(data_to_send[i],HEX);Serial.print(" ");
    }
    Serial.println("");

    // Start transmission of new data
    transmission_state = radio->startTransmit(data_to_send,data_length);
    if (DEBUG)
        Serial.print("Transmitting pack of length ");Serial.println(data_length);

    pos_spr.unloadFont();
    pos_spr.setColorDepth(16);
    pos_spr.loadFont(AA_FONT_MED, LittleFS);
    pos_spr.setTextColor(TFT_BLUE, TFT_WHITE);
    tft.setCursor(170, 147);
    pos_spr.printToSprite("  " + (String)data_length + "  ");

    // Reset transmission flag
    transmitted_flag = false;
}

void calibrateTouchscreen()
{
    // Set screen rotation
    screen_rotation = 1;
    tft.setRotation(screen_rotation);

    // Prepare screen
    tft.fillScreen(TFT_WHITE);
    tft.setSwapBytes(true);
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.setTextDatum(TL_DATUM);

    // Load small font
    tft.loadFont(AA_FONT_MED, LittleFS);

    // Initialize calibration values
    TS_MINX = 9999;
    TS_MINY = 9999;
    TS_MAXX = 0;
    TS_MAXY = 0;

    // Draw initial screen
    tft.fillRect(0,0,10,10,TFT_RED);
    tft.fillRect(310,0,10,10,TFT_RED);
    tft.fillRect(0,230,10,10,TFT_RED);
    tft.fillRect(310,230,10,10,TFT_RED);

    tft.drawString("Screen Calibration:", 40, 60);
    tft.drawString("Press corners in any order", 40, 80);
    tft.drawString("multiple times starting in", 40, 100);

    // Switch to larger font for count down timer
    tft.unloadFont();
    spr.setColorDepth(16);
    spr.loadFont(AA_FONT_LARGE, LittleFS);
    spr.setTextColor(TFT_BLACK, TFT_WHITE);

    // Middle of the screen
    int mid_x = tft.width() / 2;

    unsigned long start_time = millis();

    // Display countdown timer
    int count_down_millis = 5000;
    tft.setCursor(mid_x-30, 150);
    while (millis() < start_time + count_down_millis)
    {
        int count_down = (int)ceil((count_down_millis - (millis() - start_time))/1000);
        spr.printToSprite(" " + (String)count_down + " ");
        delay(100);
    }

    // Draw calibration background
    tft.fillScreen(TFT_WHITE);
    int mid_y = (tft.height() / 2)-50;
    tft.pushImage (mid_x-50, mid_y-60, 105, 115, TBR_Logo);
    tft.fillRoundRect(mid_x+35, mid_y+50, 20, 5, 5, TFT_WHITE);
    tft.fillRect(0,0,10,10,TFT_RED);
    tft.fillRect(310,0,10,10,TFT_RED);
    tft.fillRect(0,230,10,10,TFT_RED);
    tft.fillRect(310,230,10,10,TFT_RED);

    // Count down number location
    spr.setTextDatum(TC_DATUM);

    tft.setCursor(mid_x-40, 150);

    count_down_millis = 20000;
    start_time = millis();
    int points_selected = 0;

    // Take points for duration milliseconds
    while ((millis() < start_time + count_down_millis) || points_selected < 15)
    { 
        TSPoint p = ts.getPoint();
        if (p.z > 250) 
        {

            TS_MINX = min(p.x,TS_MINX);
            TS_MINY = min(p.y,TS_MINY);
            TS_MAXX = max(p.x,TS_MAXX);
            TS_MAXY = max(p.y,TS_MAXY);
            Serial.print("X = ");Serial.print(p.x);
            Serial.print(" Y = ");Serial.print(p.y);
            Serial.print(" Z = ");Serial.println(p.z);
            tft.fillRect(0,0,10,10,TFT_BLUE);
            tft.fillRect(310,0,10,10,TFT_BLUE);
            tft.fillRect(0,230,10,10,TFT_BLUE);
            tft.fillRect(310,230,10,10,TFT_BLUE);
            delay(200);
            tft.fillRect(0,0,10,10,TFT_RED);
            tft.fillRect(310,0,10,10,TFT_RED);
            tft.fillRect(0,230,10,10,TFT_RED);
            tft.fillRect(310,230,10,10,TFT_RED);
            points_selected++;
        }
        int count_down = (int)ceil((count_down_millis - (millis() - start_time))/1000);

        if (millis() < start_time + count_down_millis)
        {
            spr.printToSprite("  " + (String)(count_down) + "  ");
        }
        else
        {
            spr.unloadFont();
            spr.loadFont(AA_FONT_MED, LittleFS);
            tft.setCursor(0, 150);
            spr.printToSprite("  Press Each Corner  ");
        }
    }
  
    Serial.print("Min X = ");Serial.println(TS_MINX);
    Serial.print("Min Y = ");Serial.println(TS_MINY);
    Serial.print("Max X = ");Serial.println(TS_MAXX);
    Serial.print("Max Y = ");Serial.println(TS_MAXY);

    char data[100] = {0};
    char TS_MINX_char[20];
    char TS_MINY_char[20];
    char TS_MAXX_char[20];
    char TS_MAXY_char[20];
    char comma_char[2] =",";
    itoa(TS_MINX,TS_MINX_char,10);
    itoa(TS_MINY,TS_MINY_char,10);
    itoa(TS_MAXX,TS_MAXX_char,10);
    itoa(TS_MAXY,TS_MAXY_char,10);
    strcpy(data,TS_MINX_char);
    strcat(data,comma_char);
    strcat(data,TS_MINY_char);
    strcat(data,comma_char);
    strcat(data,TS_MAXX_char);
    strcat(data,comma_char);
    strcat(data,TS_MAXY_char);

    const char *filepath = "/ts_cal_data.txt";

    // Open or create the file for writing
    File file = LittleFS.open(filepath, "w");
    if (!file) 
    {
        Serial.println("Error opening file for writing.");
        return;
    }
  
    // Write a character array to the file
    size_t bytesWritten = file.write((const uint8_t *)data, strlen(data));
  
    if (bytesWritten != strlen(data)) 
    {
        Serial.println("Calibration file failed to open, using temporary values");
        file.close();
        return;
    }
  
    // Close the file
    file.close();

    testTouchscreen();

    tft.unloadFont();
    spr.unloadFont();

}

void testTouchscreen()
{

      screen_rotation = 1;
    tft.setRotation(screen_rotation);
    
    tft.fillScreen(TFT_WHITE);
    
    tft.setSwapBytes(true);

    // Set the font colour and the background colour
    tft.setTextColor(TFT_BLACK, TFT_WHITE);

    tft.loadFont(AA_FONT_MED, LittleFS);
    
    tft.drawString("Try it!! Tap the ball", 80, 100);

    delay(2000);
    
    tft.fillScreen(TFT_WHITE);

    bool point_drawn = false;

    int point_found = 0;

    while(point_found < 10)
    {
        if(!point_drawn)
        {
          tft.fillSmoothCircle(random(320), random(240), 10, TFT_PURPLE, TFT_PURPLE);
          point_drawn = true;
        }

        // Look for touchscreen touch
        TSPoint p = ts.getPoint();

        if (p.z > 250) 
        {

            int x = getTouchX(p);
            int y = getTouchY(p);

            Serial.print("p.y/x = "); Serial.print(p.y);Serial.print(" ");Serial.print(x);
            Serial.print(" p.x/y = "); Serial.print(p.x);Serial.print(" ");Serial.println(y);
            tft.fillSmoothCircle(x, y, 5, TFT_BLUE, TFT_BLUE);

            point_found++;
            point_drawn = false;

            delay(2000);
            tft.fillScreen(TFT_WHITE);
        }
    }

    tft.unloadFont();
    spr.unloadFont();

}

int getTouchY(TSPoint p)
{
    if (screen_rotation == 1)
    {
        int y = constrain(p.x, TS_MINY, TS_MAXY);
        y = map(y, TS_MINY, TS_MAXY, 0, tft.height());
        return y;
    }
    else if (screen_rotation == 0)
    {
        int y = constrain(p.y, TS_MINY, TS_MAXY);
        y = tft.height() - map(y, TS_MINY, TS_MAXY, 0, tft.height());
        return y;
    }
    else
    {
        Serial.println("Undefined screen rotation");
        return 0;
    }
}

int getTouchX(TSPoint p)
{
    if (screen_rotation == 1)
    {
        int x = constrain(p.y, TS_MINY, TS_MAXY);
        x = tft.width() - map(x, TS_MINY, TS_MAXY, 0, tft.width());
        return x;
    }
    else if (screen_rotation == 0)
    {
        int x = constrain(p.x, TS_MINY, TS_MAXY);
        x = tft.width() -map(x, TS_MINY, TS_MAXY, 0, tft.width());
        return x;
    }
    else
    {
        Serial.println("Undefined screen rotation");
        return 0;
    }
}

// Void type function iwth no arguements called when a complete packet
// is transmitted by the module
void setFlag(void) 
{
  // Indicate a packet was sent
  transmitted_flag = true;
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

    pos_spr.unloadFont();
    pos_spr.setColorDepth(16);
    pos_spr.loadFont(AA_FONT_MED, LittleFS);
    pos_spr.setTextColor(TFT_BLUE, TFT_WHITE);
    tft.setCursor(90, 24);
    pos_spr.printToSprite("  " + (String)dataForTinkerSend.voltage + "  ");
    tft.setCursor(90, 44);
    pos_spr.printToSprite("  " + (String)dataForTinkerSend.current + "  ");
    tft.setCursor(90, 64);
    pos_spr.printToSprite("  " + (String)dataForTinkerSend.SOC + "  ");

    if (DEBUG)
    {
        Serial.println("");
        Serial.print("Voltage: ");Serial.println(dataForTinkerSend.voltage);
        Serial.print("Avg Voltage: ");Serial.println(dataForTinkerSend.avg_voltage);
        Serial.print("Current: ");Serial.println(dataForTinkerSend.current);
        Serial.print("Avg Current: ");Serial.println(dataForTinkerSend.avg_current);
        Serial.print("Battery Capactity: ");Serial.println(dataForTinkerSend.battery_capacity);
        Serial.print("Battery Age: ");Serial.println(dataForTinkerSend.battery_age);
        Serial.print("Number of Cycles: ");Serial.println(dataForTinkerSend.cycle_counter);
        Serial.print("SOC: ");Serial.println(dataForTinkerSend.SOC);
        Serial.print("Temperature: ");Serial.println(dataForTinkerSend.temperature);
        Serial.println("------------------------------------------------------------");
    }
    uint16_t sendSize = 0;

    // Send data to TinkerSend radio using serial connection
    sendSize = radioTransfer.txObj(dataForTinkerSend,sendSize);
    radioTransfer.sendData(sendSize);
}

// Loop through valid baud rates and determine the current setting
bool autoSetBaudRate()
{
    // Start serial connections to send correction data to GNSS receiver
    // This loop will detect the current baud rate of the GNSS receiver
    // by sending a message and determining which buad rate returns a valid
    // ACK message
    int valid_baud_rates[10] = {115200, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600}; 
    
    // A character array for one block of GNSS data
    uint8_t serial_data[2500];

    // How long to wait for response from receiver
    int wait_duration = 250;

    // Message to reset receiver to defaults
    uint8_t res_payload_length[]={0x00, 0x02};
    int res_payload_length_length = 2;
    uint8_t res_msg_id[]={0x04};
    int res_msg_id_length = 1;
    uint8_t res_msg_body[]={0x01};
    int res_msg_body_length = 1;

    // Loop through possible baud rates
    for (int i=0;i<10;i++)
    {
        // Open the serial connection to the receiver
        Serial1.begin(valid_baud_rates[i]);

        Serial.print("Trying baud rate = ");Serial.println(valid_baud_rates[i]);
        // Send a message to reset receiver to defaults
        if (program_skytraq.sendGenericMsg(res_msg_id,
                                           res_msg_id_length,
                                           res_payload_length,
                                           res_payload_length_length,
                                           res_msg_body,
                                           res_msg_body_length) == 1)
        {
            Serial.print("Found correct baud rate of ");Serial.println(valid_baud_rates[i]);
            return true;            
        }               
        else
        {
            Serial1.end();
        }
    }

    return false;
}
