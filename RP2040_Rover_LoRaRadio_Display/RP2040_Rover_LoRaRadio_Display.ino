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
 * Firmware for RP2040 running on the rover to receive correction data directly from a base station 
 * through a LoRa network using the TinkerSend - LoRa radio (LLCC68 based). This firmware also reads 
 * data from TinkerCharge and makes it available to the ESP32 to display on a webpage.
 * This firmware sends RTCM correction data to the rover's the PX112X GNSS receiver, which
 * in turn computes an RTK solution for the rover.
 *  * !!! Note this must be compiled with the Earle Philhower RP2040 board set !!!
 * Copyright Tinkerbug Robotics 2023
 * Provided under GNU GPL 3.0 License
 */

 // !!! Note this must be compiled with the Earle Philhower RP2040 board set !!!
 
 #include <Arduino.h>

// Configuration for pins is in User_Setup.h in the TFT_eSPI library folder
#include <TFT_eSPI.h>
#include <TinyGPSPlus.h>
#include <TouchScreenTR.h>
#include <math.h>
#include <FS.h>
#include <LittleFS.h>
#include <CRC.h>
#include <SPI.h>
#include <RadioLib.h>
#include <Wire.h>
#include <SerialTransfer.h>
#include <SoftwareSerial.h>
#include <pico/stdlib.h>

// Custom libraries
#include "TBR_Logo.h"
#include <MAX17055_TR.h>
#include <programSkyTraq.h>

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

// GNSS receiver data is parsed by the TinyGPSPlus library
TinyGPSPlus gnss;
TinyGPSCustom gnss_quality(gnss, "GPGGA", 6);

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

gnssData gnss_data;

bool new_gnss_data = false;

int lastTime;
static const double deg2rad = M_PI/180.0;

// NMEA Message Scrolling
// The scrolling area must be a integral multiple of TEXT_HEIGHT
#define TEXT_HEIGHT 16 // Height of text to be printed and scrolled
#define BOT_FIXED_AREA 32 // Number of lines in bottom fixed area (lines counted from bottom of screen)
#define TOP_FIXED_AREA 0 // Number of lines in top fixed area (lines counted from top of screen)
#define YMAX 320 // Bottom of screen area

// The initial y coordinate of the top of the scrolling area
uint16_t yStart = TOP_FIXED_AREA;
// yArea must be a integral multiple of TEXT_HEIGHT
uint16_t yArea = YMAX-TOP_FIXED_AREA-BOT_FIXED_AREA;
// The initial y coordinate of the top of the bottom text line
uint16_t yDraw = YMAX - BOT_FIXED_AREA - TEXT_HEIGHT;

// Keep track of the drawing x coordinate
uint16_t xPos = 0;

// For the byte we read from the serial port
byte data = 0;

// A few test variables used during debugging
bool change_colour = 1;
bool selected = 1;

// We have to blank the top line each time the display is scrolled, but this takes up to 13 milliseconds
// for a full width line, meanwhile the serial buffer may be filling... and overflowing
// We can speed up scrolling of short text lines by just blanking the character we drew
int blank[19]; // We keep all the strings pixel lengths to optimise the speed of the top line blanking

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

// GNSS parsing
uint8_t gnss_quality_indicator = 0;
float rtk_age = 0;
float rtk_ratio = 0;
float latitude = 0.0;
float longitude = 0.0;
float altitude = 0.0;
uint32_t last_read_time;
float RSSI = 0.0;

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

// Max LoRa packet length
#define MAX_PACKET_LENGTH 256

// Max size of data received from GNSS receiver in any single burst
#define MAX_MSG_LENGTH 1024

// Serial2 is the  connection to GPS receiver's RXD2 pin that receives
// correction data

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
    //while (!Serial){};
    delay(1000);
    Serial.println("Starting ...");

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
//    Serial.println("Configure receiver to send specific NMEA messages");
//    uint8_t msg_payload_length[]={0x00, 0x0f};
//    int msg_payload_length_length = 2;
//    uint8_t msg_msg_id[]={0x64};
//    int msg_msg_id_length = 1;
//    uint8_t msg_msg_body[]={0x02, 0x01, 0x00, 0x00, 0x00,
//                            0x01, 0x00, 0x00, 0x00, 0x00, 
//                            0x00, 0x00, 0x00, 0x00};
//    int msg_msg_body_length = 14;
//    program_skytraq.sendGenericMsg(msg_msg_id,
//                                   msg_msg_id_length,
//                                   msg_payload_length,
//                                   msg_payload_length_length,
//                                   msg_msg_body,
//                                   msg_msg_body_length);
//    delay(100);
    
    // Configure receiver to output position data at 10 Hz
    // a0 a1 00 03 0e 0a 00 04 0d 0a
//    Serial.println("Configure receiver to send data at 10 Hz");
//    uint8_t bin_payload_length[]={0x00, 0x03};
//    int bin_payload_length_length = 2;
//    uint8_t bin_msg_id[]={0x0E};
//    int bin_msg_id_length = 1;
//    uint8_t bin_msg_body[]={0x0A, 0X00}; // {0x0A, 0X00}; 10 Hz
//    int bin_msg_body_length = 2;
//    program_skytraq.sendGenericMsg(bin_msg_id,
//                                   bin_msg_id_length,
//                                   bin_payload_length,
//                                   bin_payload_length_length,
//                                   bin_msg_body,
//                                   bin_msg_body_length);
    delay(100);

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

    lastTime = millis();

    current_tab_index = 0;

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
    
    delay(250);

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
    
    tft.drawString("Lat:", 3, 24);
    tft.drawString("Lon:", 3, 44);
    tft.drawString("Alt:", 3, 64);
    tft.drawString("Mode:", 3, 84);
    tft.drawString("RTK Ratio:", 3, 104);
    tft.drawString("RTK Age:", 3, 124);
    tft.drawString("RSSI:", 3, 147);
    tft.drawString("SNR:", 150, 147);
    //tft.drawString("Up:", 220, 147);

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
        // Read RTCM correction data from LoRa radio and send to GNSS receiver
        // for it to use in making RTK corrections
        readAndSendRTCMData();
 
        // Read RTK corrected NMEA data output from GNSS receiver and print to USB serial
        readAndSendNMEAData();
 
        uint8_t gps_second = 0;
    
        if (gnss.time.isUpdated() && gnss.time.second() != gps_second)
        {
            // ### Time ###
            
            // Cursor position for printing time
            tft.setCursor(3, 2);

            uint8_t gps_minute = gnss.time.minute();
            String minute_string;
            if (gps_minute < 10)
                minute_string = "0" + (String)gps_minute;
            else
                minute_string = (String)gps_minute;

            gps_second = gnss.time.second();
            String second_string;
            if (gps_second < 10)
                second_string = "0" + (String)gps_second;
            else
                second_string = (String)gps_second;
            
            gps_time_spr.printToSprite(" " + (String)gnss.date.month() + "/" + (String)gnss.date.day() + "/" + (String)gnss.date.year() + " "
                                       + (String)gnss.time.hour() + ":" + minute_string + ":" + second_string + " UTC");
    
        }
        // Print to screen updated GNSS information
        if (new_gnss_data)
        {

            new_gnss_data = false;

            // GPS position sprite setup
            pos_spr.unloadFont();
            pos_spr.setColorDepth(16);
            pos_spr.loadFont(AA_FONT_MED, LittleFS);
            pos_spr.setTextColor(TFT_BLUE, TFT_WHITE);
            
            // ### Lattitude ###
            
            // Cursor position for printing lattitude
            tft.setCursor(43, 24);

            String lat_direction_symbol = gnss.location.rawLat().negative ? "-" : "+";
            String lat_str = String(gnss.location.rawLat().billionths);

            pos_spr.printToSprite(" " + lat_direction_symbol + (String)gnss.location.rawLat().deg + "."
                                  + lat_str.substring(0, 8) + " ");

            // ### Longitude ###
            
            // Cursor position for printing longitude
            tft.setCursor(48, 44);

            String lon_direction_symbol = gnss.location.rawLng().negative ? "-" : "+";
            String lon_str = String(gnss.location.rawLng().billionths);

            pos_spr.printToSprite(" " + lon_direction_symbol + (String)gnss.location.rawLng().deg + "."
                                  + lon_str.substring(0, 8) + " ");

            // ### Altitude ###
            
            // Cursor position for printing longitude
            tft.setCursor(48, 64);

            String alt_str = String(gnss.altitude.meters());

            pos_spr.printToSprite(" " + alt_str + " ");

            // ### Fix mode ###
            
            // Cursor position for printing RTK mode
            tft.setCursor(60, 84);
            
            String rtk_mode_str;
            uint8_t mode = atoi(gnss_quality.value());
            if (mode == 0)
                rtk_mode_str = "No Fix";
            else if (mode == 1)
                rtk_mode_str = "GPS Fix";
            else if (mode == 2)
                rtk_mode_str = "DGPS Fix";
            else if (mode == 4)
                rtk_mode_str = "RTK Fix";
            else if (mode == 5)
                rtk_mode_str = "RTK Float";
            else
                rtk_mode_str = "Unk Fix";

            pos_spr.printToSprite("   " + rtk_mode_str + "     ");

            // ### Fix mode ###
            
            // Cursor position for printing RTK ratio
            tft.setCursor(110, 104);
            pos_spr.printToSprite(" " + (String)gnss_data.rtk_ratio + " ");

            // ### RTK age ###
            
            // Cursor position for printing RTK age
            tft.setCursor(105, 124);
            pos_spr.printToSprite(" " + (String)gnss_data.rtk_age + " ");

            tft.setCursor(55, 146);
            pos_spr.printToSprite(" " + (String)radio->getRSSI() + " ");

            tft.setCursor(195, 146);
            pos_spr.printToSprite(" " + (String)radio->getSNR() + " ");



            //        Serial.print("RSSI = ");Serial.println(radio->getRSSI());
//        Serial.print("SNR = ");Serial.println(radio->getSNR());

            Serial.print("rtk_ratio: ");Serial.println(rtk_ratio);
            Serial.print("rtk_age: ");Serial.println(rtk_age);

        }

        // // Look for touchscreen touch
        // TSPoint p = ts.getPoint();
        
        // if (p.z > 250) 
        // {
        //    Serial.print(" X = "); Serial.print(getTouchX(p));
        //    Serial.print(" Y = "); Serial.print(getTouchY(p));
        //    Serial.print(" Pressure = "); Serial.println(p.z);

        //    // NMEA tab button pushed
        //    if ( (getTouchX(p)>3 && getTouchX(p) < 120) && (getTouchY(p)>210 && getTouchY(p)<235))
        //    {
        //        current_tab_index = 1;
        //        Serial.println("NMEA Tab Button Pushed");
        //    }
        // }  
    }
    
    tft.unloadFont();
    gps_time_spr.unloadFont();
    pos_spr.unloadFont();

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
//        Serial.print("RSSI = ");Serial.println(radio->getRSSI());
//        Serial.print("SNR = ");Serial.println(radio->getSNR());
//        Serial.print("Received radio packet, state: ");Serial.println(state);
        
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

uint32_t last_gnss_read_time = 0;

// Variables to hold the 13th and 14th fields as float
float field13 = 0.0;
float field14 = 0.0;

// Read NMEA data from the GNSS receiver and send to USB serial
void readAndSendNMEAData()
{
    
    String nmea_sentence = "";
    // Read latest GNSS data and send to parser
    if(Serial1.available()>0)
    {   
        last_gnss_read_time = millis();
        while(millis()-last_gnss_read_time < 10)
        {
            if(Serial1.available()>0)
            {
                char ch = Serial1.read();
                gnss.encode(ch);
                //Serial.write(ch);
                if(readNMEASentence(nmea_sentence,ch))
                {
                    if(nmea_sentence.startsWith("$PSTI,030"))
                    {
                        parseRTKSentence(nmea_sentence);
                        
                    }
                    nmea_sentence = "";
                }
                
                last_gnss_read_time = millis();
            }
        }
    }
    
    // If message is updated, then populate fields with GNSS data
    if (gnss.location.isUpdated())
    {
       
        // Lattitude, longitude, altitude values
        latitude = gnss.location.lat();
        longitude = gnss.location.lng();
        altitude = gnss.altitude.meters();
        gnss_quality_indicator = (int8_t)atoi(gnss_quality.value());
        
        // Read data and pack into structure
        gnss_data.time_stamp = micros();
        gnss_data.gnss_quality_indicator = gnss_quality_indicator;
        gnss_data.rtk_ratio = rtk_ratio;
        gnss_data.rtk_age = rtk_age;
        gnss_data.latitude = latitude;
        gnss_data.longitude = longitude;
        gnss_data.altitude = altitude;
        gnss_data.RSSI = RSSI;

        new_gnss_data = true;

        // NOTE: Sending data on the swSerial interface interferes with
        // correction data.
//        Serial.print("rtk_ratio: ");Serial.println(rtk_ratio);
//        Serial.print("rtk_age: ");Serial.println(rtk_age);
        last_read_time = micros();
    }
}

// Function to read the full NMEA sentence
bool readNMEASentence(String &sentence, char c) 
{
    if (c == '\n') 
    {
      sentence += c;
      return true;
    }
    sentence += c;
    return false;
}

// Function to parse the custom PSTI,030 message and extract 13th and 14th fields
bool parseRTKSentence(const String &nmea_sentence) 
{
    int comma_index = 0;
    String field;
  
    // Parse the RTKage and RTKratio fields
    for (int i = 0; i < 15; i++) 
    {
        comma_index = nmea_sentence.indexOf(',', comma_index + 1);
        if (comma_index == -1) 
        {
            return false;  // Return false if we run out of commas early
        }
    
        // RTK age
        if (i == 13) 
        {
            field = nmea_sentence.substring(comma_index + 1, nmea_sentence.indexOf(',', comma_index + 1));
            rtk_age = field.toFloat();

        }
    
        // RTK ratio
        if (i == 14) 
        {
            field = nmea_sentence.substring(comma_index + 1, nmea_sentence.indexOf(',', comma_index + 1));
            rtk_ratio = field.toFloat();
            return true;
        }
    }
  
    // If parsing failes, return false
    return false;
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

// ##############################################################################################
// Call this function to scroll the display one text line
// ##############################################################################################
int scroll_line() {
  int yTemp = yStart; // Store the old yStart, this is where we draw the next line
  // Use the record of line lengths to optimise the rectangle size we need to erase the top line
  tft.fillRect(0,yStart,blank[(yStart-TOP_FIXED_AREA)/TEXT_HEIGHT],TEXT_HEIGHT, TFT_BLACK);

  // Change the top of the scroll area
  yStart+=TEXT_HEIGHT;
  // The value must wrap around as the screen memory is a circular buffer
  if (yStart >= YMAX - BOT_FIXED_AREA) yStart = TOP_FIXED_AREA + (yStart - YMAX + BOT_FIXED_AREA);
  // Now we can scroll the display
  scrollAddress(yStart);
  return  yTemp;
}

// ##############################################################################################
// Setup a portion of the screen for vertical scrolling
// ##############################################################################################
// We are using a hardware feature of the display, so we can only scroll in portrait orientation
void setupScrollArea(uint16_t tfa, uint16_t bfa) {
  tft.writecommand(ILI9341_VSCRDEF); // Vertical scroll definition
  tft.writedata(tfa >> 8);           // Top Fixed Area line count
  tft.writedata(tfa);
  tft.writedata((YMAX-tfa-bfa)>>8);  // Vertical Scrolling Area line count
  tft.writedata(YMAX-tfa-bfa);
  tft.writedata(bfa >> 8);           // Bottom Fixed Area line count
  tft.writedata(bfa);
}

// ##############################################################################################
// Setup the vertical scrolling start address pointer
// ##############################################################################################
void scrollAddress(uint16_t vsp) {
  tft.writecommand(ILI9341_VSCRSADD); // Vertical scrolling pointer
  tft.writedata(vsp>>8);
  tft.writedata(vsp);
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
