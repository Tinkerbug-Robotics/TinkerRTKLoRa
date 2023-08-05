/**
 * Firmware for ESP32 running on the rover with correction data sent directly over LoRa.
 * This firmware displays information about the RTK solution and data used on a webpage,
 * which is available on the ESP32's IP address (printed to serial at startup).
 * Copyright Tinkerbug Robotics 2023
 * Provided under GNU GPL 3.0 License
 */


// Libraries
#include <map> // Standard C++ library
#include <WiFi.h>
#include <TinyGPS++.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SoftwareSerial.h"
#include "SerialTransfer.h"
#include <TimeLib.h>
#include <Adafruit_NeoPixel.h>
#include <esp_task_wdt.h>

// Local files
#include "inputs.h"
#include "homepage.h"
#include "tinkercharge.h"
#include "rtk.h"
#include "map.h"
#include "gnss.h"

// ESP32 watch dog timer (s)
#define WDT_TIMEOUT 10

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// TCP client for connecting to base station ESP32
WiFiClient tcp_client;

// Max length of an RTCM data burst
#define MAX_SERIAL_LENGTH 2500

#define NEO_PIN 4
Adafruit_NeoPixel pixels(1, NEO_PIN, NEO_GRB + NEO_KHZ800);

// Software serial connection for TinkerNav data
SoftwareSerial tinkernav_serial;

// GNSS parsing
String gps_quality_text = "";
float rtk_age = 0;
int num_cycle_slip_gps = 0;
int num_cycle_slip_bds = 0;
int num_cycle_slip_gal = 0;
float rtk_east = 0.0;
float rtk_north = 0.0;
float rtk_up = 0.0;
String rtk_rec_mode = "Rover";
float rtk_ratio = 0;
float lattitude = 0.0;
float longitude = 0.0;
String date_string = "";
String time_string = "";
int SAT_TIME_OUT = 5;

// Data stored for active satellites
struct SatData
{
    unsigned long update_time;
    bool active;
    int elevation;
    int azimuth;
    int snr;
    String constellation;
} sat_data;

// Maps to contain satellite data for each supported constellation
std::map<int, SatData> gps_sat_map;
std::map<int, SatData> gal_sat_map;
std::map<int, SatData> bei_sat_map;

// GNSS receiver data is parsed by the TinyGPSPlus library
TinyGPSPlus gnss;

TinyGPSCustom gnss_quality(gnss, "GPGGA", 6);
TinyGPSCustom psti_1(gnss, "PSTI", 1);
TinyGPSCustom psti_5(gnss, "PSTI", 5);
TinyGPSCustom psti_6(gnss, "PSTI", 6);
TinyGPSCustom psti_7(gnss, "PSTI", 7);
TinyGPSCustom psti_8(gnss, "PSTI", 8);
TinyGPSCustom psti_13(gnss, "PSTI", 13);
TinyGPSCustom psti_14(gnss, "PSTI", 14);
TinyGPSCustom psti_15(gnss, "PSTI", 15);
TinyGPSCustom psti_18(gnss, "PSTI", 18);

// Tiny GPS objects for GPGSV messages, GPS satellites in view
TinyGPSCustom gps_tot_GPGSV_msgs(gnss, "GPGSV", 1);
TinyGPSCustom gps_msg_num(gnss, "GPGSV", 2);
TinyGPSCustom gps_sats_in_view(gnss, "GPGSV", 3);
TinyGPSCustom gps_sat_num[4];
TinyGPSCustom gps_elevation[4];
TinyGPSCustom gps_azimuth[4];
TinyGPSCustom gps_snr[4];

// Tiny GPS objects for GAGSV messages, Galileo satellites in view
TinyGPSCustom gal_tot_GPGSV_msgs(gnss, "GAGSV", 1);
TinyGPSCustom gal_msg_num(gnss, "GAGSV", 2);
TinyGPSCustom gal_sats_in_view(gnss, "GAGSV", 3);
TinyGPSCustom gal_sat_num[4];
TinyGPSCustom gal_elevation[4];
TinyGPSCustom gal_azimuth[4];
TinyGPSCustom gal_snr[4];

// Tiny GPS objects for GBGSV messages, BeiDou satellites in view
TinyGPSCustom bei_tot_GPGSV_msgs(gnss, "GBGSV", 1);
TinyGPSCustom bei_msg_num(gnss, "GBGSV", 2);
TinyGPSCustom bei_sats_in_view(gnss, "GBGSV", 3);
TinyGPSCustom bei_sat_num[4];
TinyGPSCustom bei_elevation[4];
TinyGPSCustom bei_azimuth[4];
TinyGPSCustom bei_snr[4];

// Library and structure for transfering data from RP2040 processor
SerialTransfer transferFromNav;

unsigned long next_connection_attempt = 0;
int connection_attempt_period = 1000;

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
} data_for_tinker_send;

// Update period for updating card based webpages
unsigned long next_update = 0;
int update_period = 1000;

// Called once on startup
void setup() 
{
    // Enable ESP32 watch dog timer and add the curent thread to it
    // The watch dog timer resets the ESP32 if there are any unhandled
    // exceptions that cause the ESP32 to hang.
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);
    
    // USB serial connection
    Serial.begin(115200);
    Serial.println("Serial Open");
    
    // Initialize NeoPixel
    pixels.begin();
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(50, 0, 0));
    pixels.show();

    // Connect to WiFi network
    Serial.print("Connecting to WiFi .");
    connectWiFi();

    // Software serial connection to RP2040
    tinkernav_serial.begin(9600, SWSERIAL_8N1, 0, 1);
    transferFromNav.begin(tinkernav_serial);

    // GNSS hardware serial connection
    Serial1.begin(115200, SERIAL_8N1, 21, 20);

    // Initialize TinyGPSCustom objects for GPGSV messages
    for (int i=0; i<4; ++i)
    {
        gps_sat_num[i].begin(  gnss, "GPGSV", 4 + 4 * i); // offsets 4, 8, 12, 16
        gps_elevation[i].begin(gnss, "GPGSV", 5 + 4 * i); // offsets 5, 9, 13, 17
        gps_azimuth[i].begin(  gnss, "GPGSV", 6 + 4 * i); // offsets 6, 10, 14, 18
        gps_snr[i].begin(      gnss, "GPGSV", 7 + 4 * i); // offsets 7, 11, 15, 19
    }

    // Initialize TinyGPSCustom objects for GAGSV messages
    for (int i=0; i<4; ++i)
    {
        gal_sat_num[i].begin(  gnss, "GAGSV", 4 + 4 * i); // offsets 4, 8, 12, 16
        gal_elevation[i].begin(gnss, "GAGSV", 5 + 4 * i); // offsets 5, 9, 13, 17
        gal_azimuth[i].begin(  gnss, "GAGSV", 6 + 4 * i); // offsets 6, 10, 14, 18
        gal_snr[i].begin(      gnss, "GAGSV", 7 + 4 * i); // offsets 7, 11, 15, 19
    }

    // Initialize TinyGPSCustom objects for GBGSV messages
    for (int i=0; i<4; ++i)
    {
        bei_sat_num[i].begin(  gnss, "GBGSV", 4 + 4 * i); // offsets 4, 8, 12, 16
        bei_elevation[i].begin(gnss, "GBGSV", 5 + 4 * i); // offsets 5, 9, 13, 17
        bei_azimuth[i].begin(  gnss, "GBGSV", 6 + 4 * i); // offsets 6, 10, 14, 18
        bei_snr[i].begin(      gnss, "GBGSV", 7 + 4 * i); // offsets 7, 11, 15, 19
    }

    // ### Web pages served by ESP32 ###
    
    // Home page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
    {
        Serial.println("Handle Home Page");
        request->send_P(200, "text/html", home_html);
    });

    // TinkerCharge data page
    server.on("/tinkercharge", HTTP_GET, [](AsyncWebServerRequest *request)
    {
        Serial.println("Handle TinkerCharge");
        request->send_P(200, "text/html", tc_html, init_tinkercharge);
    });

    // RTK data page
    server.on("/rtk", HTTP_GET, [](AsyncWebServerRequest *request)
    {
        Serial.println("Handle RTK");
        request->send_P(200, "text/html", rtk_html, init_rtk);
    });

    // GNSS data page
    server.on("/gnss", HTTP_GET, [](AsyncWebServerRequest *request)
    {
        Serial.println("Handle GNSS");
        request->send_P(200, "text/html", gnss_html, init_location);
    });

    // Map page
    server.on("/map", HTTP_GET, [](AsyncWebServerRequest *request)
    {
        Serial.println("Handle map");
        request->send_P(200, "text/html", map_html, init_location);
    });

    // Update location, called periodically to update position
    server.on("/loc",  HTTP_GET, [](AsyncWebServerRequest *request)
    {

      char lat_lng[32] = "39.2815074046,-74.558350235";
      Serial.println("Update lat/lon for map");

      if (lattitude != 0.0 && longitude != 0.0)
      {
          char lat[16];
          dtostrf(lattitude,12, 8, lat);
          char lng[16];
          dtostrf(longitude,12, 8, lng);
  
          strcpy(lat_lng, lat);
          strcat(lat_lng, ",");
          strcat(lat_lng, lng);
      }

      request->send_P(200, "text/plain", lat_lng);
    });

    // Table of detected satellites
    server.on("/sat_table",  HTTP_GET, [](AsyncWebServerRequest *request)
    {

        String sat_table = "<!DOCTYPE HTML><html>\n";
        sat_table += "<head>\n";
        sat_table += "<meta http-equiv=\'refresh\' content=\'5\'>\n";
        sat_table += "<title>Sensor Data Table</title>\n";
        sat_table += "<style>\n";
        sat_table += "table {\n";
        sat_table += "border-collapse: collapse;\n";
        sat_table += "width: 100%;\n";
        sat_table += "}\n";
        sat_table += "th, td {\n";
        sat_table += "border: 1px solid black;\n";
        sat_table += "padding: 8px;\n";
        sat_table += "text-align: left;\n";
        sat_table += "}\n";
        sat_table += "th {\n";
        sat_table += "background-color: #f2f2f2;\n";
        sat_table += "}\n";
        sat_table += "</style>\n";
        sat_table += "</head>\n";
        sat_table += "<body>\n";
        sat_table += "<h1>TinkerRTK Satellite Data</h1>\n";
        sat_table += "<p><a href='/'> Home</a></p>\n";
        sat_table += "<table id='sat_table'>\n";
        sat_table += "<tr>\n";
        sat_table += "<th>Constellation</th>\n";
        sat_table += "<th>PRN</th>\n";
        sat_table += "<th>Az</th>\n";
        sat_table += "<th>El</th>\n";
        sat_table += "<th>SNR</th>\n";
        sat_table += "</tr>\n";

        for(auto it = gps_sat_map.begin(); it != gps_sat_map.end(); it++)
        {
            sat_table += "<tr>";
            sat_table += "<td>" + String(it->second.constellation) + "</td>\n";
            sat_table += "<td>" + String(it->first) + "</td>\n";
            sat_table += "<td>" + String(it->second.azimuth) + "</td>\n";
            sat_table += "<td>" + String(it->second.elevation) + "</td>\n";
            sat_table += "<td>" + String(it->second.snr) + "</td>\n";
            sat_table += "</tr>\n";
        }

        for(auto it = gal_sat_map.begin(); it != gal_sat_map.end(); it++)
        {
            sat_table += "<tr>";
            sat_table += "<td>" + String(it->second.constellation) + "</td>\n";
            sat_table += "<td>" + String(it->first) + "</td>\n";
            sat_table += "<td>" + String(it->second.azimuth) + "</td>\n";
            sat_table += "<td>" + String(it->second.elevation) + "</td>\n";
            sat_table += "<td>" + String(it->second.snr) + "</td>\n";
            sat_table += "</tr>\n";
        }
        
        for(auto it = bei_sat_map.begin(); it != bei_sat_map.end(); it++)
        {
            sat_table += "<tr>";
            sat_table += "<td>" + String(it->second.constellation) + "</td>\n";
            sat_table += "<td>" + String(it->first) + "</td>\n";
            sat_table += "<td>" + String(it->second.azimuth) + "</td>\n";
            sat_table += "<td>" + String(it->second.elevation) + "</td>\n";
            sat_table += "<td>" + String(it->second.snr) + "</td>\n";
            sat_table += "</tr>\n";
        }
        sat_table += "</table>\n";
        
        request->send_P(200, "text/html", sat_table.c_str());
    });
    
    // Handle Web Server Events
    events.onConnect([](AsyncEventSourceClient *client)
    {
        if(client->lastId())
        {
          Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
        }
        client->send("hello!", NULL, millis(), 1000);
    });
    server.addHandler(&events);
    server.begin();
  
}

// Repeating loop
void loop() 
{

    // Tell the watch dog timer the thread is still alive
    // so that the hardware doesn't reset
    esp_task_wdt_reset();

    // Receive serial data from TinkerCharge via RP2040
    if (transferFromNav.available())
    {
        uint16_t recSize = 0;
        recSize = transferFromNav.rxObj(data_for_tinker_send, recSize);
    }

    // Read and parse latest data from GNSS receiver
    readAndParseGNSS();

    // Update webpages
    if (millis() > next_update) 
    {

        // Check connection to WiFi and reconnect if needed
        connectWiFi();
 
        // Send Events to update webpages
        events.send("ping",NULL,millis());
        events.send(String(data_for_tinker_send.voltage).c_str(),"voltage",millis());
        events.send(String(data_for_tinker_send.avg_voltage).c_str(),"avg_voltage",millis());
        events.send(String(data_for_tinker_send.current).c_str(),"current",millis());
        events.send(String(data_for_tinker_send.avg_current).c_str(),"avg_current",millis());
        events.send(String(data_for_tinker_send.battery_capacity).c_str(),"battery_capacity",millis());
        events.send(String(data_for_tinker_send.SOC).c_str(),"battery_soc",millis());
        events.send(String(data_for_tinker_send.temperature).c_str(),"tc_temp",millis());
        events.send(String(millis()/1000.0/60.0).c_str(),"up_time",millis());
        
        events.send(String(lattitude).c_str(),"lat",millis());
        events.send(String(longitude).c_str(),"lng",millis());
        
        events.send(date_string.c_str(),"gnss_date",millis());
        events.send(time_string.c_str(),"gnss_time",millis());
        events.send(gps_quality_text.c_str(),"fix",millis());
        events.send(String(rtk_age).c_str(),"rtk_age",millis());
        events.send(String(rtk_ratio).c_str(),"rtk_ratio",millis());
        events.send(rtk_rec_mode.c_str(),"rtk_mode",millis());
        events.send(String(num_cycle_slip_gps).c_str(),"cs_gps",millis());
        events.send(String(num_cycle_slip_bds).c_str(),"cs_bds",millis());
        events.send(String(num_cycle_slip_gal).c_str(),"cs_gal",millis());
        events.send(String(rtk_east).c_str(),"rtk_east",millis());
        events.send(String(rtk_north).c_str(),"rtk_north",millis());
        events.send(String(rtk_up).c_str(),"rtk_up",millis());

        next_update = millis() + update_period;

    }
}

// Connect to WiFi
void connectWiFi() 
{
    int try_count = 0;
    while ( WiFi.status() != WL_CONNECTED )
    {
        try_count++;
        WiFi.disconnect();
        WiFi.mode(WIFI_STA);
        WiFi.begin( ssid, password );
        Serial.print(".");
        if ( try_count == 10 )
        {
          ESP.restart();
        }
        Serial.print('.');
        delay(1000);
        if(WiFi.status() == WL_CONNECTED)
        {
            Serial.print(" IP Address: ");Serial.println(WiFi.localIP());
            Serial.print("The MAC address for this board is: ");Serial.println(WiFi.macAddress());
        }
    }
}

// Read and parse data from GNSS receiver
void readAndParseGNSS()
{

    // Read latest GNSS data and send to parser
    while(Serial1.available() > 0)
    {
        char ch = Serial1.read();
        gnss.encode(ch);
        //Serial.write(ch);
    }

    // If message is updated, then populate fields with GNSS data
    if (gnss_quality.isUpdated())
    {

        // Set system time
        setTime(gnss.time.hour(),gnss.time.minute(),gnss.time.second(),gnss.date.day(),gnss.date.month(),gnss.date.year());
        
         // Parse and construct strings for time and date
        date_string = String(gnss.date.month()) + "/" + String(gnss.date.day()) + "/" + String(gnss.date.year());
        String gnss_minute;
        if ((int)gnss.time.minute() < 10)
            gnss_minute = "0" + String(gnss.time.minute());
        else
            gnss_minute = String(gnss.time.minute());
            
        String gnss_second;
        if ((int)gnss.time.second() < 10)
            gnss_second = "0" + String((int)gnss.time.second());
        else
            gnss_second = String((int)gnss.time.second());
            
        time_string = String(gnss.time.hour()) + ":" + gnss_minute + ":" + gnss_second;

        // Clear NeoPixel
        pixels.clear();

        // Convert GNSS Fix type to string and set NeoPixel color
        if(atoi(gnss_quality.value())==0)
        {
           gps_quality_text = "Invalid";
           pixels.setPixelColor(0, pixels.Color(50, 0, 0));
        }
        else if(atoi(gnss_quality.value())==1)
        {
           gps_quality_text = "GPS";
           pixels.setPixelColor(0, pixels.Color(50, 50, 0));
        }
        else if(atoi(gnss_quality.value())==2)
        {
           gps_quality_text = "DGPS";
           pixels.setPixelColor(0, pixels.Color(50, 50, 0));
        }
        else if(atoi(gnss_quality.value())==4)
        {
           gps_quality_text = "RTK Fix";
           pixels.setPixelColor(0, pixels.Color(0, 0, 50));
        }
        else if(atoi(gnss_quality.value())==5)
        {
           gps_quality_text = "RTK Float";
           pixels.setPixelColor(0, pixels.Color(0, 50, 0));
        }
        else
        {
           gps_quality_text = "NA";
           pixels.setPixelColor(0, pixels.Color(50, 0, 0));
        }
           
        pixels.show();
    
        // Lattitude and longitude values
        lattitude = gnss.location.lat();
        longitude = gnss.location.lng();

    }
    if (psti_1.isUpdated())
    {
        if(strcmp(psti_1.value(),"030") == 0)
        {
            // Age of correction data if present
            rtk_age = atof(psti_14.value());
            rtk_ratio = atof(psti_15.value());
        }
        else if (strcmp(psti_1.value(),"032") == 0)
        {
            rtk_east = atof(psti_6.value());
            rtk_north = atof(psti_7.value());
            rtk_up = atof(psti_8.value());
        }
        else if (strcmp(psti_1.value(),"033") == 0)
        {
            num_cycle_slip_gps = atoi(psti_6.value());
            num_cycle_slip_bds = atoi(psti_7.value());
            num_cycle_slip_gal = atoi(psti_8.value());
        }
    }

    // Parse GPS satellite in view data
    static uint8_t gps_total_sats;
    static uint8_t gps_last_msg_num;

    if (gps_tot_GPGSV_msgs.isUpdated())
    {
        uint8_t msg_num = atoi(gps_msg_num.value());

        bool valid_msg_order = false;
        if (msg_num==1)
        {
            gps_total_sats = atoi(gps_sats_in_view.value());
            valid_msg_order = true;
        } 
        else if (msg_num == gps_last_msg_num + 1)
        {
            valid_msg_order = true;
            gps_last_msg_num = msg_num;
        }

        // Do not process after an invalid message is received in message series
        if (valid_msg_order)
        {
            uint8_t num_sats = 4;
            if( gps_total_sats < num_sats )
                num_sats = gps_total_sats;
    
            for (int i=0; i<num_sats; ++i)
            {

                int sat_num = atoi(gps_sat_num[i].value());

                // Insert or update satellite
                std::map<int,SatData>::iterator iter;

                iter = gps_sat_map.find(sat_num);

                // If the satellite already exists in the map, then update
                if(gps_sat_map.find(sat_num)!=gps_sat_map.end())
                {
                    iter->second.update_time = now();
                    iter->second.elevation = atoi(gps_elevation[i].value());
                    iter->second.azimuth = atoi(gps_azimuth[i].value());
                    iter->second.snr = atoi(gps_snr[i].value());                    
                }
                // If the satellite is new, then insert a new record with current data
                else
                {
                    SatData gps_sat;
                    gps_sat.update_time = now();
                    gps_sat.elevation = atoi(gps_elevation[i].value());
                    gps_sat.azimuth = atoi(gps_azimuth[i].value());
                    gps_sat.snr = atoi(gps_snr[i].value());
                    gps_sat.constellation = "GPS";
                    gps_sat_map.insert({sat_num , gps_sat });
                }
                
                gps_total_sats--;
    
            }
        }
        
        // Clean out satellites that have not been updated in past time_out seconds
        for (auto it = gps_sat_map.begin(); it != gps_sat_map.end(); it++) 
        {
            if(it->second.update_time < now()-SAT_TIME_OUT)
            {
                gps_sat_map.erase(it->first);
            }
        }
    }

    // Parse Galileo satellite in view data
    static uint8_t gal_total_sats;
    static uint8_t gal_last_msg_num;
    // Last in series of GAGSV messages received
    if (gal_tot_GPGSV_msgs.isUpdated())
    {
        uint8_t msg_num = atoi(gal_msg_num.value());

        bool valid_msg_order = false;
        if (msg_num==1)
        {
            gal_total_sats = atoi(gal_sats_in_view.value());
            valid_msg_order = true;
            gal_last_msg_num = 1;
        } 
        else if (msg_num == gal_last_msg_num + 1)
        {
            valid_msg_order = true;
            gal_last_msg_num = msg_num;
        }

        // Do not process after an invalid message is received in message series
        if (valid_msg_order)
        {
            uint8_t num_sats = 4;
            if( gal_total_sats < num_sats )
                num_sats = gal_total_sats;
                    
            for (int i=0; i<num_sats; ++i)
            {

                int sat_num = atoi(gal_sat_num[i].value());

                // Insert or update satellite
                std::map<int,SatData>::iterator iter;

                iter = gal_sat_map.find(sat_num);

                // If the satellite already exists in the map, then update
                if(gal_sat_map.find(sat_num)!=gal_sat_map.end())
                {
                    iter->second.update_time = now();
                    iter->second.elevation = atoi(gal_elevation[i].value());
                    iter->second.azimuth = atoi(gal_azimuth[i].value());
                    iter->second.snr = atoi(gal_snr[i].value());                    
                }
                // If the satellite is new, then insert a new record with current data
                else
                {
                    SatData gal_sat;
                    gal_sat.update_time = now();
                    gal_sat.elevation = atoi(gal_elevation[i].value());
                    gal_sat.azimuth = atoi(gal_azimuth[i].value());
                    gal_sat.snr = atoi(gal_snr[i].value());
                    gal_sat.constellation = "Galileo";
                    gal_sat_map.insert({sat_num , gal_sat });
                }
    
                gal_total_sats--;
    
            }
        }
        // Clean out satellites that have not be recently updated
        for (auto it = gal_sat_map.begin(); it != gal_sat_map.end(); it++) 
        {
            if(it->second.update_time < now()-SAT_TIME_OUT)
            {
                gal_sat_map.erase(it->first);
            }
        }
    }

    // Parse BeiDou satellite in view data
    static uint8_t bei_total_sats;
    static uint8_t bei_last_msg_num;
    // Last in series of GAGSV messages received
    if (bei_tot_GPGSV_msgs.isUpdated())
    {
        uint8_t msg_num = atoi(bei_msg_num.value());

        bool valid_msg_order = false;
        if (msg_num==1)
        {
            bei_total_sats = atoi(bei_sats_in_view.value());
            valid_msg_order = true;
            bei_last_msg_num = 1;
        } 
        else if (msg_num == bei_last_msg_num + 1)
        {
            valid_msg_order = true;
            bei_last_msg_num = msg_num;
        }

        // Do not process after an invalid message is received in message series
        if (valid_msg_order)
        {
            uint8_t num_sats = 4;
            if( bei_total_sats < num_sats )
                num_sats = bei_total_sats;
    
            for (int i=0; i<num_sats; ++i)
            {

                int sat_num = atoi(bei_sat_num[i].value());

                // Insert or update satellite
                std::map<int,SatData>::iterator iter;

                iter = bei_sat_map.find(sat_num);

                // If the satellite already exists in the map, then update
                if(bei_sat_map.find(sat_num)!=bei_sat_map.end())
                {
                    iter->second.update_time = now();
                    iter->second.elevation = atoi(bei_elevation[i].value());
                    iter->second.azimuth = atoi(bei_azimuth[i].value());
                    iter->second.snr = atoi(bei_snr[i].value());                    
                }
                // If the satellite is new, then insert a new record with current data
                else
                {
                    SatData bei_sat;
                    bei_sat.update_time = now();
                    bei_sat.elevation = atoi(bei_elevation[i].value());
                    bei_sat.azimuth = atoi(bei_azimuth[i].value());
                    bei_sat.snr = atoi(bei_snr[i].value());
                    bei_sat.constellation = "BeiDou";
                    bei_sat_map.insert({sat_num , bei_sat });
                }
    
                bei_total_sats--;
            }
        }

        // Clean out old satellites
        for (auto it = bei_sat_map.begin(); it != bei_sat_map.end(); it++) 
        {
            if(it->second.update_time < now()-SAT_TIME_OUT)
            {
                bei_sat_map.erase(it->first);
            }
        }
    }
}

// Populates initial webpage values on first load
String init_tinkercharge(const String& var)
{

    if(var == "VOLTAGE")
    {
      return String(data_for_tinker_send.voltage);
    }
    else if(var == "AVG_VOLTAGE")
    {
      return String(data_for_tinker_send.avg_voltage);
    }
    else if(var == "CURRENT")
    {
      return String(data_for_tinker_send.current);
    }
    else if(var == "AVG_CURRENT")
    {
      return String(data_for_tinker_send.avg_current);
    }
    else if(var == "BATT_CAPACITY")
    {
      return String(data_for_tinker_send.battery_capacity);
    }
    else if(var == "BATT_CHARGE")
    {
      return String(data_for_tinker_send.SOC);
    }
    else if(var == "TEMPERATURE")
    {
      return String(data_for_tinker_send.temperature);
    }
    else if(var == "UP_TIME")
    {
      return String(millis()/1000.0/60);
    }

    return String();
}

// Populates initial webpage values on first load
String init_rtk(const String& var)
{

    if(var == "RTK_MODE")
    {
      return rtk_rec_mode;
    }
    if(var == "GNSS_DATE")
    {
      return date_string;
    }
    if(var == "GNSS_TIME")
    {
      return time_string;
    }
    if(var == "UP_TIME")
    {
      return String(millis()/1000.0/60);
    }
    if(var == "FIX")
    {
      return String(gps_quality_text);
    }
    if(var == "RTK_AGE")
    {
      return String(rtk_age);
    }
    if(var == "RTK_RATIO")
    {
      return String(rtk_ratio);
    }
    if(var == "CS_GPS")
    {
      return String(num_cycle_slip_gps);
    }
    if(var == "CS_BDS")
    {
      return String(num_cycle_slip_bds);
    }
    if(var == "CS_GAL")
    {
      return String(num_cycle_slip_gal);
    }
    if(var == "RTK_EAST")
    {
      return String(rtk_east);
    }
    if(var == "RTK_NORTH")
    {
      return String(rtk_north);
    }
    if(var == "RTK_UP")
    {
      return String(rtk_up);
    }
    return String();
}

String init_location(const String& var)
{

    if(var == "LATTITUDE")
    {
      return String(lattitude);
    }
    else if(var == "LONGITUDE")
    {
      return String(longitude);
    }

    return String();
}
