/**
 * Firmware for ESP32 running on the basestation with correction data sent directly over LoRa.
 * This firmware displays information about the RTK solution and data used on a webpage,
 * which is available on the ESP32's IP address (printed to serial at startup).
 * Copyright Tinkerbug Robotics 2023
 * Provided under GNU GPL 3.0 License
 */

// Libraries
#include <map> // Standard C++ library
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SoftwareSerial.h"
#include "SerialTransfer.h"
#include <TimeLib.h>
#include <Adafruit_NeoPixel.h>
#include <esp_task_wdt.h>
#include "ParseRTCM.h"
#include <math.h>

// Local files
#include "inputs.h"
#include "homepage.h"
#include "tinkercharge.h"
#include "rtk.h"
#include "map.h"
#include "gnss.h"

// ESP32 watch dog timer (s)
#define WDT_TIMEOUT 10

// RTCM parsing variables
PARSERTCM rtcm_parser;
char rtcm_data[2500];
String rtk_rec_mode = "Base Station";
bool data_available = false;
unsigned int data_length = 0;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// LED indicator
#define NEO_PIN 4
Adafruit_NeoPixel pixels(1, NEO_PIN, NEO_GRB + NEO_KHZ800);

// Software serial connection for TinkerNav data
SoftwareSerial tinkernav_serial;

// Library and structure for transfering data from RP2040 processor
SerialTransfer transferFromNav;

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

double latitude;
double longitude;

// Base station survey
bool survey_complete = false;
String survey_complete_string = "Incomplete";

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
    //while (!Serial){}; // Pauses till serial starts. TODO: Remove if running apart from computer
    
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

    // Web pages served by ESP32

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

      if (latitude != 0.0 && longitude != 0.0)
      {
          char lat[16];
          dtostrf(latitude,12, 8, lat);
          char lng[16];
          dtostrf(longitude,12, 8, lng);
  
          strcpy(lat_lng, lat);
          strcat(lat_lng, ",");
          strcat(lat_lng, lng);
      }

      request->send_P(200, "text/plain", lat_lng);
    });
    
    // Handle Web Server Events
    events.onConnect([](AsyncEventSourceClient *client)
    {
        if(client->lastId())
        {
          Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
        }
        client->send("reconnect", NULL, millis(), 1000);
    });
    server.addHandler(&events);
    server.begin();
}

void loop() 
{

    // Tell the watch dog timer the thread is still alive
    // so that the hardware doesn't reset
    esp_task_wdt_reset();

    // Receive serial data from TinkerCharge via RP2040 if available
    if (transferFromNav.available())
    {
        uint16_t recSize = 0;
        recSize = transferFromNav.rxObj(data_for_tinker_send, recSize);
    }

    // Read and parse latest data from GNSS receiver and send to parser
    readSerialBuffer();

    // Set indicator based on status of lat/long report
    if (data_available)
    {

        // Parse RTCM data
        rtcm_parser.ReadData(rtcm_data,data_length);

        static double ecef_rss_last = 0;
        double ecef_rss = sqrt(rtcm_parser.data_struct.ecef[0]*rtcm_parser.data_struct.ecef[0] +
                               rtcm_parser.data_struct.ecef[1]*rtcm_parser.data_struct.ecef[1] +
                               rtcm_parser.data_struct.ecef[2]*rtcm_parser.data_struct.ecef[2]);

        // Latitude and Longitude reported and did not vary since last report
        if(ecef_rss > 1 && fabs(ecef_rss_last-ecef_rss) < 0.001)
        {
            latitude = rtcm_parser.getLatitude();
            longitude = rtcm_parser.getLongitude();
            
            survey_complete = true;
            survey_complete_string = "Complete";
            pixels.setPixelColor(0, pixels.Color(0, 0, 50));
            pixels.show();
        }
        // First latitude and longitude values available, but not stable
        else if (ecef_rss > 1)
        {

            latitude = rtcm_parser.getLatitude();
            longitude = rtcm_parser.getLongitude();
            
            survey_complete = false;
            survey_complete_string = "Incomplete";
            pixels.setPixelColor(0, pixels.Color(0, 50, 0));
            pixels.show();
        }
        else
        {
            // No position data is available
            survey_complete = false;
            survey_complete_string = "Incomplete";
            pixels.setPixelColor(0, pixels.Color(50, 0, 0));
            pixels.show();
        }

        ecef_rss_last = ecef_rss;
    }


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
        events.send(survey_complete_string.c_str(),"survey_complete_string",millis());
        events.send(String(latitude).c_str(),"lat",millis());
        events.send(String(longitude).c_str(),"lng",millis());

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

// Read serial buffer into a large array and send to parser
void readSerialBuffer ()
{

    // Position through current packet
    int input_pos = 0;

    // Indicates that new data is available
    data_available = false;

    // Serial character
    byte in_byte;

    // Last time data was read
    unsigned long last_read_time = 0;

    // Read a burst of GNSS data, the PX11XX receivers send RTCM
    // data in bursts of messages with hundreds of milliseconds
    // between each burst
    last_read_time = millis();

    // If serial data is still available or data was recently received
    // This loops through all data in a burst, bursts are 1000 milliseconds apart and take <<500 milliseconds
    while (Serial1.available () || (millis() - last_read_time) < 50)
    {
        if (Serial1.available ())
        {
            // Read data from serial
            in_byte = Serial1.read();

            rtcm_data[input_pos] = in_byte;

            //Serial.println(in_byte, HEX);
            input_pos++;
            last_read_time = millis();
            data_available = true;
        }
    }

    data_length = input_pos;

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
    if(var == "UP_TIME")
    {
      return String(millis()/1000.0/60);
    }
    if(var == "SURV_STRING")
    {
      return survey_complete_string;
    }

    return String();
}

String init_location(const String& var)
{

    if(var == "LATITUDE")
    {
      return String(latitude);
    }
    else if(var == "LONGITUDE")
    {
      return String(longitude);
    }

    return String();
}
