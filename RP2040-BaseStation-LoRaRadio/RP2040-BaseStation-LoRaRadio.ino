/**
 * Firmware for RP2040 running on the base station which reads RTCM correction data from the 
 * GNSS receiver and sends it to the LoRa radio. This firmware also reads data from TinkerCharge 
 * and makes it available to the ESP32 to display on a webpage. Finally, the firmware also outputs GNSS serial data
 * to the RP2040 USB serial port.
 * !!! Note this must be compiled with the Earle Philhower RP2040 board set !!!
 * This is becuase it uses software serial to communicate to the ESP32. Hardware serial is
 * used for the more time critical GNSS communications.
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
volatile bool transmitted_flag = false;

void setup() 
{

    // Pico USB Serial
    Serial.begin(115200);
    // Pauses till serial starts. Do not use when running without a computer attached
    // or it will pause indefinetly
    //while (!Serial){};

    // GNSS input/output
    // Uses default 0,1 (TX, RX) pins
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

    // External RF switches controlled by (RX enable, TX enable)
    //radio->setRfSwitchPins(L_RXEN, L_TXEN);
  
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

    // Read serial buffer and store in a large array
    if (Serial1.available())
    {
        readSerialBuffer();
    }

    // If there is complete data to transmit and the previous transmission is 
    // complete, then break up the data into LoRa packets and transmit over radio
    if (data_avail && transmitted_flag)
    {
        decomponseAndSendData();
    }

    unsigned long current_time = millis();

    // Periodically write SOC data to ESP32
    if (current_time > last_soc_time + soc_periodic || last_soc_time > current_time)
    { 
        Serial.println(millis());
        readAndSendSOC();
        last_soc_time = current_time;
    }

}

// Read serial buffer into a large array
void readSerialBuffer ()
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
            data_avail = true;
        }
    }

    data_length = input_pos;
    Serial.print("Read new data: data_length = ");Serial.println(data_length);
}

// Transmit available data over radio
void decomponseAndSendData ()
{

    int data_counter = 0;
    uint8_t rtcm_data_to_send[MAX_PACKET_LENGTH];

    // Loop over all data and break into LoRa sized packets if needed
    for (int i=0;i<data_length;i++)
    {
        // Store data into an array to send
        if (data_counter <= 250)
        {
            rtcm_data_to_send[data_counter] = rtcm_data[i];
            data_counter++;
        }

        // Decide to send data
        if (data_counter == MAX_PACKET_LENGTH || i == data_length-1)
        {
            sendTransmission(rtcm_data_to_send, data_counter);
            data_counter = 0;
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
        Serial.println(F("transmission finished!"));
    } 
    else 
    {
        Serial.print(F("Failed, code "));Serial.println(transmission_state);
    }
    
    // Clean up after transmission is finished
    // this will ensure transmitter is disabled,
    // RF switch is powered down etc.
    radio->finishTransmit();

    // Start transmission of new data
    transmission_state = radio->startTransmit(data_to_send,data_length);
    Serial.print("Transmitting pack of length ");Serial.println(data_length);

    // Reset transmission flag
    transmitted_flag = false;
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

// Loop through valid baud rates and determine the current setting
bool autoSetBaudRate()
{
    // Start serial connections to send correction data to GNSS receiver
    // This loop will detect the current baud rate of the GNSS receiver
    // by sending a message and determining which buad rate returns a valid
    // ACK message
    int valid_baud_rates[9] = {4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600}; 
    
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
    for (int i=0;i<9;i++)
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
