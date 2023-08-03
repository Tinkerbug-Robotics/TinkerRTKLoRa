/**
 * Firmware for RP2040 running on the base station to send correction data directly to a rover over 
 * a LoRa network using the TinkerSend - LoRa radio (LLCC68 based).
 * This firmware reads RTCM correction data from the PX112X GNSS receiver, packages it
 * into LoRa sized packets, and sends the data over LoRa using the TinkerSend radio.
 * Copyright Tinkerbug Robotics 2023
 * Provided under GNU GPL 3.0 License
 */

// include the library
#include <RadioLib.h>

// RP2040 pins for LoRa radio
#define L_DIO1 6 //28
#define L_DIO2 23 //16
#define L_TXEN 22 //22
#define L_RXEN 21 // 21
#define L_SS   9 // 13
#define L_BUSY 2  // 4
#define L_RST  5  //5
#define L_MISO 8 //8
#define L_MOSI 11 //11
#define L_SCK  10 //10

// LoRa radio instance
Module* mod;
LLCC68* radio;

// Create SPI instance for the LoRa radio pins
arduino::MbedSPI SPI1(L_MISO, L_MOSI, L_SCK);

// Max number of bytes in a packet
#define MAX_PACKET_LENGTH 250

// Save transmission state between loops
int transmission_state = RADIOLIB_ERR_NONE;

// Hardware serial connection to GNSS receiver on GPIO pins
UART Serial2(0, 1, 0, 0);

// A character array for one block of GNSS data
uint8_t rtcm_data[2500];

// Flag and counter for reading RTCM correction data
bool data_avail = false;
int data_length = 0;

// Flag to indicate that a packet was sent
volatile bool transmitted_flag = false;

void setup() 
{
    Serial.begin(115200);
    //while (!Serial){}; // Pauses till serial starts. TODO: Remove if running apart from computer

    // Start SPI for LoRa radio
    SPI1.begin();
    
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

    // External RF switches controlled by (RX enable, TX enable)
    //radio->setRfSwitchPins(L_RXEN, L_TXEN);
  
    // start transmitting the first packet
    Serial.print(F("Sending first packet ... "));
  
    // Transmit initial dummy data
    uint8_t byteArr[250] = {0x00};
    transmission_state = radio->startTransmit(byteArr,250);

    // Start serial connection to GNSS receiver
    Serial2.begin(115200);

    Serial.print("... LLCC68 initialization complete");
}

void loop() 
{
    // Read serial buffer and store in a large array
    if (Serial2.available())
    {
        readSerialBuffer();
    }
    // If there is complete data to transmit and the previous transmission is 
    // complete, then break up the data into LoRa packets and transmit over radio
    if (data_avail && transmitted_flag)
    {
        decomponseAndSendData();
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
    while (Serial2.available () || (millis() - last_read_time) < 50)
    {
        if (Serial2.available ())
        {
            // Read data from serial
            in_byte = Serial2.read();

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
            //Serial.print(rtcm_data[i], HEX);Serial.print(" ");
            //Serial.print(data_length);Serial.print(" ");Serial.print(i);Serial.print(" "); 
            //Serial.println(data_counter);
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
