

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include "mbedtls/aes.h"

#include <OneWire.h>
#include <DallasTemperature.h>

#include "SSD1306.h" 
#include "images.h"

#define SCK           5   // GPIO5  -- SX1278's SCK
#define MISO         19   // GPIO19 -- SX1278's MISO
#define MOSI         27   // GPIO27 -- SX1278's MOSI
#define SS           18   // GPIO18 -- SX1278's CS
#define RST          14   // GPIO14 -- SX1278's RESET
#define DI0          26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BLUE_LED      2   // GPIO2  -- System LED
#define OLED_SDA      4   // GPIO4  -- SSD1306 SDA
#define OLED_SCL     15   // GPIO15 -- SSD1306 SCL
#define OLED_RSTN    16   // GPIO16 -- SSD1306 RSTN
#define ONE_WIRE_BUS 17   // GPIO17 -- Data wire for temp sensor

#define BAND                  868E6 // EU band
#define TEMPERATURE_PRECISION     9 // DS18B20's max resolution is 9
#define COL_HEIGHT               12 // 10pt + 2pt spacing

unsigned int counter = 0;

SSD1306 display(0x3c, OLED_SDA, OLED_SCL);
String rssi = "RSSI --";
String packSize = "--";
String packet ;

mbedtls_aes_context aes;
char * key = "my-super-secret-passphrase";
char msg[17];
unsigned char buf[17];
boolean enable_aes = true;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

int numberOfDevices; // Number of temperature devices found

DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void setup() {
  pinMode(OLED_RSTN,OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  
  digitalWrite(OLED_RSTN, LOW);  // pull low to reset OLED
  delay(50); 
  digitalWrite(OLED_RSTN, HIGH); // remove OLED reset
  
  Serial.begin(115200);
  while (!Serial);
  Serial.println();
  Serial.println("UART started.");
  Serial.println();
  
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS, RST, DI0);
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  } else {
    Serial.println("LoRa started.");
  }

  // locate devices on the bus
  Serial.print("Locating 1-wire devices... ");
  
  // Start up the library
  sensors.begin();
  
  // Grab a count of devices on the wire
  numberOfDevices = sensors.getDeviceCount();
  
  Serial.print("found ");
  Serial.print(numberOfDevices, DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is "); 
  if (sensors.isParasitePowerMode()) 
    Serial.println("ON");
  else 
    Serial.println("OFF");
  
  // Loop through each device, print out address
  for(int i=0;i<numberOfDevices; i++)
  {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i))
    {
      if (sensors.validAddress(tempDeviceAddress) && (tempDeviceAddress[0] == DS18B20MODEL))  {
        Serial.print("Found device ");
        Serial.print(i, DEC);
        Serial.print(" with address: ");
        printAddress(tempDeviceAddress);
        Serial.println();
      } else {
        Serial.print("Skipping device at ");
        Serial.println(i, DEC);
      }
    }
  }
  
//  LoRa.onReceive(cbk);
//  LoRa.receive();

  Serial.println("init ok");
  display.init();
  display.flipScreenVertically();  
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  delay(1500);
}

void encrypt(unsigned char *msg) {
  if (enable_aes) {
    mbedtls_aes_init( &aes );
    mbedtls_aes_setkey_enc( &aes, (const unsigned char*) key, strlen(key) * 8 );
    mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, (const unsigned char*)msg, (unsigned char*)buf);
    mbedtls_aes_free( &aes );
    for (int i = 0; i < 16; i++) msg[i] = buf[i];
    msg[16] = '\0';
  }
}

void decrypt(unsigned char *msg) {
  if (enable_aes) {
    mbedtls_aes_init( &aes );
    mbedtls_aes_setkey_dec( &aes, (const unsigned char*) key, strlen(key) * 8 );
    mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_DECRYPT, (const unsigned char*)msg, (unsigned char*)buf);
    mbedtls_aes_free( &aes );
    for (int i = 0; i < 16; i++) msg[i] = buf[i];
    msg[16] = '\0';
  }
}

void send_packet(unsigned char *message) {
  byte b;
  char m[3];
  encrypt(message);
//  decrypt(message);
  for (int i = 0; i < 16; i++) {
    b = message[i];
    sprintf(m, "%02X\0", b);
    String M = m;
    Serial.print(M);
  }
  Serial.println(' ');
  LoRa.beginPacket();
  LoRa.write(message, 16);
  LoRa.endPacket();
}

void loop() {
  display.clear();
  
  display.drawString(0,  0*COL_HEIGHT, "Sensors found: ");
  display.drawString(90, 0*COL_HEIGHT, String(numberOfDevices));

  sprintf(msg, "Ciao Barz! %6d\0", counter/2);
  String s = msg;
  send_packet((unsigned char*)msg);

  display.drawString(0,  1*COL_HEIGHT, "Sending packet: ");
  display.drawString(90, 1*COL_HEIGHT, String(counter));
  display.drawString(0,  2*COL_HEIGHT, s);
  display.display();
  Serial.println(s);

  counter++;
  digitalWrite(BLUE_LED, HIGH);   // turn the LED on
  delay(2000);                    // wait for a second
  digitalWrite(BLUE_LED, LOW);    // turn the LED off
  delay(2000);                    // wait for a second
}
