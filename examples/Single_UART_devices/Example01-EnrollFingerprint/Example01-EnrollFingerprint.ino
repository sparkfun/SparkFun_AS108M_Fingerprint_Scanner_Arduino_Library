/*
  Enroll a fingerprint into AS-108M/AD-013 memory
  By: Ricardo Ramos
  SparkFun Electronics
  Date: June 14th, 2021
  SparkFun code, firmware, and software is released under the MIT License. Please see LICENSE.md for further details.
  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/17151

 Note: This example will work in devices with a single hardware serial port like Arduino Uno.
  
  Hardware Connections:
  - Connect the sensor to your board. Be aware that this sensor can be powered by 3.3V only!
  - Open a serial monitor at 115200bps
  
  The example below illustrates how to use the AS-108M/AD-013 with an Arduino Uno board.
*/

#include <SoftwareSerial.h>
#include "SparkFun_AS108M_Arduino_Library.h"

// Defines where the readers will be connected.
// TX_PIN : Arduino --> Reader
// RX_PIN : Arduino <-- Reader

#define TX_PIN    9       // AD-013 green wire
#define RX_PIN    8       // AD-013 blue wire

// Reader instance
AS108M as108m;

// Software serial instance with the corresponding pins
SoftwareSerial as108_serial(RX_PIN, TX_PIN);

// Function prototype for error callback function
void AS108_Callback();

void setup()
{
  // Initialize monitor serial port
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("Starting up..."));

  // Initialize reader serial port
  as108_serial.begin(57600);

  // Set built-in LED pin as output
  pinMode(LED_BUILTIN, OUTPUT);
  
  // the fingerprint scanner needs 100 ms after power up so let's wait and give it some slack also
  delay(150);

  // When calling begin we pass the reader serial port, the reader's address and an optional callback function as a parameter.
  // The library will call this function if there are any errors during operation.
  // The callback parameter is optional.
  if (as108m.begin(as108_serial, 0xffffffff, AS108_Callback) == true)
  {
    Serial.println(F("AS108M is properly connected."));
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else
  {
    Serial.println(F("AS108M not properly connected - check your connections..."));
    Serial.println(F("System halted!"));
    while (true);
  }
}

void loop()
{
  // ID holds the memory address that the fingerprint will be saved to.
  // Valid ranges are 1 to 40, inclusive
  byte ID = 1;

  Serial.print(F("Enrolling fingerprint in memory location "));
  Serial.println(ID);

  // Begin enroll process
  bool enroll = as108m.enrollFingerprint(ID);

  if (enroll == true)
  {
    // Turn on the built in LED and print out a sucess message if the operation was successful...
    Serial.print("Fingerprint enrolled successfully in memory position ");
    Serial.print(ID);
    Serial.println(" !");
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else
  {
    // ... or otherwise turn the LED off and print out a failure message
    Serial.println("Enroll failed...");
    digitalWrite(LED_BUILTIN, LOW);
  }

  // Halt
  while (true);     
 
}

// This function prints out the corresponding error message
void AS108_Callback()
{
  switch (as108m.response)
  {
  case AS108M_RESPONSE_CODES::AS108M_OK:
    // Just exit the switch
    break;

  case AS108M_RESPONSE_CODES::AS108M_DATA_PACKET_RECEIVE_ERROR:
    Serial.println(F("Packet receive error"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_NO_FINGER:
    Serial.println(F("No fingertip on scanner"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_GET_FINGERPRINT_IMAGE_FAILED:
    Serial.println(F("Get fingerprint image failed"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_FINGERPRINT_TOO_DRY_TOO_LIGHT:
    Serial.println(F("Fingerprint too dry or too light"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_FINGERPRINT_TOO_HUMID_TOO_BLURRY:
    Serial.println(F("Fingerprint too humid or too blurry"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_FINGERPRINT_TOO_AMORPHOUS:
    Serial.println(F("Fingerprint too amorphous"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_FINGERPRINT_TOO_LITTLE_MINUTIAES:
    Serial.println(F("Fingerprint too little minutiaes"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_FINGERPRINT_UNMATCHED:
    Serial.println(F("Fingerprint does not match ID"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_NO_FINGERPRINT_FOUND:
    Serial.println(F("No matching fingerprint found in search"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_MERGING_FAILED:
    Serial.println(F("Merging failed"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_ADDRESS_EXCEEDING_DATABASE_LIMIT:
    Serial.println(F("Address exceeded device limit (40)"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_TEMPLATE_READING_ERROR_INVALID_TEMPLATE:
    Serial.println(F("Template reading error or invalid template from database"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_FEATURE_UPLOAD_FAILED:
    Serial.println(F("Feature upload failed"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_CANNOT_RECEIVE_CONTINUOUS_PACKETS:
    Serial.println(F("Module cannot receive continuous packets"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_IMAGE_UPLOADING_FAILED:
    Serial.println(F("Image uploaded failed"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_IMAGE_DELETING_FAILED:
    Serial.println(F("Image deleting failed"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_FINGERPRINT_DATABASE_CLEAR_FAILED:
    Serial.println(F("Fingerprint database clear failed"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_CANNOT_IN_LOW_POWER_CONSUMPTION:
    Serial.println(F("Cannot perform task in low power mode"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_INVALID_PASSWORD:
    Serial.println(F("Invalid password"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_SYSTEM_RESET_FAILED:
    Serial.println(F("Device reset failed"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_NO_VALID_ORIGINAL_IMAGE_ON_BUFFER:
    Serial.println(F("No image in buffer"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_ONLINE_UPGRADING_FAILED:
    Serial.println(F("Upgrading failed"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_INCOMPLETE_OR_STILL_FINGERPRINT:
    Serial.println(F("Incomplete fingerprint on sensor"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_FLASH_READ_WRITE_ERROR:
    Serial.println(F("Flash read/write error"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_UNKNOWN_ERROR:
  case AS108M_RESPONSE_CODES::AS108M_UNDEFINED_ERROR:
    Serial.println(F("Undefined/unknown error"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_INVALID_REGISTER:
    Serial.println(F("Invalid register"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_REGISTER_DISTRIBUTING_CONTENT_WRONG_NUMBER:
    Serial.println(F("Register content wrong number"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_NOTEPAD_PAGE_APPOINTING_ERROR:
    Serial.println(F("Notepad appointing error"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_PORT_OPERATION_FAILED:
    Serial.println(F("Port operation failed"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_AUTOMATIC_ENROLL_FAILED:
    Serial.println(F("Automatic enroll failed"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_FINGERPRINT_DATABASE_FULL:
    Serial.println(F("Fingerprint database is full"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_MUST_VERIFY_PASSWORD:
    Serial.println(F("Verify password"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_CONTINUE_PACKET_ACK_F0:
    Serial.println(F("Existing instruction of continue data packet, ACK with 0xf0 after receiving correctly"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_CONTINUE_PACKET_ACK_F1:
    Serial.println(F("Existing instruction of continue data packet, ACK with 0xf1 after receiving correctly"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_SUM_ERROR_BURNING_FLASH:
    Serial.println(F("Checksum error burning flash"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_PACKET_FLAG_ERROR_BURNING_FLASH:
    Serial.println(F("Packet flag error when burning flash"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_PACKET_LENGTH_ERROR_BURNING_FLASH:
    Serial.println(F("Packet length error when burning flash"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_CODE_LENGTH_TOO_LONG_BURNING_FLASH:
    Serial.println(F("Code length too long when burning flash"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_BURNING_FLASH_FAILED:
    Serial.println(F("Burning flash failed"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_RESERVED:
    Serial.println(F("Reserved"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_INVALID_RESPONSE:
    Serial.println(F("Invalid response"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_BAD_CHECKSUM:
    Serial.println(F("Wrong checksum"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_ADDRESS_MISMATCH:
    Serial.println(F("Address mismatch"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_RECEIVE_TIMEOUT:
    Serial.println(F("Receive timeout"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_TOUCH_SENSOR:
    Serial.println(F("Please touch the scanner with your fingertip"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_REMOVE_FINGER:
    Serial.println(F("Please remove your fingertip from the scanner"));
    break;

  case AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE:
    Serial.println(F("No response"));
    break;

  default:
    break;
  }
}
