/*
  This is a library written for the AS108M Capacitive Fingerprint Scanner
  SparkFun sells these at its website:
https://www.sparkfun.com/products/17151

  Do you like this library? Help support open source hardware. Buy a board!

  Written by Ricardo Ramos  @ SparkFun Electronics, April 14th, 2021
  This file implements core functions available in the AS108M sensor library.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include "SparkFun_AS108M_Constants.h"
#include "SparkFun_AS108M_Arduino_Library.h"

#define SERIAL_BUFFER_SIZE 256

bool AS108M::begin(Stream& commPort, uint32_t address, void(*callBack)(void))
{
	_comm = &commPort;
	_address = static_cast<uint32_t>(address);
	if(callBack != NULL)
		pCallback = callBack;

	return isConnected();
}

bool AS108M::isConnected()
{
	// Clear response
	response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;
	
	// clear any leftover data that may be in _comm 
	// since AS108M will send 0x55 after power up
	while(_comm->available() > 0)
		_comm->read();
	
	// Send CANCEL command and wait for reply
	sendSingleByteCommand(AS108M_CANCEL);
	
	// Get data back
	readPacket();
	
	// return if we got an OK from A108M
	return (response == AS108M_RESPONSE_CODES::AS108M_OK);
}

void AS108M::sendSingleByteCommand(const byte command)
{
	const byte cmd[] = { 0x01, 0x00, 0x03, command };     
	sendPacket(cmd, 4);
}

void AS108M::sendPacket(const byte* data, byte dataSize)
{
	uint16_t checkSum = 0;
	
	// packet has 2 header bytes, 4 address bytes and 2 checksum byte
	byte packetLength = dataSize + 8;

	// checkSum *may* overflow. According to AS108M datasheet:
	// Sum is the total bytes from packet flag to Sum, the carry will be ignored if it exceed 2 bytes;
	for(byte i = 0 ; i < dataSize ; i++)
		checkSum += data[i];
		
	// Allocate packet array
	byte* packet = new byte[packetLength];

	// Assemble and transmit packet
	packet[0] = 0xef;
	packet[1] = 0x01;
	packet[2] = _address >> 24;
	packet[3] = _address >> 16;
	packet[4] = _address >> 8;
	packet[5] = static_cast<byte>(_address & 0xff);
	
	for (byte i = 0; i < dataSize; i++)
		packet[6 + i] = data[i];
		
	packet[packetLength - 2] = checkSum >> 8;
	packet[packetLength - 1] = checkSum & 0x00ff;
		
	_comm->write(packet, packetLength);
	
	// Delete the array from the heap...
	delete[] packet;
	
	// .. and nullify the pointer
	packet = NULL;
}

AS108M_PACKET_DATA AS108M::readPacket(unsigned int timeout)
{
	byte tempByte;
	uint16_t calculatedCheckSum = 0;
	uint16_t packetLength = 0;
	byte buffer[45] = { 0 };
	byte bufferIndex = 0;
	AS108M_PACKET_DATA reply;
	
	// Set response as no response
	response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;
		
	// Wait until we have nothing in the serial port
	// But time out after timeout msec have elapsed with an empty buffer
	uint32_t start = millis();
	while (_comm->available() == 0)
	{
		if (millis() - start > timeout)
		{
			response = AS108M_RESPONSE_CODES::AS108M_RECEIVE_TIMEOUT;
			return reply;
		} 
	}
	
	// Wait until completed packets arrive - needed for slow operation...
	delay(50);
	
	// Read the whole buffer into buffer array
	while(_comm->available() > 0)
	{
		buffer[bufferIndex] = (byte)_comm->read();
		bufferIndex++;
	}	
	
	// Do we have a valid header ?
	if(buffer[0] != 0xEF && buffer[1] == 0x01)
	{
		response = AS108M_RESPONSE_CODES::AS108M_INVALID_RESPONSE;
		return reply;
	}
		
	// Check if address matches the one programmed
	uint32_t receivedAddress = static_cast<uint32_t>(buffer[2]) << 24 | static_cast<uint32_t>(buffer[3]) << 16 | static_cast<uint32_t>(buffer[4]) << 8 | static_cast<uint32_t>(buffer[5]);
	_addressReplied = receivedAddress;
	if (receivedAddress != _address)
	{
		response = AS108M_RESPONSE_CODES::AS108M_ADDRESS_MISMATCH;
		return reply;
	}

	// Get current FLAG. Do not forget to sum bytes for checksum calculation !
	tempByte = buffer[6];
	calculatedCheckSum += tempByte;
	switch (tempByte)
	{
	case 0x01:
		reply.flagType = FLAG_TYPE::COMMAND;
		break;
			
	case 0x02:
		reply.flagType = FLAG_TYPE::DATA;
		break;
			
	case 0x07:
		reply.flagType = FLAG_TYPE::ACK;	
		break;
			
	case 0x08:
		reply.flagType = FLAG_TYPE::END;
		break;
			
	default:
		reply.flagType = FLAG_TYPE::INDETERMINATE;
		break;
	}
	
	if (reply.flagType == FLAG_TYPE::INDETERMINATE)
		return reply;
		
	
	// Get packet size. Do not forget to sum bytes for checksum calculation !
	// 2 is subtracted from packetLength since checksum is not part of data itself
	packetLength = buffer[7] << 8 | buffer[8];
	packetLength -= 2;
	calculatedCheckSum += buffer[7];
	calculatedCheckSum += buffer[8];
	
	// Copy all useful payload to reply struct
	for(byte i = 0 ; i < packetLength ; i++)
	{
		reply.packetData[i] = buffer[i + 9];
		calculatedCheckSum += buffer[i + 9];
	}

	// Compare checksum and set reponse accordingly
	uint16_t receivedChecksum = buffer[9 + packetLength] << 8 | buffer[10 + packetLength];
	response = (receivedChecksum != calculatedCheckSum) ? AS108M_RESPONSE_CODES::AS108M_BAD_CHECKSUM : AS108M_RESPONSE_CODES::AS108M_OK;
	return reply;
}

AS108M_RESPONSE_CODES AS108M::getResponseCode(byte response)
{
	if (response == 0x0)
		return AS108M_RESPONSE_CODES::AS108M_OK;

	if (response == 0x01)
		return AS108M_RESPONSE_CODES::AS108M_DATA_PACKET_RECEIVE_ERROR;

	if (response == 0x02)
		return AS108M_RESPONSE_CODES::AS108M_NO_FINGER;

	if (response == 0x03)
		return AS108M_RESPONSE_CODES::AS108M_GET_FINGERPRINT_IMAGE_FAILED;

	if (response == 0x04)
		return AS108M_RESPONSE_CODES::AS108M_FINGERPRINT_TOO_DRY_TOO_LIGHT;

	if (response == 0x05)
		return AS108M_RESPONSE_CODES::AS108M_FINGERPRINT_TOO_HUMID_TOO_BLURRY;

	if (response == 0x06)
		return AS108M_RESPONSE_CODES::AS108M_FINGERPRINT_TOO_AMORPHOUS;

	if (response == 0x07)
		return AS108M_RESPONSE_CODES::AS108M_FINGERPRINT_TOO_LITTLE_MINUTIAES;

	if (response == 0x08)
		return AS108M_RESPONSE_CODES::AS108M_FINGERPRINT_UNMATCHED;

	if (response == 0x09)
		return AS108M_RESPONSE_CODES::AS108M_NO_FINGERPRINT_FOUND;

	if (response == 0x0a)
		return AS108M_RESPONSE_CODES::AS108M_MERGING_FAILED;

	if (response == 0x0b)
		return AS108M_RESPONSE_CODES::AS108M_ADDRESS_EXCEEDING_DATABASE_LIMIT;

	if (response == 0x0c)
		return AS108M_RESPONSE_CODES::AS108M_TEMPLATE_READING_ERROR_INVALID_TEMPLATE;

	if (response == 0x0d)
		return AS108M_RESPONSE_CODES::AS108M_FEATURE_UPLOAD_FAILED;

	if (response == 0x0e)
		return AS108M_RESPONSE_CODES::AS108M_CANNOT_RECEIVE_CONTINUOUS_PACKETS;

	if (response == 0x0f)
		return AS108M_RESPONSE_CODES::AS108M_IMAGE_UPLOADING_FAILED;

	if (response == 0x10)
		return AS108M_RESPONSE_CODES::AS108M_IMAGE_DELETING_FAILED;

	if (response == 0x11)
		return AS108M_RESPONSE_CODES::AS108M_FINGERPRINT_DATABASE_CLEAR_FAILED;

	if (response == 0x12)
		return AS108M_RESPONSE_CODES::AS108M_CANNOT_IN_LOW_POWER_CONSUMPTION;

	if (response == 0x13)
		return AS108M_RESPONSE_CODES::AS108M_INVALID_PASSWORD;

	if (response == 0x14)
		return AS108M_RESPONSE_CODES::AS108M_SYSTEM_RESET_FAILED;

	if (response == 0x15)
		return AS108M_RESPONSE_CODES::AS108M_NO_VALID_ORIGINAL_IMAGE_ON_BUFFER;

	if (response == 0x16)
		return AS108M_RESPONSE_CODES::AS108M_ONLINE_UPGRADING_FAILED;

	if (response == 0x17)
		return AS108M_RESPONSE_CODES::AS108M_INCOMPLETE_OR_STILL_FINGERPRINT;

	if (response == 0x18)
		return AS108M_RESPONSE_CODES::AS108M_FLASH_READ_WRITE_ERROR;

	if (response == 0x19)
		return AS108M_RESPONSE_CODES::AS108M_UNDEFINED_ERROR;

	if (response == 0x1a)
		return AS108M_RESPONSE_CODES::AS108M_INVALID_REGISTER;

	if (response == 0x1b)
		return AS108M_RESPONSE_CODES::AS108M_REGISTER_DISTRIBUTING_CONTENT_WRONG_NUMBER;

	if (response == 0x1c)
		return AS108M_RESPONSE_CODES::AS108M_NOTEPAD_PAGE_APPOINTING_ERROR;

	if (response == 0x1d)
		return AS108M_RESPONSE_CODES::AS108M_PORT_OPERATION_FAILED;

	if (response == 0x1e)
		return AS108M_RESPONSE_CODES::AS108M_AUTOMATIC_ENROLL_FAILED;

	if (response == 0x1f)
		return AS108M_RESPONSE_CODES::AS108M_FINGERPRINT_DATABASE_FULL;

	if (response == 0x21)
		return AS108M_RESPONSE_CODES::AS108M_MUST_VERIFY_PASSWORD;

	if (response == 0xf0)
		return AS108M_RESPONSE_CODES::AS108M_CONTINUE_PACKET_ACK_F0;

	if (response == 0xf1)
		return AS108M_RESPONSE_CODES::AS108M_CONTINUE_PACKET_ACK_F1;

	if (response == 0xf2)
		return AS108M_RESPONSE_CODES::AS108M_SUM_ERROR_BURNING_FLASH;

	if (response == 0xf3)
		return AS108M_RESPONSE_CODES::AS108M_PACKET_FLAG_ERROR_BURNING_FLASH;

	if (response == 0xf4)
		return AS108M_RESPONSE_CODES::AS108M_PACKET_LENGTH_ERROR_BURNING_FLASH;

	if (response == 0xf5)
		return AS108M_RESPONSE_CODES::AS108M_CODE_LENGTH_TOO_LONG_BURNING_FLASH;

	if (response == 0xf6)
		return AS108M_RESPONSE_CODES::AS108M_BURNING_FLASH_FAILED;

	if ((response == 0x20) || (response > 0x21 && response <= 0xef))
		return AS108M_RESPONSE_CODES::AS108M_RESERVED;

	return AS108M_RESPONSE_CODES::AS108M_INVALID_RESPONSE;
}

AS108M_QUERY_DATA AS108M::searchFingerprint()
{	
	// Set response as no response
	response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;
	
	// Create default reply struct
	AS108M_PACKET_DATA reply;
	
	// Create default searchData struct (no finger detected)
	AS108M_QUERY_DATA searchData;
	
	// Searching is composed of three steps:
	// 1) Read Fingerprint using PS_GetImage
	// 2) Generate the image into a specific BufferID (1 in this case)
	// 3) Search the chip's memory for a fingerprint match
	
	// exit flag
	bool exit = false;
	
	do
	{
		sendSingleByteCommand(AS108M_GET_IMAGE);
		reply = readPacket();
		
		// If readPacket() did not set AS108M_OK return searchData as is
		if(response != AS108M_RESPONSE_CODES::AS108M_OK)
			return searchData;
		
		// We have a valid reply packet, so let's see if there's a finger in the sensor.
		
		switch(reply.packetData[0])
		{
		case 0x0:
			{
				exit = true;
			}
			break;

		case 0x01:
			{
				response = AS108M_RESPONSE_CODES::AS108M_DATA_PACKET_RECEIVE_ERROR;
				
				// Callback the function passed if it's not NULL
				if(pCallback != NULL)
					pCallback();
				
				return searchData;
			}
			break;

		case 0x02:
			{
				response = AS108M_RESPONSE_CODES::AS108M_NO_FINGER;
				if (pCallback != NULL)
					pCallback();
				return searchData;
			}
			break;
			
		case 0x03:
			{
				response = AS108M_RESPONSE_CODES::AS108M_GET_FINGERPRINT_IMAGE_FAILED;

				// Callback the function passed if it's not NULL
				if(pCallback != NULL)
					pCallback();
				
				return searchData;
			}
			break;

		default:
			{
				response = AS108M_RESPONSE_CODES::AS108M_UNKNOWN_ERROR;

				// Callback the function passed if it's not NULL
				if(pCallback != NULL)
					pCallback();
				
				return searchData;
			}
			break;

		}
	} while (!exit) ;
	
	// If we got this far it means we have a valid fingerprint in the scanner. Generate the char buffer...
	
	// Create and send command to genetrate CharBuffer to BufferID 1
	byte genCharBufCommand[5] = { AS108M_FLAG_COMMAND, 0x00, 0x04, AS108M_GET_CHAR, AS108M_BUFFER_ID_1 };
	sendPacket(genCharBufCommand, 5);
	
	// Empty the previously used reply struct
	memset(&reply, 0, sizeof(reply));
	
	// Set response as no response again
	response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;
	
	// Get the reply from the device
	reply = readPacket();
	
	// If readPacket() did not set AS108M_OK return searchData as is
	if(response != AS108M_RESPONSE_CODES::AS108M_OK)
	{
		// Callback the function passed if it's not NULL
		if(pCallback != NULL)
			pCallback();
				
		return searchData;
	}
	
	// Parse confirm code returned
	switch(reply.packetData[0])
	{
	
	case 0x0:
		// No errors, just exit the switch
		break;
		
	case 0x01:
		{
			response = AS108M_RESPONSE_CODES::AS108M_DATA_PACKET_RECEIVE_ERROR;

			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return searchData;
		}
		break;
		
	case 0x06:
		{
			response = AS108M_RESPONSE_CODES::AS108M_FINGERPRINT_TOO_AMORPHOUS;

			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return searchData;
		}
		break;
		
	case 0x07:
		{
			response = AS108M_RESPONSE_CODES::AS108M_FINGERPRINT_TOO_LITTLE_MINUTIAES;

			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return searchData;
		}
		break;
		
	case 0x15:
		{
			response = AS108M_RESPONSE_CODES::AS108M_NO_VALID_ORIGINAL_IMAGE_ON_BUFFER;

			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return searchData;
		}
		break;
		
	default:
		{
			response = AS108M_RESPONSE_CODES::AS108M_UNKNOWN_ERROR;

			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return searchData;
		}
		break;
	}
	
	// Final step is to search the device for a matching fingerprint from page 0 to 99
	byte searchCommand[9] = { AS108M_FLAG_COMMAND, 0x0, 0x08, AS108M_SEARCH, AS108M_BUFFER_ID_1, 0x00, 0x00, 0x00, 0x28 };
		
	sendPacket(searchCommand, 9);

	// Empty the previously used reply struct
	memset(&reply, 0, sizeof(reply));
	
	// Set response as no response again
	response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;
	
	// Get the reply from the device
	reply = readPacket();
	
	// If readPacket() did not set AS108M_OK return searchData as is
	if(response != AS108M_RESPONSE_CODES::AS108M_OK)
		return searchData;
	
	switch (reply.packetData[0])
	{
	case 0x0:
		{
			// Fingerprint match was found.
			searchData.found = true;
			searchData.pageId = reply.packetData[2];	// ID will never be more than 99 !
			searchData.matchScore = reply.packetData[3] << 8 | reply.packetData[4];
		}
		break;
		
	case 0x01:
		{
			response = AS108M_RESPONSE_CODES::AS108M_DATA_PACKET_RECEIVE_ERROR;
			
			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
		}
		break;
	
	case 0x09:
		{
			response = AS108M_RESPONSE_CODES::AS108M_NO_FINGERPRINT_FOUND;
			
			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
		}
		break;
		
	default:
		{
			response = AS108M_RESPONSE_CODES::AS108M_UNKNOWN_ERROR;

			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
		}
		break;
	}
	
	return searchData;
}

AS108M_QUERY_DATA AS108M::getFingerprintMatch(byte ID)
{
	// Set response as no response
	response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;
	
	// Create default reply struct
	AS108M_PACKET_DATA reply;
	
	// Create default searchData struct (no finger detected)
	AS108M_QUERY_DATA searchData;
	
	// Looking for a match is composed of four steps:
	// 1) Read Fingerprint using PS_GetImage
	// 2) Generate the image into a specific BufferID 1
	// 3) Load fingerprint ID (PageNumber) from the chip memory in BufferID 2 
	// 4) Call PS_Match

	// exit flag
	bool exit = false;

	do
	{
		sendSingleByteCommand(AS108M_GET_IMAGE);
		reply = readPacket();
		
		// If readPacket() did not set AS108M_OK return searchData as is
		if(response != AS108M_RESPONSE_CODES::AS108M_OK)
			return searchData;
		
		// We have a valid reply packet, so let's see if there's a finger in the sensor.
		
		switch(reply.packetData[0])
		{
		case 0x0:
			{
				exit = true;
			}
			break;

		case 0x01:
			{
				response = AS108M_RESPONSE_CODES::AS108M_DATA_PACKET_RECEIVE_ERROR;
				
				// Callback the function passed if it's not NULL
				if(pCallback != NULL)
					pCallback();
				
				return searchData;
			}
			break;

		case 0x02:
			{
				response = AS108M_RESPONSE_CODES::AS108M_NO_FINGER;

				// Callback the function passed if it's not NULL
				if (pCallback != NULL)
					pCallback();
					
				return searchData;
			}
			break;
			
		case 0x03:
			{
				response = AS108M_RESPONSE_CODES::AS108M_GET_FINGERPRINT_IMAGE_FAILED;

				// Callback the function passed if it's not NULL
				if(pCallback != NULL)
					pCallback();
				
				return searchData;
			}
			break;

		default:
			{
				response = AS108M_RESPONSE_CODES::AS108M_UNKNOWN_ERROR;

				// Callback the function passed if it's not NULL
				if(pCallback != NULL)
					pCallback();
				
				return searchData;
			}
			break;

		}
	} while (!exit) ;
	
	// Create and send command to genetrate CharBuffer
	byte genCharBufCommand[5] = { AS108M_FLAG_COMMAND, 0x00, 0x04, AS108M_GET_CHAR, AS108M_BUFFER_ID_1 };
	sendPacket(genCharBufCommand, 5);
	
	// Empty the previously used reply struct
	memset(&reply, 0, sizeof(reply));
	
	// Set response as no response again
	response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;
	
	// Get the reply from the device
	reply = readPacket();
	
	// If readPacket() did not set AS108M_OK return searchData as is
	if(response != AS108M_RESPONSE_CODES::AS108M_OK)
	{
		// Callback the function passed if it's not NULL
		if(pCallback != NULL)
			pCallback();
				
		return searchData;
	}
	
	// Parse confirm code returned
	switch(reply.packetData[0])
	{
	
	case 0x0:
		// No errors, just exit the switch
		break;
		
	case 0x01:
		{
			response = AS108M_RESPONSE_CODES::AS108M_DATA_PACKET_RECEIVE_ERROR;
			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return searchData;
		}
		break;
		
	case 0x06:
		{
			response = AS108M_RESPONSE_CODES::AS108M_FINGERPRINT_TOO_AMORPHOUS;

			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return searchData;
		}
		break;
		
	case 0x07:
		{
			response = AS108M_RESPONSE_CODES::AS108M_FINGERPRINT_TOO_LITTLE_MINUTIAES;

			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return searchData;
		}
		break;
		
	case 0x15:
		{
			response = AS108M_RESPONSE_CODES::AS108M_NO_VALID_ORIGINAL_IMAGE_ON_BUFFER;

			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return searchData;
		}
		break;
		
	default:
		{
			response = AS108M_RESPONSE_CODES::AS108M_UNKNOWN_ERROR;

			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return searchData;
		}
		break;
	}
	
	// Load ID into BufferID 2
	byte loadCommand[7] = { AS108M_FLAG_COMMAND, 0x0, 0x06, AS108M_LOAD_CHAR, AS108M_BUFFER_ID_2, 0x00, ID };
		
	sendPacket(loadCommand, 7);

	// Empty the previously used reply struct
	memset(&reply, 0, sizeof(reply));
	
	// Set response as no response again
	response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;
	
	// Get the reply from the device
	reply = readPacket();
	
	switch (reply.packetData[0])
	{
	case 0x0:
		// No errors, just exit the switch
		break;
		
	case 0x01:
		{
			response = AS108M_RESPONSE_CODES::AS108M_DATA_PACKET_RECEIVE_ERROR;
			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return searchData;
		}
		break;
		
	case 0x0b:
		{
			response = AS108M_RESPONSE_CODES::AS108M_ADDRESS_EXCEEDING_DATABASE_LIMIT;
			
			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return searchData;			
		}
		break;
		
	case 0x0c:
		{
			response = AS108M_RESPONSE_CODES::AS108M_TEMPLATE_READING_ERROR_INVALID_TEMPLATE;
			
			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return searchData;
		}
		break;
		
	default:
		{
			response = AS108M_RESPONSE_CODES::AS108M_UNKNOWN_ERROR;

			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return searchData;
		}
		break;
	}
	
	// Call match function
	byte matchCommand[4] = { AS108M_FLAG_COMMAND, 0x0, 0x03, AS108M_MATCH };
	
	sendPacket(matchCommand, 4);
	
	// Empty the previously used reply struct
	memset(&reply, 0, sizeof(reply));
	
	// Set response as no response again
	response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;
	
	// Get the reply from the device
	reply = readPacket();
	
	// If readPacket() did not set AS108M_OK return searchData as is
	if(response != AS108M_RESPONSE_CODES::AS108M_OK)
	return searchData;
	
	switch (reply.packetData[0])
	{
	case 0x0:
		{
			// Fingerprint match was found.
			searchData.found = true;
			searchData.pageId = ID;
			searchData.matchScore = reply.packetData[1] << 8 | reply.packetData[2];
		}
		break;
		
	case 0x01:
		{
			response = AS108M_RESPONSE_CODES::AS108M_DATA_PACKET_RECEIVE_ERROR;
			
			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
		}
		break;
	
	case 0x08:
		{
			response = AS108M_RESPONSE_CODES::AS108M_FINGERPRINT_UNMATCHED;
			
			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
		}
		break;
		
	default:
		{
			response = AS108M_RESPONSE_CODES::AS108M_UNKNOWN_ERROR;

			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
		}
		break;
	}
	
	return searchData;
	
}

bool AS108M::enrollFingerprint(byte ID, byte numSamples)
{
	// Set response as no response
	response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;
	
	// Create default reply struct
	AS108M_PACKET_DATA reply;
	
	// Enroll a fingerprint consist of looping numSamples times. In each itertion bufferID is incremented and the newly acquired image is stored
	// in this bufferID. After all iterations are completed a model is generated and stored in flash in position ID.
	
	// exit flag
	bool exit = false;
	
	for(byte sample = 1 ; sample <= numSamples ; sample++)
	{
		response = AS108M_RESPONSE_CODES::AS108M_TOUCH_SENSOR;
		if (pCallback != NULL)
			pCallback();
		response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;
		do
		{
			sendSingleByteCommand(AS108M_GET_IMAGE);
			memset(&reply, 0, sizeof(reply));
			reply.packetData[0] = 0xff;
			reply = readPacket();
		
			// If readPacket() did not set AS108M_OK return searchData as is
			if(response != AS108M_RESPONSE_CODES::AS108M_OK)
				return false;
		
			// We have a valid reply packet, so let's see if there's a finger in the sensor.		
			switch(reply.packetData[0])
			{
			case 0x0:
				{
					exit = true;
				}
				break;

			case 0x01:
				{
					response = AS108M_RESPONSE_CODES::AS108M_DATA_PACKET_RECEIVE_ERROR;
					// Callback the function passed if it's not NULL
					if(pCallback != NULL)
						pCallback();
				
					return false;
				}
				break;

			case 0x02:
				{
					// No finger in sensor, so loop and wait until user touches the sensor...
					response = AS108M_RESPONSE_CODES::AS108M_NO_FINGER;
					delay(200);
				}
				break;
			
			case 0x03:
				{
					response = AS108M_RESPONSE_CODES::AS108M_GET_FINGERPRINT_IMAGE_FAILED;

					// Callback the function passed if it's not NULL
					if(pCallback != NULL)
						pCallback();
				
					return false;
				}
				break;

			default:
				{
					response = AS108M_RESPONSE_CODES::AS108M_UNKNOWN_ERROR;

					// Callback the function passed if it's not NULL
					if(pCallback != NULL)
						pCallback();
				
					return false;
				}
				break;

			}
		} while (!exit) ;
	
		response = AS108M_RESPONSE_CODES::AS108M_REMOVE_FINGER;
		if (pCallback != NULL)
			pCallback();
		
		// Wait until user remove finger from sensor...
		do
		{
			sendSingleByteCommand(AS108M_GET_IMAGE);
			memset(&reply, 0, sizeof(reply));
			reply.packetData[0] = 0xff;
			reply = readPacket();
			delay(200);
		} while (reply.packetData[0] != 0x02);

		exit = false;
		
		// Create and send command to genetrate CharBuffer
		byte genCharBufCommand[5] = { AS108M_FLAG_COMMAND, 0x00, 0x04, AS108M_GET_CHAR, sample };
		sendPacket(genCharBufCommand, 5);
	
		// Empty the previously used reply struct
		memset(&reply, 0, sizeof(reply));
		reply.packetData[0] = 0xff;
		
		// Set response as no response again
		response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;
	
		// Get the reply from the device
		reply = readPacket();
	
		// If readPacket() did not set AS108M_OK return searchData as is
		if(response != AS108M_RESPONSE_CODES::AS108M_OK)
		{
			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return false;
		}
	
		// Parse confirm code returned
		switch(reply.packetData[0])
		{
	
		case 0x0:
			// No errors, just exit the switch
			break;
		
		case 0x01:
			{
				response = AS108M_RESPONSE_CODES::AS108M_DATA_PACKET_RECEIVE_ERROR;
				// Callback the function passed if it's not NULL
				if(pCallback != NULL)
					pCallback();
				
				return false;
			}
			break;
		
		case 0x06:
			{
				response = AS108M_RESPONSE_CODES::AS108M_FINGERPRINT_TOO_AMORPHOUS;

				// Callback the function passed if it's not NULL
				if(pCallback != NULL)
					pCallback();
				
				return false;
			}
			break;
		
		case 0x07:
			{
				response = AS108M_RESPONSE_CODES::AS108M_FINGERPRINT_TOO_LITTLE_MINUTIAES;
				// Callback the function passed if it's not NULL
				if(pCallback != NULL)
					pCallback();
			
				return false;
			}
			break;
		
		case 0x15:
			{
				response = AS108M_RESPONSE_CODES::AS108M_NO_VALID_ORIGINAL_IMAGE_ON_BUFFER;

				// Callback the function passed if it's not NULL
				if(pCallback != NULL)
					pCallback();
				
				return false;
			}
			break;
		
		default:
			{
				response = AS108M_RESPONSE_CODES::AS108M_UNKNOWN_ERROR;

				// Callback the function passed if it's not NULL
				if(pCallback != NULL)
					pCallback();
				
				return false;
			}
			break;
		}
	}
	
	// Generate model
	byte genModelCommand[4] = { AS108M_FLAG_COMMAND, 0x00, 0x03, AS108M_REG_MODEL };
	sendPacket(genModelCommand, 4);
	
	// Empty the previously used reply struct
	memset(&reply, 0, sizeof(reply));
	reply.packetData[0] = 0xff;
	
	// Set response as no response again
	response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;
	
	// Get the reply from the device
	reply = readPacket();
	
	// Do we have a successful merge ?
	switch(reply.packetData[0])
	{
	case 0x0:
		// No errors, just exit the switch
		break;
		
	case 0x01:
		{
			response = AS108M_RESPONSE_CODES::AS108M_DATA_PACKET_RECEIVE_ERROR;
			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return false;
		}
		break;
		
	case 0x0a:
		{
			response = AS108M_RESPONSE_CODES::AS108M_MERGING_FAILED;
			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return false;
		}
		break;
		
	default:
		{
			response = AS108M_RESPONSE_CODES::AS108M_UNKNOWN_ERROR;

			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return false;
		}
		break;
	}
	
	// Save contents into flash at address ID
	byte saveContentsCommand[7] = { AS108M_FLAG_COMMAND, 0x00, 0x06, AS108M_STORE_CHAR, AS108M_BUFFER_ID_1, 0x00, ID };
	sendPacket(saveContentsCommand, 7);
	
	// Empty the previously used reply struct
	memset(&reply, 0, sizeof(reply));
	reply.packetData[0] = 0xff;
	
	// Set response as no response again
	response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;
	
	// Get the reply from the device
	reply = readPacket();
	
	switch (reply.packetData[0])
	{
	case 0x0:
		// No errors, just exit the switch
		break;
		
	case 0x01:
		{
			response = AS108M_RESPONSE_CODES::AS108M_DATA_PACKET_RECEIVE_ERROR;
			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return false;
		}
		break;		
		
	case 0x0b:
		{
			response = AS108M_RESPONSE_CODES::AS108M_ADDRESS_EXCEEDING_DATABASE_LIMIT;
			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return false;
		}
		break;
		
	case 0x18:
		{
			response = AS108M_RESPONSE_CODES::AS108M_FLASH_READ_WRITE_ERROR;
			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return false;
		}
		break;
		
	default:
		{
			response = AS108M_RESPONSE_CODES::AS108M_UNKNOWN_ERROR;

			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return false;
		}
		break;		
	}
	
	return true;
}

bool AS108M::clearFingerprintDatabase()
{
	// Set response as no response
	response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;
	
	// Create default reply struct
	AS108M_PACKET_DATA reply;
	
	byte clearCommand[4] = { AS108M_FLAG_COMMAND, 0x00, 0x03, AS108M_EMPTY };
	sendPacket(clearCommand, 4);
	
	// Set response as no response again
	response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;
	
	// Get the reply from the device
	reply = readPacket();
	
	switch (reply.packetData[0])
	{
	case 0x0:
		// No errors, just exit the switch
		break;
		
	case 0x01:
		{
			response = AS108M_RESPONSE_CODES::AS108M_DATA_PACKET_RECEIVE_ERROR;
			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return false;
		}
		break;		
		
	case 0x11:
		{
			response = AS108M_RESPONSE_CODES::AS108M_FINGERPRINT_DATABASE_CLEAR_FAILED;
			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return false;
		}
		break;
		
	default:
		{
			response = AS108M_RESPONSE_CODES::AS108M_UNKNOWN_ERROR;

			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return false;
		}
		break;		
	}
	
	return true;
}

bool AS108M::deleteFingerprintEntry(byte ID)
{
	// Set response as no response
	response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;
	
	// Create default reply struct
	AS108M_PACKET_DATA reply;
	
	byte deleteCommand[8] = { AS108M_FLAG_COMMAND, 0x00, 0x07, AS108M_DELETE_CHAR, 0x00, ID, 0x00, 0x01 };
	sendPacket(deleteCommand, 8);
	
	// Set response as no response again
	response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;
	
	// Get the reply from the device
	reply = readPacket();
	
	switch (reply.packetData[0])
	{
	case 0x0:
		// No errors, just exit the switch
		break;
		
	case 0x01:
		{
			response = AS108M_RESPONSE_CODES::AS108M_DATA_PACKET_RECEIVE_ERROR;
			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return false;
		}
		break;		
		
	case 0x10:
		{
			response = AS108M_RESPONSE_CODES::AS108M_IMAGE_DELETING_FAILED;
			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return false;
		}
		break;
		
	default:
		{
			response = AS108M_RESPONSE_CODES::AS108M_UNKNOWN_ERROR;

			// Callback the function passed if it's not NULL
			if(pCallback != NULL)
				pCallback();
				
			return false;
		}
		break;		
	}
	
	return true;
}

uint16_t AS108M::getDatabaseSize()
{
	byte readParaCommand[4] = {AS108M_FLAG_COMMAND, 0x00, 0x03, AS108M_READ_SYS_PARAMETER};
	sendPacket(readParaCommand,4);

	// Set response as no response again
	response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;
	
	// Create default reply struct
	AS108M_PACKET_DATA reply;
	
	// Get the reply from the device
	reply = readPacket();

	if(reply.packetData[0] == 0x01)
	{
		response = AS108M_RESPONSE_CODES::AS108M_DATA_PACKET_RECEIVE_ERROR;
		// Callback the function passed if it's not NULL
		if (pCallback != NULL)
			pCallback();

		return 0;
	}

	// Database size is located in bytes 5 and 6 in reply.packetData array
	return ((reply.packetData[5] << 8) | reply.packetData[6]);
}

uint32_t AS108M::getAddress()
{
	byte readParaCommand[4] = {AS108M_FLAG_COMMAND, 0x00, 0x03, AS108M_READ_SYS_PARAMETER};
	sendPacket(readParaCommand, 4);

	// Set response as no response again
	response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;

	// Create default reply struct
	AS108M_PACKET_DATA reply;

	// Get the reply from the device
	reply = readPacket();

	if (reply.packetData[0] == 0x01)
	{
		response = AS108M_RESPONSE_CODES::AS108M_DATA_PACKET_RECEIVE_ERROR;
		// Callback the function passed if it's not NULL
		if (pCallback != NULL)
			pCallback();

		return 0;
	}

	// Address is located in bytes 9, 10, 11 and 12 in  reply.packetData array
	// return ((reply.packetData[9] << 24) | (reply.packetData[10] << 16) | (reply.packetData[11] << 8) | reply.packetData[12]);

	// Instead of retuning the reader's address from the reader's register get it from the reply header - which was already saved by readPacket function
	// This allows easy recovery of the reader's address in case it's forgotten
	return _addressReplied;
}

uint32_t AS108M::getBaudrate()
{
	byte readParaCommand[4] = {AS108M_FLAG_COMMAND, 0x00, 0x03, AS108M_READ_SYS_PARAMETER};
	sendPacket(readParaCommand, 4);

	// Set response as no response again
	response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;

	// Create default reply struct
	AS108M_PACKET_DATA reply;

	// Get the reply from the device
	reply = readPacket();

	if (reply.packetData[0] == 0x01)
	{
		response = AS108M_RESPONSE_CODES::AS108M_DATA_PACKET_RECEIVE_ERROR;
		// Callback the function passed if it's not NULL
		if (pCallback != NULL)
			pCallback();

		return 0;
	}

	// Baudrate is located in bytes 15 and 16 in the reply.packetData array
	// but since the maximum value for the multiplier is 12 (115200/9600) we
	// only need to parse the least significant byte of the baudrate information
	uint8_t multiplier = reply.packetData[16];
	return multiplier * 9600U;
}

uint8_t AS108M::getMatchThreshold()
{
	byte readParaCommand[4] = {AS108M_FLAG_COMMAND, 0x00, 0x03, AS108M_READ_SYS_PARAMETER};
	sendPacket(readParaCommand, 4);

	// Set response as no response again
	response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;

	// Create default reply struct
	AS108M_PACKET_DATA reply;

	// Get the reply from the device
	reply = readPacket();

	if (reply.packetData[0] == 0x01)
	{
		response = AS108M_RESPONSE_CODES::AS108M_DATA_PACKET_RECEIVE_ERROR;
		// Callback the function passed if it's not NULL
		if (pCallback != NULL)
			pCallback();

		return 0;
	}

	// Match threshold (or security rank) is located in bytes 7 and 8 in the reply.packetData array
	// but since the maximum value for the match threshold is 5 we only need to parse the least
	// significant byte of the match threshold information
	return reply.packetData[8];
}

bool AS108M::setMatchThreshold(uint8_t newMatchThreshold)
{
	// Match threshold is in register #5
	byte setCommand[6] = { AS108M_FLAG_COMMAND, 0x00, 0x05, AS108M_WRITE_REG, AS108M_MATCH_THRES_REG, newMatchThreshold };
	sendPacket(setCommand, 6);

	// Set response as no response again
	response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;

	// Create default reply struct
	AS108M_PACKET_DATA reply;

	// Get the reply from the device
	reply = readPacket();

	switch(reply.packetData[0])
	{
	case 0x0:
	{
		response = AS108M_RESPONSE_CODES::AS108M_OK;
		return true;
	}
	break;

	case 0x01:
	{
		response = AS108M_RESPONSE_CODES::AS108M_DATA_PACKET_RECEIVE_ERROR;
		// Callback the function passed if it's not NULL
		if (pCallback != NULL)
			pCallback();

		return false;
	}
	break;

	case 0x1a:
	{
		response = AS108M_RESPONSE_CODES::AS108M_INVALID_REGISTER;
		if (pCallback != NULL)
			pCallback();

		return false;
	}
	break;

	default:
	{
		response = AS108M_RESPONSE_CODES::AS108M_UNKNOWN_ERROR;

		// Callback the function passed if it's not NULL
		if (pCallback != NULL)
			pCallback();

		return false;
	}
	break;

	}

	return false;
}

bool AS108M::setBaudrate(AS108M_BAUDRATE newBaudrate)
{
	// Get multiplier from enum value
	uint8_t multiplier = static_cast<uint8_t>(newBaudrate);

	// Baudrate register is #4
	byte setCommand[6] = {AS108M_FLAG_COMMAND, 0x00, 0x05, AS108M_WRITE_REG, AS108M_BAUDRATE_CTRL_REG, multiplier};
	sendPacket(setCommand, 6);

	// Set response as no response again
	response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;

	// Create default reply struct
	AS108M_PACKET_DATA reply;

	// Get the reply from the device
	reply = readPacket();

	switch (reply.packetData[0])
	{
	case 0x0:
	{
		response = AS108M_RESPONSE_CODES::AS108M_OK;
		return true;
	}
	break;

	case 0x01:
	{
		response = AS108M_RESPONSE_CODES::AS108M_DATA_PACKET_RECEIVE_ERROR;
		// Callback the function passed if it's not NULL
		if (pCallback != NULL)
			pCallback();

		return false;
	}
	break;

	case 0x1a:
	{
		response = AS108M_RESPONSE_CODES::AS108M_INVALID_REGISTER;
		if (pCallback != NULL)
			pCallback();

		return false;
	}
	break;

	default:
	{
		response = AS108M_RESPONSE_CODES::AS108M_UNKNOWN_ERROR;

		// Callback the function passed if it's not NULL
		if (pCallback != NULL)
			pCallback();

		return false;
	}
	break;
	}

	return false;
}

bool AS108M::setAddress(uint32_t newAddress)
{
	// Break the 32-bit address into 8 bit chunks
	uint8_t b0 = newAddress >> 24;
	uint8_t b1 = newAddress >> 16;
	uint8_t b2 = newAddress >> 8;
	uint8_t b3 = newAddress & 0x0000ff;

	// Build the command 
	byte setCommand[8] = {AS108M_FLAG_COMMAND, 0x00, 0x07, AS108M_SET_CHIP_ADDRESS, b0, b1, b2, b3};
	
	// Send the packet
	sendPacket(setCommand, 8);

	// Set response as no response again
	response = AS108M_RESPONSE_CODES::AS108M_NO_RESPONSE;

	// Create default reply struct
	AS108M_PACKET_DATA reply;

	// Get the reply from the device
	reply = readPacket();

	switch (reply.packetData[0])
	{
	case 0x0:
	{
		response = AS108M_RESPONSE_CODES::AS108M_OK;
		return true;
	}
	break;

	case 0x01:
	{
		response = AS108M_RESPONSE_CODES::AS108M_DATA_PACKET_RECEIVE_ERROR;
		// Callback the function passed if it's not NULL
		if (pCallback != NULL)
			pCallback();

		return false;
	}
	break;

	default:
	{
		response = AS108M_RESPONSE_CODES::AS108M_UNKNOWN_ERROR;

		// Callback the function passed if it's not NULL
		if (pCallback != NULL)
			pCallback();

		return false;
	}
	break;
	}

	return false;
}