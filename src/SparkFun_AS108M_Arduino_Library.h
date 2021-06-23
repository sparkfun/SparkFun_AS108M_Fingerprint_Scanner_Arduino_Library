/*
  This is a library written for the AS108M Capacitive Fingerprint Scanner
  SparkFun sells these at its website:
https://www.sparkfun.com/products/17151

  Do you like this library? Help support open source hardware. Buy a board!

  Written by Ricardo Ramos  @ SparkFun Electronics, April 14th, 2021
  This file declares core functions available in the AS108M sensor library.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __SparkFun_AS108M_Arduino_Library__
#define __SparkFun_AS108M_Arduino_Library__
#include "SparkFun_AS108M_Constants.h"
#include <Arduino.h>

// Struct that holds packet data replied from the sensor
struct AS108M_PACKET_DATA
{
	FLAG_TYPE flagType = FLAG_TYPE::INDETERMINATE;
	byte packetLength = 0;
	byte packetData[31] = { 0 };
};


struct AS108M_QUERY_DATA
{
	// Flag that indicates if a fingerprint match was found
	bool found = false;	
	// Fingerprint database entry
	byte pageId = 0;
	// Matching fingerprint score
	unsigned int matchScore = 0;
};

class AS108M
{
private:
	// Pointer to the port used.
	Stream* _comm = NULL;
	
	// AS108M address (defaults to 0xffffffff).
	uint32_t _address = 0xffffffff;
	
	// Returns enumeration based on response value.
	AS108M_RESPONSE_CODES getResponseCode(byte response);

	// Reads a data packet from the device. Timeout in msec is optional and defaults to 5000
	AS108M_PACKET_DATA readPacket(unsigned int timeout = 5000);
	
	// Function pointer to optional callback function.
	void(*pCallback)(void) = NULL;

	
public:
	
	// Holds the last response code.
	AS108M_RESPONSE_CODES response;
	
	// Send single byte command to AS108M.
	void sendSingleByteCommand(const byte command);
	
	// Sends multiple bytes to AS108M where data array holds all user payload, from packet flag up to but excluding sum.
	void sendPacket(const byte* data, byte dataSize);
	
	// Starts the device. Callback is an optional pointer to a function that returns void and accepts void. Address is optional and defaults to 0xffffffff.
	bool begin(Stream& commPort = Serial, void(*callBack)(void) = NULL, unsigned long address = 0xffffffff);
	
	// Returns true if AS108M replies accordingly using the settings from begin.
	bool isConnected();
	
	// Zeroes the device's fingerprint database.
	bool clearFingerprintDatabase();
	
	// Returns true if a fingerprint was correctly enrolled in position ID. 
	bool enrollFingerprint(byte ID, byte numSamples = 5);
	
	// Returns true if fingerprint matches the ID passed as paramenter, false otherwise.
	AS108M_QUERY_DATA getFingerprintMatch(byte ID);
	
	// Search for the fingerprint in the device's enrolled fingerprint memory. 
	// This function will wait three attempts spaced timeBetweenRetries msec if no finger is in sensor before returning.
	AS108M_QUERY_DATA searchFingerprint();
	
	//Deletes a specific fingerprint entry from the database.
	bool deleteFingerprintEntry(byte ID);
};
#endif