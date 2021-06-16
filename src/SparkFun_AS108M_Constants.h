/*
  This is a library written for the AS108M Capacitive Fingerprint Scanner
  SparkFun sells these at its website:
https://www.sparkfun.com/products/17151

  Do you like this library? Help support open source hardware. Buy a board!

  Written by Ricardo Ramos  @ SparkFun Electronics, April 14th, 2021
  This file declares all constants used in the AS108M sensor library.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __SparkFun_AS108M_Constants__
#define __SparkFun_AS108M_Constants__

#include <Arduino.h>

// Flag types
const byte AS108M_FLAG_COMMAND =		0x01;
const byte AS108M_FLAG_DATA	=			0x02;
const byte AS108M_FLAG_ACK =			0x07;
const byte AS108M_FLAG_END =			0x08;

// Buffer ID
const byte AS108M_BUFFER_ID_1 =			0x01;
const byte AS108M_BUFFER_ID_2 =			0x02;

// Instruction definitions
const byte AS108M_GET_IMAGE =			0x01;
const byte AS108M_GET_CHAR =			0x02;
const byte AS108M_MATCH =				0x03;
const byte AS108M_SEARCH =				0x04;
const byte AS108M_REG_MODEL =			0x05;
const byte AS108M_STORE_CHAR =			0x06;
const byte AS108M_LOAD_CHAR =			0x07;
const byte AS108M_UP_CHAR =				0x08;
const byte AS108M_DOWN_CHAR =			0x09;
const byte AS108M_UP_IMAGE =			0x0a;
const byte AS108M_DOWN_IMAGE =			0x0b;
const byte AS108M_DELETE_CHAR =			0x0c;
const byte AS108M_EMPTY =				0x0d;
const byte AS108M_WRITE_REG =			0x0e;
const byte AS108M_READ_SYS_PARAMETER =	0x0f;
const byte AS108M_SET_PASSWORD =		0x12;
const byte AS108M_VERIFY_PASSWORD =		0x13;
const byte AS108M_GET_RANDOM_CODE =		0x14;
const byte AS108M_SET_CHIP_ADDRESS =	0x15;
const byte AS108M_READ_INFO_PAGE =		0x16;
const byte AS108M_WRITE_NOTEPAD =		0x18;
const byte AS108M_READ_NOTEPAD =		0x19;
const byte AS108M_VALID_TEMPLATE_NUM =	0x1d;
const byte AS108M_READ_INDEX_TABLE =	0x1f;
const byte AS108M_CANCEL =				0x30;

// Enumerations
enum class PACKET_FIELD : byte
{
	HEADER,
	ADDRESS,
	FLAG,
	LENGTH,
	PARAMETER,
	CHECKSUM,
	INVALID
};

enum class FLAG_TYPE : byte
{
	INDETERMINATE,
	COMMAND,
	DATA,
	ACK,
	END,
};

enum class AS108M_RESPONSE_CODES : byte
{
	AS108M_OK,											// 0, No error
	AS108M_DATA_PACKET_RECEIVE_ERROR,					// 1, Data packet receiving error
	AS108M_NO_FINGER,									// 2, no finger on the sensor
	AS108M_GET_FINGERPRINT_IMAGE_FAILED,				// 3, Fingerprint image acquisition failed
	AS108M_FINGERPRINT_TOO_DRY_TOO_LIGHT,				// 4, Fingerprint image is too dry or too light to generate feature
	AS108M_FINGERPRINT_TOO_HUMID_TOO_BLURRY,			// 5, Fingerprint image is too humid or too blurry to generate feature
	AS108M_FINGERPRINT_TOO_AMORPHOUS,					// 6, Fingerprint image is too amorphous to generate feature
	AS108M_FINGERPRINT_TOO_LITTLE_MINUTIAES,			// 7, Fingerprint image is in order, but with too little minutiaesor too small area to generate feature
	AS108M_FINGERPRINT_UNMATCHED,						// 8, Fingerprint unmatched
	AS108M_NO_FINGERPRINT_FOUND,						// 9, No fingerprint found in search
	AS108M_MERGING_FAILED,								// 10, Merging failed
	AS108M_ADDRESS_EXCEEDING_DATABASE_LIMIT,			// 11, SN address exceeded the range of fingerprint database when accessing it
	AS108M_TEMPLATE_READING_ERROR_INVALID_TEMPLATE,		// 12, Template reading error or invalid template from database
	AS108M_FEATURE_UPLOAD_FAILED,						// 13, Feature uploading failed
	AS108M_CANNOT_RECEIVE_CONTINUOUS_PACKETS,			// 14, Module cannot receive continuous data packet
	AS108M_IMAGE_UPLOADING_FAILED,						// 15, Image uploading failed
	AS108M_IMAGE_DELETING_FAILED,						// 16, Module deleting failed
	AS108M_FINGERPRINT_DATABASE_CLEAR_FAILED,			// 17, Fingerprint database clearing operation failed
	AS108M_CANNOT_IN_LOW_POWER_CONSUMPTION,				// 18, cannot be in low power consumption
	AS108M_INVALID_PASSWORD,							// 19, Incorrect password
	AS108M_SYSTEM_RESET_FAILED,							// 20, System reset failed
	AS108M_NO_VALID_ORIGINAL_IMAGE_ON_BUFFER,			// 21, No valid original image in buffer to generate image
	AS108M_ONLINE_UPGRADING_FAILED,						// 22, Online upgrading failed
	AS108M_INCOMPLETE_OR_STILL_FINGERPRINT,				// 23, There is incomplete fingerprint or finger stayed still between consecutive image captures
	AS108M_FLASH_READ_WRITE_ERROR,						// 24, Read/write FLASH error
	AS108M_UNDEFINED_ERROR,								// 25, Undefined error
	AS108M_INVALID_REGISTER,							// 26, Invalid register
	AS108M_REGISTER_DISTRIBUTING_CONTENT_WRONG_NUMBER,	// 27, Wrong content register distributing number
	AS108M_NOTEPAD_PAGE_APPOINTING_ERROR,				// 28, Page appointing notepad error
	AS108M_PORT_OPERATION_FAILED,						// 29, Port operation failed
	AS108M_AUTOMATIC_ENROLL_FAILED,						// 30, Automatic fingerprint enroll failed
	AS108M_FINGERPRINT_DATABASE_FULL,					// 31, Fingerprint database full
	AS108M_MUST_VERIFY_PASSWORD,						// 32, User must verify the password
	AS108M_CONTINUE_PACKET_ACK_F0,						// 33, Existing instruction of continue data packet, ACK with 0xf0 after receiving correctly
	AS108M_CONTINUE_PACKET_ACK_F1,						// 34, Existing instruction of continue data packet, ACK with 0xf1 after receiving correctly
	AS108M_SUM_ERROR_BURNING_FLASH,						// 35, Sum error when burning internal FLASH
	AS108M_PACKET_FLAG_ERROR_BURNING_FLASH,				// 36, Packet flag error when burning internal FLASH
	AS108M_PACKET_LENGTH_ERROR_BURNING_FLASH,			// 37, Packet length error when burning internal FLASH
	AS108M_CODE_LENGTH_TOO_LONG_BURNING_FLASH,			// 38, Code length too long when burning internal FLASH
	AS108M_BURNING_FLASH_FAILED,						// 39, Burning FLASH failed when burning internal FLASH;
	AS108M_RESERVED,									// 40, Reserved
	AS108M_INVALID_RESPONSE,							// 41, Invalid response
	AS108M_BAD_CHECKSUM,								// 42, Bad checksum
	AS108M_ADDRESS_MISMATCH,							// 43, Address mismatch
	AS108M_RECEIVE_TIMEOUT,								// 44, Receive timeout
	AS108M_TOUCH_SENSOR,								// 45, indicates user to touch sensor (for enrolling purposes)
	AS108M_REMOVE_FINGER,								// 46, indicates user to remove finger from the sensor (for enrolling purposes)
	AS108M_NO_RESPONSE,									// 47, no response
	AS108M_UNKNOWN_ERROR								// 48, unknown error
};
	
#endif