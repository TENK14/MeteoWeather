void setup()
{

  /* add setup code here */

}

void loop()
{

  /* add main program code here */
  /**
  * PROG: MeteoWeather
  * AUTH: TENK
  * DATE: 28.4.2016
  * DESC: Save temperature, pressure, altitude and gps position to SD card. ??humidity??
  * Fonty: https://code.google.com/p/u8glib/wiki/fontgroupu8g
  * no
  /**/


  // ------------ TEMPERATURE AND PRESSURE SENSOR ---------------- // 
#include "BMP180.h" // Pressure and Temperature sensor ///////////////////////////////////////////////
#include "cactus_io_BME280_I2C.h"

  // ---------------------- DISPLAY ------------------------------ // 
  //#include <U8glib.h> // Display
#include "OzOLED.h" // max positions are [ROW=7,COL=16] for text
  //#include <Adafruit_GFX.h>
  //#include <Adafruit_SSD1306.h>


  // -------------------- SD CARD ---------------------------- //
#include <Wire.h>
  //#include <SPI.h> // SD card communication protocol (like I2C) from Arduino IDE/lib/WIFI
  //#include <SD.h>  // SD card from Arduino IDE/lib/SD
#include <SdFat.h> // pouzivano s 328P, pokusy o zprovozneni s 1284P

  /**
  // V c:\Program Files (x86)\Arduino\hardware\mighty-1284p-1.6.3\avr\libraries\SD\examples\ReadWrite\ by to mìlo dle diskuzí fungovat pøi aktivních tìchto nastaveních

  V je soubor Sd2PinMap.h a v nem by mela byt rozsirena podminka o 1284P, coz je: #elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__)  || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__)  || defined(__AVR_ATmega644PA__)

  Dale v kodu by melo byt, coz mam:

  // Kvuli SD shieldu
  #define __AVR_ATmega1284P__
  #define MIGHTY_1284P_VARIANT
  ....
  //uint8_t const SS_PIN = SS;// 4;// SS;
  //uint8_t const MOSI_PIN = MOSI;// 5;// MOSI;
  //uint8_t const MISO_PIN = MISO;// 6;// MISO;
  //uint8_t const SCK_PIN = SCK
  ....
  pinMode(10, OUTPUT);

  pinMode(chipSelect, OUTPUT);
  digitalWrite(chipSelect, HIGH);
  /**/
  // ---------------------- GPS ------------------------------ //
  //#include <Adafruit_GPS.h>
#include "TinyGPS.h"
#include <SoftwareSerial.h>

  // ==========================================
  // ============== Consts ====================
  // ==========================================

  // Kvuli SD shieldu
#define __AVR_ATmega1284P__
#define MIGHTY_1284P_VARIANT


  /**
  #define COUNT 10
  #define TITLE "MeteoWeather"
  #define VERSION "1.6"
  #define LOOP_TIME_MS 5000
  #define FILENAME "weather.txt"
  /**/

#define TITLE "MeteoWeather"
#define VERSION "2.13"
  //#define FILENAME "weather.txt"

  // Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
  // Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false //true

  // Log file base name.  Must be six characters or less.
#define FILE_BASE_NAME "Data"

  // TIMERS
#define MS_READ_BMP180 5000
#define MS_READ_BME280 5000
#define MS_DRAW 1000
#define MS_WRITE_DATA2SD 10000

	const uint8_t chipSelect = SS;//4; // SD card pin // UNO - 10, 1284P - 4 = SS

								  /// PROGMEM
								  /// Store data in flash (program) memory instead of SRAM
								  /// The PROGMEM keyword is a variable modifier, it should be used only with the datatypes defined in pgmspace.h. 
								  /// It tells the compiler "put this information into flash memory", instead of into SRAM, where it would normally go.

								  /**/

	static const byte  PROGMEM COUNT = 10; //10;

										   /**/




										   /**
										   #define OLED_RESET 4
										   Adafruit_SSD1306 display(OLED_RESET);
										   #define XPOS 0
										   #define YPOS 1
										   /**/
										   // ------ Modes ------

										   //enum {	M_MENU, M_CLOCK, M_GPS, M_TRAVEL,M_RUNNING,		
										   //        M_SET_TIMEZONE, M_STATUS, M_FIXING};


										   // ------ Config ------

										   //byte mode = M_FIXING;	// initial mode

										   // ========================================================
										   // ==================== Global Variables ==================
										   // ========================================================

										   // We are going to use the on board LED for an indicator.
	int indicatorLed = 13;

	// Timers for reading and writing
	uint32_t timerReadBMP180 = millis();
	uint32_t timerReadBME280 = millis();
	uint32_t timerDraw = millis();
	uint32_t timerWriteData2SD = millis();

	/// ---------- BMP180 ------------ /// /////////////////////////// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	///Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
	// Store an instance of the BMP180 sensor.
	BMP180 barometer;
	float temperature, pressure, altitude, humidity;
	// Store the current sea level pressure at your location in Pascals.
	float seaLevelPressure = 101325;

	BME280_I2C bme(0x76);  // I2C using address 0x76

						   /// --------- GPS ---------------- ///
						   // gpsSerial(RxpinArduino,TxPinArduino)
						   // TxPinGPS = 3 -> RxArduino
						   // RxPinGPS = 2 -> TxArduino
						   // SoftwareSerial uart_gps(RxArduino, TxArduino);
						   //SoftwareSerial mySerial(3,2); //(4,3); //(3, 4); // GPS connectors
						   //Adafruit_GPS GPS(&mySerial);
	SoftwareSerial gpsSerial(3, 2); //(4,3); //(3,4); // create gps sensor connection
	TinyGPS gps; // create gps object
				 //long lat,lon; // create variable for latitude and longitude object (lat/lon are values in 100,000ths of a degree)
	float latitude = 0.f;
	float longitude = 0.f;
	//unsigned long date, time; // date = ddmmyy, time = hhmmsscc, age = milliseconds
	int year;
	byte month, day, hour, minute, second, hundredths;

	// this keeps track of whether we're using the interrupt
	// off by default!
	boolean usingInterrupt = false;
	void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

								/// ----------SD card ------------- ///
	SdFat sd; // File system object.
	SdFile file; // Log file.
	char filename[13] = FILE_BASE_NAME "00.log";

	char charVal[10];               //temporarily holds data from vals


									//boolean oled_enabled = true; // NOT IN USE


									//bool isInitDisplay, isInitBMP180, isInitSDCard = false;
	bool isSaved, isGPSParsed;

	bool isSDCardLoad = true;//true; //false;
	bool isOledLoad = true;
	bool isBMP180Load = false;
	bool isBME280Load = true;
	bool isGPSLoad = true;
	bool isDebug = false;


	long loopCounter = 0;
	/**
	static const unsigned char OscarLogo[] PROGMEM ={

	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
	0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0xF0,
	0xF8, 0xF8, 0xFC, 0xFE, 0xFE, 0xFF, 0x3F, 0x1F, 0x0F, 0x07, 0x07, 0x03, 0x03, 0x03, 0x03, 0x01,
	0x01, 0x03, 0x03, 0x03, 0x03, 0x07, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0xFE, 0xFE, 0xFC, 0xFC, 0xF8,
	0xF0, 0xE0, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0xE0, 0xF0, 0x10, 0x18, 0x08, 0x08, 0x08,
	0x18, 0x38, 0x70, 0xE0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xF0, 0x3C, 0x0E, 0x0F, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0x0F, 0x0F, 0x3C, 0xF0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xC0, 0x83, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00,
	0x80, 0xE0, 0x78, 0x1F, 0x37, 0x38, 0xBC, 0xFC, 0xE4, 0x80, 0x00, 0x00, 0xE0, 0xF0, 0x38, 0x28,
	0xB8, 0x98, 0x80, 0xC0, 0xE0, 0x30, 0x18, 0x88, 0xC8, 0xF8, 0xB8, 0x80, 0x3C, 0x3C, 0xEC, 0xF8,
	0x98, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xFE, 0x1F, 0x01, 0x00, 0x00, 0xC0, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xF8, 0xE0, 0xC0, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0xC0, 0xE0, 0xF0, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01,
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xE0, 0xF0, 0xF8, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE, 0xFE, 0x7E,
	0x7E, 0xFE, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFC, 0xF0, 0xE0, 0xFF, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0x60,
	0x60, 0x20, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x3F, 0x7F, 0x7F,
	0xFF, 0x8F, 0x07, 0x07, 0x03, 0x01, 0x01, 0x01, 0x00, 0xF0, 0xFC, 0xFC, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xF8, 0xFC, 0xFC, 0x00, 0x01, 0x01, 0x03, 0x03, 0x07, 0x0F, 0xDF, 0xFF,
	0x7F, 0x7F, 0x3F, 0x3F, 0x1F, 0x0F, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xF0, 0x7E, 0x1F, 0x07, 0x01, 0x00,
	0x04, 0x06, 0x03, 0x81, 0xE0, 0x6C, 0x0C, 0x04, 0x00, 0x00, 0x80, 0xC0, 0x60, 0x20, 0x20, 0xE0,
	0xE0, 0x00, 0xD0, 0xF0, 0xE0, 0x60, 0x20, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0xC0, 0xC0, 0x60, 0x20,
	0x20, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x03, 0x0F, 0x3C, 0x70, 0x60, 0xC0, 0xC0, 0xC0, 0x80, 0x80, 0x83, 0x83, 0x80, 0x80, 0x80, 0x80,
	0x80, 0x80, 0x80, 0x80, 0x81, 0x83, 0x83, 0x80, 0xC0, 0xC0, 0xC0, 0x60, 0x30, 0x1C, 0x0F, 0x01,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x06, 0x07, 0x05, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
	0x04, 0x04, 0x00, 0x07, 0x07, 0x04, 0x06, 0x02, 0x00, 0x07, 0x07, 0x04, 0x04, 0x06, 0x07, 0x07,
	0x06, 0x06, 0x07, 0x03, 0x00, 0x06, 0x07, 0x05, 0x06, 0x02, 0x80, 0x87, 0x87, 0x04, 0x84, 0xF6,
	0xFF, 0x1F, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x02, 0x02, 0x03, 0x01,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

	};
	/**/
	//=======================================================
	//================= Program Begin =======================
	//=======================================================

	void setup() {

		//OLED BEGIN
		//OzOled.init();  //initialze SEEED OLED display  
		// charge pump ON
		//OzOled.sendCommand(0x8d);
		//OzOled.sendCommand(0x14); 

		//Serial.begin(115200/*9600*/); //4.9.2016
		Serial.begin(9600); //4.9.2016


							/********************* Add these Two Lines **********************/
		pinMode(10, OUTPUT); // change this to 53 on a mega  // don't follow this!!
		digitalWrite(10, HIGH); // Add this line
		pinMode(53, OUTPUT); // change this to 53 on a mega  // don't follow this!!
		digitalWrite(53, HIGH); // Add this line

		if (isDebug)
		{
			Serial.println("--=== SETUP ===--");
			Serial.println("-----------------");
		}

		//setupDisplay();

		/* Initialise the sensor */
		/**
		if(!bmp.begin())
		{
		// There was a problem detecting the BMP085 ... check your connections //
		Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");

		u8g.firstPage();
		do {
		// W:5,H:7
		u8g.setFont(u8g_font_9x15);
		u8g.drawStr(0,37,"Cidlo nepripojeno");
		} while( u8g.nextPage() );

		while(1);
		}
		/**/

		if (isOledLoad)
		{
			/*isInitDisplay = *///setupDisplay();
			setupDisplayOzOled();
			//setupDisplayAdafruit();
		}

		if (isBMP180Load)
		{
			/*isInitBMP180 = */setupBMP180();
		}

		if (isBME280Load)
		{
			setupBME280();
		}

		if (isSDCardLoad)
		{
			/*isInitSDCard = */setupSDCard();

			if (isDebug)
			{
				Serial.print("SS: ");   Serial.println(SS);
				//    Serial.print("SS_PIN: ");   Serial.println(SS_PIN);
				Serial.print("MOSI: "); Serial.println(MOSI);
				//    Serial.print("MOSI_PIN: "); Serial.println(MOSI_PIN);
				Serial.print("MISO: "); Serial.println(MISO);
				//    Serial.print("MISO_PIN: "); Serial.println(MISO_PIN);  
				Serial.print("SCK: ");  Serial.println(SCK);
				//    Serial.print("SCK_PIN: ");  Serial.println(SCK_PIN);
			}
			//uint8_t const SS_PIN = SS;// 4;// SS;
			//uint8_t const MOSI_PIN = MOSI;// 5;// MOSI;
			//uint8_t const MISO_PIN = MISO;// 6;// MISO;
			//uint8_t const SCK_PIN = SCK
		}

		if (isGPSLoad)
		{
			//setupGPS();
			//setupTinyGPS();
			gpsSerial.begin(9600/*115200/*4800*/); // connect gps sensor
		}

		/* Display some basic information on this sensor */
		//ScreenInit();



	} // setup()

	  // Error messages stored in flash.
#define error(msg) sd.errorHalt(F(msg))


	  //char buff[30];
	char buff[71];
	void loop() {
		if (isDebug)
		{
			Serial.println("--=== Next loop ===--");
			Serial.println("---------------------");
		}
		//  counter++;
		//loopCounter++;

		// TEST

		//sprintf(buff, "TEST: %s","23:01:12 13.06.2016;961.000;21.100;444.467;[50.0355,14.3373]");
		//writeLine2SD(buff);

		/**
		sprintf(buff, "LoopCounter: %d", loopCounter);
		if(writeLine2SD(buff))
		//if (writeLine2SD("LoopCounter: "+loopCounter))
		{
		Serial.println("Zapisano");
		}
		else
		{
		Serial.println("Zapis se nezdaril!");
		}
		/**/

		if (isGPSLoad)
		{
			//  readGPS();
			readTinyGPS();
		}

		if (isBMP180Load)
		{
			readBMP180();
		}

		if (isBME280Load)
		{
			if (isDebug)
			{
				Serial.println("----Read BME280----");
			}
			readBME280();
		}

		/**
		// pro pripad, ze by bylo vice cidel
		// a kazde bychom chteli cist
		// v jinem intervalu
		if (counter > COUNT)
		{
		counter = 0;

		//readBMP085();
		readBMP180();

		//    draw();

		//    writeData2SD();
		}
		else
		{
		//pressure = temperature = altitude = 0.0f;
		}
		/**/



		/**
		// picture loop
		u8g.firstPage();
		do {
		draw();

		char buff[5];
		itoa (counter,buff,10);
		u8g.drawStr( 112, 10, buff);
		} while( u8g.nextPage() );
		/**/

		// Je treba to zapisovat v kazdem pruchodu, kvuli GPS
		draw();

		if (isSDCardLoad)
		{
			//writeData2SD_TEST();
			writeData2SD();
		}

		// Neni treba hlavni smycky, readery cidel, draw a writeData2SD maji vlastni timery
		// delay zpusob dela problemy s nacitanim GPS
		//  if(latitude > 0/*isGPSParsed*/)
		//  {
		//    // rebuild the picture after some delay
		//    delay(1000/*LOOP_TIME_MS*/); // zpozdeni zadat podle nejrychleji cteneho cidla
		//  }

	} // loop()

	  // ----------------- functions ---------------------------

	  /********************************
	  *       SETUP DISPLAY          *
	  ********************************/

	bool setupDisplayOzOled()
	{
		if (isDebug)
		{
			Serial.println("--=== setupDisplayOzOled() ===--");
			Serial.println("--------------------------------");
		}

		bool result = true;

		/**/
		OzOled.init();                   //initialze Oscar OLED display

										 // charge pump ON
		OzOled.sendCommand(0x8d);
		OzOled.sendCommand(0x14);

		// rotate 180
		OzOled.sendCommand(0xc8);
		OzOled.sendCommand(0xa1);
		OzOled.sendCommand(0x0da);
		OzOled.sendCommand(0x012);

		OzOled.setNormalDisplay();       //Set display to Normal mode
		OzOled.setHorizontalMode();      //Set addressing mode to Horizontal Mode
										 /**/
										 // Draw fixing logo
										 //  OzOled.drawBitmap(start_up_logo, 0, 0, 16, 8);

		if (isDebug)
		{
			Serial.println("--=== setupDisplayOzOled() - END ===--");
			Serial.println("---------------------------------------");
		}

		return result;
	}


	/********************************
	*        SETUP BMP180          *
	********************************/
	bool setupBMP180()
	{
		if (isDebug)
		{
			Serial.println("--=== setupBMP180() ===--");
			Serial.println("--------------------------");
		}

		bool result = true;

		// -----------------
		//  BMP180
		// -----------------
		if (isDebug)
		{
			Serial.println("Setup BMP180");
		}

		// We start the I2C on the Arduino for communication with the BMP180 sensor.
		Wire.begin();
		// Set up the Indicator LED.
		pinMode(indicatorLed, OUTPUT);
		// We create an instance of our BMP180 sensor.
		barometer = BMP180(); ////////////////////////////////////
							  // We check to see if we can connect to the sensor.
		if (barometer.EnsureConnected()) //////////////////////////
										 //  if (false)
		{
			if (isDebug)
			{
				Serial.println("Connected to BMP180."); // Output we are connected to the computer.
			}

			digitalWrite(indicatorLed, HIGH); // Set our LED.

											  // When we have connected, we reset the device to ensure a clean start.
			barometer.SoftReset(); //////////////////////////////////
								   // Now we initialize the sensor and pull the calibration data.
			barometer.Initialize(); /////////////////////////////////
		}
		else
		{
			if (isDebug)
			{
				Serial.println("No sensor found.");
			}

			digitalWrite(indicatorLed, LOW); // Set our LED.
			result = false;
		}

		if (isDebug)
		{
			Serial.println("--=== setupBMP180() - END ===--");
			Serial.println("-------------------------------");
		}

		return result;
	}

	/********************************
	*        SETUP BMPE280         *
	********************************/
	bool setupBME280()
	{
		if (isDebug)
		{
			Serial.println("--=== setupBME280() ===--");
			Serial.println("--------------------------");
		}

		if (!bme.begin()) {
			if (isDebug)
			{
				Serial.println("Could not find a valid BME280 sensor, check wiring!");
			}

			while (1);
		}

		bme.setTempCal(-1);

		if (isDebug)
		{
			Serial.println("--=== setupBME280() - END ===--");
			Serial.println("-------------------------------");
		}
	}

	/********************************
	*       SETUP SD CARD          *
	********************************/
	bool setupSDCard()
	{
		if (isDebug)
		{
			Serial.println("--=== setupSDCard() ===--");
			Serial.println("-------------------------");
		}

		const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
		//  char filename[13] = FILE_BASE_NAME "00.csv";
		//  filename = FILE_BASE_NAME "00.csv";

		//  char fileNameTmp[13] = FILE_BASE_NAME "00.log";
		//  sprintf(filename,"%s",fileNameTmp);

		bool result = true;

		/**/
		// Pro ATmega1284P
		//pinMode(10, OUTPUT);   
		//pinMode(SS, OUTPUT);
		//digitalWrite(SS, HIGH);

		pinMode(10, OUTPUT);

		pinMode(chipSelect, OUTPUT);
		digitalWrite(chipSelect, HIGH);
		/**/

		// SdFat
		//if (!sd.begin(chipSelect, SPI_HALF_SPEED)) {
		if (!sd.begin(chipSelect)) {
			sd.initErrorHalt();
		}

		// Find an unused file name.
		if (BASE_NAME_SIZE > 6) {
			error("FILE_BASE_NAME too long");
		}
		while (sd.exists(filename)) {
			if (filename[BASE_NAME_SIZE + 1] != '9') {
				filename[BASE_NAME_SIZE + 1]++;
			}
			else if (filename[BASE_NAME_SIZE] != '9') {
				filename[BASE_NAME_SIZE + 1] = '0';
				filename[BASE_NAME_SIZE]++;
			}
			else {
				error("Can't create file name");
			}
		}

		//  if (!file.open(filename, O_CREAT | O_WRITE | O_EXCL)) {
		//    error("file.open");
		//  }

		//char buff[30]; //4.9.2016
		if (file.open(filename, O_CREAT | O_WRITE | O_EXCL))
		{
			if (isDebug)
			{
				Serial.println("Writing to test.txt...");
			}

			file.println("Start Writing:::");
			isSaved = true;
		}
		else {
			// if the file didn't open, print an error:
			sprintf(buff, "error opening %s", filename);

			if (isDebug)
			{
				Serial.println(buff);
			}

			isSaved = false;
		}

		file.close(); // 14.6.2016 - testovani

		if (isDebug)
		{
			Serial.println("--=== setupSDCard() - END ===--");
			Serial.println("-------------------------------");
		}

		return result;
	}

	/********************************
	*       SETUP GPS MODUL        *
	********************************/
	// Inicialization of GPS modul is in the Setup() method.

	/********************************
	*         GPS FUNCTIONS        *
	********************************/

	//uint32_t timerReadGPS = millis();
	void readTinyGPS()
	{
		if (isDebug)
		{
			Serial.println("--=== readTinyGPS() ===--");
			Serial.println("-------------------------");
		}

		/**
		// if millis() or timer wraps around, we'll just reset it
		if (timerReadGPS > millis())  timerReadGPS = millis();

		// approximately every 2 seconds or so, print out the current stats
		if (millis() - timerReadGPS <= 2000)
		{
		return;
		}
		timerReadGPS = millis(); // reset the timer
		/**/

		while (gpsSerial.available()) { // check for gps data
			char/*byte*/ c = gpsSerial.read();

			if (isDebug)
			{
				Serial.print(c);
			}

			if (gps.encode(c)) { // encode gps data
				isGPSParsed = true;
				getgps(gps);
				/**/
				//      gps.get_position(&lat,&lon); // get latitude and longitude
				//      gps.get_datetime(&date, &time);

				// Define the variables that will be used
				//      float latitude, longitude;
				// Then call this function
				gps.f_get_position(&latitude, &longitude);

				// You can now print variables latitude and longitude      
				if (isDebug)
				{
					Serial.print("Lat/Long: ");
					Serial.print(latitude, 5);
					Serial.print(", ");
					Serial.println(longitude, 5);
				}

				// Same goes for date and time
				//      int year;
				//      byte month, day, hour, minute, second, hundredths;
				gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths);


				// Print data and time
				if (isDebug)
				{
					Serial.print("Date: "); Serial.print(month, DEC); Serial.print("/");
					Serial.print(day, DEC); Serial.print("/"); Serial.print(year);
					Serial.print("  Time: "); Serial.print(hour, DEC); Serial.print(":");
					Serial.print(minute, DEC); Serial.print(":"); Serial.print(second, DEC);
					Serial.print("."); Serial.println(hundredths, DEC);
				}

				//Since month, day, hour, minute, second, and hundr

				// display position
				//      Serial.print("Position: ");
				//      Serial.print("lat: ");Serial.print(lat);Serial.print(" ");// print latitude
				//      Serial.print("lon: ");Serial.println(lon); // print longitude
				/**/
			}
			else
			{
				//     Serial.println("GPS data not available...");
				isGPSParsed = false;
			}
			//Serial.print("COUNTER: "); Serial.println(counter);
		}

		if (isDebug)
		{
			Serial.println("--=== readTinyGPS() - END ===--");
			Serial.println("-------------------------------");
		}

	}

	// The getgps function will get and print the values we want.
	void getgps(TinyGPS &gps)
	{
		if (isDebug)
		{
			Serial.println("--=== getgps(TinyGPS &gps) ===--");
			Serial.println("--------------------------------");
		}

		// To get all of the data into varialbes that you can use in your code, 
		// all you need to do is define variables and query the object for the 
		// data. To see the complete list of functions see keywords.txt file in 
		// the TinyGPS and NewSoftSerial libs.

		// Define the variables that will be used
		//float latitude, longitude;
		// Then call this function
		gps.f_get_position(&latitude, &longitude);

		// You can now print variables latitude and longitude  
		if (isDebug)
		{
			Serial.print("Lat/Long: ");
			Serial.print(latitude, 5);
			Serial.print(", ");
			Serial.println(longitude, 5);
		}

		// Same goes for date and time
		//int year;
		//byte month, day, hour, minute, second, hundredths;
		gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths);

		// Print data and time
		if (isDebug)
		{
			Serial.print("Date: "); Serial.print(month, DEC); Serial.print("/");
			Serial.print(day, DEC); Serial.print("/"); Serial.print(year);
			Serial.print("  Time: "); Serial.print(hour, DEC); Serial.print(":");
			Serial.print(minute, DEC); Serial.print(":"); Serial.print(second, DEC);
			Serial.print("."); Serial.println(hundredths, DEC);
		}
		//Since month, day, hour, minute, second, and hundr

		// Here you can print the altitude and course values directly since 
		if (isDebug)
		{
			// there is only one value for the function
			Serial.print("Altitude (meters): "); Serial.println(gps.f_altitude());
			// Same goes for course
			Serial.print("Course (degrees): "); Serial.println(gps.f_course());
			// And same goes for speed
			Serial.print("Speed(kmph)\\: "); Serial.println(gps.f_speed_kmph());
			Serial.println();
		}


		// -----------------------
		long lat, lon; // create variable for latitude and longitude object (lat/lon are values in 100,000ths of a degree)
		unsigned long date, time; // date = ddmmyy, time = hhmmsscc, age = milliseconds
								  //char buff[30]; //4.9.2016

		gps.get_position(&lat, &lon); // get latitude and longitude
		gps.get_datetime(&date, &time/*, &age*/);

		if (isDebug)
		{
			sprintf(buff, "D:%d;%d", date, time);
			Serial.println(buff);

			Serial.print("La/Lo: ");
			Serial.print(lat);
			Serial.print(", ");
			Serial.println(lon);

			Serial.print("Position: ");
			Serial.print(lat, 4);
			Serial.print(", ");
			Serial.println(lon, 4);

			char c_Lat[10];
			char c_Lon[10];
			//  lon = 450230; // DEBUG
			dtostrf((float)((float)lat / 1000000.f), 4, 4, c_Lat);
			dtostrf((float)((float)lon / 1000000.f), 4, 4, c_Lon);
			sprintf(buff, "[%s,%s]", c_Lat, c_Lon);
			Serial.println(buff);
		}

		// -----------------------


		// Here you can print statistics on the sentences.
		unsigned long chars;
		unsigned short sentences, failed_checksum;
		gps.stats(&chars, &sentences, &failed_checksum);
		//Serial.print("Failed Checksums: ");Serial.print(failed_checksum);
		//Serial.println(); Serial.println();

		if (isDebug)
		{
			Serial.println("--=== getgps(TinyGPS &gps) - END ===--");
			Serial.println("--------------------------------------");
		}

	}

	/********************************
	*    READ DATA FROM BMP180     *
	********************************/
	char c_millis[21];
	char status;
	void readBMP180()
	{
		if (isDebug)
		{
			Serial.println("--=== readBMP180() ===--");
			Serial.println("--------------------------");
		}

		// if millis() or timer wraps around, we'll just reset it
		if (timerReadBMP180 > millis())  timerReadBMP180 = millis();

		// approximately every 2 seconds or so, print out the current stats
		if (millis() - timerReadBMP180 <= MS_READ_BMP180) // 5s
		{
			return;
		}
		timerReadBMP180 = millis(); // reset the timer

		ltoa(millis() / 1000, c_millis, 10);
		sprintf(buff, "--=== readBMP180(%s) ===--", c_millis);
		writeLine2SD(buff);

		/**/
		if (isDebug)
		{
			Serial.println("----Read BMP180----");
		}
		// Retrive the current pressure in Pascals.
		//long currentPressure = barometer.GetPressure();
		pressure = barometer.GetPressure() / 100;

		// Retrive the current altitude (in meters). Current Sea Level Pressure is required for this.
		altitude = barometer.GetAltitude(seaLevelPressure);

		// Retrive the current temperature in degrees celcius.
		temperature = barometer.GetTemperature();
		/**/

		if (isDebug)
		{
			Serial.println("--=== readBMP180() - END ===--");
			Serial.println("------------------------------");
		}
	}

	void readBME280()
	{
		if (isDebug)
		{
			Serial.println("----Read BME280----");
		}

		/**/

		// if millis() or timer wraps around, we'll just reset it
		if (timerReadBME280 > millis())  timerReadBMP180 = millis();

		// approximately every 2 seconds or so, print out the current stats
		if (millis() - timerReadBME280 <= MS_READ_BME280) // 5s
		{
			return;
		}
		timerReadBME280 = millis(); // reset the timer

									//-----------------------------------------------

		bme.readSensor();

		// Retrive the current pressure in hecto Pascals.
		pressure = bme.getPressure_HP();

		// Retrive the current temperature in degrees celcius.
		temperature = bme.getTemperature_C();

		// Retrive the current humidity in percentage
		humidity = bme.getHumidity();

		// Retrive the current altitude (in meters). Current Sea Level Pressure is required for this.
		altitude = bme.getAltitude(seaLevelPressure);

		/**/
		if (isDebug)
		{
			Serial.println("--=== readBME280() - END ===--");
			Serial.println("------------------------------");
		}
	}

	void WriteGPS2SerialPort()
	{
		/**
		Serial.print("\nTime: ");
		Serial.print(GPS.hour, DEC); Serial.print(':');
		Serial.print(GPS.minute, DEC); Serial.print(':');
		Serial.print(GPS.seconds, DEC); Serial.print('.');
		Serial.println(GPS.milliseconds);
		Serial.print("Date: ");
		Serial.print(GPS.day, DEC); Serial.print('/');
		Serial.print(GPS.month, DEC); Serial.print("/20");
		Serial.println(GPS.year, DEC);
		Serial.print("Fix: "); Serial.print((int)GPS.fix);
		Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
		if (GPS.fix)
		{
		Serial.print("Location: ");
		Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
		Serial.print(", ");
		Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
		Serial.print("Location (in degrees, works with Google Maps): ");
		Serial.print(GPS.latitudeDegrees, 4);
		Serial.print(", ");
		Serial.println(GPS.longitudeDegrees, 4);

		Serial.print("Speed (knots): "); Serial.println(GPS.speed);
		Serial.print("Angle: "); Serial.println(GPS.angle);
		Serial.print("Altitude: "); Serial.println(GPS.altitude);
		Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
		}
		/**/

		//    Serial.print("Position: ");
		//    Serial.print(lat, 4);
		//    Serial.print(", "); 
		//    Serial.println(lon, 4);
	}

	/********************************
	*    SHOW DATA ON DISPLAY      *
	********************************/

	int counter = COUNT;
	char c_counter[3];
	void draw(void) {
		if (isDebug)
		{
			Serial.println("--=== draw() ===--");
			Serial.println("------------------");
		}

		// if millis() or timer wraps around, we'll just reset it
		if (timerDraw > millis())  timerDraw = millis();

		// approximately every 2 seconds or so, print out the current stats
		if (millis() - timerDraw <= MS_DRAW) // 1s
		{
			return;
		}
		timerDraw = millis(); // reset the timer

		counter++;
		//itoa (counter,c_counter,10);
		ltoa(millis() / 1000, c_millis, 10);

		//sprintf(buff, "--=== draw(%s-%s) ===--", c_counter,c_millis);
		//writeLine2SD(buff);

		OzOled.printString(c_millis, 0, 1);


		if (counter > COUNT)
		{
			counter = 0;
		}

		//char buff[30]; //4.9.2016

		///  OzOled.clearDisplay(); 

		/// Header  
		sprintf(buff, "%s %s", TITLE, VERSION);
		if (isDebug)
		{
			Serial.println(buff);
		}
		OzOled.printString(buff, 0, 0);


		/// Statuses
		if (isDebug)
		{
			Serial.print("Counter: ");
			Serial.println(counter);
		}
		//itoa (counter,buff,10);
		sprintf(buff, "%-2d", counter);
		OzOled.printString(buff, 14, 1);//15

										/// GPS status
		if (isDebug)
		{
			Serial.print(isGPSParsed ? "G" : "g");
		}

		//OzOled.printString( (isGPSParsed ? "G" : "g"), 12, 1);
		if (isGPSParsed)
		{
			OzOled.printString("G", 12, 1);
		}
		else
		{
			OzOled.printString("g", 12, 1);
		}


		/// Save status
		if (isDebug)
		{
			Serial.print(isSaved ? "S" : "s");
		}

		//OzOled.printString( (isSaved ? "S" : "s"), 13, 1);//14
		if (isSaved)
		{
			OzOled.printString("S", 13, 1);
		}
		else
		{
			OzOled.printString("s", 13, 1);
		}

		// date+time  
		//  sprintf(buff,"D:%d;%d",date,time);
		//  sprintf(buff,"D:%d.%d;%d:%d:%d",day,month,hour,minute,second);
		sprintf(buff, "D:%02d:%02d:%02d %02d.%02d.%04d", hour, minute, second, day, month, year);
		if (isDebug)
		{
			Serial.println(buff);
		}
		OzOled.printString(buff, 0, 2);

		/////////
		// BMP180
		/////////  
		/**
		/// Pressure
		dtostrf(pressure, 4, 3, charVal);
		sprintf(buff, "P:%s hPa",charVal);
		Serial.println(buff);
		OzOled.printString(buff, 0, 3);

		/// Temperature
		dtostrf(temperature, 4, 3, charVal);
		sprintf(buff, "T:%s °C",charVal);
		Serial.println(buff);
		OzOled.printString(buff, 0, 4);

		/// Altitude
		//dtostrf(bmp.pressureToAltitude(seaLevelPressure,event.pressure), 4, 3, charVal);
		dtostrf(altitude, 4, 3, charVal);
		sprintf(buff, "A:%s m", charVal);
		Serial.println(buff);
		OzOled.printString(buff, 0, 5);
		/**/

		/////////
		// BME280
		/////////
		/// Pressure  
		dtostrf(pressure, 4, 3, charVal);
		sprintf(buff, "P:%s hPa", charVal);
		if (isDebug)
		{
			Serial.println(buff);
		}
		OzOled.printString(buff, 0, 3);

		/// Temperature
		dtostrf(temperature, 4, 3, charVal);
		sprintf(buff, "T:%s °C", charVal);
		if (isDebug)
		{
			Serial.println(buff);
		}
		OzOled.printString(buff, 0, 4);

		/// Altitude
		//dtostrf(bmp.pressureToAltitude(seaLevelPressure,event.pressure), 4, 3, charVal);
		dtostrf(altitude, 4, 3, charVal);
		sprintf(buff, "A:%s m", charVal);
		if (isDebug)
		{
			Serial.println(buff);
		}
		OzOled.printString(buff, 0, 5);

		/// Humidity
		dtostrf(humidity, 4, 3, charVal);
		sprintf(buff, "H:%s %", charVal);
		if (isDebug)
		{
			Serial.println(buff);
		}
		OzOled.printString(buff, 0, 6);




		/**/

		/// GPS
		char c_Lat[10];
		char c_Lon[10];
		//  dtostrf(GPS.latitudeDegrees, 4, 4, c_Lat);
		//  dtostrf(GPS.longitudeDegrees, 4, 4, c_Lon);   

		//  lon = 450230; // DEBUG
		//  dtostrf((float)((float)lat/100000.f), 4, 4, c_Lat);
		//  dtostrf((float)((float)lon/100000.f), 4, 4, c_Lon);   
		//  sprintf(buff,"[%s,%s]",c_Lat,c_Lon);
		dtostrf(latitude, 4, 4, c_Lat);
		dtostrf(longitude, 4, 4, c_Lon);
		sprintf(buff, "Lat:%s, Lon:%s", c_Lat, c_Lon);
		//  sprintf(buff,"[%.5f,%.5f]",latitude,longitude);
		if (isDebug)
		{
			Serial.println(buff);
		}
		OzOled.printString(buff, 0, 7);

		if (isDebug)
		{
			Serial.println("--=== draw() - END ===--");
			Serial.println("------------------------");
		}
	}


	void ScreenInit()
	{
		if (isDebug)
		{
			Serial.println("--=== ScreenInit() ===--");
			Serial.println("------------------------");
		}

		//Draw fixing logo
		//OzOled.drawBitmap(OscarLogo/*start_up_logo*/, 0, 0, 16, 8);
		delay(2000);
		OzOled.clearDisplay();

		if (isDebug)
		{
			Serial.println("--=== ScreenInit() - END ===--");
			Serial.println("------------------------------");
		}
	}

	/********************************
	*     WRITE DATA TO SD CARD    *
	********************************/

	char c_Lat[10];
	char c_Lon[10];
	void writeData2SD()
	{
		if (isDebug)
		{
			Serial.println("--=== writeData2SD() ===--");
			Serial.println("--------------------------");

			Serial.print("Filename: ");
			Serial.println(filename);
		}

		// if millis() or timer wraps around, we'll just reset it
		if (timerWriteData2SD > millis())  timerWriteData2SD = millis();

		// approximately every 2 seconds or so, print out the current stats
		if (millis() - timerWriteData2SD <= MS_WRITE_DATA2SD) // 10s
		{
			return;
		}
		timerWriteData2SD = millis(); // reset the timer

									  //ltoa(millis()/1000,c_millis,10);
									  //sprintf(buff, "--=== writeData2SD(%s) ===--",c_millis);
									  //writeLine2SD(buff);

									  //char buff[30];
									  //sprintf(buff,"%02d:%02d:%02d %02d.%02d.%04d;",hour,minute,second,day,month,year);

									  // Debug
									  //  ltoa(millis()/1000,c_millis,10);
		sprintf(buff, "[[%d]]", millis() / 1000);
		sprintf(buff, "%s %02d:%02d:%02d %02d.%02d.%04d;", buff, hour, minute, second, day, month, year);
		//file.print(buff);
		write2SD(buff);
		if (isDebug)
		{
			Serial.print(buff);
		}

		/////////////////// 
		// read from BMP180 / BME280
		///////////////////  
		/**
		// pressure
		dtostrf(pressure, 4, 3, charVal);
		//dtostrf((barometer.GetPressure() / 100), 4, 3, charVal);
		sprintf(buff,"%s",charVal);


		// temperature
		dtostrf(temperature, 4, 3, charVal);
		// Retrive the current temperature in degrees celcius.
		//dtostrf(barometer.GetTemperature(), 4, 3, charVal);
		sprintf(buff,"%s;%s",buff,charVal);

		// altitude
		dtostrf(altitude, 4, 3, charVal);
		// Retrive the current altitude (in meters). Current Sea Level Pressure is required for this.
		//dtostrf(barometer.GetAltitude(seaLevelPressure), 4, 3, charVal);
		sprintf(buff,"%s;%s;",buff,charVal);
		/**/

		///////////////////
		// read from BME280
		///////////////////
		// pressure
		//dtostrf(bme.getPressure_HP(), 4, 3, charVal);
		dtostrf(pressure, 4, 3, charVal);
		sprintf(buff, "%s", charVal);

		// temperature
		//dtostrf(bme.getTemperature_C(), 4, 3, charVal);
		dtostrf(temperature, 4, 3, charVal);
		sprintf(buff, "%s;%s", buff, charVal);

		// humidity
		//dtostrf(bme.getHumidity(), 4, 3, charVal);  
		dtostrf(humidity, 4, 3, charVal);
		sprintf(buff, "%s;%s", buff, charVal);

		// altitude
		//dtostrf(bme.getAltitude(seaLevelPressure), 4, 3, charVal);  
		dtostrf(altitude, 4, 3, charVal);
		sprintf(buff, "%s;%s;", buff, charVal);


		//file.print(buff);
		write2SD(buff);
		if (isDebug)
		{
			Serial.print(buff);
		}


		dtostrf(latitude, 4, 4, c_Lat);
		dtostrf(longitude, 4, 4, c_Lon);
		sprintf(buff, "[%s,%s]", c_Lat, c_Lon);
		//file.println(buff);
		writeLine2SD(buff);
		if (isDebug)
		{
			Serial.println(buff);
		}


		// ===========================================


		/**
		//if (file.open(filename, O_CREAT | O_WRITE | O_EXCL))
		if (file.open(filename, O_CREAT | O_APPEND | O_WRITE))
		{
		Serial.println("Writing to test.txt...");
		//char *buff;
		//buff = setFileLine();

		Serial.println("--===Before write Date and Time ===--");
		// date + time
		//      sprintf(buff,"[[%d;%d]]",date,time);
		// Same goes for date and time
		//      gps.crack_datetime(&year,&month,&day,&hour,&minute,&second,&hundredths);
		sprintf(buff,"%02d:%02d:%02d %02d.%02d.%04d;",hour,minute,second,day,month,year);
		file.print(buff);
		Serial.println(buff);

		//      buff[0] = '/0';
		//      file.println();

		Serial.println("--===Before write Pressure and Temperature and Altitude ===--");
		// pressure
		dtostrf(pressure, 4, 3, charVal);
		sprintf(buff,"%s",charVal);


		// temperature
		dtostrf(temperature, 4, 3, charVal);
		sprintf(buff,"%s;%s",buff,charVal);

		// altitude
		dtostrf(altitude, 4, 3, charVal);
		sprintf(buff,"%s;%s;",buff,charVal);

		file.print(buff);
		Serial.println(buff);


		Serial.println("--===Before write GPS ===--");
		// ===========================================
		// ----------------- GPS ---------------------
		//      sprintf(buff,"%d:%d %d.%d.20%d",GPS.hour,GPS.minute,GPS.day,GPS.month,GPS.year);
		//      file.println(buff);

		//      sprintf(buff,"F:%d,Q:%d",(int)GPS.fix,(int)GPS.fixquality);
		//      file.println(buff);

		char c_Lat[10];
		char c_Lon[10];
		//      dtostrf(GPS.latitudeDegrees, 4, 4, c_Lat);
		//      dtostrf(GPS.longitudeDegrees, 4, 4, c_Lon);

		//      dtostrf((float)((float)lat/100000.f), 4, 4, c_Lat);
		//      dtostrf((float)((float)lon/100000.f), 4, 4, c_Lon);
		//      sprintf(buff,"[%s,%s]",c_Lat,c_Lon);

		dtostrf(latitude, 4, 4, c_Lat);
		dtostrf(longitude, 4, 4, c_Lon);
		sprintf(buff,"[%s,%s]",c_Lat,c_Lon);

		//      sprintf(buff,"[%.5f,%.5f]",latitude,longitude);

		//      sprintf(buff,"[%d,%d]",lat/100000,lon/100000);
		Serial.println(buff);


		// ===========================================

		file.println(buff);

		// close the file:
		//      file.close();
		Serial.println("done.");
		Serial.println("--=== DONE ===--");
		// Force data to SD and update the directory entry to avoid data loss.
		if (!file.sync() || file.getWriteError()) {
		error("write error");
		isSaved = false;
		Serial.println("--=== Sync Error ===--");
		}
		isSaved = true;
		file.close(); // 14.6.2016 - testovani
		} else {
		//      // if the file didn't open, print an error:
		//      sprintf(buff, "error opening %s",filename);
		//      Serial.println(buff);
		isSaved = false;
		Serial.println("--=== isSaved = FALSE ===--");
		}

		Serial.println("--=== writeData2SD() - END ===--");
		Serial.println("--------------------------------");

		/**/
	} // writeData2SD()



	bool write2SD(char *s)
	{
		if (isDebug)
		{
			Serial.println("---------------------writeLine2SD()");
		}
		if (file.open(filename, O_CREAT | O_APPEND | O_WRITE))
		{
			file.print(s);

			// Force data to SD and update the directory entry to avoid data loss.
			if (!file.sync() || file.getWriteError())
			{
				file.close();
				return false;
			}
			file.close();
		}

		return true;
	}

	//bool writeLine2SD(String s)
	bool writeLine2SD(char *s)
	{
		if (isDebug)
		{
			Serial.println("---------------------writeLine2SD()");
		}
		if (file.open(filename, O_CREAT | O_APPEND | O_WRITE))
		{
			file.println(s);

			// Force data to SD and update the directory entry to avoid data loss.
			if (!file.sync() || file.getWriteError())
			{
				file.close();
				return false;
			}
			file.close();
		}
		else // pokus o opetovne zapsani do noveho souboru
		{
			setupSDCard();
			writeLine2SD(s);
		}

		return true;
	}
}
