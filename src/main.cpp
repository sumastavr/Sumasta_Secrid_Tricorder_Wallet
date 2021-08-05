#include <Arduino.h>
#define FASTLED_ALLOW_INTERRUPTS 0
//#define FASTLED_INTERRUPT_RETRY_COUNT 1

//#define BLYNK_PRINT Serial

//#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <FastLED.h>
#include <Adafruit_Sensor.h>
#include <TimeLib.h>
#include <WidgetRTC.h>
#include <Adafruit_APDS9960.h>
#include <Adafruit_BME280.h>
#include <MPU6050.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_VEML6070.h>
#include <Adafruit_VL53L0X.h>

#include <Adafruit_CCS811.h>

BlynkTimer timer;
WidgetRTC rtc;

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "f39679acbc844dbb8ae6927c22abd88d";

// Your WiFi credentials.
// Set password to "" for open networks.

//char ssid[] = "ASUS";
//char pass[] = "EmmAStrAAt12";

String currentTimex ="Sumasta";

// RGB Shades data output to LEDs is on pin 5
#define LED_PIN  D5

// RGB Shades color order (Green/Red/Blue)
#define COLOR_ORDER RGB
//#define CHIPSET     WS2811
#define CHIPSET     WS2812

// Global maximum brightness value, maximum 255
#define MAXBRIGHTNESS 255
#define STARTBRIGHTNESS 25

// Cycle time (milliseconds between pattern changes)
#define cycleTime 5000

// Hue time (milliseconds between hue increments)
//#define hueTime 30
#define hueTime 60


// Time after changing settings before settings are saved to EEPROM
#define EEPROMDELAY 2000

#define BUTTON D3

boolean active=true;
uint8_t currentPWM = 100;

#include <Wire.h>
// Include FastLED library and other useful files

#include <EEPROM.h>
#include "font.h"

void initTCS();
uint16_t getLux();
void showTemperature();
void confetti();
void showBanner();
void showPressure();
void showHumidity();
void glitter();
void demo();
void adjustBrightnessAccel();
void menuSelectAccel();
void initBME();
void initMPU();
float getTiltAccel();
void initCCS811();
void debugCCS811();
void debugBME();
void  initVL53(); 
void debugDistance(); 
void initVEML(); 
void debugVEML();
void initTCS(); 
void debugTCS(); 
void initAPDS(); 
void debugAPDS(); 
		
    
   void initMPU(); 
    
   void debugMPU();
   void switchMenuDebug(char menuNow);
   void selectMenu();

  void runGyroSerial();
  void runTemperature();
void runHumidity();
void runPressure();
void runAltitute();

void runVEML();
void moooiRealDistance();
void runAccelSerial();

		void runLux();

		void runCCT();

		void runRGBSensor();

		void runGesture();
void runTVOC();
void runCO2();
void debugAPDSnoloop();
void threeSine();
void plasma();
void rider();
void slantBars();
void sideRain();

void hueCycle(byte incr);
void showFlashTextRedGreenFade(String text, int colorHue);
void showFlashText(String text);

void fadeAll(byte fadeIncr);
void runTextRGB(String text, int speed, byte r, byte g, byte b) ;

void showMenuTitle(byte menuCode) ;


void showBanner();

void showBannerGift() ;
void runText(String text, int speed);
void confirmBlink();
void scrollArray(byte scrollDir) ;

uint8_t XY( uint8_t x, uint8_t y);

void autoDimmingHold();
void selectRandomPalette();
void loadCharBuffer(byte character);

void dimmingRoutine();
void autoDimming();
void dimmingRoutineDistance();
void moooiDistanceProximityDim();

// Global variables
boolean effectInit = false; // indicates if a pattern has been recently switched
uint16_t effectDelay = 0; // time between automatic effect changes
unsigned long effectMillis = 0; // store the time of last effect function run
unsigned long cycleMillis = 0; // store the time of last effect change
unsigned long currentMillis; // store current loop's millis value
unsigned long hueMillis; // store time of last hue change
unsigned long eepromMillis; // store time of last setting change
byte currentEffect = 0; // index to the currently running effect
boolean autoCycle = true; // flag for automatic effect changes
boolean eepromOutdated = false; // flag for when EEPROM may need to be updated
byte currentBrightness = STARTBRIGHTNESS; // 0-255 will be scaled to 0-MAXBRIGHTNESS

CRGBPalette16 currentPalette(RainbowColors_p); // global palette storage

typedef void (*functionList)(); // definition for list of effect function pointers
extern const byte numEffects;

// Increment the global hue value for functions that use it
byte cycleHue = 0;
byte cycleHueCount = 0;


// Params for width and height
const byte kMatrixWidth = 24;
const byte kMatrixHeight = 5;


#define NUM_LEDS (kMatrixWidth * kMatrixHeight)
CRGB leds[ NUM_LEDS ];


byte charBuffer[5] = {0};

// This function will return the right 'led index number' for 
// a given set of X and Y coordinates on your RGB Shades. 
// This code, plus the supporting 80-byte table is much smaller 
// and much faster than trying to calculate the pixel ID with code.
#define LAST_VISIBLE_LED 242

// Runs one time at the start of the program (power up or reset)
void setup() {


  // write FastLED configuration data
  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, LAST_VISIBLE_LED + 1);

  //EEPROM.write(77, 40);
  // set global brightness value
  FastLED.setBrightness(scale8(STARTBRIGHTNESS, MAXBRIGHTNESS));
  //FastLED.setBrightness(EEPROM.read(77));

  //rtc.begin();
  Serial.begin(115200);

  Serial.println(EEPROM.read(77));

  Serial.println(numEffects);

  pinMode(BUTTON, INPUT_PULLUP);
  
  Serial.print("Setting soft-AP ... ");
  
  //Serial.println(WiFi.softAP("SUMAS+A is in Proximity", "",1,0,12) ? "Ready" : "Failed!");

  WiFi.forceSleepBegin();

  //debug();

}

void adjustBrightness() {

	return;
	// skip this function below, auto adjust brightness function below not optimized

	Wire.endTransmission();
	delay(10);

	initTCS();
	delay(100);

	int luxNow = getLux();

	Serial.print("LUX Now: ");
	Serial.println(luxNow);

	Wire.endTransmission();

	switch (luxNow) {
	case 0 ... 30: 
		FastLED.setBrightness(0);
		FastLED.show();
		while (1) {
			delay(500);
			luxNow=getLux();
			if (luxNow > 30) break;
			Serial.println(luxNow);
		}
		adjustBrightness();
		break;
	case 31 ... 250: FastLED.setBrightness(20); break;
	case 251 ... 1000: FastLED.setBrightness(40); break;
	case 1001 ... 10000: FastLED.setBrightness(80); break;
	case 10001 ... 20000: FastLED.setBrightness(120); break;
	case 20001 ... 30000: FastLED.setBrightness(160); break;
	default: FastLED.setBrightness(200); break;


	}

}

void debug() {
	Serial.println("Start Debug");
	for (int i = 0; i < 5; i++) {
		Serial.println(currentTimex.charAt(i));
	}
	while (1);
}

// list of functions that will be displayed
functionList effectList[] = {threeSine,
                             showTemperature,                      
                             plasma,
							 showBanner,
							 confetti,
                             rider,
							 showPressure,
							 glitter,
							 showHumidity,
                             slantBars,
							 sideRain,
                            };

const byte numEffects = (sizeof(effectList)/sizeof(effectList[0]));

byte menuNow = 1;
byte previousMenu = menuNow;
byte menuCounter = 0;
bool flagAdjustBrightness = true;

// Runs over and over until power off or reset
void loop(){
	demo();
}

void demo() {
	menuNow = 0;
	Serial.println("Start");

	while (1) {
		
		//checkButton();

		if (digitalRead(BUTTON) == LOW) {
			//delay(200);
			
			int basicCounter = 0;
			boolean timerPassed = false;
			while (digitalRead(BUTTON) == LOW) {
				basicCounter++;

				if (basicCounter > 800) {
					adjustBrightnessAccel();
					timerPassed = true;
				} 
				delay(1);
			}

			if (!timerPassed) {		
				menuSelectAccel();
			}
			
			
		}

		//Serial.println("Start1");

		currentMillis = millis(); // save the current timer value
								  
		if (currentMillis - cycleMillis > cycleTime && autoCycle == true) {		
			cycleMillis = currentMillis;
			if (++currentEffect >= numEffects) currentEffect = 0; flagAdjustBrightness = true; // loop to start of effect list
			effectInit = false; // trigger effect initialization when new effect is selected
			//adjustBrightness();
		}

		//Serial.println("Start1a");

		// increment the global hue value every hueTime milliseconds
		if (currentMillis - hueMillis > hueTime) {
			hueMillis = currentMillis;
			hueCycle(1); // increment the global hue value
		}

		//Serial.println("Start1b");

		// run the currently selected effect every effectDelay milliseconds
		if (currentMillis - effectMillis > effectDelay) {
			effectMillis = currentMillis;
			effectList[currentEffect](); // run the selected effect function
			random16_add_entropy(1); // make the random values a bit more random-ish
		}

		//Serial.println("Start2");

		// run a fade effect too if the confetti effect is running
		if (effectList[currentEffect] == confetti) fadeAll(1);

		
		if (currentEffect == 1 && flagAdjustBrightness) {
			adjustBrightness();
			flagAdjustBrightness = false;
		}
		

		// Serial debug for all sensors
		/*
		if (Serial.available() > 0) {
			switchMenuDebug(Serial.read());
		}
		*/

		//Serial.println("Start3");

		FastLED.show(); // send the contents of the lt
		FastLED.delay(1000 / 200);

	}

}

void checkButton() {
	if (digitalRead(BUTTON) == LOW) {

		delay(200);
		Serial.println("Main menu!");
		while (digitalRead(BUTTON) == LOW);

		menuSelectAccel();
		//switchMenu();
		//switchMenuDebug();
	}

	if (Serial.available() > 0) {
		switchMenuDebug(Serial.read());
	}

}

void adjustBrightnessAccel() {
	for (int i = 0; i < NUM_LEDS*2; i++) {
		leds[i] = CRGB::BlueViolet;
	}
	FastLED.show();

	delay(2000);

	Wire.endTransmission();
	delay(100);
	initBME();
	delay(200);
	Wire.endTransmission();
	delay(100);
	initMPU();

	int intensity;

	while (1) {
		
		float tilt = getTiltAccel();

		if (tilt < 1.00) {
			intensity = tilt * 100;

			intensity = map(intensity, 0, 100, 0, 200);

			FastLED.setBrightness(intensity);
			FastLED.show();
			delay(100);

		}	

		if (digitalRead(BUTTON) == LOW) break;
		delay(25);
	}

	delay(1000);

}

void switchMenuDebug(char menuNow) {
	
	switch (menuNow) {
		case '1': Wire.begin(); delay(300); Wire.endTransmission(); delay(100); initCCS811(); debugCCS811(); break;
		case '2': Wire.endTransmission(); delay(200); initBME(); debugBME(); break;
		case '3': Wire.endTransmission(); delay(200); initVL53(); debugDistance(); break;
		case '4': Wire.endTransmission(); delay(200); initVEML(); debugVEML(); break;
		case '5': Wire.endTransmission(); delay(200); initTCS(); debugTCS(); break;
		case '6': Wire.endTransmission(); delay(200); initAPDS(); debugAPDS(); break;
		case '7': Wire.endTransmission(); delay(200); initMPU(); debugMPU(); break;
	}
}

#define TOTALMENU	16

String menuTitleShort[TOTALMENU + 1] = {	" ","TEMP",// 1
											"HUMI",// 2 
											"BARO",// 3
											"ALTI",// 4
											"UVLV",// 5
											"TOFD",// 6 
											"LUXL",// 7
											"CCTL",// 8
											"RGBC",// 9
											"GEST",// 10
											"CO2L",// 11
											"TVOC",// 12
											"ACCE",// 13
											"GYRO",// 14
											"EXIT" }; //15

void menuSelectAccel() {

	Serial.println("ENTER Accel Menu:");

	showFlashText("MENU");
	showFlashText("MENU");

	Wire.endTransmission();
	delay(100);
	initBME();
	delay(200);
	Wire.endTransmission();
	delay(100);
	initMPU();

	long timeoutThisMenu = millis();

	while (1) {

		if (getTiltAccel() > 31.00) {
			menuNow++;
			if (menuNow >= TOTALMENU) menuNow = 1;

			showFlashTextRedGreenFade(menuTitleShort[menuNow],menuNow * 15);
			do {
				delay(25);
			}while (getTiltAccel() > 31.00);

			delay(500);

			Serial.print("Menu Now: ");
			Serial.println(menuNow);
			timeoutThisMenu = millis();

		}

		if (digitalRead(BUTTON) == LOW) {
			delay(250);
			while (digitalRead(BUTTON) == LOW);
			selectMenu();
		}

		if (millis() - timeoutThisMenu > 60000) {
			Serial.println("Timeout");
			demo();
		}

		delay(25); // 40hz sample
	}
}


void selectMenu() {

	//showMenuTitle(menuNow-1);

	switch (menuNow) {
	// Temperature measurement
	case 1: // BME can start right away, no preconditional initialization
		Wire.endTransmission(); 
		delay(200); 
		initBME();
		runTemperature();
		break;
	
	// Humidity measurements
	case 2: // // BME can start right away, no preconditional initialization
		Wire.endTransmission();
		delay(200);
		initBME();
		runHumidity();
		break;
	
	// Air Pressure measurements
	case 3: // // BME can start right away, no preconditional initialization
		Wire.endTransmission();
		delay(200);
		initBME();
		runPressure();
		break;

	// Calculated altitude
	case 4: // // BME can start right away, no preconditional initialization
		Wire.endTransmission();
		delay(200);
		initBME();
		runAltitute();
		break;

	// Measure UV level Index
	case 5: // 

		// check if previously used i2c bus is wire2, initialize dummy connection with bme. 
		if (previousMenu >= 7 && previousMenu <= 10) {
			Wire.endTransmission();
			delay(200);
			initBME();
		}

		Wire.endTransmission();
		delay(200);
		initVEML();
		runVEML();
		break;

	// Measure distance TOF
	case 6: //

		// check if previously used i2c bus is wire2, initialize dummy connection with bme. 
		if (previousMenu >= 7 && previousMenu <= 10) {
			Wire.endTransmission();
			delay(200);
			initBME();
		}

		Wire.endTransmission();
		delay(200);
		initVL53();
		moooiRealDistance();
		break;
	
	// Measure LUX level Index
	case 7: // 

		// check if previously used i2c bus is wire2, initialize dummy connection with bme. 
		if (previousMenu >= 7 && previousMenu <= 10) {
			Wire.endTransmission();
			delay(200);
			initBME();
		}

		Wire.endTransmission();
		delay(200);
		initTCS();
		runLux();
		break;
	
	// Measure CCT level Index
	case 8: // 
		Wire.endTransmission();
		delay(200);
		initTCS();
		runCCT();
		break;

	// Measure RGB level Index
	case 9: // 
		Wire.endTransmission();
		delay(200);
		initTCS();
		runRGBSensor();
		break;

	// Measure Air Gesture
	case 10: // 
		Wire.endTransmission();
		delay(200);
		initAPDS();
		runGesture();
		break;
	
	// Measure Air Gesture
	case 11: // 
		runTextRGB("Init.. restart unit if no response", 75,200,0,0);
		Wire.endTransmission();
		delay(300);
		initAPDS();
		debugAPDSnoloop();
		debugAPDSnoloop();
		Wire.endTransmission();
		delay(500);
		Wire.begin();
		delay(500);
		Wire.endTransmission();
		delay(200);
		initCCS811();
		delay(200);
		initCCS811();
		runCO2();
		break;
	
		// Measure Air Gesture
	case 12: // 
		runTextRGB("Init.. restart unit if no response", 75, 200, 0, 0);
		Wire.endTransmission();
		delay(300);
		initAPDS();
		debugAPDSnoloop();
		debugAPDSnoloop();
		Wire.endTransmission();
		delay(500);
		Wire.begin();
		delay(500);
		Wire.endTransmission();
		delay(200);
		initCCS811();
		runTVOC();
		break;

	// accelerometer demo
	case 13:

		runTextRGB("Connect USB W/ Serial interface 9600 bps", 75, 200, 0, 0);
		// check if previously used i2c bus is wire2, initialize dummy connection with bme. 
		if (previousMenu >= 7 && previousMenu <= 10) {
			Wire.endTransmission();
			delay(200);
			initBME();
		}

		Wire.endTransmission();
		delay(200);
		initMPU();
		runAccelSerial();
		break;

	// Gyroscope measurements
	case 14:
		runTextRGB("Connect USB W/ Serial interface 9600 bps", 75, 200, 0, 0);
		// check if previously used i2c bus is wire2, initialize dummy connection with bme. 
		if (previousMenu >= 7 && previousMenu <= 10) {
			Wire.endTransmission();
			delay(200);
			initBME();
		}

		Wire.endTransmission();
		delay(200);
		initMPU();
		runGyroSerial();
		break;
		break;

	// exit
	case 15:
		demo();
		break;
	}


	// assign previous menu to current menu
	previousMenu = menuNow;

	/*

	switch (menuNow) {
	case 1: Wire.begin(); delay(200); Wire.endTransmission(); delay(100); initCCS811(); runCO2(); break;
	case 2: runTVOC(); break;
	case 3: Wire.endTransmission(); delay(100); initBME(); runTemperature(); break;
	case 4: runHumidity(); break;
	case 5: runPressure(); break;
	case 6: runAltitute(); break;
	case 7: Wire.endTransmission(); delay(100); initVL53(); runTestDistance(); break;
	case 8: FastLED.setBrightness(STARTBRIGHTNESS); moooiRealDistance(); break;
	case 9: Wire.endTransmission(); delay(100); initVEML(); runVEML(); break;
	case 10: Wire.endTransmission(); delay(100); initTCS(); runLux(); break;
	case 11: runCCT(); break;
	case 12: runRGBSensor(); break;
	case 13: Wire.endTransmission(); delay(100); initAPDS(); runGesture(); break;
	case 14: runText("  rebooting.... ",90); demo(); break;//Serial.println("RESET"); WiFi.forceSleepBegin(); wdt_reset(); ESP.restart(); while (1)wdt_reset(); break;
	}

	*/
}

void rese() {
	byte rs[5];

	for (int x = 0; x < 7; x++) {
		rs[x] = 9;
	}
}

String MenuTitles[TOTALMENU] = { "Gas Sensor: Carbon Dioxide Level",			//1
						"Gas Sensor: Total Volatile Organic Compound",	//2
						"Temperature Sensor: Ambient in Celcius",		//3
						"Humidity Sensor: Ambient in percentage",		//4
						"Pressure Sensor: Ambient in Pascal",			//5
						"Approximated Altitude: In meters",				//6
						"Distance Sensor: Time of Flight, dimming mode",//7
						"Distance sensor: time of flight real distance",//8
						"UV Level sensor: Ambient UV index level",		//9
						"Lux Light level Sensor",						//10
						"CCT Light Level Sensor",						//11
						"Color sensor: RGB Intensity",					//12
						"Simple Gesture sensor Swipe"					//13
						};

void showMenuTitle(byte menuCode) {
	runText(MenuTitles[menuCode], 90);
}

void showBanner() {
	//runText((String)bme.readPressure() + " hPa", 100);
	runText("Sumasta Tricorder Wallet", 85);
}

void showBannerGift() {
	//runText((String)bme.readPressure() + " hPa", 100);
	runText("Michel Germe, SIGNIFY LIFI Venture", 85);
}


Adafruit_APDS9960 apds;

void checkButton();
void showFlashText(String text);

void initAPDS() {
	if (!apds.begin()) {
		Serial.println("failed to initialize device! Please check your wiring.");
	}
	else Serial.println("Device initialized!");
}

void debugAPDS() {
	//gesture mode will be entered once proximity mode senses something close
	apds.enableProximity(true);
	apds.enableGesture(true);

	while (1) {
		//read a gesture from the device
		uint8_t gesture = apds.readGesture();
		if (gesture == APDS9960_DOWN) Serial.println("v");
		if (gesture == APDS9960_UP) Serial.println("^");
		if (gesture == APDS9960_LEFT) Serial.println("<");
		if (gesture == APDS9960_RIGHT) Serial.println(">");
		
		checkButton();
		delay(5);
	}
}

void debugAPDSnoloop() {
	//read a gesture from the device
	uint8_t gesture = apds.readGesture();
	if (gesture == APDS9960_DOWN) Serial.println("v");
	if (gesture == APDS9960_UP) Serial.println("^");
	if (gesture == APDS9960_LEFT) Serial.println("<");
	if (gesture == APDS9960_RIGHT) Serial.println(">");

}

void runGesture() {
	//gesture mode will be entered once proximity mode senses something close
	apds.enableProximity(true);
	apds.enableGesture(true);
	while (1) {
		//read a gesture from the device
		uint8_t gesture = apds.readGesture();
		if (gesture == APDS9960_DOWN) { Serial.println("v"); showFlashText("RIGHT"); }
		if (gesture == APDS9960_UP) {Serial.println("^"); showFlashText("LEFT");}
		if (gesture == APDS9960_LEFT) { Serial.println("<"); showFlashText("DOWN"); }
		if (gesture == APDS9960_RIGHT) { Serial.println(">"); showFlashText(" UP "); }

		checkButton();
		delay(5);
	}
}




#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
					 //Adafruit_BME280 bme(BME_CS); // hardware SPI
					 //Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

unsigned long delayTime;

void runTextRGB(String text, int speed, byte r, byte g, byte b);
void runText(String text, int speed);
void checkButton();

#define SCROLL_SPEED_FAST	50
#define SCROLL_SPEED_NORMAL	75
#define SCROLL_SPEED_SLOW	100

void initBME() {

	bool status;

	// default settings
	// (you can also pass in a Wire library object like &Wire2)
	Wire.begin(4, 5);
	status = bme.begin();
	if (!status) {
		Serial.println("Could not find a valid BME280 sensor, check wiring!");
		//while (1);
	}

	delayTime = 1000;

	Serial.println();
}

void showTemperature() {
	//runText((String)bme.readTemperature() + " Celcius", 100);

	initBME();
	delay(10);

	float offset = 0.0;
	runTextRGB((String)(bme.readTemperature()-offset) + " Celcius System Temp", SCROLL_SPEED_NORMAL,20,random(250),random(250));
}

void showPressure() {
	//runText((String)bme.readPressure() + " hPa", 100);
	runTextRGB((String)bme.readPressure() + " hPa", SCROLL_SPEED_NORMAL,random(250),20, random(250));
}

void showHumidity() {
	//runText((String)bme.readHumidity() + " %", 100);
	runTextRGB((String)bme.readHumidity() + " %", SCROLL_SPEED_NORMAL, random(250),random(250),20);
}

void showAltitude() {
	runText((String)bme.readAltitude(SEALEVELPRESSURE_HPA) + " Meters above sea level", SCROLL_SPEED_NORMAL);
}


void debugBME() {
	while (1) {

		Serial.print("Temperature = ");
		Serial.print(bme.readTemperature());
		Serial.println(" *C");

		//runText((String)bme.readTemperature() + " Celcius", 60);

		Serial.print("Pressure = ");

		Serial.print(bme.readPressure() / 100.0F);
		Serial.println(" hPa");

		Serial.print("Approx. Altitude = ");
		Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
		Serial.println(" m");

		Serial.print("Humidity = ");
		Serial.print(bme.readHumidity());
		Serial.println(" %");

		Serial.println();
		
		delay(500);
		checkButton();
	}
}

void runTemperature() {
	while (1) {

		Serial.print("Temperature = ");
		Serial.print(bme.readTemperature());
		Serial.println(" *C");

		runText(" "+(String)bme.readTemperature()+" celcius System Temp", SCROLL_SPEED_NORMAL);

		delay(100);
		checkButton();
	}
}

void runHumidity() {
	while (1) {

		Serial.print("Humid = ");
		Serial.print(bme.readHumidity());
		Serial.println(" *%");

		runText(" " + (String)bme.readHumidity() + " %", SCROLL_SPEED_NORMAL);

		delay(100);
		checkButton();
	}
}

void runPressure() {
	while (1) {

		Serial.print("Press = ");
		Serial.print(bme.readPressure());
		Serial.println(" *PA");

		runText(" " + (String)bme.readPressure() + " hPA", SCROLL_SPEED_NORMAL);

		delay(100);
		checkButton();
	}
}

void runAltitute() {
	while (1) {

		Serial.print("Alt = ");
		Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
		Serial.println(" *PA");

		runText(" " + (String)bme.readAltitude(SEALEVELPRESSURE_HPA) + " Meters", SCROLL_SPEED_NORMAL);

		delay(100);
		checkButton();
	}
}


#define NUMBUTTONS 2
#define MODEBUTTON 4
#define BRIGHTNESSBUTTON 3

#define BTNIDLE 0
#define BTNDEBOUNCING 1
#define BTNPRESSED 2
#define BTNRELEASED 3
#define BTNLONGPRESS 4
#define BTNLONGPRESSREAD 5

#define BTNDEBOUNCETIME 20
#define BTNLONGPRESSTIME 1000

unsigned long buttonEvents[NUMBUTTONS];
byte buttonStatuses[NUMBUTTONS];
byte buttonmap[NUMBUTTONS] = {BRIGHTNESSBUTTON, MODEBUTTON};

void updateButtons() {
  for (byte i = 0; i < NUMBUTTONS; i++) {
    switch (buttonStatuses[i]) {
      case BTNIDLE:
        if (digitalRead(buttonmap[i]) == LOW) {
          buttonEvents[i] = currentMillis;
          buttonStatuses[i] = BTNDEBOUNCING;
        }
        break;

      case BTNDEBOUNCING:
        if (currentMillis - buttonEvents[i] > BTNDEBOUNCETIME) {
          if (digitalRead(buttonmap[i]) == LOW) {
            buttonStatuses[i] = BTNPRESSED;
          }
        }
        break;

      case BTNPRESSED:
        if (digitalRead(buttonmap[i]) == HIGH) {
          buttonStatuses[i] = BTNRELEASED;
        } else if (currentMillis - buttonEvents[i] > BTNLONGPRESSTIME) {
          buttonStatuses[i] = BTNLONGPRESS;
        }
        break;

      case BTNRELEASED:
        break;

      case BTNLONGPRESS:
        break;

      case BTNLONGPRESSREAD:
        if (digitalRead(buttonmap[i]) == HIGH) {
          buttonStatuses[i] = BTNIDLE;
        }
        break;
    }
  }
}

byte buttonStatus(byte buttonNum) {

  byte tempStatus = buttonStatuses[buttonNum];
  if (tempStatus == BTNRELEASED) {
    buttonStatuses[buttonNum] = BTNIDLE;
  } else if (tempStatus == BTNLONGPRESS) {
    buttonStatuses[buttonNum] = BTNLONGPRESSREAD;
  }

  return tempStatus;

}

void doButtons() {
  
  // Check the mode button (for switching between effects)
  switch (buttonStatus(0)) {

    case BTNRELEASED: // button was pressed and released quickly
      cycleMillis = currentMillis;
      if (++currentEffect >= numEffects) currentEffect = 0; // loop to start of effect list
      effectInit = false; // trigger effect initialization when new effect is selected
      eepromMillis = currentMillis;
      eepromOutdated = true;
      break;

    case BTNLONGPRESS: // button was held down for a while
      autoCycle = !autoCycle; // toggle auto cycle mode
      confirmBlink(); // one blue blink: auto mode. two red blinks: manual mode.
      eepromMillis = currentMillis;
      eepromOutdated = true;
      break;

  }

  // Check the brightness adjust button
  switch (buttonStatus(1)) {

    case BTNRELEASED: // button was pressed and released quickly
      currentBrightness += 51; // increase the brightness (wraps to lowest)
      FastLED.setBrightness(scale8(currentBrightness, MAXBRIGHTNESS));
      eepromMillis = currentMillis;
      eepromOutdated = true;
      break;

    case BTNLONGPRESS: // button was held down for a while
      currentBrightness = STARTBRIGHTNESS; // reset brightness to startup value
      FastLED.setBrightness(scale8(currentBrightness, MAXBRIGHTNESS));
      eepromMillis = currentMillis;
      eepromOutdated = true;
      break;

  }
  
}


Adafruit_CCS811 ccs;
void checkButton();
void runText(String text, int speed);

void initCCS811() {

	Serial.println(Wire.status());
	Wire.begin(4, 5);
	
	if (!ccs.begin()) {
		Serial.println("Failed to start sensor! Please check your wiring.");
		delay(500);
		checkButton();

		Wire.flush();
		delay(100);
		Wire.status();
		delay(100);
		Wire.clearWriteError();
		delay(100);
		Wire.endTransmission();
		delay(100);

		//initCCS811();
		//while (1);
		ESP.reset();
	}

	//calibrate temperature sensor
	while (!ccs.available());
	float temp = ccs.calculateTemperature();
	ccs.setTempOffset(temp - 25.0);
}

void debugCCS811() {
	while (1) {
		FastLED.show();
		checkButton();
		if (ccs.available()) {
			float temp = ccs.calculateTemperature();
			if (!ccs.readData()) {
				Serial.print("CO2: ");
				Serial.print(ccs.geteCO2());
				Serial.print("ppm, TVOC: ");
				Serial.print(ccs.getTVOC());
				Serial.print("ppb   Temp:");
				Serial.println(temp);
			}
			else {
				Serial.println("ERROR!");
				ESP.reset();
				//while (1);
			}
		}
		delay(500);
	}
}

void runCO2() {
	byte pastLevel = 0;
	while (1) {
		checkButton();
		if (ccs.available()) {
			float temp = ccs.calculateTemperature();
			if (!ccs.readData()) {
				Serial.print("CO2: ");
				Serial.println(pastLevel);
				runText("  "+(String)ccs.geteCO2() + " ppm ", SCROLL_SPEED_NORMAL);
				//byte currentLevel = map(ccs.geteCO2(), 400, 5000, 0, 24);
				//showBar(currentLevel,pastLevel);
				//pastLevel = currentLevel;
			}
			else {
				Serial.println("ERROR!");
			}
		}
		delay(100);
	}
}


void runTVOC() {
	while (1) {
		checkButton();
		if (ccs.available()) {
			float temp = ccs.calculateTemperature();
			if (!ccs.readData()) {
				Serial.print("ppm, TVOC: ");
				Serial.print(ccs.getTVOC());
				runText("  " + (String)ccs.getTVOC() + " ppb ", SCROLL_SPEED_NORMAL);
			}
			else {
				Serial.println("ERROR!");
			}
		}
		delay(200);
	}
}






void threeSine() {

  static byte sineOffset = 0; // counter for current position of sine waves

  // startup tasks
  if (effectInit == false) {
    effectInit = true;
    effectDelay = 20;
  }

  // Draw one frame of the animation into the LED array
  for (byte x = 0; x < kMatrixWidth; x++) {
    for (int y = 0; y < kMatrixHeight; y++) {

      // Calculate "sine" waves with varying periods
      // sin8 is used for speed; cos8, quadwave8, or triwave8 would also work here
      byte sinDistanceR = qmul8(abs(y * (255 / kMatrixHeight) - sin8(sineOffset * 9 + x * 16)), 2);
      byte sinDistanceG = qmul8(abs(y * (255 / kMatrixHeight) - sin8(sineOffset * 10 + x * 16)), 2);
      byte sinDistanceB = qmul8(abs(y * (255 / kMatrixHeight) - sin8(sineOffset * 11 + x * 16)), 2);

      leds[XY(x, y)] = CRGB(255 - sinDistanceR, 255 - sinDistanceG, 255 - sinDistanceB);
    }
  }

  sineOffset++; // byte will wrap from 255 to 0, matching sin8 0-255 cycle

}


// RGB Plasma
void plasma() {

  static byte offset  = 0; // counter for radial color wave motion
  static int plasVector = 0; // counter for orbiting plasma center

  // startup tasks
  if (effectInit == false) {
    effectInit = true;
    effectDelay = 10;
  }

  // Calculate current center of plasma pattern (can be offscreen)
  int xOffset = cos8(plasVector / 256);
  int yOffset = sin8(plasVector / 256);

  // Draw one frame of the animation into the LED array
  for (int x = 0; x < kMatrixWidth; x++) {
    for (int y = 0; y < kMatrixHeight; y++) {
      byte color = sin8(sqrt(sq(((float)x - 7.5) * 10 + xOffset - 127) + sq(((float)y - 2) * 10 + yOffset - 127)) + offset);
      leds[XY(x, y)] = CHSV(color, 255, 255);
    }
  }

  offset++; // wraps at 255 for sin8
  plasVector += 16; // using an int for slower orbit (wraps at 65536)

}


// Scanning pattern left/right, uses global hue cycle
void rider() {

  static byte riderPos = 0;

  // startup tasks
  if (effectInit == false) {
    effectInit = true;
    effectDelay = 5;
    riderPos = 0;
  }

  // Draw one frame of the animation into the LED array
  for (byte x = 0; x < kMatrixWidth; x++) {
    int brightness = abs(x * (256 / kMatrixWidth) - triwave8(riderPos) * 2 + 127) * 3;
    if (brightness > 255) brightness = 255;
    brightness = 255 - brightness;
    CRGB riderColor = CHSV(cycleHue, 255, brightness);
    for (byte y = 0; y < kMatrixHeight; y++) {
      leds[XY(x, y)] = riderColor;
    }
  }

  riderPos++; // byte wraps to 0 at 255, triwave8 is also 0-255 periodic

}


// Shimmering noise, uses global hue cycle
void glitter() {

  // startup tasks
  if (effectInit == false) {
    effectInit = true;
    effectDelay = 15;
  }

  // Draw one frame of the animation into the LED array
  for (int x = 0; x < kMatrixWidth; x++) {
    for (int y = 0; y < kMatrixHeight; y++) {
      leds[XY(x, y)] = CHSV(cycleHue, 255, random8(5) * 63);
    }
  }

}


// Fills saturated colors into the array from alternating directions
void colorFill() {

  static byte currentColor = 0;
  static byte currentRow = 0;
  static byte currentDirection = 0;

  // startup tasks
  if (effectInit == false) {
    effectInit = true;
    effectDelay = 45;
    currentColor = 0;
    currentRow = 0;
    currentDirection = 0;
    currentPalette = RainbowColors_p;
  }

  // test a bitmask to fill up or down when currentDirection is 0 or 2 (0b00 or 0b10)
  if (!(currentDirection & 1)) {
    effectDelay = 45; // slower since vertical has fewer pixels
    for (byte x = 0; x < kMatrixWidth; x++) {
      byte y = currentRow;
      if (currentDirection == 2) y = kMatrixHeight - 1 - currentRow;
      leds[XY(x, y)] = currentPalette[currentColor];
    }
  }

  // test a bitmask to fill left or right when currentDirection is 1 or 3 (0b01 or 0b11)
  if (currentDirection & 1) {
    effectDelay = 20; // faster since horizontal has more pixels
    for (byte y = 0; y < kMatrixHeight; y++) {
      byte x = currentRow;
      if (currentDirection == 3) x = kMatrixWidth - 1 - currentRow;
      leds[XY(x, y)] = currentPalette[currentColor];
    }
  }

  currentRow++;

  // detect when a fill is complete, change color and direction
  if ((!(currentDirection & 1) && currentRow >= kMatrixHeight) || ((currentDirection & 1) && currentRow >= kMatrixWidth)) {
    currentRow = 0;
    currentColor += random8(3, 6);
    if (currentColor > 15) currentColor -= 16;
    currentDirection++;
    if (currentDirection > 3) currentDirection = 0;
    effectDelay = 300; // wait a little bit longer after completing a fill
  }


}

// Emulate 3D anaglyph glasses
void threeDee() {

  // startup tasks
  if (effectInit == false) {
    effectInit = true;
    effectDelay = 50;
  }

  for (byte x = 0; x < kMatrixWidth; x++) {
    for (byte y = 0; y < kMatrixHeight; y++) {
      if (x < 7) {
        leds[XY(x, y)] = CRGB::Blue;
      } else if (x > 8) {
        leds[XY(x, y)] = CRGB::Red;
      } else {
        leds[XY(x, y)] = CRGB::Black;
      }
    }
  }

  leds[XY(6, 0)] = CRGB::Black;
  leds[XY(9, 0)] = CRGB::Black;

}

// Random pixels scroll sideways, uses current hue
#define rainDir 0
void sideRain() {

  // startup tasks
  if (effectInit == false) {
    effectInit = true;
    effectDelay = 30;
  }

  scrollArray(rainDir);
  byte randPixel = random8(kMatrixHeight);
  for (byte y = 0; y < kMatrixHeight; y++) leds[XY((kMatrixWidth - 1) * rainDir, y)] = CRGB::Black;
  leds[XY((kMatrixWidth - 1)*rainDir, randPixel)] = CHSV(cycleHue, 255, 255);

}

// Pixels with random locations and random colors selected from a palette
// Use with the fadeAll function to allow old pixels to decay
void confetti() {

  // startup tasks
  if (effectInit == false) {
    effectInit = true;
    effectDelay = 10;
    selectRandomPalette();
  }

  // scatter random colored pixels at several random coordinates
  for (byte i = 0; i < 4; i++) {
    leds[XY(random16(kMatrixWidth), random16(kMatrixHeight))] = ColorFromPalette(currentPalette, random16(255), 255); //CHSV(random16(255), 255, 255);
    random16_add_entropy(1);
  }

}


// Draw slanting bars scrolling across the array, uses current hue
void slantBars() {

  static byte slantPos = 0;

  // startup tasks
  if (effectInit == false) {
    effectInit = true;
    effectDelay = 5;
  }

  for (byte x = 0; x < kMatrixWidth; x++) {
    for (byte y = 0; y < kMatrixHeight; y++) {
      leds[XY(x, y)] = CHSV(cycleHue, 255, quadwave8(x * 32 + y * 32 + slantPos));
    }
  }

  slantPos -= 4;

}


int c = 0;
#define NORMAL 0
#define RAINBOW 1
#define charSpacing 1

int showText(String textToShow, byte style, CRGB fgColor, CRGB bgColor) {
	static byte currentCharColumn = 0;
	static byte paletteCycle = 0;
	static CRGB currentColor;
	static byte bitBuffer[25] = { 0 };
	static byte bitBufferPointer = 0;
	static int scrolledChar;

	// startup tasks
	if (effectInit == false) {
		c = 0;
		scrolledChar = 0;
		effectInit = true;
		effectDelay = 35;
		currentCharColumn = 0;
		loadCharBuffer(textToShow.charAt(c++));
		currentPalette = RainbowColors_p;
		for (byte i = 0; i < kMatrixWidth; i++) bitBuffer[i] = 0;
	}

	paletteCycle += 15;


	if (currentCharColumn < 5) { // characters are 5 pixels wide
		bitBuffer[(bitBufferPointer + kMatrixWidth - 1) % kMatrixWidth] = charBuffer[currentCharColumn]; // character
	}
	else {
		bitBuffer[(bitBufferPointer + kMatrixWidth - 1) % kMatrixWidth] = 0; // space
	}

	CRGB pixelColor;
	for (byte x = 0; x < kMatrixWidth; x++) {
		for (byte y = 0; y < 5; y++) { // characters are 5 pixels tall
			if (bitRead(bitBuffer[(bitBufferPointer + x) % kMatrixWidth], y) == 1) {
				if (style == RAINBOW) {
					pixelColor = ColorFromPalette(currentPalette, paletteCycle + y * 16, 255);
				}
				else {
					pixelColor = fgColor;
				}
			}
			else {
				pixelColor = bgColor;
			}
			leds[XY(x, y)] = pixelColor;
		}
	}

	currentCharColumn++;
	if (currentCharColumn > (4 + charSpacing)) {
		currentCharColumn = 0;

		char nextChar = textToShow.charAt(c++);

		if (nextChar == 0) { // null character at end of strong
			c = 0;
			nextChar = textToShow.charAt(c++);
		}
		loadCharBuffer(nextChar);
	}

	bitBufferPointer++;
	scrolledChar++;
	if (bitBufferPointer > 23) bitBufferPointer = 0;

	return scrolledChar/5;

}

void runText(String text, int speed) {

	effectInit = false;

	text += "        ";

	while (showText(text, RAINBOW, 0, CRGB::Black)<text.length()+4) {
		FastLED.show();
		FastLED.delay(100 - speed);
		checkButton();
	}

}


void runTextRGB(String text, int speed, byte r, byte g, byte b) {

	effectInit = false;

	text += "        ";

	while (showText(text, NORMAL, CRGB(r,g,b), CRGB::Black)<text.length() + 4) {
		FastLED.show();
		FastLED.delay(100 - speed);
		checkButton();
	}

}

void showBar(byte level, byte pastLevel) {
	
	byte intensity = map(level, 0, 24, 0, 255);
		
	if (level > pastLevel) {
		for (int x = pastLevel; x <level; x++) {
				for (int y = 0; y < 5; y++) {
					leds[XY(x, y)] = CRGB(intensity, 255 - intensity, 0);
				}
		}
	}else{
		for (int x = pastLevel; x > level; x--) {
			for (int y = 0; y < 5; y++) {
				leds[XY(x, y)] = CRGB::Black;
			}
		}
	}
	FastLED.show();
		//FastLED.show();
		//FastLED.delay(10);
}

int flashText(String textToShow, byte style, CRGB fgColor, CRGB bgColor) {
	static byte currentCharColumn = 0;
	static byte paletteCycle = 0;
	static CRGB currentColor;
	static byte bitBuffer[25] = { 0 };
	static byte bitBufferPointer = 24;
	static int scrolledChar;

	// startup tasks
	if (effectInit == false) {
		c = 0;
		scrolledChar = 0;
		effectInit = true;
		effectDelay = 35;
		currentCharColumn = 0;
		loadCharBuffer(textToShow.charAt(c++));
		currentPalette = RainbowColors_p;
		for (byte i = 0; i < kMatrixWidth; i++) bitBuffer[i] = 0;
	}

	paletteCycle += 15;


	if (currentCharColumn < 5) { // characters are 5 pixels wide
		bitBuffer[(bitBufferPointer + kMatrixWidth - 1) % kMatrixWidth] = charBuffer[currentCharColumn]; // character
	}
	else {
		bitBuffer[(bitBufferPointer + kMatrixWidth - 1) % kMatrixWidth] = 0; // space
	}

	CRGB pixelColor;
	for (byte x = 0; x < kMatrixWidth; x++) {
		for (byte y = 0; y < 5; y++) { // characters are 5 pixels tall
			if (bitRead(bitBuffer[(bitBufferPointer + x) % kMatrixWidth], y) == 1) {
				if (style == RAINBOW) {
					pixelColor = ColorFromPalette(currentPalette, paletteCycle + y * 16, 255);
				}
				else {
					pixelColor = fgColor;
				}
			}
			else {
				pixelColor = bgColor;
			}
			leds[XY(x, y)] = pixelColor;
		}
	}

	currentCharColumn++;
	if (currentCharColumn > (4 + charSpacing)) {
		currentCharColumn = 0;

		char nextChar = textToShow.charAt(c++);

		if (nextChar == 0) { // null character at end of strong
			c = 0;
			nextChar = textToShow.charAt(c++);
		}
		loadCharBuffer(nextChar);
	}

	bitBufferPointer++;
	scrolledChar++;
	if (bitBufferPointer > 23) bitBufferPointer = 0;

	return scrolledChar;

}

void showFlashText(String text) {

	if (text.length() == 1)text += "   ";
	if (text.length() == 2)text += "  ";
	if (text.length() == 3)text += " ";

	effectInit = false;
	while (flashText(text, NORMAL, CRGB::BlueViolet, CRGB::Black) < 24) {
	} 

	FastLED.show();
}

void showFlashTextRedGreenFade(String text, int colorHue) {

	if (text.length() == 1)text += "   ";
	if (text.length() == 2)text += "  ";
	if (text.length() == 3)text += " ";

	if (colorHue > 255) colorHue = 255;

	effectInit = false;
	while (flashText(text, NORMAL, CRGB(colorHue,255-colorHue,0), CRGB::Black) < 24) {
	}
	FastLED.show();
}

String fab[3] = { "Hello pamungkas sumasta ",
"",
"" };


// Scroll a text string

void scrollText(byte message, byte style, CRGB fgColor, CRGB bgColor) {
  static byte currentCharColumn = 0;
  static byte paletteCycle = 0;
  static CRGB currentColor;
  static byte bitBuffer[25] = {0};
  static byte bitBufferPointer = 0;
  
  // startup tasks
  if (effectInit == false) {
    c=0;
    effectInit = true;
    effectDelay = 35;
    currentCharColumn = 0;
	loadCharBuffer(fab[message].charAt(c++));
    currentPalette = RainbowColors_p;
    for (byte i = 0; i < kMatrixWidth; i++) bitBuffer[i] = 0;
  }

  paletteCycle += 15;


  //Serial.println("C");

  if (currentCharColumn < 5) { // characters are 5 pixels wide
    bitBuffer[(bitBufferPointer + kMatrixWidth - 1) % kMatrixWidth] = charBuffer[currentCharColumn]; // character
  } else {
    bitBuffer[(bitBufferPointer + kMatrixWidth - 1) % kMatrixWidth] = 0; // space
  }

  //Serial.println("D");

  CRGB pixelColor;
  for (byte x = 0; x < kMatrixWidth; x++) {
    for (byte y = 0; y < 5; y++) { // characters are 5 pixels tall
      if (bitRead(bitBuffer[(bitBufferPointer + x) % kMatrixWidth], y) == 1) {
        if (style == RAINBOW) {
          pixelColor = ColorFromPalette(currentPalette, paletteCycle+y*16, 255);
        } else {
          pixelColor = fgColor;
        }
      } else {
        pixelColor = bgColor;
      }
      leds[XY(x, y)] = pixelColor;
    }
  }

  //Serial.println("E");

  currentCharColumn++;
  if (currentCharColumn > (4 + charSpacing)) {
    currentCharColumn = 0;
    //currentMessageChar++;
	
	//Serial.println("1");
    
	char nextChar = fab[message].charAt(c++);
	//Serial.println("2");

    if (nextChar == 0) { // null character at end of strong
      c = 0;
	  nextChar = fab[message].charAt(c++);
    }
	//Serial.println("3");
    loadCharBuffer(nextChar);
  }



  bitBufferPointer++;
  if (bitBufferPointer > 23) bitBufferPointer = 0;
  
  FastLED.delay(30);

  /*
  Serial.print("BIT: ");
  Serial.println(bitBufferPointer);
  Serial.print("CCC: ");
  Serial.println(currentCharColumn);
  Serial.print("CMC: ");
  //Serial.println(currentMessageChar);

  Serial.println();
  */
 // delay(500);

}



void scrollTextZero() {
  /*
	// Blynk.run();
	String S_H;
	if(hour()<10){
		S_H=String(hour());
		S_H="0"+S_H;
	}else{
		S_H=String(hour());
	}
	
	String S_M;
	if(minute()<10){
		S_M=String(minute());
		S_M="0"+S_M;
	}else{
		S_M=String(minute());
	}
	*/
	 //currentTimex = String("  "+S_H+":"+S_M+"  "+"\0");
	// currentTimex="  "+currentTimex+"  ";
    // scrollText(0, RAINBOW, 0,CRGB::Black);

currentTimex += "\0";
 scrollText(0, RAINBOW, 0,CRGB::Black);
 
}

void scrollTextOne() {
	currentTimex = "  Saturday  ";
	currentTimex+='\0';
  scrollText(1, NORMAL,CRGB::Red, CRGB::Black);
}

void scrollTextTwo() {
  scrollText(2, RAINBOW, 0, CRGB::Black);
}

void scrollTextThree() {
  scrollText(3, RAINBOW, 0, CRGB::Black);
}

void scrollTextFour() {
  scrollText(4, RAINBOW, 0, CRGB::Black);
}






MPU6050 mpu;
void checkButton();
void switchLED();

void initMPU() {

	while (!mpu.begin(MPU6050_SCALE_250DPS, MPU6050_RANGE_16G))
	{
		Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
		checkButton();
		ESP.restart();
		delay(1000);
	}

	//checkMPUSettings();

}

float getTiltAccel() {
	Vector rawAccel = mpu.readScaledAccel();
	return rawAccel.YAxis;
}

void checkMPUSettings()
{
	Serial.println();

	Serial.print(" * Sleep Mode:            ");
	Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

	Serial.print(" * Clock Source:          ");
	switch (mpu.getClockSource())
	{
	case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
	case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
	case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
	case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
	case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
	case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
	case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
	}

	Serial.print(" * Accelerometer:         ");
	switch (mpu.getRange())
	{
	case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
	case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
	case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
	case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;
	}

	Serial.print(" * Accelerometer offsets: ");
	Serial.print(mpu.getAccelOffsetX());
	Serial.print(" / ");
	Serial.print(mpu.getAccelOffsetY());
	Serial.print(" / ");
	Serial.println(mpu.getAccelOffsetZ());

	Serial.println();

	float temp = mpu.readTemperature();

	Serial.print(" Temp = ");
	Serial.print(temp);
	Serial.println(" *C");

	delay(3000);
}

void debugMPU() {

	Vector rawAccel = mpu.readScaledAccel();

	while (1) {
		rawAccel = mpu.readScaledAccel();
		
		Serial.print(rawAccel.YAxis);
		//Serial.print('\t');
		//Serial.print(rawAccel.YAxis);
		//Serial.print('\t');
		//Serial.print(rawAccel.ZAxis);
		Serial.println();
		delay(25);
		checkButton();
	}
}

void runAccelSerial() {

	Vector rawAccel = mpu.readScaledAccel();

	while (1) {
		rawAccel = mpu.readScaledAccel();

		Serial.print(rawAccel.YAxis);
		Serial.print('\t');
		Serial.print(rawAccel.YAxis);
		Serial.print('\t');
		Serial.print(rawAccel.ZAxis);
		Serial.println();
		delay(25);
		checkButton();
	}
}

void runGyroSerial() {
	Vector rawGyro = mpu.readRawGyro();

	while (1) {
		rawGyro = mpu.readRawGyro();

		Serial.print(rawGyro.YAxis);
		Serial.print('\t');
		Serial.print(rawGyro.YAxis);
		Serial.print('\t');
		Serial.print(rawGyro.ZAxis);
		Serial.println();
		delay(25);
		checkButton();
	}
}

boolean ledstatus = true;
#define THRESHOLD	16000
int counter = 0;

long doubleTapTimer;
int doubleTapInterval = 300;
//uint8_t currentPWM = 100;

void moooiTest() {
	initMPU();
	Serial.println("Enter Test Mooi");

	switchLED();
	FastLED.setBrightness(MAXBRIGHTNESS);
	delay(2000);

	while (1) {
		//FastLED.show();
		Vector rawAccel = mpu.readRawAccel();
		//Vector rawGyro = mpu.readRawGyro();

		//Serial.println(rawGyro.ZAxis);

		//if (digitalRead(BUTTON) == LOW) moooiTestDistance();
		
		if (rawAccel.ZAxis > THRESHOLD) {
			
			delay(20);
			do {
				rawAccel = mpu.readRawAccel();
				delay(5);
			} while (rawAccel.ZAxis > THRESHOLD);


			if (millis() - doubleTapTimer < doubleTapInterval) {
				Serial.print("DOUBLE TAP: ");
				dimmingRoutine();
			} else {
				Serial.print("TAP: ");
				switchLED();
			}

			Serial.println(counter++);

			doubleTapTimer = millis();

		}
				
		delay(5);
	}
}


boolean increaseDim = true;

long incrTimer;

void dimmingRoutine() {

	// Turn our current led on to white, then show the leds
	while (1) {
		

		if (millis() - incrTimer > 10) {
			if (increaseDim) currentPWM++;
			if (!increaseDim) currentPWM--;
			
			if (currentPWM == 0) increaseDim = true;
			if (currentPWM == 255) increaseDim = false;

			for (int whiteLed = 0; whiteLed < 242; whiteLed++) {
				leds[whiteLed] = CRGB(currentPWM, currentPWM, currentPWM);
			}
			// Show the leds (only one of which is set to white, from above)    
			FastLED.show();
			incrTimer = millis();

		}

		
		Vector rawAccel = mpu.readRawAccel();

		if (rawAccel.ZAxis>THRESHOLD) {
			
			increaseDim = !increaseDim;
			if (!increaseDim) ledstatus = true;
			Serial.println("OUT");
			delay(20);

			do {
				rawAccel = mpu.readRawAccel();
				delay(5);
			} while (rawAccel.ZAxis > THRESHOLD);
			
			break;
		}
		
		//Serial.println(currentPWM);
		delay(5);
	}

}



Adafruit_TCS34725 tcs = Adafruit_TCS34725(0x00, TCS34725_GAIN_1X);


void initTCS() {
	if (tcs.begin()) {
		Serial.println("Found sensor");
	}
	else {
		Serial.println("No TCS34725 found ... check your connections");
		while (1);
	}
}

void debugTCS() {

	while (1) {
		checkButton();

		uint16_t r, g, b, c, colorTemp, lux;

		tcs.getRawData(&r, &g, &b, &c);
		colorTemp = tcs.calculateColorTemperature(r, g, b);
		lux = tcs.calculateLux(r, g, b);

		Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
		Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
		Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
		Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
		Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
		Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
		Serial.println(" ");

		delay(1000);
	}
}

uint16_t getLux() {

	uint16_t r, g, b, c, colorTemp, lux;
	tcs.getRawData(&r, &g, &b, &c);
	lux = tcs.calculateLux(r, g, b);

	return lux;

}

void runLux() {
	while (1) {
		uint16_t r, g, b, c, colorTemp, lux;

		tcs.getRawData(&r, &g, &b, &c);
		//colorTemp = tcs.calculateColorTemperature(r, g, b);
		lux = tcs.calculateLux(r, g, b);

		runText(" " + (String)lux + " lux  ", SCROLL_SPEED_NORMAL);

		delay(100);
	}
}

void runCCT() {
	while (1) {
		uint16_t r, g, b, c, colorTemp, lux;

		tcs.getRawData(&r, &g, &b, &c);
		colorTemp = tcs.calculateColorTemperature(r, g, b);
		//lux = tcs.calculateLux(r, g, b);

		runText(" " + (String)colorTemp + " kelvin  ", SCROLL_SPEED_NORMAL);

		delay(100);
	}
}

void runRGBSensor() {
	while (1) {
		uint16_t r, g, b, c, colorTemp, lux;

		tcs.getRawData(&r, &g, &b, &c);
		//colorTemp = tcs.calculateColorTemperature(r, g, b);
		//lux = tcs.calculateLux(r, g, b);

		runTextRGB(" " + (String)r + " ", SCROLL_SPEED_NORMAL,150,0,0);
		runTextRGB(" " + (String)g + " ", SCROLL_SPEED_NORMAL, 0, 150, 0);
		runTextRGB(" " + (String)b + " ", SCROLL_SPEED_NORMAL, 0, 0, 150);

		delay(100);
	}
}



Adafruit_VEML6070 uv = Adafruit_VEML6070();

void initVEML() {
	Serial.println("VEML6070 Test");
	uv.begin(VEML6070_1_T);
}

void debugVEML() {
	while (1) {
		Serial.print("UV light level: "); 
		Serial.println(uv.readUV());
		checkButton();
		delay(1000);
	}
}

void runVEML() {
	while (1) {
		Serial.print("UV light level: ");
		Serial.println(uv.readUV());
		checkButton();

		runText(" " + (String)uv.readUV() + " UV Index  ", SCROLL_SPEED_NORMAL);

		delay(500);
	}
}



Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void initVL53() {

	Serial.println("Adafruit VL53L0X test");
	if (!lox.begin()) {
		Serial.println(F("Failed to boot VL53L0X"));
		while (1);
	}

}

void debugDistance() {
	
	while (1) {
		checkButton();
		VL53L0X_RangingMeasurementData_t measure;

		//Serial.print("Reading a measurement... ");
		lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

		if (measure.RangeStatus != 4) {  // phase failures have incorrect data
			Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
			showFlashText((String)measure.RangeMilliMeter);
		}
		else {
			//Serial.println(" out of range ");
		}

		delay(50);
	}

}

#define THRESHOLDDISTANCE	200
byte colorSelection = 0;

void runTestDistance() {

	switchLED();
	FastLED.setBrightness(MAXBRIGHTNESS);

	while (1) {
		//FastLED.show();
		VL53L0X_RangingMeasurementData_t measure;
		lox.rangingTest(&measure, false);

		checkButton();

		long hoverCounter=0;

		if (measure.RangeMilliMeter < THRESHOLDDISTANCE && measure.RangeStatus != 4) {

			int startDistance = measure.RangeMilliMeter;
			delay(20);
			do {
				lox.rangingTest(&measure, false);
				hoverCounter += 1;

				if (hoverCounter > 20) {
					hoverCounter = 0;
					dimmingRoutineDistance();
					ledstatus = false;
				}
				Serial.println("Counting");

				if (measure.RangeMilliMeter < startDistance - 50 && hoverCounter>5) {
					Serial.println("Change Color");
					colorSelection++;
					if (colorSelection > 3) colorSelection = 0;
					ledstatus = false;
					switchLED();
					break;
				}

				delay(5);
			} while (measure.RangeMilliMeter < THRESHOLDDISTANCE && measure.RangeStatus != 4);


			if (millis() - doubleTapTimer < doubleTapInterval) {
				Serial.print("DOUBLE TAP: ");
				autoDimming();
				doubleTapTimer = millis();
			}
			else {
				Serial.print("TAP: ");
				switchLED();
			}

			Serial.println(counter++);

			doubleTapTimer = millis();

		}

		delay(5);
	}
}

VL53L0X_RangingMeasurementData_t measure;

void dimmingRoutineDistance() {

	byte prevTemp = 0;

	// Turn our current led on to white, then show the leds
	do {

		int temp = measure.RangeMilliMeter - 50;
		if (temp > 255) temp = 255;
		if (temp < 0) temp = 0;

		
		/*
		if (temp > (currentPWM) && begin) {
			for (int x = temp; x < currentPWM; x += 3) {
				for (int whiteLed = 0; whiteLed < 242; whiteLed++) {
					leds[whiteLed] = CRGB(x, x, x);
				}
				FastLED.show();
				delay(1);
			}
			begin = false;
		}
		else if (temp < currentPWM && begin) {
			for (int x = temp; x > currentPWM; x-=3) {
				for (int whiteLed = 0; whiteLed < 242; whiteLed++) {
					leds[whiteLed] = CRGB(x, x, x);
				}
				FastLED.show();
				delay(1);
			}
			begin = false;
		}
		*/
		
		currentPWM = temp;
		
		
		for (int whiteLed = 0; whiteLed < 242; whiteLed++) {
			switch (colorSelection) {
			case 0: leds[whiteLed] = CRGB(currentPWM, currentPWM, currentPWM); break;
			case 1: leds[whiteLed] = CRGB(currentPWM, 0, 0); break;
			case 2: leds[whiteLed] = CRGB(0, currentPWM, 0); break;
			case 3: leds[whiteLed] = CRGB(0, 0, currentPWM); break;
			}
		}
		
		// Show the leds (only one of which is set to white, from above)    
		FastLED.show();
		
		lox.rangingTest(&measure, false);

		prevTemp = currentPWM;
		
		delay(10);
	} while (measure.RangeStatus != 4 && measure.RangeMilliMeter < THRESHOLDDISTANCE+300);

	do {
		lox.rangingTest(&measure, false);
		delay(5);
	} while (measure.RangeMilliMeter < THRESHOLDDISTANCE+200 && measure.RangeStatus != 4);

}

void switchLED() {
	ledstatus = !ledstatus;
	Serial.println("Switch LED");
	for (int whiteLed = 0; whiteLed < 242; whiteLed = whiteLed + 1) {
		// Turn our current led on to white, then show the leds

		if (ledstatus) {
			
			switch (colorSelection) {
			case 0: leds[whiteLed] = CRGB(currentPWM, currentPWM, currentPWM); break;
			case 1: leds[whiteLed] = CRGB(currentPWM, 0, 0); break;
			case 2: leds[whiteLed] = CRGB(0, currentPWM, 0); break;
			case 3: leds[whiteLed] = CRGB(0, 0, currentPWM); break;
			}	
		}
		else {
			leds[whiteLed] = CRGB::Black;
			//FastLED.setBrightness(0);
		}
		// Show the leds (only one of which is set to white, from above)    
	}
	FastLED.show();
}

void autoDimming() {

	boolean increasing = true;

	do {

		lox.rangingTest(&measure, false);

		if (measure.RangeMilliMeter < THRESHOLDDISTANCE) {
			Serial.println("Stop AutoDimming");
			ledstatus = true;
			break;
		}

		if(increasing)currentPWM+=4;
		if (!increasing)currentPWM-=4;
		
		if (currentPWM <= 1) increasing = true;
		if (currentPWM >= 250)increasing = false;

		for (int whiteLed = 0; whiteLed < 242; whiteLed++) {
			switch (colorSelection) {
			case 0: leds[whiteLed] = CRGB(currentPWM, currentPWM, currentPWM); break;
			case 1: leds[whiteLed] = CRGB(currentPWM, 0, 0); break;
			case 2: leds[whiteLed] = CRGB(0, currentPWM, 0); break;
			case 3: leds[whiteLed] = CRGB(0, 0, currentPWM); break;
			}
		}

		FastLED.show();
		delay(5);

	}while (1);

	do {
		lox.rangingTest(&measure, false);
		delay(5);
	} while (measure.RangeMilliMeter < THRESHOLDDISTANCE + 200 && measure.RangeStatus != 4);

	// Show the leds (only one of which is set to white, from above)    
	

}

void moooiDistanceHoldDim() {

	runText("moooi test mode: hold dim", 90);

	initVL53();
	switchLED();
	FastLED.setBrightness(MAXBRIGHTNESS);

	while (1) {
		//FastLED.show();
		VL53L0X_RangingMeasurementData_t measure;
		lox.rangingTest(&measure, false);

		long hoverCounter = 0;

		if (digitalRead(BUTTON) == LOW) {
			delay(100);
			while (digitalRead(BUTTON) == LOW);
			moooiDistanceProximityDim();
		}

		if (measure.RangeMilliMeter < THRESHOLDDISTANCE && measure.RangeStatus != 4) {

			int startDistance = measure.RangeMilliMeter;
			delay(20);
			do {
				lox.rangingTest(&measure, false);
				hoverCounter += 1;

				if (hoverCounter > 10) {
					hoverCounter = 0;
					autoDimmingHold();
					ledstatus = false;
				}
				Serial.println("Counting");

				/*
				if (measure.RangeMilliMeter < startDistance - 50 && hoverCounter>5) {
					Serial.println("Change Color");
					colorSelection++;
					if (colorSelection > 3) colorSelection = 0;
					ledstatus = false;
					switchLED();
					break;
				}
				*/

				delay(5);
			} while (measure.RangeMilliMeter < THRESHOLDDISTANCE && measure.RangeStatus != 4);


			if (millis() - doubleTapTimer < doubleTapInterval) {
				Serial.print("DOUBLE TAP: ");
				//autoDimming();
				doubleTapTimer = millis();
			}
			else {
				Serial.print("TAP: ");
				switchLED();
			}

			Serial.println(counter++);

			doubleTapTimer = millis();

		}

		delay(5);
	}
}

void moooiDistanceProximityDim() {

	FastLED.setBrightness(STARTBRIGHTNESS);
	runText("moooi test mode: Proximity dim", 90);

	switchLED();
	FastLED.setBrightness(MAXBRIGHTNESS);

	while (1) {
		//FastLED.show();
		VL53L0X_RangingMeasurementData_t measure;
		lox.rangingTest(&measure, false);

		long hoverCounter = 0;

		if (digitalRead(BUTTON) == LOW) {
			delay(100);
			while (digitalRead(BUTTON) == LOW);
			moooiRealDistance();
		}

		if (measure.RangeMilliMeter < THRESHOLDDISTANCE && measure.RangeStatus != 4) {

			int startDistance = measure.RangeMilliMeter;
			delay(20);
			do {
				lox.rangingTest(&measure, false);
				hoverCounter += 1;

				if (hoverCounter > 10) {
					hoverCounter = 0;
					dimmingRoutineDistance();
					ledstatus = false;
				}
				Serial.println("Counting");

				delay(5);
			} while (measure.RangeMilliMeter < THRESHOLDDISTANCE && measure.RangeStatus != 4);


			if (millis() - doubleTapTimer < doubleTapInterval) {
				Serial.print("DOUBLE TAP: ");
				//autoDimming();
				doubleTapTimer = millis();
			}
			else {
				Serial.print("TAP: ");
				switchLED();
			}

			Serial.println(counter++);

			doubleTapTimer = millis();

		}

		delay(5);
	}
}

void autoDimmingHold() {

	boolean increasing = true;

	do {

		lox.rangingTest(&measure, false);

		/*
		if (measure.RangeMilliMeter < THRESHOLDDISTANCE) {
			Serial.println("Stop AutoDimming");
			ledstatus = true;
			break;
		}
		*/

		byte steps = 4;
		if (currentPWM < 100) steps = 4;
		if (currentPWM < 50) steps = 3;
		if (currentPWM < 25) steps = 2;
		if (currentPWM < 12) steps = 1;


		if (increasing)currentPWM += steps;
		if (!increasing)currentPWM -= steps;

		if (currentPWM <= 1) increasing = true;
		if (currentPWM >= 250)increasing = false;

		for (int whiteLed = 0; whiteLed < 242; whiteLed++) {
			switch (colorSelection) {
			case 0: leds[whiteLed] = CRGB(currentPWM, currentPWM, currentPWM); break;
			case 1: leds[whiteLed] = CRGB(currentPWM, 0, 0); break;
			case 2: leds[whiteLed] = CRGB(0, currentPWM, 0); break;
			case 3: leds[whiteLed] = CRGB(0, 0, currentPWM); break;
			}
		}

		FastLED.show();
		delay(3);

	} while (measure.RangeMilliMeter < THRESHOLDDISTANCE && measure.RangeStatus != 4);

	do {
		lox.rangingTest(&measure, false);
		delay(5);
	} while (measure.RangeMilliMeter < THRESHOLDDISTANCE + 200 && measure.RangeStatus != 4);
}

void moooiRealDistance() {

	//FastLED.setBrightness(STARTBRIGHTNESS);
	//runText("moooi test mode: Real Distance Show", 90);

	//switchLED();
	//FastLED.setBrightness(MAXBRIGHTNESS);

	while (1) {
		checkButton();
		VL53L0X_RangingMeasurementData_t measure;

		if (digitalRead(BUTTON) == LOW) {
			delay(100);
			while (digitalRead(BUTTON) == LOW);
			moooiDistanceHoldDim();
		}

		//Serial.print("Reading a measurement... ");
		lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!


		if (measure.RangeStatus != 4 && measure.RangeMilliMeter<500) {  // phase failures have incorrect data
			Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
			showFlashTextRedGreenFade((String)measure.RangeMilliMeter, map(measure.RangeMilliMeter,30,500,0,255));
		}
		else {
			//Serial.println(" out of range ");
		}

		delay(50);
	}

}


uint8_t XY( uint8_t x, uint8_t y)
{
  // any out of bounds address maps to the first hidden pixel
  if( (x >= kMatrixWidth) || (y >= kMatrixHeight) ) {
    return (LAST_VISIBLE_LED + 1);
  }

/*
  const uint8_t ShadesTable[] = {
     68,  0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 69,
     29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14,
     30, 31, 32, 33, 34, 35, 36, 70, 71, 37, 38, 39, 40, 41, 42, 43,
     57, 56, 55, 54, 53, 52, 51, 72, 73, 50, 49, 48, 47, 46, 45, 44,
     74, 58, 59, 60, 61, 62, 75, 76, 77, 78, 63, 64, 65, 66, 67, 79
  };
*/


const byte ShadesTable[] = {
     0,   1,  2,    3,    4,    5,    6,    7,    8,    9,    10,   11,   12,   13,   14,   15,   16,   17,   18,   19,   20,     21,   22,   23,
     47,  46, 45,  44,    43,   42,   41,   40,   39,   38,   37,   36,   35,   34,   33,   32,   31,   30,   29,   28,   27,     26,   25,   24,
     48,  49, 50,  51,    52,   53,   54,   55,   56,   57,   58,   59,   60,   61,   62,   63,   64,   65,   66,   67,   68,     69,   70,   71,
       95, 94,  93,    92,   91,   90,   89,   88,   87,   86,   85,   84,   83,   82,   81,  80, 79,   78,   77,   76,   75,     74,   73,   72,
     96, 97,  98, 99,  100,  101,  102,   103,  104,  105,  106,  107,  108,  109,  110,  111,  112,  113,  114,  115,  116,   117,   118,  119
     
  };


/*
  const byte ShadesTable[] = {
     11,   10,  9,    8,    7,    6,    5,    4,    3,    2,   1,   0,   12,   13,   14,   15,   16,   17,   18,   19,   20,     21,   22,   23,
     47,  46, 45,  44,    43,   42,   41,   40,   39,   38,   37,   36,   35,   34,   33,   32,   31,   30,   29,   28,   27,     26,   25,   24,
     48,  49, 50,  51,    52,   53,   54,   55,   56,   57,   58,   59,   60,   61,   62,   63,   64,   65,   66,   67,   68,     69,   70,   71,
     96,  95, 94,  93,    92,   91,   90,   89,   88,   87,   86,   85,   84,   83,   82,   81,   80,   79,   78,   77,   76,     75,   73,   72,
     97,  98, 99,  100,  101,  102,   103,  104,  105,  106,  107,  108,  109,  110,  111,  112,  113,  114,  115,  116,   117,   118,  119,  120
     
  };
*/
/*
  if(y==0 ||y==2 ||y==4){
    return ShadesTable[((24*y)+x)];
  }else{
    return ShadesTable[((24*(y+1))-x)];
  }
  */
  byte f = (y * kMatrixWidth) + x;
  byte j = ShadesTable[f];
  return j*2+1;
  
}



void hueCycle(byte incr) {
    cycleHueCount = 0;
    cycleHue+=incr;
}

// Set every LED in the array to a specified color
void fillAll(CRGB fillColor) {
  for (byte i = 0; i < NUM_LEDS; i++) {
    leds[i] = fillColor;
  }
}

// Fade every LED in the array by a specified amount
void fadeAll(byte fadeIncr) {
  for (byte i = 0; i < NUM_LEDS; i++) {
    leds[i] = leds[i].fadeToBlackBy(fadeIncr);
  }
}

// Shift all pixels by one, right or left (0 or 1)
void scrollArray(byte scrollDir) {
  
    byte scrollX = 0;
    for (byte x = 1; x < kMatrixWidth; x++) {
      if (scrollDir == 0) {
        scrollX = kMatrixWidth - x;
      } else if (scrollDir == 1) {
        scrollX = x - 1;
      }
      
      for (byte y = 0; y < kMatrixHeight; y++) {
        leds[XY(scrollX,y)] = leds[XY(scrollX + scrollDir*2 - 1,y)];
      }
    }
  
}


// Pick a random palette from a list
void selectRandomPalette() {

  switch(random8(8)) {
    case 0:
    currentPalette = CloudColors_p;
    break;
    
    case 1:
    currentPalette = LavaColors_p;
    break;
    
    case 2:
    currentPalette = OceanColors_p;
    break;
    
    case 4:
    currentPalette = ForestColors_p;
    break;
    
    case 5:
    currentPalette = RainbowColors_p;
    break;
    
    case 6:
    currentPalette = PartyColors_p;
    break;
    
    case 7:
    currentPalette = HeatColors_p;
    break;
  }

}

// Interrupt normal operation to indicate that auto cycle mode has changed
void confirmBlink() {

  if (autoCycle) { // one blue blink, auto mode active
    fillAll(CRGB::DarkBlue);
    FastLED.show();
    FastLED.delay(200);
    fillAll(CRGB::Black);
    FastLED.delay(200);
  } else { // two red blinks, manual mode active
    fillAll(CRGB::DarkRed);
    FastLED.show();
    FastLED.delay(200);
    fillAll(CRGB::Black);
    FastLED.delay(200);
    fillAll(CRGB::DarkRed);
    FastLED.show();
    FastLED.delay(200);
    fillAll(CRGB::Black);
    FastLED.delay(200);
  }

}
/*
// Determine flash address of text string
unsigned int currentStringAddress = 0;
void selectFlashString(byte string) {
  currentStringAddress = pgm_read_word(&stringArray[string]);
}
*/

// Fetch font character bitmap from flash

void loadCharBuffer(byte character) {
  byte mappedCharacter = character;
  if (mappedCharacter >= 32 && mappedCharacter <= 95) {
    mappedCharacter -= 32; // subtract font array offset
  } else if (mappedCharacter >= 97 && mappedCharacter <= 122) {
    mappedCharacter -= 64; // subtract font array offset and convert lowercase to uppercase
  } else {
    mappedCharacter = 96; // unknown character block
  }
  
  for (byte i = 0; i < 5; i++) {
    charBuffer[i] = pgm_read_byte(Font[mappedCharacter]+i);
  }
  
}

/*
// Fetch a character value from a text string in flash
char loadStringChar(byte string, byte character) {
  return (char) pgm_read_byte(currentStringAddress + character);
}

// write EEPROM value if it's different from stored value
void updateEEPROM(byte location, byte value) {
  if (EEPROM.read(location) != value) EEPROM.write(location, value);
}

// Write settings to EEPROM if necessary
void checkEEPROM() {
  if (eepromOutdated) {
    if (currentMillis - eepromMillis > EEPROMDELAY) {
      updateEEPROM(0, 99);
      updateEEPROM(1, currentEffect);
      updateEEPROM(2, autoCycle);
      updateEEPROM(3, currentBrightness);
      eepromOutdated = false;
    }
  }
}
*/