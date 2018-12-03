//===================================================================================
// Using an AtTiny85 to turn on a relay on PIN0 for 10 seconds every so-often. 
// Can be used to water plants etc.
//
// The Arduino shuts down to about 0.006 to 0.007 milliAmps while sleeping, so we
// should get decent battery life. Every 8 seconds we wake up and do something.
//
// Power Usage:
//
// 1. Sleeping is 0.006 to 0.007 mA. (6-7 microAmps - nice!)
// 2. Reading the sensor is 7 mA.
// 3. Delay loops are 51 mA!
// 4. Relay on is 70 mA.
//
// The capacity of a Panasonic CR123A is about 1300 (they say 1500) milliAmpHours. 
//
// That's 21.185 YEARS on sleep mode!
// That's 7.738 DAYS constantly reading the sensor.
// That's 1.062 DAYS in a delay loop.
// That's 18.571 HOURS running the relay.
//
// Every hour we check the sensor and water the plant if necessary for 10 seconds.
//
// Norman Dunbar
// 16th June 2018.
//===================================================================================


//===================================================================================
// AtTiny85 Pin Usage.
//===================================================================================
//               RST -->  o V o   <-- VCC
// SENSOR_VCC = PIN3 -->  o   o   <-- PIN2 = LED
// SENSOR_GND = PIN4 -->  o   o   <-- PIN1 = SENSOR_D0
//               GND -->  o___o   <-- PIN0 = RELAY Driver (See below)
//
//                      AtTiny85 
//===================================================================================


//===================================================================================
// Circuit Diagram
//===================================================================================
//
//                                      +---------------------------+---+--- VCC (5V)
//                                      |                           |   |
//                                      |                        C1 |   |
//                                      |                          ===  |
//                     +---------------------------+               ===  +--> Relay VCC
//                     |                |          |                | 
//                     |       __ __    |          |                |
// D0 on Sensor  <-----+      O  V  O---+          |                |
// VCC on Sensor <------------O     O--------------|--------+       |
// GND on Sensor <------------O     O---------+----+        |       |
//                        +---O_____O----+    |             =  R3   |
//                        |              |    =  R2        | |      |
//                        |     U1       |   | |            =       |
//                        |              |    =             |       |
//                        |              |    |             | LED1  |
//                        |              |    |             V       |
//                        |              |    |            ---      |
//                        |              |    |             |       |
//                        +--------------|----+-------------+--+----+-+----- GND
//                                       |                     |      |
//                                       |                     |      |
//                                       |                  C  |      |
//                                       |     __  R1    +-->--+      +---> Relay GND
//                                       +----|__|-------|        T1
//                                                     B +-----+
//                                                          E  |
//                                                             |
//                                                             +----------> Relay IN1
//
//===================================================================================


//===================================================================================
// BOM
//===================================================================================
//
// C1  100nF ceramic.
// R1  1K 1/4 Watt 5%.
// R2  10K 1/4 watt 5%.
// R3  560R 1/4 watt 5%.
// T1  2N2222A NPN transistor.
// U1  Atmel AtTiny85.
// LED1 3mm LED.
// 
// Sensor is a cheap Ebay Chinese soil moisture sensor with a D0 and A0 output, VCC 
// and GND inputs.I'm using the D0 output as it's either on (too dry) or off (too
// wet). The A0 is too variable due to stray capacitance etc.
//
// Relay is a SONGLE SRD-05VDC-SL-C suitable for Arduino. Mine is a 2 way relay, but 
// they come in sets of one through 8 I believe. Use what you have. 
//===================================================================================


//===================================================================================
// Credits.
//===================================================================================
// Uses Low-Power library by Ortegafernando at:
//
// https://github.com/ortegafernando/Low-Power 
//
// which is a fork of RocketScream's Low-Power library at:
//
// https://github.com/rocketscream/Low-Power 
//
// modified to support the AtTiny family of microcontrollers.
//
// In case of vanishing repositories, I have forked the Ortegafernando's to:
//
// https://github.com/NormanDunbar/Low-Power.
//
// (Just in case Fernando ever 'goes away'.
//
// Credit to the above though for saving me the effort of writing a library for the
// AtTiny family. You have to love Open Source!
//===================================================================================


//===================================================================================
// WARNING
//===================================================================================
// All output pins should be reconfigured as input during sleeping. This
// cuts down on power use too.
//===================================================================================


//===================================================================================
// Code starts here.
//===================================================================================
#include "LowPower.h"

#define      RELAY PIN0     // Relay driver transistor goes here.
#define  SENSOR_D0 PIN1     // Digital pin from sensor goes here.
#define        LED PIN2     // LED - "I'm awake!"
#define SENSOR_VCC PIN3     // Power to the sensor when we are awake.
#define SENSOR_GND PIN4     // Ground to the sensor when we are awake.

//-----------------------------------------------------------------------------------
// Your relay may be directly connected, in which case, ON is LOW and OFF is HIGH.
// Mine is connected with a 2n2222 NPN transistor because I like to see things ON
// when HIGH and OFF when LOW. I'm old fashioned that way.
//-----------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------
// Select one set of values below depending on your circuit.
//-----------------------------------------------------------------------------------
#define RELAY_ON  HIGH      // Enable relay through driver transistor.
#define RELAY_OFF LOW       // Disable relay through driver transistor.

// #define RELAY_ON  LOW       // Enable relay directly.
// #define RELAY_OFF HIGH      // Disable relay directly.

//-----------------------------------------------------------------------------------
// Set these to determine how often we really wake up and check things.
// The AtTiny will wake every 8 seconds anyway, there's nothing we can
// do about that, but by counting each 8 seconds wake up, we can delay 
// for any time we want really.Provided it's a multiple of 8 seconds!
//-----------------------------------------------------------------------------------

#define TESTING 3           // When testing, check every TESTING * 8 seconds.
#define TEST_RELAY 2        // Seconds to run the relay at startup.

#define EVERY_HOUR 450      // How many 8 seconds in one hour.
#define EVERY_3_HOURS 1350  // How many 8 seconds in three hours.
#define EVERY_4_HOURS 1800  // How many 8 seconds in four hours.

//#define TIME_CHECK TESTING  // For testing purposes only.
#define TIME_CHECK EVERY_4_HOURS  // Normal running time to wake up.
#define SECONDS_ON 15       // Time the pump runs for. In seconds.


//===================================================================================
//                                                                         flashLED()
//===================================================================================
// Turn on the LED for for a given number of milliSeconds. Then off again. The LED is
// configured as OUTPUT then flashed, then reconfigured as INPUT so that the system
// can go to sleep afterwards. (OUTPUT pins need to be INPUT when sleeping.)
//===================================================================================
void flashLED(unsigned _delay)
{
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  delay(_delay);
  digitalWrite(LED, LOW);  

  // Required for safe sleeping.
  pinMode(LED, INPUT); 
}

//===================================================================================
//                                                                            setup()
//===================================================================================
// Turn on the LED for 2 seconds at startup, and set the sensor signal pin to be an
// INPUT pin. These are fine through sleep modes, only OUTPUTs need reconfiguring.
// (According to ATMEL anyway - and they should know.)
// BEWARE: This starts the pump running at startup too, as a check.
//===================================================================================
void setup()
{
  // LED gets flashed once at startup.
  flashLED(2000);
  
  // Sensor.Stays as input while sleeping.
  pinMode(SENSOR_D0, INPUT);

  // Fire the relay, for a quick test run.
  // Or quick splash I suppose!
  startRelay(TEST_RELAY);
}




//===================================================================================
//                                                                       readSensor()
//===================================================================================
// Power up the sensor. Wait a bit to settle, then read it. We do this to avoid stray
// capacitance on the sensor and to avoid excessive wear on the probes.
// The pins are reconfigured as INPUT while sleeping.
//===================================================================================
bool readSensor()
{
  bool readSensor = LOW;
  
  // Ready to power up.
  pinMode(SENSOR_VCC, OUTPUT);
  pinMode(SENSOR_GND, OUTPUT);

  // Power on.
  digitalWrite(SENSOR_GND, LOW);
  digitalWrite(SENSOR_VCC, HIGH);

  // Settling time.
  delay(2000);

  // Read the sensor.
  readSensor = digitalRead(SENSOR_D0);

  // Power off and Shut down for sleeping.
  digitalWrite(SENSOR_VCC, LOW);
  pinMode(SENSOR_VCC, INPUT);
  pinMode(SENSOR_GND, INPUT);

  // Return reading.
  return readSensor;
}


//===================================================================================
//                                                                       startRelay()
//===================================================================================
// Set up the relay pin after sleeping, power on the relay to drive the pump 
// then after a delay, power down and prepare the relay pin for sleeping again.
//===================================================================================
void startRelay(int SecondsOn) {
   // Start the relay.
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, RELAY_ON);

  // Delay SECONDS_ON seconds.
  delay(SecondsOn * 1000);

  // Relay off again.
  digitalWrite(RELAY, RELAY_OFF);
  pinMode(RELAY, INPUT);
}


//===================================================================================
//                                                                             loop()
//===================================================================================
// Every 8 seconds, wake up and check the counter. If it has been 'a while' since we
// last checked the sensor, check it now. If the sensor says that the plant needs 
// water, water the plant for SECONDS_ON seconds.
//===================================================================================
void loop() 
{
  static unsigned short wakeUpCounter = 0;

  // After a wake up call from the WDT...
  // Flash the LED.(Heartbeat)
  flashLED(100);

  // Do we need to chack the sensor yet?
  if (wakeUpCounter >= TIME_CHECK) {
    
    // Reset the counter.
    wakeUpCounter = 0;
 
    // If sensor is HIGH, we need water.
    if (readSensor()) {
      // Start the relay.
      startRelay(SECONDS_ON);
    }  
  }
    
  // Nothing done. Increment the counter.
  wakeUpCounter++;

  // Enter power down state for 8s with ADC and BOD module disabled
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
}
