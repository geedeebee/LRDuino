// Uncomment the following for debug printing
#define DEBUG


// MOTORBIKE SENSOR/LOGGER
//
// Forked by Greg Brown 11/10/2020 from LRDuino by Ben Anderson Version 0.93?/
// A motorcycle (Suzuki RGV 250 Gamma) auxillary display/logger (1991, therefore non-OBD!)
// Key sensor data to show/store: CHT x 2, Tire Temp x 2, ambient temp, gear position, lean angle (NB: gear position is fcn(rpm, wheelspeed) )
//
//  Hardware:
//  Arduino Mega2560
//  TFT 1.44" 128x128 (Adafruit) with builtin uSD reader
//  i2c sensors (BNO055 IMU, MXL90164 IR temperature x2)
//  Analog sensors - AD8495 K-type Thermocouple amplifier x2
//  Level shifter for i2c bus
//  Rotary encoder with button
//  Sparkfun RTC (i2c interface)
//  SparkFun OpenLogger (connected to Serial3 @ 9600 baud)
//
//  To do:
//  BUGS:
//  P3 Not quite enough text cleared on bargraph max scale when a peak above limit occurs

//
//  DEV:
//  1. Check/rewrite the sensor code fault stuff (depending on final choice of sensors)
//  2. Write the GPI code - count interrupts form Hall sensors (Wheel speed) & Tach - reuse old code for this!
//  2. Decide on bigger screen (can use touch UI for this?). Maybe use newer UTFT and UTouch libs?!
//  3. Reorder the sensors in the include file (user provides the index) - user editable?
//  3. DET/knock sensor http://www.bylund-automotive.com/educative/knock/ and https://forums.parallax.com/discussion/156362/automotive-knock-sensor-interface
//                        and http://archive.siliconchip.com.au/cms/A_108910/article.html
//  3. Upgrade to Adafruit_MLX90640 which is a temperature sensor array - can provide a "view" across the whole tire!?
//
//  HARDWARE:
//  1. PCB LAYOUT
//  2. Housing design/printing & mount for triple clamp
//  3. Sensor mounts - especially the tire sensors... how to keep clean and safe?



#include <Adafruit_MLX90614.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SdFat.h>
#include <SPI.h>
#include <Adafruit_ImageReader.h>
#include <Adafruit_ImageReader_EPD.h>
#include <elapsedMillis.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ezButton.h>
#include <SparkFunDS1307RTC.h>

#include "DebugUtils.h"
#include "encoder.h"      // code to handle the rotary encoder
#include "sensorConfig.h" // These are my definitions for the sensors

#define DEBUG false

// PINOUTS - VERY IMPORTANT! THESE DIFFER FOR THE 3.2" display version
  #define TFT_CS     10   // pins needed for TFT setup
  #define TFT_RST    9  
  #define TFT_DC     8
  #define SD_CS      SS    
  
  // 52 is MISO by default on Mega2560 (from the SD card)
  // 51 is MOSI by default on Mega2560 (to the TFT Display)
  // 53 is SS by default on Mega2560
 
  // A4 is I2C SDA
  // A5 is I2C SCL
  
  //The A/B encoder pins  i.e. 2 and 3 are defined in the encoder.h include
  int encoderSwitchPin = 4; //push button switch

  #define DIVISOR      1023

  #define MAX(x,y) (x>y?x:y)                // Useful macros for max, min and range
  #define MIN(x,y) (x>y?y:x)
  
  #define BNO055_SAMPLERATE_DELAY_MS (50)  //GB TODO: CHECK WHAT IS ALLOWABLE HERE

  // Config for AD8495 thermocouple amplifers
  #define CHT1_PIN  A0
  #define CHT2_PIN  A1
  #define AREF  5.0
  #define ADC_RESOLUTION 10
  #define VREF 1.2


  
//CONSTRUCTORS

SdFat SD;
Adafruit_ST7735 display1 = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);
Adafruit_ImageReader reader(SD);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_MLX90614 mlx2= Adafruit_MLX90614(0x5B); // The sensor has been reprogrammed (using the Sparkfun library example sketch) to 0x5B
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
ezButton button(4);  // create ezButton object that attach to pin 7
elapsedMillis UPDATEMillis;
elapsedMillis LOGMillis;

// GLOBAL VARIABLES
uint16_t UPDATEInterval = 500; 
uint16_t LOGInterval = 1000; 

uint8_t sensecount = 0;
const uint8_t totalsensors = 7; //this is the actual number of sensor definitons
uint8_t rotation = 0; // incremented by 1 with each button press - it's used to tell the drawdisplay functions which sensor data they should output.
enum UI_State {TEXT, GRAPH, CONFIG};
enum UI_State Screen_State;

enum LOGGER_State {OFF, ON};
enum LOGGER_State Logger_State;

const int SHORT_PRESS_TIME = 1000; // 1000 milliseconds
const int LONG_PRESS_TIME  = 1000; // 1000 milliseconds
unsigned long pressedTime  = 0;
unsigned long releasedTime = 0;
bool isPressing = false;
bool isLongDetected = false;

void setup()   {
  Serial.begin(9600);  //GB TODO: Only need this for debugging, can comment out for production
  Serial3.begin(9600); // OpenLogger device is on this port

  rtc.begin(); 
  //rtc.autoTime();
  rtc.set24Hour();  
  
  display1.initR(INITR_144GREENTAB);   // initialize the ST7735S chip 
  display1.setRotation(1);
  display1.cp437(1);   // The fix to enable (or disable) Code Page 437-compatible charset
  display1.fillScreen(ST77XX_BLACK); 
  
  //GREETING SPLASHSCREEN 
  display1.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  display1.setTextSize(1);
  display1.setCursor(10,20);
  display1.print(getTime());  // from RTC
  display1.setTextColor(ST77XX_BLUE, ST77XX_BLACK);  
  display1.setTextSize(4);
  display1.setCursor(55,50);
  display1.print(char(226));  // char(226) is Greek Gamma ;) Also, char(2) is a smiley see https://en.wikipedia.org/wiki/Code_page_437
  delay(1000);  

  if(!SD.begin(SD_CS)) { // alternatively use a different speed? like "SD.begin(SD_CS, SD_SCK_MHZ(25))"  
    display1.setTextSize(2);
    display1.setTextColor(ST77XX_RED, ST77XX_BLACK);
    display1.setCursor(5,50);
    display1.fillScreen(ST7735_BLACK);
    display1.println(F("SD begin() failed"));
    for(;;); // Fatal error, do not continue
  }

//  ALTERNATIVE GRAPHICAL SPLASHSCREEN
//  ImageReturnCode stat;
//  stat = reader.drawBMP("/SUZUKI.BMP", display1, 0, 0);
//  delay(1000);  

  // INITIALIZE THE BNO055 IMU
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    display1.setTextSize(2);
    display1.setTextColor(ST77XX_RED, ST77XX_BLACK);
    display1.fillScreen(ST7735_BLACK);    
    display1.setCursor(5,50);
    display1.println(F("IMU failure")); 
    delay(1000);   
  }
  bno.setExtCrystalUse(true);

  // INITIALIZE THE MLX90614 IT SENSORS
  mlx.begin(); 
  mlx2.begin();

  // GET THE ENCODER READY
  beginEncoders();
  button.setDebounceTime(50); // set debounce time of the rotart encode button to 50 milliseconds
    
  // Count the number of active sensors
  // first we need to dimension the array
  for (uint8_t thisSensor = 0; thisSensor < totalsensors; thisSensor++) {
    if (senseactive[thisSensor]==true) {
      sensecount++;
    }
  }

  // SET THE INITIAL STATE FOR THE SCREEN AND LOGGER
  Screen_State = TEXT; // START IN THE TEXT VIEW 
  Logger_State = OFF;  // NO LOGGING TO START

  display1.fillScreen(ST7735_BLACK);   // Finally, let's clear the TFT display 
}
// END OF SETUP


////////////////////////////////
///// M A I N   L O O P ////////
////////////////////////////////

void loop() {

  // USER INTERACTIONS
  
  if (updateEncoders()) {
    rotation = encoder[0].count;
  }

  button.loop(); // MUST call the loop() function first

  if(button.isPressed()){
    pressedTime = millis();
    isPressing = true;
    isLongDetected = false;
  }

  if(button.isReleased()) {
    isPressing = false;
    releasedTime = millis();

    long pressDuration = releasedTime - pressedTime;

    if( pressDuration < SHORT_PRESS_TIME )
      Screen_State = (Screen_State + 1)%3;
      DEBUG_PRINTLN("Short press");
      display1.fillScreen(ST77XX_BLACK);
  }

  if(isPressing == true && isLongDetected == false) {
    long pressDuration = millis() - pressedTime;

    if( pressDuration > LONG_PRESS_TIME ) {
      isLongDetected = true;
      resetPeaks();
      Logger_State = (Logger_State + 1)%2; // GB TODO: Just for testing
      DEBUG_PRINTLN("Long press");
    }
  }


  // PROCESS THE SCREEN STATE
  if (Screen_State == GRAPH && rotation == 6) {  // NOTE: rotation==6 ==> "Lean Angle" and this gets faster refresh rate
      UPDATEInterval = 100;
    } else {
        UPDATEInterval = 500;
      }

  if (UPDATEMillis >= UPDATEInterval) {
    // TIME TO TAKE THE SENSOR READINGS
    
    if (senseactive[0]) {
      sensevals[0] = readCHT1Temp(0); // read CHT1 (Left cylinder head) from AD8495 on A0  
      updatePEAK(0); //  CHT 1 (Left)
    }

    if (senseactive[1]) {
      sensevals[1] = readCHT2Temp(1); // read CHT2 (Right cylinder head)  from AD8495 on A1
      updatePEAK(1); // CHT 2 (Right)
    }
    
    if (senseactive[2]) {
      sensevals[2] = readTireFrontTemp(2); // read Front Tire Temperature from i2c HXL90614 #1 (0x5A)    
      updatePEAK(2); // 
    }
    
    if (senseactive[3]) {
      sensevals[3] = readTireRearTemp(3); // read Rear Tire Temperature from i2c HXL90614 #2 (0x5B)
      updatePEAK(3);
    }
    
    if (senseactive[4]) {
      sensevals[4] = readAmbientTemp(4); // read Ambient Temperature: from i2c HXL90614 #1 (0x5A) 
      updatePEAK(4); //
    }

    if (senseactive[5]) {
      sensevals[5] = readGear(5); // GB TODO: GEAR POSITION 
      //updatePEAK(5); // peaks not needed for GPI
    }
    
    if (senseactive[6]) {
      sensevals[6] = readLeanAngle(6); // Inclinometer - Y (roll) angle      
      updatePEAK(6);
    }
    
    // Update TFT Display
    switch (Screen_State) {
      case TEXT:
        drawDISPLAY_text();
        break;
      case GRAPH:
        drawDISPLAY_bar();
        break;
      case CONFIG:
        drawDISPLAY_config();
        break;
      } 

    UPDATEMillis = 0; //RESET THE COUNTER
  }
  if (LOGMillis >= LOGInterval) {
  //TIME TO LOG (IF LOGGER IS ON)  
    if (Logger_State) {
      doLog(); //GB TODO: Put this behind a check whether the logger state is on/off
    }
    LOGMillis = 0; //RESET THE COUNTER
  }
}

////////////////////////////////////////////
//   E N D   O F   M A I N   L O O P      //
////////////////////////////////////////////

void drawDISPLAY_bar(void) { // Bar Graph display

  uint8_t sensor1 = posrot(1);
  drawSensor(0, display1, sensor1);  // arguments (Y offset, display object, sensor to plot)
  drawBarGraph(10, display1, sensor1);  

}


void drawDISPLAY_text(void) { //  Text Summary display

  uint8_t sensor1 = posrot(1);
  uint8_t sensor2 = posrot(2); 
  uint8_t sensor3 = posrot(3); 
  uint8_t sensor4 = posrot(4);   
   
  drawSensor(0, display1, sensor1);  // arguments (Y offset, display object ref, sensor to plot)
  drawSensor(35, display1, sensor2);  // arguments (Y offset, display object ref, sensor to plot)
  drawSensor(70, display1, sensor3);  // arguments (Y offset, display object ref, sensor to plot)
  drawSensor(105, display1, sensor4);  // arguments (Y offset, display object ref, sensor to plot)   
}

void drawDISPLAY_config(void) {
  // GB TODO: options edit screen(s) - Here is just something to text UI for now
  display1.setTextSize(2);
  display1.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  display1.setTextWrap(false);
  display1.setCursor(10, 10); 
  display1.print("\n1. Edit\n2. Save"); 
  display1.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
  display1.print("\n3. Exit");    
}

// Helper Functions

void drawSensor(uint8_t y, Adafruit_ST7735 &refDisp, uint8_t sensor) {
  uint8_t xoffset = 0;
  int8_t rolltemp = 0;
  
  refDisp.setTextSize(2);
  refDisp.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  refDisp.setTextWrap(false);

  refDisp.setCursor(46, y + 7);

  String newVal;
  newVal = valIfnoErr(sensor);
  for (int jj=newVal.length(); jj<3; jj++)
  {
    newVal.concat(" ");
  }
  refDisp.print(newVal); 

  xoffset = (3 * 13) + 5 ; // work out width of the characters so we can move the cursor to the correct position to display our units symbol

//  if (sensefault[sensor] > 0 || sensor == 5) {  // normal size text if it's an error message or it's our low coolant warning sensor
  if (sensefault[sensor] > 0) {  // normal size text if it's an error message
    
    refDisp.setTextSize(2);
    refDisp.setCursor(46, y + 7);
  }  else {                                     // else use a smaller font for the units display
    refDisp.setTextSize(1);
    refDisp.setCursor(46 + xoffset, y + 7);
  }

// GB: fixed for units overwrite/clearing TODO: GPI display issues....
  String newUnits;
  newUnits = units(sensor);
  for (int kk=newUnits.length(); kk<3; kk++)
  {
    newUnits.concat(" ");
  }
  refDisp.print(newUnits);   
  //refDisp.println(units(sensor));

  // THIS IS INSTEAD OF THE SENSOR BMP ICON
  refDisp.setTextSize(2);
  refDisp.setCursor(0, y + 7);
  refDisp.println(sensorLabel[sensor]);    //Sensor Label is defined in the sensorConfig.h

  // Sensor warnings
  refDisp.setTextSize(2);
  refDisp.setTextColor(ST77XX_RED, ST77XX_BLACK);

  refDisp.setCursor(112, y+8); 
  refDisp.println(" ");  // Clear the space from previous warning
  
  if (hiloWARN(sensor)) {  
    refDisp.setCursor(112, y+8); 
    refDisp.println(char(19));  //char(19) is a double exclamation !!
  }

  if (faultWARN(sensor) == 1) {
    refDisp.setCursor(112, y+8); 
    refDisp.println("X");  
  }

  if (Logger_State) {
    refDisp.drawRect(0, 0, 128, 128, ST77XX_RED); //Border of the screen
  } else {
      refDisp.drawRect(0, 0, 128, 128, ST77XX_BLACK); //Border of the screen    
    }
}


void drawBarGraph(uint8_t y, Adafruit_ST7735 &refDisp, uint8_t sensor) {
  int padding = 0;
  float scalerange = 0;
  int scaleposmin = 0;
  
  refDisp.drawLine(11, y+52, 11, 54, ST77XX_WHITE); // draw our gauge and scale markings
  refDisp.drawLine(64, y+52, 64, 54, ST77XX_WHITE);
  refDisp.drawLine(116, y+52, 116, 54, ST77XX_WHITE);
  refDisp.drawRect(11, y+42, 106, 10, ST77XX_WHITE); //Border of the bar chart
  if (senseminvals[sensor] < 0) { // Work out a positive range of values that we need to plot
    scaleposmin = senseminvals[sensor] * -1;
    scalerange = scaleposmin + sensemaxvals[sensor];
  } else {
    scaleposmin = senseminvals[sensor];
    scalerange = sensemaxvals[sensor] - scaleposmin;
  }
  refDisp.fillRect(14, y+44, 100, 6, ST77XX_BLACK); // erase the old bar just before drawing the new one!
  if (sensor == 6) {
    refDisp.fillRect((100 / scalerange * (sensevals[sensor] + scaleposmin))+12, y+44, 5 , 6, ST77XX_WHITE); //Draws a marker for lean angle
  } else {
    refDisp.fillRect(14, y+44, (100 / scalerange * (sensevals[sensor] + scaleposmin)), 6, ST77XX_WHITE); //Draws a filled bar for all other sensors
  }

  refDisp.fillRect(13 + (100 / scalerange * (sensepeakvals[sensor] + scaleposmin)), y+44, 3, 6, ST77XX_RED); // draw the peak value marker;
  if (sensor == 6) {   // ONLY SHOW MIN PEAK FOR LEAN ANGLE
    refDisp.fillRect(13 + (100 / scalerange * (sensepeakminvals[sensor] + scaleposmin)), y+44, 3, 6, ST77XX_RED); // draw the min peak value marker;
  }

  if (sensevals[sensor] < 100) { // adjust padding for the low value so it looks nice
    padding = 0;
  } else {
    padding = -4;
  }
  refDisp.fillRect(8, y+33, 110, 7, ST77XX_BLACK); // erase the text with a big rectangle. Maybe needs to be faster...
  refDisp.setTextSize(1); 
  refDisp.setCursor(8 + padding + (100 / scalerange * (sensepeakvals[sensor] + scaleposmin)), y+33); // set cursor with padding
  refDisp.println(String(sensepeakvals[sensor])); // write the *max* peak val

  if (sensor == 6) {
    refDisp.setCursor(8 + padding + (100 / scalerange * (sensepeakminvals[sensor] + scaleposmin)), y+33); // set cursor with padding
    refDisp.println(String(sensepeakminvals[sensor])); // write the *minimum* peak val
  }
  
  refDisp.setCursor(8, y+57);
  refDisp.println(String(senseminvals[sensor])); // draw the minumum value
  refDisp.setCursor(58, y+57);
  if (senseminvals[sensor] < 100) { // adjust padding for the low value so it looks nice
    padding = 8;
  } else {
    padding = 0;
  }

  String midPointText;
  midPointText = static_cast<int>(((sensemaxvals[sensor] - senseminvals[sensor]) / 2) - scaleposmin);
  for (int jj=midPointText.length(); jj<3; jj++)
  {
    midPointText.concat(" ");
  }
  refDisp.print(midPointText); 

  if (sensemaxvals[sensor] < 100) { // adjust padding for the high value so it looks nice/doesnt wrap off screen
    padding = 10;
  } else {
    padding = 5;
  }

  refDisp.setCursor(100 + padding, y+57);
  refDisp.println(String(sensemaxvals[sensor])); // draw the maximum value
}


bool hiloWARN(uint8_t sensorZ) {
  // this function toggles an error flag if the current sensor is above it's high warning parameter - since the display is redrawn every 250ms it appears to flash
  if (sensefault[sensorZ] > 0) { // we don't want to display a high or low warning if there's a sensor fault (ie wiring issue etc).
    return (false);
  }
  if (sensevals[sensorZ] > sensewarnhivals[sensorZ] || sensevals[sensorZ] < sensewarnlowvals[sensorZ]) {
    if (warnhistatus[sensorZ] == true) {
      warnhistatus[sensorZ] = false; // we toggle the value so that the warning triangle flashes based on the interval we are looping at in loop()
    } else {
      warnhistatus[sensorZ] = true;
    }
  } else {
    warnhistatus[sensorZ] = false;
  }
  return (warnhistatus[sensorZ]);
}


uint8_t faultWARN(uint8_t sensorZ) {
  // this function alternates a flag between 1 & 2 if it is set - since the display is redrawn every 250ms it appears to flash
  if (sensefault[sensorZ] > 0) {
    if (sensefault[sensorZ] == 1) {
      sensefault[sensorZ] = 2; // we toggle the value so that the fault icon flashes based on the interval we are looping at in loop()

    } else {
      sensefault[sensorZ] = 1;
    }
  }
  return (sensefault[sensorZ]);
}


void toggleFault(uint8_t sensorPin) {
  // toggles the fault state of a sensor (to make our warning symbols flash)
  if (sensefault[sensorPin] == 2) {
    sensefault[sensorPin] = 2; // 2 is animation off
  
  } else {
    sensefault[sensorPin] = 1; // 1 is animation on
  }
}


void updatePEAK(uint8_t sensorPin) {
  sensepeakvals[sensorPin] = MAX(sensevals[sensorPin],sensepeakvals[sensorPin]);
  sensepeakminvals[sensorPin] = MIN(sensevals[sensorPin],sensepeakminvals[sensorPin]); // Now also capturing peak minimum values (e.g. for lean angle)
}


String units(uint8_t sensor) { // returns the units associated with the sensor, or some fault text
  // if a fault is set return ERR
  if (sensefault[sensor] > 0 ) {
    return (F("?"));
  }
  // if no fault then return the correct units (saves us some memory usage)

  switch (senseunits[sensor]) {
    case 0:
      return (F("C"));
    case 1:
      return (F("psi"));
    case 2:
      if (sensefault[sensor] > 0) {
        return (F("Low"));
      }
      return (F(" OK"));
    case 3:
      return (F("Deg"));
    case 4:
      return (F(" "));
  }
}


String valIfnoErr(uint8_t sensor) { //prevents values being displayed if we are in fault state OR this is a boolean sensor (coolant level)
  String text = String(sensevals[sensor]);
  // if a fault is set return an empty string
 
  if (sensefault[sensor] > 0 ) {
  
    return (F(""));
  }
  return (text);
  
}


uint8_t posrot(uint8_t location) { // this is used to shift our array of data around the screens
  
  uint8_t count=0;
  uint8_t pos[sensecount];

  // now we populate the array with the active sensors
  for (uint8_t locthisSensor = 0; locthisSensor < totalsensors; locthisSensor++) { 
    if (senseactive[locthisSensor] == true) {
      pos[count]=locthisSensor;
      count++;
    }
  }
  //uint8_t pos[] = {0, 1, 2, 3, 4, 5, 6};
  // return the correct sensor for the current location  
  location = location - 1 + rotation;

  if (location > count-1) {
    location = location % count;
  }
  return (pos[location]);
}


int doFaults(int constraint, int checkval, int retval, uint8_t index) { //
  if (checkval < constraint) {
    toggleFault(index); //fault!
    sensepeakvals[index] = senseminvals[index]; //set the peak value to minimum (tidies the display)
    retval = senseminvals[index]; //return minimum value
  } else {
    sensefault[index] = 0; // no fault
  }
  return (retval);
}


  // SENSOR READING FUNCTIONS
  // Each is very specific to type of sensor (analog pin vs i2c vs SPI etc)
  // Note also the code defining each fault condition will need to be modified!

int readERR2081(uint8_t sensorPin, uint8_t index) {
  int raw = 0;     // variable to store the raw ADC input value
  float Vin = 3.3; // variable to store the measured VCC voltage
  float Vout = 0;  // variable to store the output voltage
  int R2 = 1000;   // variable to store the R2 value
  float Rth = 0;   // variable to store the thermistor value

  // THERMISTOR CODE
  raw = analogRead(sensorPin);      // Reads the Input PIN
  Vout = (Vin / DIVISOR) * raw;     // Calculates the Voltage on the Input PIN
  Rth = ((R2 * Vin) / Vout) - R2;   // Calculates the Resistance of the Thermistor
  float steinhart;                  // This next stage calculates the temp from the resistance
  steinhart = Rth / 2012.2;         // (R/Ro)  therm @ 25C = 1986
  steinhart = log(steinhart);       // ln(R/Ro)
  steinhart /= 3502;                // 1/B * ln(R/Ro) b coefficient = 3344
  steinhart += 1.0 / (25 + 273.15); // + (1/To) nominal temp is 25c
  steinhart = 1.0 / steinhart;      // Invert
  steinhart -= 273.15;              // convert to C
  // END THERMISTOR CODE

  // FAULT checking
  // Sensors should be connected with a 1K pulldown resistor - if there's is a connection fault a low raw read will indicate this.
  return (doFaults(DIVISOR/100, raw, int(steinhart), index));
}

bool readCoolantLevel(uint8_t sensorPin, uint8_t index) {
  // sensor is normally closed
  // use a pulldown resistor to enable fault monitoring
  int CoolantLevel;
  CoolantLevel = analogRead(sensorPin);
  // process any faults
  return ((bool)doFaults(DIVISOR/2, CoolantLevel, CoolantLevel, index));
}


int readLeanAngle(uint8_t index) {
  // POSSIBLE TYPES:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  return (doFaults(0, 0, int(euler.z()), index));
}


int readAmbientTemp(uint8_t index) {
  int ambTemp;
//  if (!mlx.searchDevice()) {
//    return (doFaults(1, 0, 0, index)); // set fault state if the MLX90416 is not connected **TBD**
//  }
  ambTemp = int(mlx.readAmbientTempC());
  return (doFaults(DIVISOR/100, ambTemp, ambTemp, index));
}


int readTireFrontTemp(uint8_t index) {
  int objTemp;
//  if (!mlx.searchDevice()) {
//    return (doFaults(1, 0, 0, index)); // set fault state if the MLX90416 is not connected **TBD**
//  }
  objTemp = int(mlx.readObjectTempC());
  return (doFaults(DIVISOR/100, objTemp, objTemp, index));
}


int readTireRearTemp(uint8_t index) {
  int objTemp;
//  if (!mlx.searchDevice()) {
//    return (doFaults(1, 0, 0, index)); // set fault state if the MLX90416 is not connected **TBD**
//  }
  objTemp = int(mlx2.readObjectTempC());
  return (doFaults(DIVISOR/100, objTemp, objTemp, index));
}


int readGear(uint8_t index) {
  int gearNum = 2; // GB TODO: I OBVIOUSLY NEED TO DO A LOT MORE WORK HERE! CURRENTLY THROWING A FAULT CODE JUST FOR UI CHECKING
  return (doFaults(999, gearNum, gearNum, index));
}


void resetPeaks() {
  // reset all the sensor peaks (max/min) currently stored
  for (int i = 0; i < totalsensors; i++) {
    sensepeakvals[i] = 0;
    sensepeakminvals[i] = 0;
  }
}


int readCHT1Temp(uint8_t index) {
  float reading, voltage, temperature;
  reading = analogRead(CHT1_PIN);
  voltage = reading * (AREF / (pow(2,ADC_RESOLUTION)-1));
  temperature = (voltage - VREF)/0.005;
  return (doFaults(DIVISOR/100, int(temperature), int(temperature), index)); 
}


int readCHT2Temp(uint8_t index) {
  float reading, voltage, temperature;
  reading = analogRead(CHT2_PIN);
  voltage = reading * (AREF / (pow(2,ADC_RESOLUTION)-1));
  temperature = (voltage - VREF)/0.005;
  return (doFaults(DIVISOR/100, int(temperature), int(temperature), index)); 
}


void doLog() {
  Serial3.println(getTime()
  +","+String(sensevals[0])+","+String(sensepeakvals[0])+","+String(sensepeakminvals[0])
  +","+String(sensevals[1])+","+String(sensepeakvals[1])+","+String(sensepeakminvals[1])
  +","+String(sensevals[2])+","+String(sensepeakvals[2])+","+String(sensepeakminvals[2])
  +","+String(sensevals[3])+","+String(sensepeakvals[3])+","+String(sensepeakminvals[3])
  +","+String(sensevals[4])+","+String(sensepeakvals[4])+","+String(sensepeakminvals[4]) 
  +","+String(sensevals[5])+","+String(sensepeakvals[5])+","+String(sensepeakminvals[5])
  +","+String(sensevals[6])+","+String(sensepeakvals[6])+","+String(sensepeakminvals[6]));
}

String getTime()
{
  String tempString = "";
  rtc.update();
  
  tempString += String(rtc.hour()) + ":"; // Print hour
  if (rtc.minute() < 10)
    tempString += '0'; // Print leading '0' for minute
  tempString += String(rtc.minute()) + ":"; // Print minute
  if (rtc.second() < 10)
    tempString += '0'; // Print leading '0' for second
  tempString += String(rtc.second()); // Print second

  if (rtc.is12Hour()) // If we're in 12-hour mode
  {
    // Use rtc.pm() to read the AM/PM state of the hour
    if (rtc.pm()) tempString += " PM"; // Returns true if PM
    else tempString += " AM";
  }
  
  tempString += " ";

#ifdef PRINT_USA_DATE
  tempString += String(rtc.month()) + "/" +   // Print month
                 String(rtc.date()) + "/";  // Print date
#else
  tempString += String(rtc.date()) + "/" +    // (or) print date
                 String(rtc.month()) + "/"; // Print month
#endif
  tempString += String(rtc.year());        // Print year
  return tempString;
}
