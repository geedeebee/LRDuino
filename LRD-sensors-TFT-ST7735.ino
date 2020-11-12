// LRDuino by Ben Anderson
// Version 0.93

// Forked by Greg Brown 11/10/2020
// Intended as a motorcycle (Suzuki RGV 250 Gamma) auxillary display (1991, therefore non-OBD!)
// I will probably limit the number and type of display to just 1 (cost and space)
// Key sensor data to show/store: CHT x 2, Tire Temp x 2, coolant temp, gear position, lean (NB: gear position is f(rpm, wheelspeed) )
//
// Looks like the Uno is not big enough to run eerything, so aiming on using a Mega2560
// The display I have available is a TFT 1.44" 128x128 (Adafruit) with builtin uSD reader
//
// To do:
//  1. check/rewrite the sensor code especially the fault stuff, to suit my sensors
//  2. debug the warning/fault code w.r.t. the UI 
//  3. Data logging?
//  4. Eye candy for the start screen (Suzuki Logo) if there is still memory available
//  5. DET/knock sensor?? http://www.bylund-automotive.com/educative/knock/ and https://forums.parallax.com/discussion/156362/automotive-knock-sensor-interface
//                        and http://archive.siliconchip.com.au/cms/A_108910/article.html

#include <Adafruit_MLX90614.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SdFat.h>
#include <SPI.h>
#include <Adafruit_MAX31856.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <FaBo3Axis_ADXL345.h> //a smaller ADXL library so it fits on the Uno
#include "sensorConfig.h"
#include <Adafruit_ImageReader.h>
#include <Adafruit_ImageReader_EPD.h>

//  WARNING! Be sure to adjust the following pinout details to suit your hardware

  #define TFT_CS     10   // pins needed for TFT setup
  #define TFT_RST    9  
  #define TFT_DC     8
  #define SD_CS      4    // Chip select line for SD card
  
  // 12 is MISO by default on Uno (from the SD card)
  // 13 is MOSI by default on Uno (to the TFT Display)
  //
  // 52 is MISO by default on Uno (from the SD card)
  // 51 is MOSI by default on Uno (to the TFT Display)
  // 53 is SS by default on Mega

  #define MAX_CS       5  // Multiple software SPI because adafruit ST7735 & MAX31856 libraries won't play nicely on the same bus
  #define MAX_DC       2  // MISO DI
  #define MAX_CLK      7  // CLK
  #define MAX_MOSI     6  // DO
  
  // A4 is I2C SDA
  // A5 is I2C SCL
  
  #define ROTBUT       3 // our input button 
  #define DIVISOR      1023

// These are the current sensor inputs (analog) but I will not need these. Need to rewrite the sensor polling code from line 169 onwards
//  A0  Boost
//  A1  Tbox temp
//  A2  Oil pressure
//  A3  Oil Temp
//  A4  Coolant Level

  
//CONSTRUCTORS

Adafruit_MAX31856 max = Adafruit_MAX31856(MAX_CS, MAX_DC, MAX_MOSI, MAX_CLK); // note the separate SPI buses
Adafruit_ST7735 display1 = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);
FaBo3Axis accel;
SdFat SD;
Adafruit_ImageReader reader(SD);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// GLOBAL VARIABLES

uint8_t sensecount = 0;
const uint8_t totalsensors = 7; //this is the actual number of definitons above
uint8_t rotation = 0; // incremented by 1 with each button press - it's used to tell the drawdisplay functions which sensor data they should output.

// the follow variable is a long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long previousMillis = 0;        // will store last time the displays were updated
uint16_t interval = 250;        // interval at which to update displays (milliseconds)



void setup()   {
  Serial.begin(9600);
  
  tftInit(display1);

  //configure pin as an input and enable the internal pull-up resistor, this is for a button to toggle the rotation of sensors shown
  pinMode(ROTBUT, INPUT_PULLUP);

  max.begin(); //initialise the MAX31856
  max.setThermocoupleType(MAX31856_TCTYPE_K); // and set it to a K-type thermocouple - adjust for your hardware!

  accel.configuration(); // initialise ADXL345
  accel.powerOn();

  mlx.begin(); 
  
  // count the number of active sensors
  // first we need to dimension the array
  for (uint8_t thisSensor = 0; thisSensor < totalsensors; thisSensor++) {
    if (senseactive[thisSensor]==true) {
      sensecount++;

    }
  }





}
// END OF SETUP


////////////////////////////////
///// M A I N   L O O P ////////
////////////////////////////////

void loop() {

  unsigned long currentMillis = millis(); //store the current time

  // USER INTERACTION
  bool butVal = digitalRead(ROTBUT); // read the button state
  if (butVal == LOW) {
    rotation = rotation + 1; // rotate the sensors on the screen when button is pressed
    previousMillis = previousMillis - (interval + 1); // force an update of the screens.
    if (rotation == sensecount) { // this should be total number of sensors in the main sensor array
      rotation = 0;
    }
    display1.fillScreen(ST77XX_BLACK); //force a clearing of the screen only when sensors are rotated
  }

  if (currentMillis - previousMillis > interval) { // only read the sensors and draw the screen if desired "interval" has passed
    // save the last time we updated
    previousMillis = currentMillis;

    // TAKE THE SENSOR READINGS - GB: NEED TO UPDATE THESE FOR MY NEW SENSORS (OR ANYTIME I CHANGE THEM!!!)

    if (senseactive[0]) {
      sensevals[0] = readBoost(A0, 0); // read boost off A0 and store at index 0
      updatePEAK(0); // TURBO
    }

    if (senseactive[1]) {
      sensevals[1] = readERR2081(A1, 1); // read A1, currently the Gearbox oil temp sensor
      updatePEAK(1); // TBOX OIL TEMP
    }
    
    if (senseactive[2]) {
      //sensevals[2] = readMAX(2); //read EGT from the Max31856
      sensevals[2] = readAmbientTemp(2); //read EGT from the Max31856     
      updatePEAK(2); // EGT
    }
    
    if (senseactive[3]) {
      sensevals[3] = readPress(A2, 3); // placeholder at the moment but should be very similar to the boost reading if a cheap pressure sensor is used (ie one which returns a linear voltage 0-5v based on presure)
      updatePEAK(3); // OIL PRESSURE
    }
    
    if (senseactive[4]) {
      sensevals[4] = readERR2081(A3, 4); // read A7, store at index 4 currently the Engine oil temp sensor
      updatePEAK(4); // OIL TEMP
    }

    if (senseactive[5]) {
      sensevals[5] = readCoolantLevel(A4, 5); // read A6, to check if coolant level is low
      //updatePEAK(5); // Coolant Level - no need to set a max as this is boolean
    }
    
    if (senseactive[6]) {
      sensevals[6] = readADXL(6); // Inclinometer - Y (roll) angle
      updatePEAK(6);
    }

    // Update TFT Display
    drawDISPLAY1();
  
  }
}

void drawDISPLAY1(void) { // DISPLAY 1 Main guage display

  uint8_t sensor1 = posrot(1);
  uint8_t sensor2 = posrot(2); 
  uint8_t sensor3 = posrot(3); 
  uint8_t sensor4 = posrot(4);   
   
  drawSensorGB(0, display1, sensor1);  // arguments (Y offset, display object ref, sensor to plot)
  //drawBarGraph(display1, sensor1);  // GB: NEED TO UPDATE REFRESH SCHEME FOR BARGRAPH
  drawSensorGB(35, display1, sensor2);  // arguments (Y offset, display object ref, sensor to plot)
  drawSensorGB(70, display1, sensor3);  // arguments (Y offset, display object ref, sensor to plot)
  drawSensorGB(105, display1, sensor4);  // arguments (Y offset, display object ref, sensor to plot)   
}

// Helper Functions

void drawSensorGB(uint8_t y, Adafruit_ST7735 &refDisp, uint8_t sensor) {
  uint8_t xoffset = 0;
  String temp;
  int8_t rolltemp = 0;
  
  refDisp.setTextSize(2);
  refDisp.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  refDisp.setTextWrap(false);

  refDisp.setCursor(46, y + 7);
  //refDisp.println(valIfnoErr(sensor));
  String xxx;
  xxx = valIfnoErr(sensor);
  for (int jj=xxx.length(); jj<3; jj++)
  {
    xxx.concat(" ");
  }
  refDisp.print(xxx); 
  
  temp = valIfnoErr(sensor);
  //xoffset = (temp.length() * 13) + 5 ; // work out width of the characters so we can move the cursor to the correct position to display our units symbol
  xoffset = (3 * 13) + 5 ; // work out width of the characters so we can move the cursor to the correct position to display our units symbol

  if (sensefault[sensor] > 0 || sensor == 5) {  // normal size text if it's an error message or it's our low coolant warning sensor
    refDisp.setTextSize(2);
    refDisp.setCursor(46, y + 7);
  }  else {                                     // else use a smaller font for the units display
    refDisp.setTextSize(1);
    refDisp.setCursor(46 + xoffset, y + 7);
  }

  refDisp.println(units(sensor));

  // THIS IS INSTEAD OF THE SENSOR TYPE ICON
  refDisp.setTextSize(2);
  refDisp.setCursor(0, y + 7);
  refDisp.println(sensorLabel[sensor]);

  // Sensor warnings
  refDisp.setFont(&FreeSansBold12pt7b);
  refDisp.setTextSize(1);
  refDisp.setTextColor(ST77XX_RED, ST77XX_BLACK);
  
  if (hiloWARN(sensor)) {  
    refDisp.setCursor(112, y+22); 
    refDisp.println("!");  
  }
  else {
    refDisp.setCursor(112, y+22); 
    refDisp.println(" "); 
  }
  
  if (faultWARN(sensor) == 1) {
    refDisp.setCursor(112, y+22); 
    refDisp.println("X");  
  }
  else {
    refDisp.setCursor(112, y+22); 
    refDisp.println(" "); 
  }
   
  refDisp.setFont(); //reset to basic font 
}
///////////////////////////END OF GB TEXT ONLY VERSION//////////////
////////////////////////////////////////////////////////////////////


void drawBarGraph(Adafruit_ST7735 &refDisp, uint8_t sensor) {
  int padding = 0;
  float scalerange = 0;
  int scaleposmin = 0;
  
  refDisp.drawLine(11, 52, 11, 54, ST77XX_WHITE); // draw our gauge and scale markings
  refDisp.drawLine(64, 52, 64, 54, ST77XX_WHITE);
  refDisp.drawLine(116, 52, 116, 54, ST77XX_WHITE);
  refDisp.drawRect(11, 42, 106, 10, ST77XX_WHITE); //Border of the bar chart
  if (senseminvals[sensor] < 0) { // Work out a positive range of values that we need to plot
    scaleposmin = senseminvals[sensor] * -1;
    scalerange = scaleposmin + sensemaxvals[sensor];
  } else {
    scaleposmin = senseminvals[sensor];
    scalerange = sensemaxvals[sensor] - scaleposmin;
  }
  refDisp.fillRect(14, 44, (100 / scalerange * (sensevals[sensor] + scaleposmin)), 6, ST77XX_WHITE); //Draws the bar depending on the sensor value

  refDisp.drawLine(13 + (100 / scalerange * (sensepeakvals[sensor] + scaleposmin)), 41, 13 + (100 / scalerange * (sensepeakvals[sensor] + scaleposmin)), 50, ST77XX_WHITE); // draw the peak value line;
  if (sensevals[sensor] < 100) { // adjust padding for the low value so it looks nice
    padding = 0;
  } else {
    padding = -4;
  }
  refDisp.setCursor(8 + padding + (100 / scalerange * (sensepeakvals[sensor] + scaleposmin)), 33); // set cursor with padding
  refDisp.setTextSize(1);
  refDisp.println(String(sensepeakvals[sensor])); // and write the peak val
  refDisp.setCursor(8, 57);
  refDisp.println(String(senseminvals[sensor])); // draw the minumum value
  refDisp.setCursor(58, 57);
  if (senseminvals[sensor] < 100) { // adjust padding for the low value so it looks nice
    padding = 8;
  } else {
    padding = 0;
  }
  refDisp.println(String(static_cast<int>(((sensemaxvals[sensor] - senseminvals[sensor]) / 2) - scaleposmin))); // draw the midpoint value
  if (sensemaxvals[sensor] < 100) { // adjust padding for the high value so it looks nice/doesnt wrap off screen
    padding = 10;
  } else {
    padding = 5;
  }
  refDisp.setCursor(100 + padding, 57);
  refDisp.println(String(sensemaxvals[sensor])); // draw the maximum value
}

bool hiloWARN(uint8_t sensorZ) {
  // this function toggles a an error flag if the current sensor is above it's high warning parameter - since the display is redrawn every 250ms it appears to flash
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
  // toggles the fault strate of a sensor (to make our warning symbols flash)
  if (sensefault[sensorPin] == 2) {
    sensefault[sensorPin] = 2; // 2 is animation off
  } else {
    sensefault[sensorPin] = 1; // 1 is animation on
  }
}

void updatePEAK(uint8_t sensorPin) {
  // stores the current value of a sensor if it is above the previously stored high value
  if (sensevals[sensorPin] >= senseminvals[sensorPin]) { // only do this if the value is above the min
    if (sensevals[sensorPin] <= sensemaxvals[sensorPin]) { // only do this if the value is below the max
      if (sensevals[sensorPin] > sensepeakvals[sensorPin]) {
        sensepeakvals[sensorPin] = int(sensevals[sensorPin]); //if we have a new max then store it
      }
    }
  }
}

String units(uint8_t sensor) { // returns the units associated with the sensor, or some fault text
  // if a fault is set return ERR
  if (sensefault[sensor] > 0 && sensor != 5) {
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
  }
}

String valIfnoErr(uint8_t sensor) { //prevents values being displayed if we are in fault state OR this is a boolean sensor (coolant level)
  String text = String(sensevals[sensor]);
  // if a fault is set return an empty string

  if (sensor == 5) {
    return (F(""));
  }
  if (sensefault[sensor] > 0 || sensor == 5) {
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

// Sensor reading code.

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


int readBoost(uint8_t sensorPin, uint8_t index) {
  int rawval;
  float kpaval;
  float boost;
  rawval = analogRead(sensorPin);       // Read MAP sensor raw value on analog port 0
  kpaval = rawval * 0.4878;             // convert to kpa
  boost = kpaval * 0.145038 - 14.5038;  // Convert to psi and subtract atmospheric (sensor is absolute pressure)
  // process any faults
  return (doFaults(DIVISOR/100, rawval, int(boost), index));
}


int readMAX(uint8_t index) {
  // Make sure you remove the delay(250) from the adafruit_MAX31856 readThermocoupleTemperature() - otherwise the screen rotation and refresh slows down to approx every 500ms
  int t;
  t = max.readThermocoupleTemperature();
  // process any faults
  return (doFaults(max.readFault(), 0, t, index));
}


int readPress(uint8_t sensorPin, uint8_t index) {
  //just a dummy at present
  int p;
  p = analogRead(sensorPin);
  // process any faults
  return (doFaults(DIVISOR/100, p, p, index));
}


bool readCoolantLevel(uint8_t sensorPin, uint8_t index) {
  // sensor is normally closed
  // use a pulldown resistor to enable fault monitoring
  int CoolantLevel;
  CoolantLevel = analogRead(sensorPin);
  // process any faults
  return ((bool)doFaults(DIVISOR/2, CoolantLevel, CoolantLevel, index));
}


int readADXL(uint8_t index) {
  // use fabo calls on uno/nano (smaller code/ram footprint
  int ax, ay, az;

  if (!accel.searchDevice()) {
    return (doFaults(1, 0, 0, index)); // set fault state if the ADXL345 is not connected
  }
  accel.readXYZ(&ax, &ay, &az); //ADXL345
  
  // we're only interested in one axis for vehicle roll - picth may go in at a later date (just needs some gfx knocking up)
  //  double xAngle = atan( ax / (sqrt(square(ay) + square(az))));
  double yAngle = atan( ay / (sqrt(sq(ax) + sq(az))));
  //  double zAngle = atan( sqrt(square(ax) + square(ay)) / az);
  //  xAngle *= 180.00;
  yAngle *= 180.00;
  //  zAngle *= 180.00;
  //  xAngle /= 3.141592;
  yAngle /= 3.141592;
  //  zAngle /= 3.141592;
  return (int(yAngle));
}

//int readMMA7660(uint8_t index) { 
//  int8_t x, y, z;
//
////  if (!accel.searchDevice()) {
////    return (doFaults(1, 0, 0, index)); // set fault state if the ADXL345 is not connected
////  }
//
//  acc.getXYZ(&x,&y,&z);
//  
//  // we're only interested in one axis for vehicle roll - picth may go in at a later date (just needs some gfx knocking up)
//  //  double xAngle = atan( x / (sqrt(square(y) + square(z))));
//  double yAngle = atan( y / (sqrt(sq(x) + sq(z))));
//  //  double zAngle = atan( sqrt(square(x) + square(y)) / z);
//  //  xAngle *= 180.00;
//  yAngle *= 180.00;
//  //  zAngle *= 180.00;
//  //  xAngle /= 3.141592;
//  yAngle /= 3.141592;
//  //  zAngle /= 3.141592;
//  return (int(yAngle));
//}


int readAmbientTemp(uint8_t index) {
  int ambTemp;
//  if (!mlx.searchDevice()) {
//    return (doFaults(1, 0, 0, index)); // set fault state if the MLX90416 is not connected
//  }
  ambTemp = int(mlx.readAmbientTempC());
  return (doFaults(DIVISOR/100, ambTemp, ambTemp, index));
}


int readObjectTemp(uint8_t index) {
  int ambTemp;
//  if (!mlx.searchDevice()) {
//    return (doFaults(1, 0, 0, index)); // set fault state if the MLX90416 is not connected
//  }
  ambTemp = int(mlx.readObjectTempC());
  return (doFaults(DIVISOR/100, ambTemp, ambTemp, index));
}


void tftInit(Adafruit_ST7735 tft) {
  tft.initR(INITR_144GREENTAB);   // initialize the ST7735S chip 
  tft.setRotation(1);
  tft.cp437(1);   // The fix to enable (or disable) Code Page 437-compatible charset
  tft.fillScreen(ST77XX_BLACK); 
  
  //tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  //tft.setTextSize(4);
  //tftlay1.setCursor(55,50);
  //tft.print(char(226));  // char(226) is Greek Gamma ;) Also, char(2) is a smiley see https://en.wikipedia.org/wiki/Code_page_437
  //delay(1000); 
   
  if(!SD.begin(SD_CS, SD_SCK_MHZ(25))) { // ESP32 requires 25 MHz limit
    Serial.println(F("SD begin() failed"));
    for(;;); // Fatal error, do not continue
  }
  
  ImageReturnCode stat;
  stat = reader.drawBMP("/SUZUKI.BMP", display1, 0, 0);
  delay(3000); 
  tft.fillScreen(ST7735_BLACK);;   // clears the screen and buffer
}
