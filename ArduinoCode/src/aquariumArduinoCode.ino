#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define LEDPIN 13
//Number of Items
#define numAnalogs 3
#define Channels_Count_Arduino 10
#define numFloats 5
#define numAlarms 4
#define startPin 2

//Analogs
#define MPH0 A0
#define MPH1 A1

//Floats
#define floatStart 15

// Tank  Range
#define tankTempLow 25  //in Celsius 77F
#define tankTempHigh  28  //in Celsius 82.4F
#define tankPhLow 7.8
#define tankPhHigh 8.2  //not actually used
#define caPhLow 6.5
#define caPhHigh 6.7  //not actually used

//OneWire
#define ONE_WIRE_BUS 14

//Serial Comms
#define serialBuffer 8

//PumpOrder
#define pumpReturn 0
#define pumpRecirc 1
#define pumpCa 2
#define pumpTopOff 3

//Output Channel Mappings
int Ch[Channels_Count_Arduino + 1];
int myLights[] = { 0, 1 }; // Light1, Light2
int myPumps[] = { 2, 3, 4, 5 }; // return, recirc, CA, topoff
int myOther[] = { 6, 7, 8, 9 }; // heater, chiller, CO2, OM
//Arrays
//int incomingByte[serialBuffer];
char incomingByte[serialBuffer];
float analogs[numAnalogs];
int analogsTrigger = 0;
float analogsPin[] = { 0, 0 };
float yIntercept[] = { 0.00, 0.00 };

int floatTrigger = 0;
int floatPin[numFloats];
int floatAverage[numFloats] = { 0, 0, 0, 0, 0 }; // Top-Off Low, Normal Sump, Sump Low, Tank High, Protein High
int alarms[numAlarms]; //  Top-Off Low, Sump Low, Tank High, Protein High

//Alarms
int alarmSet = 0;

// We don't need to trigger Alarms every iteration of the loop... every half second should be fine.
unsigned long previousMillis = 0;
const long interval = 1000;

//OneWire Setup for Temperature Sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//PH Calibration Data
// need to modify all calibration values to work for both probes
#include "EEPROM.h" //Standard EEPROM Library
//I got this from shorting the nbc's cnetre pin to outer pin [simulated the perfect probe] open serial and its the mv reading that you want to put here
#define Healthy1_mv 1.96
#define Vs 5
int addresCalibrationPH4[2] = { 0, 100 };  // convert to array of int
int addresCalibrationPH7[2] = { 50, 150 }; // convert to array of int
//************** Some values for working out the ph*****************//
float mvReading = 0;
float Slope[2] = { 3.5, 3.5 }; // convert to array
float mvReading_7[2] = { 0, 0 }; // convert to array
float mvReading_4[2] = { 0, 0 }; // convert to array
int value = 0;
float average = 0;

// **************************************************************************************************************************
void setup() {
  pinMode(LEDPIN, OUTPUT);
  Serial.begin(9600);                     // set up Serial at 9600 bps N81

  // initialize Timer1
  timerSetup();
  sensorSetup();
  digitalSetup();
  floatSetup();
  alarmSetup();
  Read_Eprom(0); //need to convert All calibration functions to work for both probes
  Read_Eprom(1);
  Slope_calc(0);
  Slope_calc(1);

}

ISR(TIMER1_OVF_vect) {
  sensors.requestTemperatures();
}

void loop() {
  unsigned long currentMillis = millis();

  readPiData();
  floatCheck();
  analogGet();

  if ( currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    analogTemp();
    analogCheck();
    alarmCheck();
  }
}


//*************************************************************************
//  FUNCTIONS
//*************************************************************************

// ALARMS ***********************************************
void alarmSetup( void ) {
  for (int i = 0; i <= numAlarms; i++) {
    alarms[i] = 0;
  }
}

void alarmCheck( void ) {

  if ((floatAverage[1] == 1) && (floatAverage[0] == 0)) { // if normal sump is low, and top-off is high turn on topoff
    floatAlarm(1, myPumps[pumpTopOff]);
  } else {
    floatAlarm(0, myPumps[pumpTopOff]);
  }

  if (floatAverage[0] == 1) { // if top-off is low, turn off top-off
    floatAlarm(0, myPumps[pumpTopOff]);
    alarms[0] = 1;
  } else {
    alarms[0] = 0;
  }

  if (floatAverage[2] == 1) { // if Sump Low, turn off Return Pump
    floatAlarm(0, myPumps[pumpReturn]);
    alarms[1] = 1;
    alarmSet = 1;
    digitalWrite(LEDPIN, HIGH);
  } else {
    if (alarmSet == 0) {
      alarms[1] = 0;
      digitalWrite(LEDPIN, LOW);
    }
  }

  if (floatAverage[3] == 1) { // if Tank High, turn off Return Pump
    floatAlarm(0, myPumps[pumpReturn]);
    alarms[2] = 1;
    alarmSet = 1;
    digitalWrite(LEDPIN, HIGH);
  } else {
    if (alarmSet == 0) {
      alarms[2] = 0;
      digitalWrite(LEDPIN, LOW);
    }
  }

  if (floatAverage[4] == 1) { //Protein High
    alarms[3] = 1;
  } else {
    alarms[3] = 0;
  }
}


// ANALOGS **********************************************
void analogGet(void) {

  analogsTrigger++;
  analogsPin[0] += analogRead(MPH0) * Vs / 1024;
  analogsPin[1] += analogRead(MPH1) * Vs / 1024;

  if (analogsTrigger == 100) { // need to convet these so that 3.5 is replaced with PH slope
    analogs[0] = Slope[0] * ((analogsPin[0] / 100)) + yIntercept[0];
    analogs[1] = Slope[1] * ((analogsPin[1] / 100)) + yIntercept[1];
    analogsTrigger = 0;
    analogsPin[0] = 0;
    analogsPin[1] = 0;

  }


}

void analogTemp(void) {
  float val;
  val = sensors.getTempCByIndex(0);
  // Sanity check due to oddities with Temp Measurement
  if ( (10 < val) && (val < 50) ) {
    analogs[2] = val;
  }
}

void analogCheck(void) {
  // We Want to keep the Tank temperature between two points using both
  // a heater and a chiller. To help reduce cycling heaters/chillers on/off
  // we will do the following:
  //
  // In the event the temperature gets too low, turn on a heater
  // wait until the temperature is 1 degree hotter than the low point to turn
  // the heater back off
  //
  // If the tank gets too hot, turn on the chiller. Wait until the temperature
  // is 1 degree colder than the hot temperature to turn chiller back off
  //
  // Turn Heater ON / OFF
  if ( tankTempLow > analogs[2] ) {
    ledMatrix_ON(myOther[0]);
  }
  else {
    if(digitalRead(myOther[0])){
      if( tankTempLow+.2 < analogs[2]){
        ledMatrix_OFF(myOther[0]);
      }
    }
  }

  // Turn Chiller ON/ OFF Verify that the main Return pump is running before turning on
  if ( (tankTempHigh < analogs[2]) && digitalRead(myPumps[pumpReturn]) ) {
    ledMatrix_ON(myOther[1]);
  }
  else { // After temp is lowered proper amount OR if main Pump is turned off
    if(digitalRead(myOther[1])){
      if( tankTempHigh-.2 > analogs[2]){
        ledMatrix_OFF(myOther[1]);
      }
    }
  }

  // The Reaction to CO2 changes is so slow, there's no need to debounce using
  // the above algorithm like for temperature.
  // Turn CO2 ON/ OFF
  if ( tankPhLow > analogs[0] || caPhLow > analogs[1]) {
    ledMatrix_OFF(myOther[2]);
  } else {
    ledMatrix_ON(myOther[2]);
  }

}

// DIGITALS *********************************************

void digitalSetup(void) {
  for (int i = 1; i <= Channels_Count_Arduino; i++) Ch[i] = startPin + i - 1 ;          // Starting pin assignment on Arduino incrementing up to total pin assigned
  for (int i = 1; i <= Channels_Count_Arduino; i++) {
    pinMode(Ch[i], OUTPUT); // declare all arduino pins as an OUTPUT
    ledMatrix_OFF(i - 1); // Set all outputs HIGH since Relays are active LOW
  }
}

void ledMatrix_ON(int index) {
  // Relays connected directly to Arduino
  digitalWrite(Ch[index + 1], LOW);

}

void ledMatrix_OFF(int index) {
  // Relays connected directly to Arduino
  digitalWrite(Ch[index + 1], HIGH);

}

// FLOATS *****************************************
void floatSetup(void) {
  for (int i = 0; i < numFloats; i++) {
    pinMode(floatStart + i, INPUT_PULLUP);
  }
}

void floatCheck(void) {
  //Check waterTopOff
  floatTrigger++;
  for (int i = 0; i < numFloats; i++) {
    floatPin[i] = floatPin[i] + digitalRead(floatStart + i);
  }
  if (floatTrigger == 100) {
    for (int i = 0; i < numFloats; i++) {
      floatAverage[i] = floatPin[i] / 100;
      floatPin[i] = 0;
    }
    floatTrigger = 0;
  }
}

void floatAlarm( int thisFloat, int thisPump ) {
  if (thisFloat == 1) {
    ledMatrix_ON(thisPump);
  }
  else {
    ledMatrix_OFF(thisPump);
  }

}

// ONEWIRE *****************************************
void sensorSetup(void) {
  sensors.begin();
  sensors.setWaitForConversion(false);
}

// SERIALS *****************************************

void readPiData() {
  int setChan = -1;
  byte i = 0;
  char probe;
  float val;
  //  if (Serial.available() >= serialBuffer ) {
  //    for (i = 0; i <= serialBuffer; i++) {
  //      incomingByte[i] = Serial.read(); // All the channels are read plus 1 for header starting with incomingByte[0]
  //    }

  if (Serial.available() > 0 ) {
    Serial.readBytesUntil('X', incomingByte, 8);


    int lookup = incomingByte[1] - '0';
    switch (incomingByte[0]) {
      case 'L':
        setChan = myLights[lookup];
        break;
      case 'P':
        setChan = myPumps[lookup];
        break;
      case 'O':
        setChan = myOther[lookup];
        break;
      case 'h':
        Serial.println(analogs[0]);
        break;
      case 'i':
        Serial.println(analogs[1]);
        break;
      case 't':
        Serial.println(analogs[2]);
        break;
      case 'R':
        lookup = incomingByte[2] - '0';
        switch (incomingByte[1]) {
          case 'L':
            Serial.println(digitalRead(Ch[myLights[lookup] + 1]));
            break;
          case 'P':
            Serial.println(digitalRead(Ch[myPumps[lookup] + 1]));
            break;
          case 'O':
            Serial.println(digitalRead(Ch[myOther[lookup] + 1]));
            break;
        }
        break;
      case 'A':
        lookup = incomingByte[2] - '0';
        switch (incomingByte[1]) {
          case 'R':
            alarmSet = 0;
            break;
          default:
            Serial.println(alarms[lookup]);
        }
        setChan = -1;
        break;
      case 'C':
        probe = incomingByte[1];

        switch(probe){
          case '0':
            CalibratePH(0);
            break;
          case '1':
            CalibratePH(1);
        }


        setChan = -1;
        break;

      default:
        setChan = -1;
        break;
    }
    if (setChan != -1) {
      switch (incomingByte[2]) {
        // Relays are active LOW, so ON = OFF and vice versa
        case '0':
          ledMatrix_OFF(setChan);
          break;
        case '1':
          ledMatrix_ON(setChan);
          break;
      }
    }
  }
}

// TIMER *****************************************************
void timerSetup(void) {
  cli();         // disable global interrupts
  TCCR1A = 0;    // set entire TCCR1A register to 0
  TCCR1B = 0;    // set entire TCCR1B register to 0
  // (as we do not know the initial  values)

  // enable Timer1 overflow interrupt:
  TIMSK1 |= (1 << TOIE1); //Atmega8 has no TIMSK1 but a TIMSK register

  // Set CS10 bit so timer runs at clock speed: (no prescaling)
  TCCR1B |= (1 << CS10); // Sets bit CS10 in TCCR1B
  TCCR1B |= (1 << CS12);
  // This is achieved by shifting binary 1 (0b00000001)
  // to the left by CS10 bits. This is then bitwise
  // OR-ed into the current value of TCCR1B, which effectively set
  // this one bit high. Similar: TCCR1B |= _BV(CS10);

  // enable global interrupts:
  sei();

}

//PH Calibration
//*************************** Checking what we stored in non volatile memory last time ************//
void Read_Eprom(int phprobe) {

  mvReading_7[phprobe] = EEPROM.read(addresCalibrationPH7[phprobe]);
  delay(10);

  mvReading_4[phprobe] = EEPROM.read(addresCalibrationPH4[phprobe]);
  delay(10);
};

//*************************Take Ten Readings And Average ****************************//
float ReadPH(int phprobe) { // Return the average mV reading
  int i = 0;
  unsigned long sum = 0;
  long reading = 0;

  while (i <= 20) {
    if(phprobe) { reading = analogRead(MPH1) * Vs / 1024; }
    else { reading = analogRead(MPH0) * Vs / 1024; }

    sum = sum + reading;
    delay(10);
    i++;
  }
  average = sum / i;
  return average;
}

//******************** calculating the PhMeter Parameters ***************************************//
void Slope_calc(int phprobe) {

  Slope[phprobe] = (7-4)/(mvReading_7[phprobe]-mvReading_4[phprobe]);
  yIntercept[phprobe] = 7 - (Slope[phprobe] * mvReading_7[phprobe]);
}

//******************************* Checks if Select button is held down and enters Calibration routine if it is ************************************//
void CalibratePH(int phprobe) {

  // Could use Serial.println to let the Arduino control the Pi at this point.

  //Update Screen to place Probe in pH 7
  Serial.print("Starting Calibration for PH Probe");
  Serial.println(phprobe);
  Serial.println("Place probe in pH 7");
  //Give user 30 seconds to comply
  delay(15000);

  //We are giving the probe 1 minute to settle
  Serial.println("Settling...");
  delay(60000);
  Serial.println("Reading pH7");
  mvReading_7[phprobe] = ReadPH(phprobe);
  EEPROM.write(addresCalibrationPH7[phprobe], mvReading_7[phprobe]);
  //Update Screen to rinse probe and
  //Place probe  pH4 Calibration
  Serial.println("Rinse and put pH 4");
  //Give user 30 seconds to comply
  delay(15000);
  //We are giving the probe 1 minute to settle
  Serial.println("Settling......");
  delay(60000);
  Serial.println("Reading pH4");
  mvReading_4[phprobe] = ReadPH(phprobe);
  EEPROM.write(addresCalibrationPH4[phprobe], mvReading_4[phprobe]);

  Slope_calc(phprobe); // Slope is now correct

  Serial.print("pH 7 mV Value: ");
  Serial.println(mvReading_7[phprobe]);
  Serial.print("pH 4 mV Value: ");
  Serial.println(mvReading_4[phprobe]);
  Serial.print("yIntercept: ");
  Serial.println(yIntercept[phprobe]);
  Serial.print("Slope: ");
  Serial.println(Slope[phprobe]);

  return;
};
