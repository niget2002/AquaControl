/*

    Prepping Environment:

    Get the pH 7 and pH 4 solutions
    Get a cup of RO/DI water
    Take Probes out of aquarium and rinse in RO DI solution
    Start Calibration process


*/

#include "EEPROM.h" //Standard EEPROM Library


//*********************** User defined variables ****************************//
//pH meter Analog output to Arduino Analog Input 0
#define PHPin A0
//I got this from shorting the nbc's cnetre pin to outer pin [simulated the perfect probe] open serial and its the mv reading that you want to put here
#define Healthy1_mv 1.96
#define Vs 5
#define addresCalibrationPH4 0
#define addresCalibrationPH7 50

//************** Some values for working out the ph*****************//
float mvReading = 0;
float phValue = 0;
float Slope = 0;
float mvReading_7 = 0;
float mvReading_4 = 0;
float offset = 0;
int value = 0;
float average = 0;


//************************************** Setup Loop Runs Once ****************//
void setup()
{
  Serial.begin(9600);
  Read_Eprom();
  Slope_calc();
}


//******************** Main Loops runs Forver ************************************//
void loop()
{
  //All these functions are put below the main loop, keeps the loop logic easy to see
  CalibratePH();
  delay(100);
};

//*************************** Checking what we stored in non volatile memory last time ************//
void Read_Eprom() {

  //************** Restart Protection Stuff ********************//
  //the 254 bit checks that the adress has something stored to read [we dont want noise do we?]
  value = EEPROM.read(addresCalibrationPH7);
  mvReading_7 = value * Vs / 256;
  delay(10);

  value = EEPROM.read(addresCalibrationPH4);
  mvReading_4 = value * Vs / 256;
  delay(10);
};



//*************************Take Ten Readings And Average ****************************//
float ReadPH() {
  int i = 0;
  unsigned long sum = 0;
  long reading = 0;

  while (i <= 20) {
    reading = analogRead(PHPin);
    sum = sum + reading;
    delay(10);
    i++;
  }
  average = sum / i;

  //Converting to mV reading and then to pH
  mvReading = average * Vs / 1024;
  //phValue=mvReading*K_PH;
  phValue = (7 - ((mvReading_7 - mvReading) * Slope));

  //voltage = avergearray(pHArray, ArrayLenth)*5.0/1024;
  //pHValue = 3.5*voltage+Offset


  return mvReading;
}


//******************** calculating the PhMeter Parameters ***************************************//
void Slope_calc() {
  offset = Healthy1_mv - mvReading_7;
  Slope = 3 / (Healthy1_mv - mvReading_4 - offset);

}


//******************************* Checks if Select button is held down and enters Calibration routine if it is ************************************//
void CalibratePH() {

  // Could use Serial.println to let the Arduino control the Pi at this point.

  //Update Screen to place Probe in pH 7
  Serial.println("Place probe in pH 7");
  //Give user 30 seconds to comply
  delay(30000);

  //We are giving the probe 1 minute to settle
  Serial.println("Settling...");
  delay(60000);
  Serial.println("Reading pH7");
  mvReading_7 = ReadPH();
  value = average / 4;
  EEPROM.write(addresCalibrationPH7, value);

  Slope_calc(); // Slope will be false until after the pH4 reading has been taken

  //Update Scree to rinse probe and
  //Place probe  pH4 Calibration
  Serial.println("Rinse and put pH 4");
  //Give user 30 seconds to comply
  delay(30000);


  //We are giving the probe 1 minute to settle
  Serial.println("Settling......");
  delay(60000);
  Serial.println("Reading pH4");
  mvReading_4 = ReadPH();
  value = average / 4;
  EEPROM.write(addresCalibrationPH4, value);

  Slope_calc(); // Slope is now correct

  Serial.print("pH 7 Value: ");
  Serial.println(mvReading_7);
  Serial.print("pH 4 Value: ");
  Serial.println(mvReading_4);
  Serial.print("Offset: ");
  Serial.println(offset);
  Serial.print("Slope: ");
  Serial.print(Slope);

  //Update Screen to rinse probe and
  //Place back in tank

  return;
};

