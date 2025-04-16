#include <Wire.h>  // Necessary for I2C communication
#include <LiquidCrystal.h>
#include <ArduinoBLE.h>

//Initialize Bluetooth Interface and Data Transfer
BLEService newService("180A");
BLEStringCharacteristic readChar("34aaae89-580a-46a3-b1d8-bcd82222bd56", BLERead | BLENotify,100);
BLEStringCharacteristic writeChar("6fb9899b-169f-4313-bd0a-7f3c25fb7516", BLEWrite,20);


//Initialize Variables
int accel = 0x53;  // I2C address for this sensor (from data sheet)
float x, y, z, acc, weight,height,goal,calsperstep,stridelength;
const int WindowTime = 1;  //Window time in seconds
float LastSwitch,distancetraveled = 0;
int FirstZeroIndex,stepcounter = 0;
const int SamplingRate = 40;  //Sampling rate in Hz
const int zero_buffer = 5;
const int WindowSize = ceil(WindowTime * SamplingRate);  //Number of Data Point per window
float accVals[WindowSize] = {1000};
bool stepchange,calsactive,distactive,goalactive, downstroke,weightreceived,heightreceived,goalreceived,BluetoothRequest,GoalCelebrated,weightrequestprinted,heightrequestprinted,goalrequestprinted = false;
bool ArrayFull = true;
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

//Initialize custom pixel blocks for LCD Progress Bar Display
byte hollowBlock[8] = {B11111,B10001,B10001,B10001,B10001,B10001,B10001,B11111};
byte fullBlock[8] = {B11111,B11111,B11111,B11111,B11111,B11111,B11111,B11111};


void setup() {
  Serial.begin(9600);
  Wire.begin();                   // Initialize serial communications
  Wire.beginTransmission(accel);  // Start communicating with the device
  Wire.write(0x2D);               // Enable measurement
  Wire.write(8);                  // Get sample measurement
  Wire.endTransmission();

  for (int i = 0; i < WindowSize; i++) { //Initialize all acceleration values to 1000 (well beyond the range our conditioned sensor would ever reach)
    accVals[i] = 1000;
  }
  lcd.begin(16, 2);
  lcd.createChar(0,fullBlock);
  lcd.createChar(1,hollowBlock);

  //Pause to initialize bluetooth connection
  while (!Serial){
  if (!BLE.begin()) {
    Serial.println("Waiting for ArduinoBLE");

  }}

  BLE.setLocalName("ArduinoStepCount");
  BLE.setAdvertisedService(newService);
  newService.addCharacteristic(readChar);
  newService.addCharacteristic(writeChar);
  BLE.addService(newService);

  BLE.advertise();
  Serial.println("Bluetooth device active");
  lcd.setCursor(0, 0);
  lcd.print("Awaiting BLE");
  lcd.setCursor(0, 1);
  lcd.print("Connection");
}

void FindStepWaveform(float* accVals, const int WindowSize, int zero_buffer) {
  stepchange = false;
  for (int i = 0; i <= WindowSize - (zero_buffer + 2); i++) {
    if (accVals[i] >= 1) {
      int oneCounter = 0;
      while ((i + oneCounter + 1 < WindowSize) && accVals[i + oneCounter + 1] >= 1) {  //while you haven't run out of space and the next uncounted item is also a one
        oneCounter++;
      }
      if (oneCounter >= zero_buffer && (i + oneCounter + 1 < WindowSize) && accVals[i + oneCounter + 1] == 0) {
        stepcounter++;
        stepchange = true;
        // Set the region to 0s (to mark it as processed)
        for (int j = 0; j < oneCounter + 1; j++) {
          accVals[i + j] = 0;
        }
        // Skip over the step region
        i += oneCounter + 1;
      }
    }
  }
}

void loop() {
  BLEDevice central = BLE.central();  // Wait for a BLE central

  if (central) {
    if(!weightreceived){ //Wait for user to input weight
      if (weightrequestprinted ==false){
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Input Weight");
        lcd.setCursor(0,1);
        lcd.print("(kg)");
        weightrequestprinted = true;
      }
      if (writeChar.written()) {
        weight = (writeChar.value()).toFloat();
        Serial.print("Received input number: ");
        Serial.println(weight);
        weightreceived = true;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Weight Set: ");
        lcd.setCursor(0,1);
        lcd.print(weight);
        lcd.print("kg");
        delay(1500);
      }
      
  }
  if(weightreceived && !heightreceived){//Once weight is received, wait for user to input height
    if (heightrequestprinted == false){
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Input Height");
        lcd.setCursor(0,1);
        lcd.print("(cm)");
        heightrequestprinted = true;
    }
    if (writeChar.written()) {
        height = (writeChar.value()).toFloat();
        Serial.print("Received input number: ");
        Serial.println(height);
        heightreceived = true;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Height Set: ");
        lcd.setCursor(0,1);
        lcd.print(height);
        lcd.print("cm");
        calsperstep = 0.0000023655*weight*height; //calories burned per step
        stridelength = 0.414 * height/100; //stride length in meters
        delay(1500);
        
      }
      
  }
  if(heightreceived && !goalreceived){//Once height is received, wait for user to input goal
    if (goalrequestprinted == false){
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Input Distance");
        lcd.setCursor(0,1);
        lcd.print("Goal (M)");
        goalrequestprinted = true;
    }
    if (writeChar.written()) {
        goal = (writeChar.value()).toFloat();
        Serial.print("Received input number: ");
        Serial.println(goal);
        goalreceived = true;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Goal Set: ");
        lcd.setCursor(0,1);
        lcd.print(goal);
        lcd.print("M");
        delay(1500);
      }
      
  }
      
  }
if (goalreceived){ // Once the final parameter is received, begin collecting accelerometer date
  delay(floor(1000 / SamplingRate));
  Wire.beginTransmission(accel);
  Wire.write(0x32);  // Prepare to get readings for sensor (address from data sheet)
  Wire.endTransmission(false);
  Wire.requestFrom(accel, 6, true);              // Get readings (2 readings per direction x 3 directions = 6 values)
  int16_t x = Wire.read() | (Wire.read() << 8);  // Parse x values
  int16_t y = Wire.read() | (Wire.read() << 8);  // Parse y values
  int16_t z = Wire.read() | (Wire.read() << 8);  // Parse z values

  x = x / 256; //Intentional integer division digitally improves SNR by subduing noise
  y = y / 256; //Intentional integer division digitally improves SNR by subduing noise
  z = z / 256; //Intentional integer division digitally improves SNR by subduing noise
  acc = sqrt((x * x + y * y + z * z));
}


  ArrayFull = true;
  for (int i = 0; i < WindowSize; i++) { //parse array to determine whether it is full
    if (accVals[i] == 1000) {
      ArrayFull = false;
      FirstZeroIndex = i;
      break;
    }
  }

  if (!ArrayFull) { //Populate array depending on if it is full or still has some initial values
    accVals[FirstZeroIndex] = acc;
  } else {  //Check for steps then shift window to the left
    FindStepWaveform(accVals, WindowSize, zero_buffer);
    for (int i = 0; i < WindowSize - 1; i++) {
      accVals[i] = accVals[i + 1];
    }
    accVals[WindowSize - 1] = acc;
  }
  if (stepchange == true && central) { //Update the 
    lcd.clear();
    lcd.print("Steps: ");
    lcd.print(stepcounter);
    lcd.setCursor(0,1);
    lcd.print("Calories: ");
    lcd.print(calsperstep * stepcounter);
    calsactive = true;
    
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    distancetraveled = stridelength * stepcounter;
    readChar.writeValue("Steps: " + String(stepcounter) +"\n" +
                    " | Distance: " + String(distancetraveled)+"\n" +
                    " | Calories: " + String(calsperstep * stepcounter)+"\n" +
                    " | Goal Progress: " + String(distancetraveled / goal * 100) + "%");

    Serial.println("Data sent to Bluetooth");
     
  }

      float GoalPercent = distancetraveled/goal; //"Celebration" functionality logic
      if (GoalPercent >= 1 && !GoalCelebrated){
        for (int i=0; i<7; i++){
          lcd.clear();
          delay(250);
          lcd.print("WOOHOO");
          lcd.setCursor(0, 1);
          lcd.print("GOAL MET!!!");
          delay(750);
        }
        GoalCelebrated = true;
      }

  if (millis()-LastSwitch >= 5000 && goalreceived){
    lcd.clear();
    lcd.print("Steps: ");
    lcd.print(stepcounter);
    lcd.setCursor(0,1);
    
    if (calsactive){
      lcd.print("Dist. (M): ");
      distancetraveled = stridelength * stepcounter;
      lcd.print(distancetraveled);
      calsactive = false;
      distactive = true;
    }
    else if (distactive){
      lcd.print("Progress: ");
      int NumBlocks = round(GoalPercent*6);
      
      for (int i = 0; i<6;i++){
        if (i<NumBlocks){
          lcd.write(byte(0));
        }
        else{
          lcd.write(byte(1));
        }
      }
      distactive = false;
      goalactive = true;
    }

    else if (goalactive){
      lcd.print("Calories: ");
      lcd.print(calsperstep * stepcounter);
      calsactive = true;
      distactive = false;
    }
    
    LastSwitch = millis();
  }
  Serial.println(acc);
}
