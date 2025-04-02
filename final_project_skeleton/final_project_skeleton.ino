#include <Wire.h> // Necessary for I2C communication
int accel = 0x53; // I2C address for this sensor (from data sheet)
float x, y, z, acc;
const int WindowSize = 10;
float accVals[WindowSize] = {0};
int FirstZeroIndex = 0;
bool ArrayFull = true;

void setup() {
  Serial.begin(9600);
  Wire.begin(); // Initialize serial communications
  Wire.beginTransmission(accel); // Start communicating with the device
  Wire.write(0x2D); // Enable measurement
  Wire.write(8); // Get sample measurement
  Wire.endTransmission();
  for(int i = 0; i < WindowSize;i++){
    Serial.println("");
    accVals[i] = 0;
  }
}

void loop() {
  delay(200);
  Wire.beginTransmission(accel);
  Wire.write(0x32); // Prepare to get readings for sensor (address from data sheet)
  Wire.endTransmission(false);
  Wire.requestFrom(accel, 6, true); // Get readings (2 readings per direction x 3 directions = 6 values)
  x = (Wire.read() | Wire.read() << 8); // Parse x values
  y = (Wire.read() | Wire.read() << 8); // Parse y values
  z = (Wire.read() | Wire.read() << 8); // Parse z values
  acc = sqrt((x*x+y*y+z*z));
  ArrayFull = true;
  for(int i = 0; i < WindowSize;i++){
    if (accVals[i] == 0){
      ArrayFull = false;
      FirstZeroIndex = i;
      break;
    }
    }
  
  if (!ArrayFull) {
    accVals[FirstZeroIndex] = acc;
  }
  else {
    for (int i=0; i < WindowSize-1; i++){
      accVals[i] = accVals[i+1];
    }
    accVals[WindowSize-1]= acc;
  }

for (int i = 0; i < WindowSize;i++){
  Serial.print(accVals[i]);
  Serial.print(",");
}
Serial.println("END Data Point Acquisition");
Serial.println("");
Serial.println("");

  //Serial.print("x = "); // Print values
  //Serial.println(x);
  //Serial.print(", y = ");
 // Serial.println(y);
  //Serial.print(", z = ");
  //Serial.println(z);
  //Serial.print(", acc = ");
  //Serial.println(acc);
}