#include <Wire.h>  // Necessary for I2C communication
int accel = 0x53;  // I2C address for this sensor (from data sheet)
float x, y, z, acc;
const int WindowTime = 1;     //Window time in seconds
const int SamplingRate = 50;  //Sampling rate in Hz
const int zero_buffer = 5;
const int WindowSize = ceil(WindowTime*SamplingRate);  //Number of Data Point per window
float accVals[WindowSize] = {1000};
int FirstZeroIndex = 0;
bool ArrayFull = true;
bool downstroke = false;
int stepcounter = 0;
bool a1 = false;
//const float cutoff = 5.0; //cutoff frequency of digital low pass
//float filtval = 0.0;
//float alpha = ((1.0/SamplingRate )/ ((1.0/(2.0*PI*cutoff)) + 1.0/SamplingRate));


void setup() {
  Serial.begin(9600);
  Wire.begin();                   // Initialize serial communications
  Wire.beginTransmission(accel);  // Start communicating with the device
  Wire.write(0x2D);               // Enable measurement
  Wire.write(8);                  // Get sample measurement
  Wire.endTransmission();
  for (int i = 0; i < WindowSize; i++) {
    accVals[i] = 1000;
  }
}

void FindStepWaveform(float* accVals, const int WindowSize, int zero_buffer) {
  for (int i = 0; i <= WindowSize - (zero_buffer + 2); i++) {
    if (accVals[i] >= 1) {
      int oneCounter = 0;
      while ((i + oneCounter + 1 < WindowSize) && accVals[i + oneCounter + 1] >= 1) {  //while you haven't run out of space and the next uncounted item is also a one
        oneCounter++;
      }
      if (oneCounter >= zero_buffer && (i + oneCounter + 1 < WindowSize) && accVals[i + oneCounter + 1] == 0) {
        stepcounter++;
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


//   for (int i = 0; i <= WindowSize - (zero_buffer + 2); i++) {
//     if (accVals[i] >= 1) {
//       int zeroCounter = 0;
//       while ((i + zeroCounter + 1 < WindowSize) && accVals[i + zeroCounter + 1] == 0) {  //while you haven't run out of space and the next uncounted item is also a zero
//         zeroCounter++;
//       }
//       if (zeroCounter >= zero_buffer && (i + zeroCounter + 1 < WindowSize) && accVals[i + zeroCounter + 1] == 1) {
//         stepcounter++;
//         // Set the region to 1s (to mark it as processed)
//         for (int j = 0; j < zeroCounter + 1; j++) {
//           accVals[i + j] = 1;
//         }
//         // Skip over the step region
//         i += zeroCounter + 1;
//       }
//     }
//   }
// }

  void loop() {
    delay(floor(1000 / SamplingRate));
    Wire.beginTransmission(accel);
    Wire.write(0x32);  // Prepare to get readings for sensor (address from data sheet)
    Wire.endTransmission(false);
    Wire.requestFrom(accel, 6, true);              // Get readings (2 readings per direction x 3 directions = 6 values)
    int16_t x = Wire.read() | (Wire.read() << 8);  // Parse x values
    int16_t y = Wire.read() | (Wire.read() << 8);  // Parse y values
    int16_t z = Wire.read() | (Wire.read() << 8);  // Parse z values

    x = x / 256;
    y = y / 256;
    z = z / 256;
    acc = sqrt((x * x + y * y + z * z));


    //filtval =  (alpha * acc) + (1-alpha) * filtval;

    //Serial.println("START");
    //Serial.println(z);
    //Serial.println(y);
    //Serial.println(x);
    //Serial.println(acc);
    Serial.println(acc);
    Serial.print("Steps: ");
    Serial.println(stepcounter);
    ArrayFull = true;
    for (int i = 0; i < WindowSize; i++) {
      if (accVals[i] == 1000) {
        ArrayFull = false;
        FirstZeroIndex = i;
        break;
      }
    }

    if (!ArrayFull) {
      accVals[FirstZeroIndex] = acc;
    } else {  //Check for steps then shift window to the left
      FindStepWaveform(accVals, WindowSize, zero_buffer);
      for (int i = 0; i < WindowSize - 1; i++) {
        accVals[i] = accVals[i + 1];
      }
      accVals[WindowSize - 1] = acc;
    }

    // for (int i = 0; i < WindowSize;i++){

    //   Serial.print(accVals[i]);
    //   Serial.print(",");
    // }
    // Serial.println("END Data Point Acquisition");
    //Serial.println("");
    //Serial.println("");

    //Serial.print("x = "); // Print values
    //Serial.println(x);
    //Serial.print(", y = ");
    // Serial.println(y);
    //Serial.print(", z = ");
    //Serial.println(z);
    //Serial.print(", acc = ");
    //Serial.println(acc);
  }