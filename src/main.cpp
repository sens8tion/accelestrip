#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <FastLED.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }

  #define NUM_LEDS 10
  #define LED_PIN D6

  }
  Serial.println("MPU6050 Found!");

  //setup motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(1);
  mpu.setInterruptPinLatch(true);	// Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  Serial.println("");
  // delay(1);
  
}

void loop() {


  CRGB leds[NUM_LEDS];
  FastLED.addLeds<WS2812B, LED_PIN>(leds, NUM_LEDS);

  while( true ){
    CRGB newdot = CRGB::Black;

    if(mpu.getMotionInterruptStatus()) {
      /* Get new sensor events with the readings */
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      /* Print out the values */
      // Serial.print("AccelX:");
      // Serial.print(a.acceleration.x);
      // Serial.print(",");
      // Serial.print("AccelY:");
      // Serial.print(a.acceleration.y);
      // Serial.print(",");
      // Serial.print("AccelZ:");
      // Serial.print(a.acceleration.z);
      // Serial.print(", ");
      // Serial.print("GyroX:");
      // Serial.print(g.gyro.x);
      // Serial.print(",");
      // Serial.print("GyroY:");
      // Serial.print(g.gyro.y);
      // Serial.print(",");
      // Serial.print("GyroZ:");
      // Serial.print(g.gyro.z);
      // Serial.println("");

      newdot = CRGB((a.acceleration.x+10), (a.acceleration.y+10), (a.acceleration.z+10));

    }

    for(int dot = 0; dot < (NUM_LEDS-1); dot++) { 
      // Serial.print(dot);
      // Serial.print(": ");
      leds[dot] = leds[dot+1];
      // for(int ax = 0; ax < 3; ax ++){
        // Serial.print(" ");
        // Serial.print(leds[dot][ax]); 
      // }
      // Serial.print(", ");
      // delay(30);
    }
    // Serial.println("");

    leds[NUM_LEDS-1] = newdot;

    FastLED.show();
    delay(5); // it appears _no delay_ overloads something and causes the control to reboot
  }
}