/* Engineering Capstone PWM tester for motors using pwm channel expander
   Landon Ayaan Caiman
   11/23/2023
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel PWM test!");

  pwm.begin();
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // 0-1600   run at 50hz pwm

  // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
  // some i2c devices dont like this so much so if you're sharing the bus, watch
  // out for this!
  Wire.setClock(400000);
    for (int i=0; i < 16; i++){
     pwm.setPWM(i, 0, map(0,0,1023,204,409));
  }   
  delay(5000);
}

void loop() {
  for(int j = 0; j < 1024; j++){
     pwm.setPWM(0, 0, map(j,0,1023,204,409));
     //pwm.setPWM(1, 0, map(j,0,1023,204,409));  
  Serial.println(j);
  delay(6);
  }
    for(int j = 1023; j > 0; j--){
     pwm.setPWM(0, 0, map(j,0,1023,204,409));  
     //pwm.setPWM(1, 0, map(j,0,1023,204,409));
  Serial.println(j);
  delay(6);
  }  
  
}