/* 
   Easy AED
   Engineering Capstone PID Test
   Ayaan Landon Caiman
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <TFLI2C.h>      // TFLuna-I2C Library v.0.2.0
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

TFLI2C tflI2C;

int16_t  tfDist;    // distance in centimeters
int16_t  tfAddr = TFL_DEF_ADR;  // use this default I2C address or
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_BNO055 bno = Adafruit_BNO055(55);
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);
uint8_t motor1 = 0;
uint8_t motor2 = 1;
uint8_t motor3 = 2;
uint8_t motor4 = 3;
double motor1Power = 0;
double motor2Power = 0;
double motor3Power = 0;
double motor4Power = 0;
void writeMotor(uint8_t motorNum, double motorVal){
    //normalize motor inputs
    pwm.setPWM(motorNum, 0, map(motorVal*1023,0,1023,204,409));
    //Serial.println(motorVal*1023);
}
double totalT = 0;
int tickCounter = 0;
double prevPitchError, prevYawError, prevRollError, prevThrottleError;
double integralPitchError=0, integralYawError=0, integralRollError=0, integralThrottleError=0;
long tStamp = 0;
//output should be {front Left, front Right, back Left, back Right}
/*
drone motor config:
u4(cc) u1(c)
u3(c) u2(cc)
| u1 |    |1 −1 1 −1| |uthrottle|
| u2 |    |1 −1 −1 1| |uroll    |
| u3 | =  |1 1 −1 −1| |upitch   |
| u4 |    |1 1 1  1 | |uyaw     |
uthrottle should be calculated with a height PID controller
uroll, upitch, uyaw should be calculated with an angular PID controller
*/
//same config as above matrix
//THROTTLE ROLL PITCH YAW
double PIDCONSTANTS[4][3] = {
  {0.2, 0, 0},//0.28
  {0.005, 0, 0},//0.005
  {0.005, 0, 0},//0.005
  {0.005, 0, 0}//0
};
double powerOffset = 0.56;//0.56
double initPowerOffset = powerOffset;
bool firstTime = true;
double target[4] = {0.25, 0.0, 0.0, 0.0};
double initHeight = target[0];

int startTime = 0;
double yawStart, rollStart, pitchStart;
bool done = false;

void emergencyRampDown(){
  //0.5 second ramp down
  double motor1Inc = motor1Power / 100.0;
  double motor2Inc = motor2Power / 100.0;
  double motor3Inc = motor3Power / 100.0;
  double motor4Inc = motor4Power / 100.0;
  for(int i = 0; i < 100; i++){
    motor1Power -= motor1Inc;
    motor2Power -= motor2Inc;
    motor3Power -= motor3Inc;
    motor4Power -= motor4Inc;
    writeMotor(motor1, motor1Power);
    writeMotor(motor2, motor2Power);
    writeMotor(motor3, motor3Power);
    writeMotor(motor4, motor4Power);
    delay(5);
  }
  //just to make sure:
  motor1Power = 0;
  motor2Power = 0;
  motor3Power = 0;
  motor4Power = 0;
  writeMotor(motor1, 0);
  writeMotor(motor2, 0);
  writeMotor(motor3, 0);
  writeMotor(motor4, 0);
}
const int signMatrix[4][4] = {
    {1, -1, 1, -1},
    {1, -1, -1, 1},
    {1, 1, -1, -1},
    {1, 1, 1, 1}
  };
void controlPID(double throttleError, double pitchError,double yawError,double rollError){
  double deltaT = (millis() - tStamp);//in milliseconds
  totalT += deltaT;
  tickCounter++;
  tStamp = millis();
  double uthrottle, uroll, upitch, uyaw;

  uthrottle = PIDCONSTANTS[0][0]*throttleError + PIDCONSTANTS[0][1]*(throttleError - prevThrottleError)/deltaT + PIDCONSTANTS[0][2]*(integralThrottleError+(throttleError*deltaT));
  uroll = PIDCONSTANTS[1][0]*rollError + PIDCONSTANTS[1][1]*(rollError - prevRollError)/deltaT + PIDCONSTANTS[1][2]*(integralRollError+(rollError*deltaT));
  upitch = PIDCONSTANTS[2][0]*pitchError + PIDCONSTANTS[2][1]*(pitchError - prevPitchError)/deltaT + PIDCONSTANTS[2][2]*(integralPitchError+(pitchError*deltaT));
  uyaw = PIDCONSTANTS[3][0]*yawError + PIDCONSTANTS[3][1]*(yawError - prevYawError)/deltaT + PIDCONSTANTS[3][2]*(integralYawError+(yawError*deltaT));
  integralPitchError += pitchError*deltaT;
  integralThrottleError += throttleError*deltaT;
  integralYawError += yawError*deltaT;
  integralRollError += rollError*deltaT;
  double inputs[4] = {uthrottle,uroll,upitch,uyaw};
  double outputs[4] = {0.02,0,0,0.02};//0.02, 0, 0, 0.02
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      outputs[i] += signMatrix[i][j]*inputs[j];
    }
    outputs[i] += powerOffset;
    outputs[i] = max(min(outputs[i], 1), 0);
  }
  if((abs(outputs[0] - motor1Power) > 0.15 || abs(outputs[1] - motor2Power) > 0.15 || abs(outputs[2] - motor3Power) > 0.15 || abs(outputs[3] - motor4Power) > 0.15) && !firstTime){
    done = true;
    Serial.println("Emergency Stopped, Motor Change Too Fast");
    Serial.print("Motor 1 Prev, Curr: ");
    Serial.print(motor1Power);
    Serial.print(", ");
    Serial.println(outputs[0]);
    Serial.print("Motor 2 Prev, Curr: ");
    Serial.print(motor2Power);
    Serial.print(", ");
    Serial.println(outputs[1]);
    Serial.print("Motor 3 Prev, Curr: ");
    Serial.print(motor3Power);
    Serial.print(", ");
    Serial.println(outputs[2]);
    Serial.print("Motor 4 Prev, Curr: ");
    Serial.print(motor4Power);
    Serial.print(", ");
    Serial.println(outputs[3]);
    Serial.print("Yaw Error: ");
    Serial.println(yawError);
    Serial.print("Roll Error: ");
    Serial.println(rollError);
    Serial.print("Pitch Error: ");
    Serial.println(pitchError);
    emergencyRampDown();
    return;
    }
    motor1Power = outputs[0];
    motor2Power = outputs[1];
    motor3Power = outputs[2];
    motor4Power = outputs[3];
    writeMotor(motor1, motor1Power);
    writeMotor(motor2, motor2Power);
    writeMotor(motor3, motor3Power);
    writeMotor(motor4, motor4Power);
    // Serial.println("throttleError: "+String(throttleError)+", rollError: "+String(rollError)+", pitchError: "+String(pitchError)+", yawError: "+String(yawError));
    // //Serial.println("uthrottle: "+String(uthrottle)+", uroll: "+String(uroll)+", upitch: "+String(upitch)+", uyaw: "+String(uyaw));
    // Serial.println("motor 1: "+String(outputs[0])+", motor 2: "+String(outputs[1])+", motor 3: "+String(outputs[2])+", motor 4: "+String(outputs[3]));
    firstTime = false;
}

//this is the setup function for the arduino
void setup() {
  Serial.begin(9600);
  Serial.println("PID test!");
  Serial.println("MAKE SURE YOUR SERIAL IS IN NEW LINE!!!");
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // 0-1600   run at 50hz pwm

  // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
  // some i2c devices dont like this so much so if you're sharing the bus, watch
  // out for this!
  Wire.setClock(400000);
  for(int i=0; i < 16; i++){
     pwm.setPWM(i, 0, map(0,0,1023,205,409));
  }
    /* Initialise the sensor */
    if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  Serial.println("bno begun");
  delay(1000);
  bno.setExtCrystalUse(true);
  delay(1000);

  Serial.println("Done Setup. Once motors are plugged in, enter start in serial for rampup");

  while(Serial.available() == 0){}
  String cmd = Serial.readStringUntil("\n").c_str();
  Serial.print("Entered:");
  Serial.print(cmd);
  if(cmd != "start\n"){
    Serial.println("You did not enter start. Ending the code and you have to restart.");
    done = true;
    return;
  }

  Serial.println("Ramping Up");

  sensors_event_t event;
  bno.getEvent(&event);
  yawStart = event.orientation.x;
  rollStart = event.orientation.y;
  pitchStart = event.orientation.z;
  double powerInc = powerOffset / 1000.0;
  for(int i = 0; i < 1000; i++){
    writeMotor(motor1, powerInc * i);
    writeMotor(motor2, powerInc * i);
    writeMotor(motor3, powerInc * i);
    writeMotor(motor4, powerInc * i);
    delay(1);
  }
  motor1Power = powerOffset;
  motor2Power = powerOffset;
  motor3Power = powerOffset;
  motor4Power = powerOffset;
  startTime = millis();
  Serial.println("Rampup Done, Void Setup Done");
  Serial.println("PID Control Starting");
}

bool landed = false;
bool timeUp = false;
int rampMarker = 0;
int endTime = 6000;

double height = 0.0;
void loop() {

  //manual commands
  if(Serial.available() > 0){
    String cmd = Serial.readStringUntil("\n").c_str();
    Serial.println("Emergency Manual Stop");
    emergencyRampDown();
    done = true;
  }

  //code not running
  if(done){
    Serial.println("Code ended");
    Serial.print("Milliseconds Per Tick: ");
    Serial.println(totalT/tickCounter);
    delay(2000);
    return;
  }
  
  //update time
  int t = millis();

  //time check then ramps down (after intervals at least 100 milliseconds lower height, then power offset)
  if(t - startTime > endTime && t - rampMarker > 100){
    if(!timeUp){
    Serial.println("Time Up, Lowering Height");
    timeUp = true;
    }
    rampMarker = t;
    target[0] = max(target[0] - initHeight / 50.0, 0);

    //on the ground start ramping down offset
    if(target[0] == 0){
      if(!landed){
        Serial.println("Landed! Now Ramping Down Power Offset");
        landed = true;
      }
      //decreasing power offset
      powerOffset = max(0, powerOffset - initPowerOffset / 30.0);
      if(powerOffset == 0){
        Serial.println("Powered Off");
        //make sure it's 0
        writeMotor(motor1, 0);
        writeMotor(motor2, 0);
        writeMotor(motor3, 0);
        writeMotor(motor4, 0);
        done = true;
        return;
      }
    }
  }

  //read sensors and calculate errors
  if( tflI2C.getData( tfDist, tfAddr)) // If read okay...
  {
    height = tfDist / 100.0;
  }

  //angle sensor read
  sensors_event_t event;
  bno.getEvent(&event);
  double angleYaw = event.orientation.x - yawStart;
  double angleRoll = event.orientation.y - rollStart;
  double anglePitch = event.orientation.z - pitchStart;
  double pitchError = target[1] - anglePitch;
  double rollError = target[2] - angleRoll;
  double yawError = target[3] - angleYaw;

  //error calculations
  double throttleError = 0.0;
  throttleError = target[0] - height * cos(abs(anglePitch) * PI / 180.0) * cos(abs(angleRoll) * PI / 180.0);
  pitchError = -pitchError;
  rollError = -rollError;

  //optimize direction of error
  pitchError += (pitchError > 180)*-360.0 + (pitchError < -180)*360.0;
  yawError += (yawError > 180)*-360.0 + (yawError <  -180)*360.0;
  rollError += (rollError > 180)*-360.0 + (rollError < -180)*360.0;

  //angle error limit exceeded
  if(abs(pitchError) > 20 || abs(yawError) > 20 || abs(rollError) > 20){
    Serial.println("Emergency Stop: Angle Limit Exceeded");
    Serial.print("Yaw Error: ");
    Serial.println(yawError);
    Serial.print("Roll Error: ");
    Serial.println(rollError);
    Serial.print("Pitch Error: ");
    Serial.println(pitchError);
    emergencyRampDown();
    done = true;
    return;
  }

  //main control function
  controlPID(throttleError, pitchError, yawError, rollError);
}