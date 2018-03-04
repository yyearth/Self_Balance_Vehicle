//BalanceCar_V2.0.ino  
//author: yyearth  mail: yyearth1@icloud.com
/*
2016.12.7  22:50--Add mpu6050, Encoder, Motor support code  
                  Write balance PD function calBalencePwm
                  Set Kp,Kd roughly
    .12.13 15:51--Add prime speed control PI function
    .12.16 17:00--Modify the PD funnction calBalencePwm, 
                  saperate control line, piecewise linear. Get better.
           15:26--Undo calBalencePwm...
    .12.16 17.40--Expand control circle period(add variable ctrlCount, 5ms*Max(ctrlCount)) 
                  to get more precise measurment of rotation of wheels.
                  Add getSpeedL, getSpeedR function.
    .12.20 12:54--Turn the Kp of PI function calVelicityPwm, control circle 5ms.
    .12.21 23:14--Add function calTurnPwm and turn the param Kd. 
    .12.23 --:----Add Bluetooth remote control code.             
*/

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "AFMotor.h"

//#include <SoftwareSerial.h>
//SoftwareSerial DebugSerial(A2, A3); // RX, TX

//#define BLYNK_PRINT DebugSerial
//#include <BlynkSimpleStream.h>



//#define INTERRUPT_PIN   //mpu6050 interrupt pin 
#define GYROOFFSET1 -1
#define GYROOFFSET2 -1
#define UPRIGHT -3
//pins for Encoder
#define SPD_INT_R 2
#define SPD_PUL_R A0
#define SPD_INT_L 3
#define SPD_PUL_L A1

#define LEDPIN 13

MPU6050 mpu;
AF_DCMotor motor_L(4);
AF_DCMotor motor_R(3);

uint8_t ctrlCount;
int balancePwm,velocityPwm,turnPwm;
int motorPwmL,motorPwmR;

char cmd;
int speedL,speedR;
int cLR,cFB;

//char auth[] = "0c85abc3bbc74b25bce28060d84bfa17";

bool blinkState = false;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
int16_t gyro[3];        // [x, y, z]            gyro vector
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  
	Wire.begin();
  //DebugSerial.begin(9600);
	Serial.begin(9600);
    
    //pinMode(INTERRUPT_PIN, INPUT);
 // initialize device
	  Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();
 // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
 // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again
 // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
 // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        
        //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }

  	//pinMode(LEDPIN, OUTPUT);
    dmpDataReady();
    //initial encoder pins
	pinMode(SPD_PUL_R, INPUT);
	pinMode(SPD_PUL_L, INPUT);
	attachInterrupt(digitalPinToInterrupt(SPD_INT_L), speed_int_l, RISING);
	attachInterrupt(digitalPinToInterrupt(SPD_INT_R), speed_int_r, RISING);
 //Blynk.begin(Serial, auth);
  
}

//BLYNK_WRITE(V1) {
//   cLR = param[0].asInt();
//   cFB = param[1].asInt();
//}

void loop() {
 
 	if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt &&fifoCount < packetSize) {    
        //Blynk.run();
        
        

        // other program behavior stuff here
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
    }

    // reset interrupt flag and get INT_STATUS byte
    //mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        if (ctrlCount>=0){                      //5ms

		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		mpu.dmpGetGyro(gyro, fifoBuffer);
        
        //Serial.print(ypr[1] * 180/M_PI);Serial.print("\t");Serial.println(gyro[1]);
		ypr[1] = ypr[1] * 180/M_PI;
        //Blynk.syncVirtual(V1);
        //Blynk.run();

        while (Serial.available() > 0) {
          cmd =  Serial.read();
          Serial.println(cmd);
    
          
          
          } 

       switch (cmd) {
           case '4':
             cFB = -9;
             break;
           case '3':
             cFB = -7;
             break;
           case '2':
             cFB = -5;
             //cLR = 0;
             break;  
           case '1':          //F
             cFB = -3;
             //cLR = 0;
             //cmd = 0;
             break;
          
           case 'B':           //B
             cFB = 2;
             //cLR = 0;
             //cmd = 0;
             break;
          
           
           case 'L':           //L
             //cFB = 0;
             cLR = 120;
            // cmd = 0;
             break;
           case 'R':           //R
             //cFB = 0;
             cLR = -100;
             //cmd = 0;
             break;
           case 'o':
      
             cLR = cFB = 0;
             break;
           default:
             //cmd = 0;
             cLR = cFB = 0;
            
       }

		balancePwm = calBalancePwm(ypr[1],gyro[1]);
		velocityPwm = calVelocityPwm(getSpeedL(),getSpeedR(),cFB);
        turnPwm = calTurnPwm(gyro[2],cLR);


        
        //Serial.println(gyro[2]); 
        //Serial.print(velocityPwm);Serial.print("\t");
        //Serial.print(getSpeedL());Serial.print("\t");Serial.println(getSpeedR());  
        //Serial.print(balancePwm);Serial.print("\t");
        //
		
        motorPwmL = balancePwm + velocityPwm + turnPwm;
        motorPwmR = balancePwm + velocityPwm - turnPwm;
        //Serial.print(motorPwmL);Serial.print("\t");Serial.println(motorPwmR);
        //motorPwmL = 254;
        //motorPwmR = 254;
        if(motorPwmL > 254)  motorPwmL = 254;
        else if(motorPwmL < -254) motorPwmL = -254;
        if(motorPwmR > 254)  motorPwmR = 254;
        else if(motorPwmR < -254) motorPwmR = -254;

        if(ypr[1]>40.0||ypr[1]<-40.0) motorPwmR = motorPwmL = 0;
        
        if(motorPwmL < 0){
            motor_L.run(BACKWARD);
            motor_L.setSpeed(-motorPwmL);  
         }else{
            motor_L.run(FORWARD);
            motor_L.setSpeed(motorPwmL); 
         }
          if(motorPwmR < 0){
             motor_R.run(BACKWARD);
             motor_R.setSpeed(-motorPwmR);  
          }else{
             motor_R.run(FORWARD);
             motor_R.setSpeed(motorPwmR);
          }

        
        
        
         
		// Serial.print("ypr\t");
		// Serial.print(ypr[0] * 180/M_PI);
		// Serial.print("\t");
		//Serial.print(ypr[1] * 180/M_PI);
		//Serial.print("\t");
		//Serial.println(ypr[2] * 180/M_PI);
		//Serial.print("   \t");
		//Serial.print(ypr[0]);
		//Serial.print("\t");
        //Serial.print(ypr[1]);
        //Serial.print("\t");
		//Serial.println(ypr[2]);
		// Serial.print("gyro\t");
		// Serial.print(gyro[0]);
		// Serial.print("\t");
	    // Serial.println(gyro[1]);
		// Serial.print("\t");
		// Serial.println(gyro[2]);
       
        // blink LED to indicate activity
        ctrlCount = 0;
        //blinkState = !blinkState;
        //digitalWrite(LEDPIN, blinkState);

    }else{
        //Blynk.run();
        ctrlCount++;
    }
    }

}


int calBalancePwm(float Angle,float Gyro){  
    float Kp1=7.72,Kd1=-2.86;//12.65*0.6=7.59   3.4*0.6=2.04
    int pwm;
    
    if(Angle>35.0||Angle<-35.0)  return 0; 
    
    pwm=Kp1*(Angle - UPRIGHT)+(Gyro - GYROOFFSET1)*Kd1; 
         
    return pwm;
}

int calVelocityPwm(int encoderL,int encoderR,float ctrl){
    static float Kp = -17;
    static float Ki = -0.001;//Kp/2000;
    static int pwm;
    static float encoderI,encoder,encoderLast;

    encoderLast = (encoderL + encoderR) - ctrl;
    encoder *= 0.7;
    encoder += encoderLast*0.3;

    encoderI += encoder;
    if (encoderI>5000) encoderI = 5000;
    if (encoderI<-5000) encoderI = -5000;
    
    pwm = Kp*encoder + Ki*encoderI;

    //Blynk.virtualWrite(V2, encoderL + encoderR);
    
    return pwm;
}

int calTurnPwm(float GyroZ,int ctrl){
    float Kd = -2;
    int pwm;

    pwm = Kd*(GyroZ - GYROOFFSET2) + ctrl;

    return pwm;


}
int getSpeedL(){
    int s;
    s = speedL;
    speedL = 0;
    //Serial.print(s);Serial.print("\t");  
    return s;
}

int getSpeedR(){
    int s;
    s = speedR;
    speedR = 0;
    //Serial.println(s);
    return s;
}

void speed_int_l()
{
  //Serial.print(speedL);Serial.print("\t");Serial.println(speedR);
  if (digitalRead(SPD_PUL_L))
    speedL-=1;
  else
    speedL+=1;
}

void speed_int_r()
{
  //Serial.print(speed_l);Serial.print("\t");Serial.println(speed_r);  
  //Serial.print(speedL);Serial.print("\t");Serial.println(speedR);
  if (digitalRead(SPD_PUL_R))
    speedR-=1;
  else
    speedR+=1;
}
