/* V6.0 Actuator ECU throttle
 * NEW VDO actuator without poti
This sketach can be used to control a cruise control servo actuator over CAN. 
*/

//________________import can libary https://github.com/sandeepmistry/arduino-CAN
#include <CAN.h>


//________________this values needs to be define for each car
int PERM_ERROR = 4; // will allow a diffrence between targetPressure and currentPressure
int minPot = 1; //measured at actuators lowest position
int maxPot = 1000; //measured at actuators highest position
int maxPressure = 430; // the max pressure your actuator is able to aply
int minPressure = 75; //the pressure in stand still
float maxACC_CMD = 500; //the max Value which comes from OP
float minACC_CMD = 0; //the min Value which comes from OP



//________________define_pins
int cancel_pin = 3;
int pressurePin = A2;
int brakelightPin = A5;

int M_DIR = 8; // LOW is Left / HIGH is Right
int M_PWM = 9; // 255 is run / LOW is stopp
int S_DIR = 7; // LOW is Left / HIGH is Right
int S_PWM = 6; // 255 is run / LOW is stopp

//________________values
int targetPressure = 0;
int currentPressure;

float ACC_CMD_PERCENT = 0;
float ACC_CMD = 0;
float ACC_CMD1 = 0;
boolean cancel = false;


void setup() {
    
//________________begin Monitor - only use it for debugging
 Serial.begin(115200);

//________________begin CAN
CAN.begin(500E3);

//________________set up pin modes
pinMode(cancel_pin, INPUT_PULLUP);
pinMode(pressurePin, INPUT);
pinMode(M_DIR, OUTPUT);
pinMode(M_PWM, OUTPUT);
pinMode(S_DIR, OUTPUT);
pinMode(S_PWM, OUTPUT);
pinMode(brakelightPin, OUTPUT);
digitalWrite(S_DIR, HIGH);

}

void loop() {
//________________read cancel pin
cancel = true; //(digitalRead(cancel_pin)); 

//________________read pressure sensor
currentPressure = (analogRead(pressurePin));

//________________light up brake lights
if (currentPressure >= 75)
  {
   analogWrite(brakelightPin, 255);
  }
else 
  {
   analogWrite(brakelightPin, 0);
  }
Serial.println(currentPressure);

//________________read ACC_CMD from CANbus
 CAN.parsePacket();

 if (CAN.packetId() == 0x343)
      {
      uint8_t dat[8];
      for (int ii = 0; ii <= 7; ii++) {
        dat[ii]  = (char) CAN.read();
        }
        ACC_CMD = ((dat[0] << 8 | dat[1] << 0) * -1); 
        }    

//________________calculating ACC_CMD into ACC_CMD_PERCENT
if (ACC_CMD >= minACC_CMD) {
    ACC_CMD1 = ACC_CMD;
    }
else {
    ACC_CMD1 = minACC_CMD;
    }
       
ACC_CMD_PERCENT = ((100/(maxACC_CMD - minACC_CMD)) * (ACC_CMD1 - minACC_CMD));

//________________calculating tagetpressure from ACC_CMD_PERCENT
float targetPressure = (((ACC_CMD_PERCENT / 100) * (maxPressure - minPressure)) + minPressure); // conversion from ACC_CMD_PERCENT % into targetpressure
  
//________________do nothing if ACC_CMD is 0
if (ACC_CMD_PERCENT == 0){
   analogWrite(S_PWM, 0);  //open solenoid
   analogWrite(M_PWM, 0);  //stop Motor
}

//________________do nothing while cancel, but read if it's still cancel
while (!cancel) {
   analogWrite(S_PWM, 0);  //open solenoid
   analogWrite(M_PWM, 0);  //stop Motor
   cancel = (digitalRead(cancel_pin));
   }

//________________close solenoid
analogWrite(S_PWM, 255);
   
//________________press or release the pedal to match targetPressure & respect endpoints
if (abs(currentPressure - targetPressure) >= PERM_ERROR)
  {
    if (currentPressure < targetPressure)
        { 
        analogWrite(M_PWM, 255);  //run Motor
        digitalWrite(M_DIR, HIGH); //motor driection left
        }    
    else if (currentPressure > targetPressure)
        {       
        analogWrite(M_PWM, 255);   //run Motor
        digitalWrite(M_DIR, LOW); //motor driection right
        }
  }  
  
//________________if we match target position, just stay here
else {
     analogWrite(M_PWM, 0);   //stop Motor
     }  

//________________print stuff if you want to DEBUG

//Serial.print("ACC_CMD_");
//Serial.print(ACC_CMD);
//Serial.print("_____");
//Serial.print(ACC_CMD_PERCENT);
//Serial.print("_%");
//Serial.print("_____");
//Serial.print("target_");
//Serial.print(targetPressure);
//Serial.print("_____");
//Serial.print("Pressure");
//Serial.println(currentPressure);
//Serial.println("");

}
