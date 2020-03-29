// V7.0_throttle
// Detect user override


//________________import can libary https://github.com/sandeepmistry/arduino-CAN
#include <CAN.h>

//________________this values needs to be define for each car
int PERM_ERROR = 2; // will allow a diffrence between targetPressure and currentPressure
int USER_ERROR = 2; // to detect user input
int minPot = 915; //measured at actuators lowest position
int maxPot = 1002; //measured at actuators highest position
float maxACC_CMD = 1430; //the max Value which comes from OP
float minACC_CMD = 477; //the min Value which comes from OP

//________________define_pins
int interruptPin = 3;
int potPin = A1;
int M_pin1 = 9;
int M_pin2 = 7;
int S_pin1 = 6;

//________________values

int targetPosition = 0;
uint8_t spd = 0x0;
int potiPosition = 0;
float ACC_CMD_PERCENT = 0;
float ACC_CMD = 0;
float ACC_CMD1 = 0;
boolean break_pressed = false;

void setup() {
//________________beginn Serial Monitor
Serial.begin(115200);

//________________begin CAN TODO: test if we need this
CAN.begin(500E3);

//________________set up pin modes
pinMode(interruptPin, INPUT);
pinMode(potPin, INPUT);    
    
pinMode(M_pin1, OUTPUT);
pinMode(M_pin2, OUTPUT);
pinMode(S_pin1, OUTPUT);

}

void loop() {
//________________read poti Position
break_pressed = (digitalRead(interruptPin));
  

//________________read poti Position
int potiPosition = (analogRead(potPin)); //read from pinA01

//________________read CAN Packet
 CAN.parsePacket();

//________________read ACC_CMD
 if (CAN.packetId() == 0x200)
      {
      uint8_t dat[8];
      for (int ii = 0; ii <= 7; ii++) {
        dat[ii]  = (char) CAN.read();
        }
        ACC_CMD = (dat[0] << 8 | dat[1] << 0); 
       } 

//________________read Speed
 if (CAN.packetId() == 0xb4)
      {
      int spdin = CAN.parseInt();
      spd = (spdin / 100);
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
float targetPosition = (((ACC_CMD_PERCENT / 100) * (maxPot - minPot)) + minPot); // conversion from brake value in % into pressure

//________________close solenoid
if (break_pressed) {
   digitalWrite(S_pin1, HIGH);
   }
   
//________________open solenoid when brake pressed
if (!break_pressed) {
   digitalWrite(S_pin1, LOW);
   digitalWrite(M_pin1, LOW);  //stop Motor
   digitalWrite(M_pin2, LOW);  //stop Motor
   }

//________________press or release the Pedal to go to targetPressure
//________________respect end points by reading poti pin

if (abs(potiPosition - targetPosition) >= PERM_ERROR)
  {
    if ((potiPosition < targetPosition) && (potiPosition < maxPot))
        { 
        digitalWrite(M_pin1, HIGH); //press the pedal
        digitalWrite(M_pin2, LOW); //press the pedal
        }    
    else if ((potiPosition > targetPosition) && (potiPosition >= minPot))
        {       
         digitalWrite(M_pin1, LOW); //release the pedal
         digitalWrite(M_pin2, HIGH); //release the pedal
        }
  }  
else {
     digitalWrite(M_pin1, LOW);  //stop Motor
     digitalWrite(M_pin2, LOW);  //stop Motor
     }  
     
//________________detect user override Gas

if ((potiPosition > (targetPosition + USER_ERROR)) && (targetPosition > minPot)){
Serial.println("USER INPUT GAS");
}

//________________set new set_speed when brake pressed
if (!break_pressed) {
  
  boolean flag1 = true;
  
  //0x1d3 msg PCM_CRUISE_2
  uint8_t dat2[8];
  dat2[0] = 0x0;
  dat2[1] = (flag1 << 7) & 0x80 | 0x28;
  dat2[2] = spd;
  dat2[3] = 0x0;
  dat2[4] = 0x0;
  dat2[5] = 0x0;
  dat2[6] = 0x0;
  dat2[7] = can_cksum(dat2, 7, 0x1d3);
  CAN.beginPacket(0x1d3);
  for (int ii = 0; ii < 8; ii++) {
    CAN.write(dat2[ii]);
  }
  CAN.endPacket();
  }
     
//________________print values
Serial.print("ACC_CMD_");
Serial.print(ACC_CMD);
Serial.print("_____");
Serial.print(ACC_CMD_PERCENT);
Serial.print("_%");
Serial.print("_____");
Serial.print("target_");
Serial.print(targetPosition);
Serial.print("_____");
Serial.print("Position_");
Serial.print(potiPosition);
Serial.print("_____");
Serial.print(spd);
Serial.print("_km/h");
Serial.println("");

}  // end of loop


//________________generate toyota can cheksum
int can_cksum (uint8_t *dat, uint8_t len, uint16_t addr) {
  uint8_t checksum = 0;
  checksum = ((addr & 0xFF00) >> 8) + (addr & 0x00FF) + len + 1;
  //uint16_t temp_msg = msg;
  for (int ii = 0; ii < len; ii++) {
    checksum += (dat[ii]);
  }
  return checksum;
}
