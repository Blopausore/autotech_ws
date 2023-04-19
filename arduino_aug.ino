#include <Servo.h>

#define AngleOrder 97
#define SpeedOrder 98
#define changeMoovOrder 99
#define changeRotaOrder 100

#define PINangle  10
#define PINspeed  9


uint8_t order = 0;
uint8_t arg = 0;


const int angleZero = 45;
const int angleScale = 45;

const int speedScale = 5;


int dirSpeed = 1; // 1: forward | -1 : backward
int dirAngle = 1; // 1: to right| -1 : to left

Servo angle;
Servo esc;

void changeSpeed() {
  float nextVal = (dirSpeed*arg*speedScale/255);
  int newVAl = (int) nextVal;
  esc.write(100 + newVAl); //shame on my code
}

void changeRot() {
  float nextVal = (dirAngle*arg*angleScale/255);
  // nextVal : [-angleScale, angleScale]
  Serial.println(nextVal);
  int newVAl = (int) nextVal;
  // write : [angleZero - angleScale, angleZero + angleScale]
  angle.write(angleZero + newVAl);//shame on my code
}

void initMotorServo() {
  //esc.write(180);
  //delay(1000); //wait for the motor 
  //esc.write(0);
  //delay(1000);
  esc.write(93);
  angle.write(90);
}

void initArdui() {
  Serial.begin(115200);
  pinMode(PINangle, OUTPUT);
  pinMode(PINspeed, OUTPUT);

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  digitalWrite(2, LOW); // GND
  digitalWrite(3, HIGH); // 5V
  
  angle.attach(PINangle, 0, 2000);
  esc.attach(PINspeed, 0, 2000);
  initMotorServo();
  
  Serial.println("init done");

}

void changePWMAngle(){
  changeRot();
}

void changePWMSpeed() {
  changeSpeed();
}

void orderActualise(uint8_t* order, uint8_t* arg) {
  (* order) = Serial.read();
  (* arg) = Serial.read();
}

void orderManager(){
  // just a case 
  if (order == AngleOrder) {
    changePWMAngle();
    Serial.println("angle");
    Serial.println(arg);
  }
  else if (order == SpeedOrder) {
    changePWMSpeed();
    Serial.println("speed");
  }
  
  // arg == 0 => dir = -1 | arg == 1 => dir = 1
  else if (order == changeMoovOrder){
    dirSpeed = -1 + 2 * arg;  
  }
  else if (order == changeRotaOrder) {
    dirAngle = -1 + 2 * arg;
  }
}


//------------------------------------------
void setup() {
 initArdui();
}

void loop() {
  if (Serial.available() >= 2) {
    orderActualise(&order, &arg);
    orderManager();
  }
  
}