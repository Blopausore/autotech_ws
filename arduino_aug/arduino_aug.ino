#include <Servo.h>

#define AngleOrder 97
#define SpeedOrder 98
#define changeMoovOrder 99
#define changeRotaOrder 100

#define PINangle  11
#define PINspeed  9


uint8_t order = 0;
uint8_t arg = 0;

int dirSpeed = 1; // -1: forward | 1 : backward
int dirAngle = 1; // 1: to right| -1 : to left


// For speed arg : [-255, 255] 
// For angle arg : [-90, 90]

Servo angle;
Servo esc;

void writeSpeed() {
  float nextVal = (dirSpeed*arg);
  int newVal = (int) nextVal;
  Serial.println(arg);
  Serial.println(dirSpeed);
  esc.write(newVal); 
}

void writeRot() {
  float nextVal = (dirAngle*arg + 90.);
  int newVal = (int) nextVal;
  Serial.println(arg);
  Serial.println(dirAngle);
  angle.write(newVal);
}

void initMotorServo() {
  esc.write(180);
  delay(1000); //wait for the motor 
  esc.write(0);
  delay(1000);
}

void initArdui() {
  Serial.begin(115200);
  pinMode(PINangle, OUTPUT);
  pinMode(PINspeed, OUTPUT);

  pinMode(3, OUTPUT);
  digitalWrite(3, HIGH); // 5V
  
  angle.attach(PINangle, 0, 2000);
  esc.attach(PINspeed, 0, 2000);
  delay(1000);
  initMotorServo();
  
  Serial.println("init done");
}


void orderActualise(uint8_t* order, uint8_t* arg) {
  (* order) = Serial.read();
  (* arg) = Serial.read();
}

void orderManager(){
  // just a case 
  if (order == AngleOrder) {
    Serial.println("angle");
    writeRot();
  }
  else if (order == SpeedOrder) {
    Serial.println("speed");
    writeSpeed();
    
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
