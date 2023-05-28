#include <Servo.h>

#define AngleOrder 97
#define SpeedOrder 98
#define changeMoovOrder 99
#define changeRotaOrder 100

#define PINangle  11
#define PINspeed  10


uint8_t order = 0;
uint8_t arg = 0;


Servo angle;
Servo esc;

void writeSpeed() {
  int newVal = (int) arg;
  Serial.println(newVal);
  esc.write(newVal - 128); 
}

void writeRot() {
  int newVal = (int) arg;
  Serial.println(newVal);
  angle.write(newVal);
}

void initMotorServo() {
  esc.write(180);
  esc.write(0);
  delay(4000);
  esc.write(120);
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

  angle.write(45);

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
/*
  // arg == 0 => dir = -1 | arg == 1 => dir = 1
  else if (order == changeMoovOrder){
    dirSpeed = -1 + 2 * arg;  
  }
  else if (order == changeRotaOrder) {
    dirAngle = -1 + 2 * arg;
  }*/
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
