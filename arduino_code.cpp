#include <Servo.h>

#define AngleOrder 97
#define SpeedOrder 98
#define changeMoovOrder 99
#define changeRotaOrder 100

#define PINangle  10
#define PINspeed  9



uint8_t order = 0;
uint8_t arg = 0;

int8_t isForward = 1; //1 => forward, -1 => backward
int8_t isRight = 1; //1=> right, -1 => left
int8_t* ptrIsFor = &isForward;
int8_t* ptrIsSpee = &isRight;


Servo angle;
Servo esc;

void changeSpeed() {
  float nextVal = (isForward*10*arg/255);
  int newVAl = (int) nextVal;
  esc.write(100 + newVAl); //shame on my code
}

void changeRot() {
  float nextVal = (isForward*90*arg/255);
  int newVAl = (int) nextVal;
  angle.write(90 + newVAl);//shame on my code
}

void changeMoov() {
  if (arg == 0) {
    (* ptrIsSpee) = -1;
  }
  else {
    (* ptrIsSpee) = 1;
  }
}

void changeRota() {
  if (arg == 0) {
    (* ptrIsFor) = -1;
  }
  else {
    (* ptrIsFor) = 1;
  }  
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
  digitalWrite(2, LOW);
  digitalWrite(3, HIGH);
  
  angle.attach(PINangle,1000,2000);
  esc.attach(PINspeed,0,2000);
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
  }
  else if (order == SpeedOrder) {
    changePWMSpeed();
    Serial.println("speed");
  }
  else if (order == changeMoovOrder) {
    changeMoov();
  }
  else if (order == changeRotaOrder) {
    changeRota();
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