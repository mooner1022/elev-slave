#include <AccelStepper.h>
#include <MultiStepper.h>
#include <VL53L0X.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SimpleTimer.h>

int stepPin=3;
int dirPin=2;
int sensorResetPin=5;
int moveSpeed=500;
int tolerance = 3;
//1
//int floorDistance[] = {45,180,335,455};
//2
//int floorDistance[] = {35,180,330,430};
//3
int floorDistance[] = {50,200,360,465};

//100mm/s
//0.1mm/ms
//10ms/mm
SimpleTimer timer;
VL53L0X sensor;
boolean isMoving = false;
boolean isMovingInaccurate=false;
boolean isDecreacingSpeed=false;
boolean inaccurateMovingDirection=false;
int decreaceSpeed = 500;
int targetFloor = 1;
AccelStepper stepper(1, stepPin, dirPin);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(4, 4, NEO_GRB + NEO_KHZ800);
uint32_t black = strip.Color(0, 0, 0, 0);

void setup() {
  Serial.begin(9600);
  resetSensor();
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
  sensor.startContinuous(2);
  strip.begin();
  strip.show();
}

void loop() {
  timer.run();
  if (isMoving) {
    if (isMovingInaccurate) {
      stepper.setSpeed(inaccurateMovingDirection?moveSpeed:-moveSpeed);
      stepper.runSpeed();
    } else if (isDecreacingSpeed) {
      if (decreaceSpeed>10) {
        stepper.setSpeed(inaccurateMovingDirection?decreaceSpeed/2:-decreaceSpeed/2);
        stepper.runSpeed();
        decreaceSpeed=decreaceSpeed-1;
        delayMicroseconds(2500);
      } else {
        isDecreacingSpeed=false;
        decreaceSpeed=moveSpeed;
      }
    } else {
      int distance = sensor.readRangeContinuousMillimeters();
      //int distance = 100;
      if (distance>=(floorDistance[targetFloor-1]+tolerance)) {
        stepper.setSpeed(-moveSpeed);
        stepper.runSpeed();
      } else if (distance<=(floorDistance[targetFloor-1]-tolerance)) {
        stepper.setSpeed(moveSpeed);
        stepper.runSpeed();
      } else {
        setLed(targetFloor,2);
        isMoving = false;
      }
    }
  } else {
    if(Serial.available()){
      String inStr=Serial.readStringUntil('\n');
      inStr.trim();
      if(!isMoving) {
        if (inStr.equals("1")) {
          moveToFloor(1);
        } else if (inStr.equals("2")) {
          moveToFloor(2);
        } else if (inStr.equals("3")) {
          moveToFloor(3);
        } else if (inStr.equals("4")) {
          moveToFloor(4);
        } else if (inStr.equals("S")) {
          sendState();
        }
      }
    }
  }
}

void setLed(int floorNum, int colorType) {
  strip.fill(black, 0, 9);
  int ledNum=floorNum-1;
  int R=0,G=0,B=0;

  if(colorType==0){
    R=0;
    G=0;
    B=0;
  } else if(colorType==1) {
    R=128;
    G=0;
    B=0;
  } else if(colorType==2) {
    R=0;
    G=128;
    B=0;
  }
  strip.setPixelColor(ledNum, R, G, B);
  strip.show();
}

void moveToFloor(int floor) {
  if (!isMoving) {
    targetFloor = floor;
    isMoving=true;
    isMovingInaccurate=true;
    int distance = sensor.readRangeContinuousMillimeters();
    inaccurateMovingDirection = distance<floorDistance[targetFloor-1];
    int inaccurateMillis = (abs(distance-floorDistance[targetFloor-1])-30)*10;
    Serial.println(distance-floorDistance[targetFloor-1]);
    Serial.println(inaccurateMillis);
    if (inaccurateMillis<0) {
      isMovingInaccurate=false;
      isDecreacingSpeed=false;
      isMoving=false;
    } else if (inaccurateMillis<500) {
      setLed(floor, 1);
      timer.setTimeout(inaccurateMillis, [](){
        isMovingInaccurate=false;
        isDecreacingSpeed=false;
      });
    } else {
      setLed(floor, 1);
      timer.setTimeout(inaccurateMillis, [](){
        isMovingInaccurate=false;
        isDecreacingSpeed=true;
      });
    }
  }
}

void resetSensor() {
  pinMode(sensorResetPin, OUTPUT);
  digitalWrite(sensorResetPin, LOW);

  delay(500);
  Wire.begin();

  pinMode(sensorResetPin, INPUT);
  delay(150);
  sensor.init(true);
  delay(100);
  sensor.setAddress((uint8_t)22);
  sensor.setTimeout(500);
}

void sendState() {
  int moveState;
  if (isMoving || isMovingInaccurate || isDecreacingSpeed) {
    moveState=inaccurateMovingDirection?1:-1;
  } else {
    moveState=0;
  }
  Serial.print(targetFloor);
  Serial.print(",");
  Serial.print(moveState);
  Serial.println();
}//층, 이동 방향, 상태
/*
void calculateSpeed() {
  int currentDis = sensor.readRangeSingleMillimeters();
  diff = preDis<currentDis?currentDis-preDis:preDis-currentDis;

  Serial.print("calculated speed: ");
  Serial.print(diff);
  Serial.println("mm/s");

  sensor.startContinuous(2);
  isCalcFinished=true;
}
*/
