#include<Arduino.h>

#define LEFT_MOTOR_FWD 11
#define LEFT_MOTOR_BWD 10
#define RIGHT_MOTOR_BWD 9
#define RIGHT_MOTOR_FWD 6


// --------------Sensors Section-------------------

#define CALIBRATE_SIZE 10
#define NUM_SENSORS 6
uint8_t Sensors[]= {A0 , A1 , A2 ,A3, A4 , A5};


long int avg_black[NUM_SENSORS];
long int avg_white[NUM_SENSORS];

int thresh[NUM_SENSORS];
int SensVal[NUM_SENSORS] = {0} ;

// --------------PID Section-------------------
int SensorCo[NUM_SENSORS] = {-9,-7,-1,1,7,9};

int rightBiasWeights[NUM_SENSORS] = {-4, -3, -2, 7, 11, 15};

float errorP = 0;
float errorI = 0;
float errorD = 0;
float lastError = 0;

float Kp = 40;
float Ki = 0.0;
float Kd = 2; 

int baseSpeed = 200;
int hexagonSpeed = 160;
// ----------------Flags---------------------
unsigned long startTime;

// ---------------Hexagon--------------------
bool hexagonFlag=false;

bool hexagonStarted = false;

const unsigned long HEXA_DELAYMS = 6500;
const unsigned long SPEED_DELAYMS = 20000;

// ---------------Brain--------------------
const unsigned long BRAIN_DELAYMS = 55000;
const unsigned long FINISH_DELAYMS = 180000;

bool brainFlag=false;
bool brainStarted = false;

// ----------------------Helper Functions-------------------------
// ##########################Sensors###########################

void calibrate(){

  digitalWrite(LED_BUILTIN , LOW);
  // #WHITE CALIBRATE
  for (int i = 0 ; i < NUM_SENSORS ; i++){
    int sum =  0 ;
    for (int j = 0 ; j < CALIBRATE_SIZE ; j++){
      sum += analogRead(Sensors[i]);
      delay(10);
    }
    avg_white[i]= sum / CALIBRATE_SIZE ;
  }

  digitalWrite(LED_BUILTIN , HIGH);
  delay(3000);
  digitalWrite(LED_BUILTIN , LOW);

  // #BLACK CALIBRATE

  for (int i = 0 ; i < NUM_SENSORS ; i++){
    int sum =  0 ;
    for (int j = 0 ; j < CALIBRATE_SIZE ; j++){
      sum += analogRead(Sensors[i]);
      delay(10);
    }
    avg_black[i]= sum / CALIBRATE_SIZE;
  }
  
  for (int i = 0 ; i < NUM_SENSORS ; i++){
    thresh[i] = (avg_black[i] + avg_white[i])/2;
    Serial.println(thresh[i]);
  }
}

void readsensors(){
for (int i = 0; i < NUM_SENSORS; i++) {
    int sum = 0;
    for (int j = 0; j < 3; j++) { // Average 3 readings for noise reduction
      sum += analogRead(Sensors[i]);
      delay(1);
    }
    int reading = sum / 3;
    SensVal[i] = (reading > thresh[i]) ? 1 : 0;
  }
}

void invertedReadsensors(){
for (int i = 0; i < NUM_SENSORS; i++) {
    int sum = 0;
    for (int j = 0; j < 3; j++) { // Average 3 readings for noise reduction
      sum += analogRead(Sensors[i]);
      delay(1);
    }
    int reading = sum / 3;
    SensVal[i] = (reading > thresh[i]) ? 0 : 1;
  }
}

void printSensVal(){
  readsensors();
  for(int i = 0 ; i<NUM_SENSORS;i++ ){

    if(SensVal[i] == 1 ){
      Serial.print("black");
    }else{
      Serial.print("white");
    }
    Serial.print("\t");
  }
  Serial.println();
}

// ##########################Motors########################

void driveMotors(int left, int right) {
  // Left motor
  if (left > 0) {
    analogWrite(LEFT_MOTOR_FWD, abs(left));
    analogWrite(LEFT_MOTOR_BWD, 0);
  } else {
    analogWrite(LEFT_MOTOR_FWD, 0);
    analogWrite(LEFT_MOTOR_BWD, abs(left));
  }
  // Right motor
  if (right > 0) {
    analogWrite(RIGHT_MOTOR_FWD, abs(right));
    analogWrite(RIGHT_MOTOR_BWD, 0);
  } else {
    analogWrite(RIGHT_MOTOR_FWD, 0);
    analogWrite(RIGHT_MOTOR_BWD, abs(right));
  }
}

// ##########################PID###########################

float calculatePID(int Sensor[]) {
  float weightedSum = 0;
  int sensorSum = 0;
  
  for (int j = 0; j < NUM_SENSORS; j++) {
    weightedSum += SensVal[j] * Sensor[j]; 
    sensorSum += SensVal[j];
  }
  
  if (sensorSum == 0) {
    return lastError * Kp;
  }
  if (sensorSum == NUM_SENSORS) {
    Serial.println("All sensors on black! Continuing with last correction...");
    return lastError * Kp;
  }
  errorP = weightedSum;  
  
  errorI += errorP;
  errorI = constrain(errorI, -1000, 1000); 

  errorD = errorP - lastError;
  lastError = errorP;

  return Kp * errorP + Ki * errorI + Kd * errorD;
}

bool allBlack(){
  for(int sensor : SensVal){
    if(not sensor) return false;
  }
  return true;
}

int countBlack(){
  int sum = 0;
  for(int sensor : SensVal){
    if(not sensor) sum++;
  }
  return sum;
}

// ----------------------Main Block--------------------------

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN , OUTPUT);
  calibrate();
  pinMode(LEFT_MOTOR_FWD, OUTPUT);
  pinMode(LEFT_MOTOR_BWD, OUTPUT);
  pinMode(RIGHT_MOTOR_FWD, OUTPUT);
  pinMode(RIGHT_MOTOR_BWD, OUTPUT);

  pinMode(2, INPUT_PULLUP);
  while (digitalRead(2)==1){}

  startTime = millis();
}

void loop(){
  readsensors();
  // ++++++++++++++++++++++++++| Hexagon Flag Detection |+++++++++++++++++++++++++++++++++++++

  if ( ((millis() - startTime > HEXA_DELAYMS))
  &&(!hexagonStarted)
  && (
    (SensVal[2] == 1 && SensVal[3] == 1 && SensVal[4] == 1 && SensVal[5] == 1)
    ||(SensVal[0] == 1 && SensVal[1] == 1 && SensVal[2] == 1) 
    || allBlack())){
      digitalWrite(LED_BUILTIN , HIGH);
      hexagonFlag = true;
      driveMotors(0,0);
      delay(200);
      driveMotors(0,160);
      for (int i = 0; i < 58; i++)
      {
        driveMotors(-160,-220);
        delay(10);
      }
      hexagonStarted=true;
  }
  if(millis()-startTime>SPEED_DELAYMS && hexagonStarted){
    hexagonSpeed = baseSpeed-30;
  }
  if (millis()-startTime>BRAIN_DELAYMS&& countBlack()>=4 && !brainFlag)
  {
    brainFlag=true;
    brainStarted=true;
    driveMotors(0,0);
    delay(200);
  }
  if (millis()-startTime>FINISH_DELAYMS && allBlack())
  {
    driveMotors(0,0);
    while (true)
    {
      /* code */
    }
  }  
  // ++++++++++++++++++++++++++| Brain Flag Detection |+++++++++++++++++++++++++++++++++++++

  // ++++++++++++++++++++++++++| Moving Through The Map |+++++++++++++++++++++++++++++++++++

  if (hexagonFlag){
      float correction = calculatePID(rightBiasWeights);
      int leftSpeed = constrain(hexagonSpeed - correction, -255, 255);
      int rightSpeed = constrain(hexagonSpeed + correction, -255, 255);
      driveMotors(rightSpeed, leftSpeed);
      delay(20);  
  }
  else if(brainFlag)
  {
    invertedReadsensors();
      float correction = calculatePID(rightBiasWeights);
      int leftSpeed = constrain(hexagonSpeed - correction, -255, 255);
      int rightSpeed = constrain(hexagonSpeed + correction, -255, 255);
      driveMotors(rightSpeed, leftSpeed);
      delay(18);
  }
  else {
    // Normal Logic
    float correction = calculatePID(SensorCo);
    int leftSpeed = constrain(baseSpeed - correction, -255, 255);
    int rightSpeed = constrain(baseSpeed + correction, -255, 255);
    driveMotors(rightSpeed, leftSpeed);
  }
  delay(20);
}

