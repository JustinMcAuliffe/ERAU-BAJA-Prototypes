#include <Adafruit_GFX.h>    
#include <MCUFRIEND_kbv.h> 

#define LCD_CS A3 
#define LCD_CD A2 
#define LCD_WR A1 
#define LCD_RD A0 
#define LCD_RESET A4 

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#include<TinyGPS++.h>
#include<SoftwareSerial.h>


MCUFRIEND_kbv tft;

SoftwareSerial ss(0, 1);
static const int RXPin = 0, TXPin = 1;
static const uint32_t GPSBaud = 9600;
// The TinyGPS++ object
TinyGPSPlus gps;
// The serial connection to the GPS device
uint8_t trigPin;
uint8_t echoPin;
void displayModeRecovery(uint8_t failCode);

bool mCHR;
bool mCOA;
bool mPUS;

void setup() {
  Serial.begin(9600);
  ss.begin(GPSBaud);
    tft.reset();
  tft.begin(0x9486);
  tft.setRotation(1);
  tft.fillScreen(BLACK);

}

void loop() {
  //Check if on standby
  if(checkifStandby(speedX(),ultrasonic_sensors(1,1))){
      displayStandby();
      Serial.println(speedX());
  }else{ 
    //Check mode 
    if(OnModeCharge(speedX())){
        changeMode(1);
        //Update calcs from new data
        //Display data
        dataDump();

    }else if(OnModeCoast(speedX())){
        changeMode(2);
        dataDump();

    }else if(OnModePush(speedX())){
        changeMode(3);
        dataDump();
    }
    else{
      
    }
  }
  /*tft.fillScreen(BLACK);
  displaySpeedValue(18);
  tft.fillScreen(BLACK);
  displayModeCoast();
  delay(1000);*/
  //displaybootScreenPrompt1();
}

void changeMode(uint8_t x){
    //1 - CHR
    //2 - COA
    //3 - PUS
    if(x == 1){
      if(mCHR == false){
        displayModeCharge();
        mCHR = true;
        //Reset other modes
        mCOA = false;
        mPUS = false;
      }else{
        //Do nothing
      }
    }else if(x == 2){
      if(mCOA == false){
        displayModeCoast();
        mCOA = true;
        //Reset other modes
        mCHR = false;
        mPUS = false;
      }else{
        //Do nothing
      }
    }else if(x == 3){
      if(mPUS == false){
        displayModePush();
        mPUS = true;
        //Reset other modes
        mCOA = false;
        mCHR = false;
      }else{
        //Do nothing
      }
    }
}

void dataDump(){
   displaySpeedValue(speedX());
   displayAltitude(altitudeX());
   displaylatitude(latitude());
   displaylongitude(longitude());
   displayTime(timeX());
   displayDate(dateX());
   //displayRideHeightFront(ultrasonic_sensors(1,1)); //Dummy Value
   displayPlacement(detectPlacement(ultrasonic_sensors(1,1)));//Dummy Values
   displayAcceleration(1);
   displayDecceleration(1);
   displayAvgSpeed(1);
}




void displaySpeedValue(uint8_t speedValue){
    int x, y = 80;
    if(speedValue < 10){
      x = 190;
    }else{
      x = 100;
    }
    tft.setCursor(x,y);
    tft.setTextColor(WHITE);
    tft.setTextSize(22);
    tft.print(speedValue);
}

void displayModeRecovery(uint8_t failCode){
  tft.fillScreen(BLUE);
  tft.setCursor(10,140);  
  tft.setTextColor(WHITE);
  tft.setTextSize(7.5);
  tft.print("MODE-RECVRY");
  tft.setCursor(150,240);  
  tft.setTextColor(WHITE);
  tft.setTextSize(1.6);
  tft.print("FAIL - " + failCode); 
  delay(1000);
}

void displayMotion(String motion){
    tft.setCursor(10,140);
    tft.setTextColor(WHITE);
    tft.setTextSize(1.6);
    tft.print("Motion: " + motion);
}

void displaylatitude(long int latitude){
    tft.setCursor(360,60);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.print("Latitude: " + latitude);
}

void displaylongitude(long int longitude){
    tft.setCursor(360,100);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.print("Longitude: " + longitude);
}

void displayAltitude(int32_t altitude){
    tft.setCursor(360,140);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.print("Altitude: " + altitude);
}

void displaySatellites(uint32_t satellites){
    tft.setCursor(360,180);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.print("Satellites: " + satellites);
}

void displayTime(uint32_t time){
    tft.setCursor(10,30);
    tft.setTextColor(WHITE);
    tft.setTextSize(2.6);
    tft.print("Time: " + time);
}

void displayDate(uint32_t date){
    tft.setCursor(300,30);
    tft.setTextColor(WHITE);
    tft.setTextSize(2.6);
    tft.print("Date: " + date);
}

void displayAcceleration(uint32_t acc){
    tft.setCursor(20,70);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.print("Accel: " + acc);
}

void displayDecceleration(uint32_t decc){
    tft.setCursor(20,110);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.print("Deccel: " + decc);
}

void displayAvgSpeed(uint32_t avgS){
    tft.setCursor(20,150);
    tft.setTextColor(WHITE);
    tft.setTextSize(2);
    tft.print("Avg Speed: " + avgS);
}
void displayBoardAngleRoll(uint8_t boardAngle){

} //Angle to the right

void displayBoardAnglePitch(uint8_t boardAngle){

}

//Placement is either "Raised" or "On Surface"
void displayPlacement(String placement){
    tft.setCursor(190,260);
    tft.setTextColor(WHITE);
    tft.setTextSize(2.2);
    tft.print(placement);
}

void displayStandby(){
  tft.setCursor(65,120);
  tft.setTextColor(WHITE);
  tft.setTextSize(5);
  tft.print("STANDBY MODE");
  tft.setCursor(150,240);
  tft.setTextColor(WHITE);
  tft.setTextSize(1.6);
  tft.print("NO MOVEMENT DETECTED");
}

void displaystatusFAIL(uint8_t failCode){
  tft.fillScreen(BLACK);
  tft.setCursor(80,140);
  tft.setTextColor(WHITE);
  tft.setTextSize(9);
  tft.print("FAIL - " + failCode);
  delay(1000);
  tft.fillScreen(BLACK);
}

void displaystatusSUCCESS(){
  tft.fillScreen(BLACK);
  tft.setCursor(50,140);  
  tft.setTextColor(WHITE);
  tft.setTextSize(9);
  tft.print("SUCCESS - 1");
  delay(1000);
  tft.fillScreen(BLACK);
}

void displayModeCoast(){
  tft.fillScreen(BLACK);
  tft.setCursor(28,140);  
  tft.setTextColor(WHITE);
  tft.setTextSize(7.5);
  tft.print("MODE-COAST");
  delay(1000);
  tft.fillScreen(BLACK);
}

void displayModeCharge(){
  tft.fillScreen(BLACK);
  tft.setCursor(4,140);  
  tft.setTextColor(WHITE);
  tft.setTextSize(7.5);
  tft.print("MODE-CHARGE");
  delay(1000);
  tft.fillScreen(BLACK);
}


void displayModePush(){
  tft.fillScreen(BLACK);
  tft.setCursor(39,140);  
  tft.setTextColor(WHITE);
  tft.setTextSize(7.5);
  tft.print("MODE-PUSH");
  delay(1000);
  tft.fillScreen(BLACK);
}

void displaybootScreenPrompt1(){
    //Boot OS (Version 1.0)
  //2022 Nyameaama Gambrah, All rights reserved
  //START
  //GPS               NEO 6M GPS MODULE
  //Ultrasonic        HC-SR04 MODULE
  tft.setTextColor(WHITE);
  tft.setTextSize(1.6);
  tft.setCursor(10,30);
  tft.print("Boot OS (Version 1.0)");
  tft.setCursor(10,50);
  tft.print("(2022) (C) Nyameaama Gambrah, All rights reserved");
  tft.setCursor(10,70);
  tft.print("START");
  tft.setCursor(10,90);
  tft.print("GPS               NEO 6M GPS MODULE x1");
  tft.setCursor(10,110);
  tft.print("Ultrasonic        HC-SR04 MODULE x1");
  tft.setCursor(10,130);
  tft.print("Time since boot: " + millis());
}

void displaybootScreenPrompt2(){
  tft.setTextColor(WHITE);
  tft.setTextSize(1.6);
  tft.setCursor(10,150);
  tft.print("Check for gps module connected...........");
}

double ultrasonic_sensors(int trigPin,int echoPin){
    long duration,microseconds_to_cm;
      pinMode(trigPin, OUTPUT);
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      pinMode(echoPin, INPUT);
      duration = pulseIn(echoPin, HIGH);
      microseconds_to_cm = duration / 29 / 2;
    return microseconds_to_cm;
    delay(100);
  }

//Latitude
double latitude(){
  gps.encode(ss.read());
  if (gps.location.isUpdated()){
    return gps.location.lat();
  }else{
    return 0;
  }
}
//Longitude
double longitude(){
  gps.encode(ss.read());
  if (gps.location.isUpdated()){
    return gps.location.lng();
  }else{
    return 0;
  }
}
//Date
uint32_t dateX(){
  gps.encode(ss.read());
  if (gps.location.isUpdated()){
    return gps.date.value();
  }else{
    return 0;
  }
}
//Time
uint32_t timeX(){
  gps.encode(ss.read());
  if (gps.location.isUpdated()){
    return gps.time.value();
  }else{
    return 0;
  }
}
//Speed
int32_t speedX(){
  gps.encode(ss.read());
  if (gps.location.isUpdated()){
    return gps.speed.kmph();
  }else{
    return 0;
  }
}
//Altitude
int32_t altitudeX(){
  gps.encode(ss.read());
  if (gps.location.isUpdated()){
    return gps.altitude.kilometers();
  }else{
    return 0;
  }
}
//Satellites in use
uint32_t satellitesX(){
  gps.encode(ss.read());
  if (gps.location.isUpdated()){
    return gps.satellites.value();
  }else{
    return 0;
  }
}

bool checkGPS(){
    bool check = false;
    if(millis() < 5000){delay(5000 - millis());}
    if (millis() > 5000 && gps.charsProcessed() < 10){
        check = true;
    }
    return check;
}

bool checkUltrasonics(){
    bool check = false;
    double samples[10];
    for(uint8_t i = 0; i < 9; ++i){
        samples[i] = ultrasonic_sensors(7,6);//Dummy pin numbers
    }
    double sum = 0;
    for(uint8_t i = 0; i < 9; ++i){
        sum += samples[i];
    }
    double avg = sum / 10;
    check = (avg <= 0) ? false : true;
    return check;
}

bool checkifStandby(int32_t speed, double floorDist){
    bool Standby = false;
    double staticRideHeightRange[2] = {2,3}; //Dummy values
    if(speed < 1 || floorDist > staticRideHeightRange[1]){
      Standby = true;
    }
    return Standby;
}

String detectPlacement(double ultrasonics){
    String surf = "On Surface";
    String raised = "Raised";
    double staticRideHeightRange[2] = {2,3}; //Dummy values
    String ret = (ultrasonics > staticRideHeightRange[1]) ? raised : surf;
    return ret;
}

bool detectMotion(int32_t speed){
    bool motion = false;
    if(speed > 1){
        motion = true;
    }
    return motion;
}

bool OnModeCharge(int32_t speed){
    bool modeCaseTrue = (speed <= 10) ? true : false;
    return modeCaseTrue;
}

bool OnModeCoast(int32_t speed){
    bool modeCaseTrue = (speed <= 19 && speed > 10) ? true : false;
    return modeCaseTrue;
}

bool OnModePush(int32_t speed){
     bool modeCaseTrue = (speed > 19) ? true : false;
    return modeCaseTrue;
}
