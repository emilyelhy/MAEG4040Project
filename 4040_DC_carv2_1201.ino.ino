/*
   Hardware Timer as an Encoder interface.

  /*
      Pins associated with each timer are
      Timer1;  // LEFT MOTOR PA8-9
      Timer2;  // LEFT MOTOR PA0-1
      Timer3;  // reserved PA6-7 / SPI
      Timer4;  // LEFT MOTOR PB6-7


   COUNTING DIRECTION:
   0 means that it is upcounting, meaning that Channel A is leading Channel B

   EDGE COUNTING:

   mode 1 - only counts pulses on channel B
   mode 2 - only counts pulses on Channel A
   mode 3 - counts on both channels.

*/


#include "SPI.h"
#include "TFT_22_ILI9225.h"
#include <stdio.h>
 #include "hw.h"

enum MOT_dir {CW, CCW, BW, FW, STP, BAK, LLT, LRT, RRT, RLT};

// DC MOTOR L
int PWML = PB0;
int IN1 = PB8;
int IN2 = PB9;

//encoderL, (TIMER 2)
int AENCL = PA0;
int BENCL = PA1;

// DC MOTOR R
int PWMR = PB1;
int IN3 = PB10;
int IN4 = PB11;

//encoderR, (TIMER 1)
int AENCR = PA8;
int BENCR = PA9;

//TFT
#define TFT_RST PB14
#define TFT_RS PB13
#define TFT_CS PB15 // SS
#define TFT_SDI PA7 // SI
#define TFT_CLK PA5 // SCK
#define TFT_LED 99
#define TFT_BRIGHTNESS 99

// LINE Cam
#define Cam_AO PA4
#define Cam_CLK PB5
#define Cam_SI PB12
#define ARR_SIZE 128
#define DT 10

#define MID 69
// #define BLK 200 //!!!
#define BLK 180 //!!!
#define NSL 55
#define NSR 53

//uint16_t Position[];

int avrLVL = 0;

unsigned long XXtime;
uint16_t camARR[ARR_SIZE];
uint16_t PcamARR[ARR_SIZE];

uint16_t *camptr;
uint16_t *Pcamptr;


// PID Constants
float Kp = 25;
float Ki = 0;
float Kd = 15;

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;
int previous_pos = 0;
int flag1 = 0;     // 0 = no turn, 1 = line on left, 2 = right
boolean flag2 = false;
unsigned long startTime;


//#I2C1
#define SCL PB6
#define SDA PB7
//ADC
#define adcPWR PA2

// Variables and constants
uint16_t x, y;
boolean flag = false;

//Encoder stuff

int en_countL = 0;
int en_countR = 0;
int speedL = 0;
int speedR = 0;

char str[50];
//uint8_t Chk_speed_flag = 0;

// Use hardware SPI (faster - on Uno: 13-SCK, 12-MISO, 11-MOSI)
//TFT_22_ILI9225 tft = TFT_22_ILI9225(TFT_RST, TFT_RS, TFT_CS, TFT_LED, TFT_BRIGHTNESS);

TFT_22_ILI9225 tft = TFT_22_ILI9225(TFT_RST, TFT_RS, TFT_CS, 0, 10);

void init_hardware_timer_encoder(uint8 T_num)  // R motor
{
  if (T_num == 1)
  {
    //define the Timer channels as inputs.
    pinMode(PA8, INPUT_PULLUP);  //channel A
    pinMode(PA9, INPUT_PULLUP);  //channel B
    // value to count up to : 16 bit so max is 0xFFFF = 65535

    TIMER1_BASE->ARR = 0xFFFF;

    //per datasheet instructions
    TIMER1_BASE->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 );  //step 1 and 2
    TIMER1_BASE->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);  // step 3 and 4
    TIMER1_BASE->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;   //step 5
    TIMER1_BASE->CR1 |= TIM_CR1_CEN ;     //step 6
    TIMER1_BASE->CNT = 0;
  }
  else if (T_num == 2)   // L motor
  {
    //define the Timer channels as inputs.
    pinMode(PA0, INPUT_PULLUP);  //channel A
    pinMode(PA1, INPUT_PULLUP);  //channel B
    // value to count up to : 16 bit so max is 0xFFFF = 65535

    TIMER2_BASE->ARR = 0xFFFF;

    //per datasheet instructions
    TIMER2_BASE->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 );  //step 1 and 2
    TIMER2_BASE->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);  // step 3 and 4
    TIMER2_BASE->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;   //step 5
    TIMER2_BASE->CR1 |= TIM_CR1_CEN ;     //step 6
    TIMER2_BASE->CNT = 0;
  }
}






// the setup function runs once when you press reset or power the board
// DC MOTOR L

int tt = 0;
uint8_t change = 0;

int spL = 150, spR = 150;
float tx = 0;



void setup() {

  Pcamptr = &PcamARR[0];
  tft.begin();
  tft.clear();
  tft.setOrientation(2);
  init_hardware_timer_encoder(1);  // R m

  init_CAM();
  camptr = &camARR[0];
  init_hardware_timer_encoder(2); // L motor

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWML, PWM);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(PWMR, PWM);
  tft.setFont(Terminal12x16);

  mainPage(0);

  pinMode(Cam_CLK, OUTPUT);
  pinMode(Cam_SI, OUTPUT);
  pinMode(PC13, OUTPUT);
  XXtime =  millis();
}

// the loop function runs over and over again forever


void loop() {

  if ((tt++) % 10 < 5)
  {
    mainPage(1); mainPage(2); mainPage(3); mainPage(4); mainPage(5); mainPage(6);
  }
  //motorMOVE(FW, 80, 80);
//  if ((tt) % 1000 < 500)
//  {    motorMOVE(FW, 40, 40);}
//  else
//  {    motorMOVE(BW, 40, 40);}

  Chk_speedX(&speedL, &speedR);
  // tracking algoritm
  tracking_algoritm();

  Cam_plot(0); // clear
  getCameraZ();  // get data
  Cam_plot(1);  // plot
  if (digitalRead(PC13))
    digitalWrite(PC13, LOW);
  else
    digitalWrite(PC13, HIGH);
  XXtime = millis() - XXtime;
}


void Chk_speedX(int *lL, int *rR)
{

  int tmp_spdL = 0, tmp_spdR = 0;

  tmp_spdL = TIMER2_BASE->CNT;
  tmp_spdR = TIMER1_BASE->CNT;
  delay(50);
  *lL = TIMER2_BASE->CNT - tmp_spdL ;
  *rR = TIMER1_BASE->CNT - tmp_spdR ;

}

int getBLK(uint16_t camARR[]) {
  int left = -1, right = -1;
  for (int i = 0; i < ARR_SIZE; i++) {
    if (camARR[i] <= BLK) {
      int count = 0;
      for (int j = i + 1; j < i + 3; j++) {
        if (camARR[j] <= BLK) count++;
      }
      if (count == 2) {
        left = i;
        break;
      }
    }
  }

  for (int i = ARR_SIZE - 1; i >= 0; i--) {
    if (camARR[i] <= BLK) {
      int count = 0;
      for (int j = i - 1; j > i - 3; j--) {
        if (camARR[j] <= BLK) count++;
      }
      if (count == 2) {
        right = i;
        break;
      }
    }
  }
  if (right - left > 115) return -10;
  else if (right - left > 50 && left < 5) return -3;
  else if (right - left <= 30) return (right + left) / 2;
  else if (abs(right - MID) > abs(left - MID)) return right;
  else return left;
}

/*
int getVELO(uint16_t pos, poslate, millis() ) {

  vel = (pos-poslast)/millis
  return
  }
 
}

*/
void tracking_algoritm()
{
  // USER DEFINE
  int pos = getBLK(camARR);
  if (pos == -10 && millis() > 10000) {
    motorMOVE(FW, NSL, NSR);
    delay(800);
    motorMOVE(STP, 0, 0);
    delay(2000);
    //flag = 0;
    exit(1);
  }
  else if (pos == -1){
    if (flag1 == 0){
      if (previous_pos < MID){
        motorMOVE(CW, 40, 40);
        flag1 = 1;
      } else{
        motorMOVE(CCW, 40, 40);
        flag1 = 2;
      }
    } else if (flag1 == 1) motorMOVE(CW, 40, 40);
      else if (flag1 == 2) motorMOVE(CCW, 40, 40);  
  }
  /*
  else if (flag1 != 0) {
    //motorMOVE(BW, 32, 32);
    if (pos == -1){
      if (previous_pos < MID || flag == 1) {
        motorMOVE(CW, 40, 40);
        flag1 = 1;
      }
      else if (previous_pos >= MID || flag == 2){
        motorMOVE(CCW, 40, 40);
        flag1 = 2;
      }
    } else flag1 = 0;
  }
  */
  /*
  else if (pos == -1){ 
    if (previous_pos < MID) {
        motorMOVE(CW, 40, 40);
      }
    else if (previous_pos >= MID ){
        motorMOVE(CCW, 40, 40);
        
    motorMOVE(BW, 34, 34);
  
  }*/
  
  
  else if (pos == -3) {
    motorMOVE(CW, 0, NSR);
    delay(500);
    flag1 = 0;
    //startTime = millis();

 
  }

  else if (pos >= 0 && pos < 49){
    motorMOVE(FW, map(pos, 0, 49, 0, NSL), NSR);
    flag1 = 0; 
  }
  else if (pos >= 89 && pos < ARR_SIZE){
    motorMOVE(FW, NSL, map(pos, 89, ARR_SIZE - 1, NSR, 0));
    flag1 = 0;
  }
/*
  else if (pos >= 0 && pos < 54)
    motorMOVE(FW, 0, map(pos, 0, 53, NSR-5, 30));
  else if (pos >= 84 && pos < ARR_SIZE)
    motorMOVE(FW, map(pos, 84, ARR_SIZE - 1, 30, NSL-5), 0);
  else if (pos >= 54 && pos < MID)
  motorMOVE(FW, map(pos, 54, MID - 1, NSL-5, NSL), NSR);
  */

  else if (pos >= 44 && pos < MID){
    flag1 = 0;
    motorMOVE(FW, map(pos, 44, MID - 1, 45, NSL), NSR);
  }
  else if (pos >= MID && pos < 94){
    flag1 = 0;
    motorMOVE(FW, NSL, map(pos, MID, 93, NSR, 43)); 
  }
  else {
    motorMOVE(FW, NSL, NSR);
    flag1 = 0;
  }

  previous_pos = pos;
}

/*
void calculate_pid()
{
  if (pos < 44 && pos >=94)
  error = 69 - pos;
  else 
  error = 0;
  
  P = error;
  I = I + previous_I;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
  previous_error = error;
}

void motor_control()
{
  // Calculating the effective motor speed:
  int left_motor_speed = NSL - PID_value;
  int right_motor_speed = NSR + PID_value;
  
  motorMOVE(FW, left_motor_speed, right_motor_speed);
}

*/

void motorMOVE(MOT_dir actX, uint8_t spdL , uint8_t spdR )
{
  analogWrite(PWML, spdL);
  analogWrite(PWMR, spdR);
  switch (actX)
  {
    case 0:  //CW
      digitalWrite(IN1, 0);
      digitalWrite(IN2, 1);
      digitalWrite(IN3, 0);
      digitalWrite(IN4, 1);
      break;
    case 1:  //CCW
      digitalWrite(IN1, 1);
      digitalWrite(IN2, 0);
      digitalWrite(IN3, 1);
      digitalWrite(IN4, 0);
      break;
    case 2:  // forward  FW
      digitalWrite(IN1, 0);
      digitalWrite(IN2, 1);
      digitalWrite(IN3, 1);
      digitalWrite(IN4, 0);
      break;
    case 3:    // backward BW
      digitalWrite(IN1, 1);
      digitalWrite(IN2, 0);
      digitalWrite(IN3, 0);
      digitalWrite(IN4, 1);
      break;
    case 4:  // stop  STP
      digitalWrite(IN1, 0);
      digitalWrite(IN2, 0);
      digitalWrite(IN3, 0);
      digitalWrite(IN4, 0);
      break;
    case 5:   // bake  BAK   //enum MOT_dir {CW, CCW, FW, BW, STP, BAK, LLT, LRT, RRT, RLT};
      digitalWrite(IN1, 1);
      digitalWrite(IN2, 1);
      digitalWrite(IN3, 1);
      digitalWrite(IN4, 1);
      break;
    case 6:  //  LEFT turn BY L MTR LLT
      digitalWrite(IN1, 1);
      digitalWrite(IN2, 0);
      digitalWrite(IN3, 1);
      digitalWrite(IN4, 1);
      break;
    case 7:  //  right turn BY L MTR LRT
      digitalWrite(IN1, 0);
      digitalWrite(IN2, 1);
      digitalWrite(IN3, 1);
      digitalWrite(IN4, 1);
      break;
    case 8:  //  RIGHT turn BY R MTR RRT
      digitalWrite(IN1, 1);
      digitalWrite(IN2, 1);
      digitalWrite(IN3, 1);
      digitalWrite(IN4, 0);
      break;
    case 9:    //  LEFT turn BY R MTR RLT
      digitalWrite(IN1, 1);
      digitalWrite(IN2, 1);
      digitalWrite(IN3, 0);
      digitalWrite(IN4, 1);
      break;
  }
}



void mainPage(uint8_t sel)
{
  float tBat = 0.0;
  int tmp = 0;
  switch (sel)
  {
    case 0: tft.drawText(12 * 3, 16 * sel, "MAE 4040" , COLOR_WHITE);
      tft.drawText(1, 16 * 1, "CYC ms:", COLOR_YELLOW);
      tft.drawText(1, 16 * 2, "Lp cnt:", COLOR_YELLOW);
      tft.drawText(1, 16 * 3, "Rp cnt:", COLOR_YELLOW);
      tft.drawText(1, 16 * 4, "avrLVL:", COLOR_YELLOW);
      tft.drawText(1, 16 * 5, "BatLVL:", COLOR_YELLOW);
      tft.drawLine(0, 16 * 8, tft.maxX(), 16 * 8, COLOR_BLUE);
      tft.drawLine(0, tft.maxY() - 2, tft.maxX(), tft.maxY() - 2, COLOR_YELLOW);
      tft.drawLine(0, tft.maxY() - 2, tft.maxX(), tft.maxY() - 2, COLOR_YELLOW);
      tmp = map(50, 0, 230, 16 * 8, (16 * 8) + 90); //16 * 6, (16 * 8) + 90)
      tft.drawLine(10, tmp, 20, tmp,  0x0470);
      tft.drawLine(10, tmp + 20, 20, tmp + 20.,  0x0470);
      tft.drawLine(10, tmp + 40, 20, tmp + 40.,  0x0470);
      tft.drawLine(10, tmp + 60, 20, tmp + 60.,  0x0470);
      //plot()

      break;
    case 1:
      tft.drawText(12 * 8, 16 * sel, "          ", COLOR_GREEN);
      sprintf(str, "%ld", XXtime );  XXtime =  millis();
      tft.drawText(12 * 8, 16 * sel, str, COLOR_GREEN);
      break;
    case 2:
      tft.drawText(12 * 8, 16 * sel, "          ", COLOR_GREEN);
      sprintf(str, "%d", speedL );
      tft.drawText(12 * 8, 16 * sel, str, COLOR_GREEN);
      break;
    case 3:
      tft.drawText(12 * 8, 16 * sel, "          ", COLOR_GREEN);
      sprintf(str, "%d", speedR );
      tft.drawText(12 * 8, 16 * sel, str, COLOR_GREEN);
      break;
    case 4:
      tft.drawText(12 * 8, 16 * sel, "          ", COLOR_GREEN);
      sprintf(str, "%d", avrLVL );
      tft.drawText(12 * 8, 16 * sel, str, COLOR_GREEN);
      break;
    case 5:
      tBat = (analogRead(adcPWR) * 13.6) / 4096;
      tft.drawText(12 * 8, 16 * sel, "          ", COLOR_GREEN);
      sprintf(str, "%2.2f", tBat );
      tft.drawText(12 * 8, 16 * sel, str, COLOR_GREEN);
      break;
    case 6:
      break;
    case 7:
      tft.drawText(1, 16 * 3, "R spd:", COLOR_YELLOW);
      tft.drawText(12 * 6, 16 * 3, "         ", COLOR_GREEN);
      sprintf(str, "%d", speedR );
      tft.drawText(12 * 6, 16 * 3, str, COLOR_GREEN);
      break;
  }
}


/******************************************************************/
// Camera routines

// LINE Cam
//#define Cam_AO PA4
//#define Cam_CLK PB5
//#define Cam_SI PB12

/******************************************************************/

void init_CAM()
{
  pinMode(Cam_CLK, OUTPUT);
  pinMode(Cam_SI, OUTPUT);
  CAMrefresh();
}

void CAMrefresh()
{
  digitalWrite(Cam_CLK, LOW);
  digitalWrite(Cam_SI, HIGH);

  digitalWrite(Cam_CLK, HIGH);
  digitalWrite(Cam_SI, LOW);


  digitalWrite(Cam_CLK, LOW);

  for (uint16_t j = 0; j < ARR_SIZE; j++)
  {
    digitalWrite(Cam_CLK, HIGH);
    delayMicroseconds(DT);
    digitalWrite(Cam_CLK, LOW);
  }
  delayMicroseconds(DT);
  digitalWrite(Cam_SI, HIGH);
  digitalWrite(Cam_CLK, HIGH);
  delayMicroseconds(DT);
  digitalWrite(Cam_SI, LOW);
  digitalWrite(Cam_CLK, LOW);

}



void getCameraZ()
{
  uint16_t *TMPptr, tt;

  CAMrefresh();

  //SWAP pointers
  TMPptr = camptr;
  camptr = Pcamptr;
  Pcamptr = TMPptr;

  for (int j = 0; j < ARR_SIZE; j++)
  {
    tt = analogRead(Cam_AO);
    *(camptr + j) = map(tt, 0, 230, 16 * 6, (16 * 8) + 90);
    if ((j > 54) && (j  < 74))
      avrLVL = (avrLVL + tt) / 2;

    digitalWrite(Cam_CLK, HIGH);
    delayMicroseconds(DT);
    digitalWrite(Cam_CLK, LOW);
    delayMicroseconds(DT);
  }
}




void Cam_plot(uint8_t wr)
{
  for (int j = 0; j < ARR_SIZE; j++)
  {
    if (wr)
      tft.drawPixel(j + 20, *(camptr + j), 0x8f70);
    else
      tft.drawPixel(j + 20, *(camptr + j), 0);
  }
}





/******************************************************************/
