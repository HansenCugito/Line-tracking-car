/*
   Hardware Timer as an Encoder interface.

  /*
      Pins associated with each timer are
      Timer1;  // RIGHT ENCODER MOTOR PA8-9
      Timer2;  // LEFT ENCODER MOTOR PA0-1
      Timer3;  // reserved PA6-7 / SPI
      Timer4;  // RIGHT MOTOR PB6-7
      Timer5;  // LEFT MOTOR PB8-9


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

unsigned long XXtime;

//#I2C1
#define SCL PB6
#define SDA PB7
//ADC
#define adcPWR PA2

//Encoder
int en_countL = 0;
int en_countR = 0;
int speedL = 0;
int speedR = 0;

char str[50];

void init_hardware_timer_encoder(uint8 T_num)  // R motor 
{
  if (T_num == 1)
  {
    //define the Timer channels as inputs.
    pinMode(PA8, INPUT_PULLUP);  //channel A 
    pinMode(PA9, INPUT_PULLUP);  //channel B
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

    TIMER2_BASE->ARR = 0xFFFF;

    //per datasheet instructions
    TIMER2_BASE->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0 );  //step 1 and 2
    TIMER2_BASE->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);  // step 3 and 4
    TIMER2_BASE->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;   //step 5
    TIMER2_BASE->CR1 |= TIM_CR1_CEN ;     //step 6
    TIMER2_BASE->CNT = 0;
  }
}
int tt = 0;

int spL = 150, spR = 150;

void setup() {

  pinMode(IN1, OUTPUT); //PB8
  pinMode(IN2, OUTPUT); //PB9
  pinMode(PWML, PWM); //PB0

  pinMode(IN3, OUTPUT); //PB10
  pinMode(IN4, OUTPUT); //PB11
  pinMode(PWMR, PWM); //PB1

  pinMode(Cam_CLK, OUTPUT); //CAM CLK IS PB5 PIN , clk is clock input signal
  pinMode(Cam_SI, OUTPUT); // CAM SI IS PB12 PIN , si is serial input signal 
  pinMode(PC13, OUTPUT); 
}

void loop() {
                
  int bc=0;
  int blackcount=0;
  int blacksum=0;
  uint16_t pp;
                                 
      blackcount=0; // reset blacksum&count
      blacksum=0;
      bc=0;
      
      CAMrefresh();
      
      for (uint8_t j = 0; j < ARR_SIZE; j++) // 20 ms x 128 = 2560 micro s
  {
    pp = analogRead(Cam_AO); // read signal from camera 0<tt<230
    //analogread returns int data type corresponding to the ADC, 12 bits 0-4095, voltage=3.3 V
    
    if( pp < (135 ) )  // we want more white than black , we want background to be 100% white
             { // if a black pixel is found, add blackcount to keep the count of black pixels and add the blackposition
              blackcount++;
              blacksum+=j;
                if(j>50) // threshold for the T-line blackcount, normally theres only 10 black count, at the T-line theres 50+blackcount 
                {
                    if(blackcount>50) // open loop control to capture the flag(T line) which has the highest black count to stop at the final circle
                    {          
                    motorMOVE(BW,100,100);
                    delay(500);
                    motorMOVE(BAK,0,0);
                    delay(10000);                  
                    }
                 }             
             }
    digitalWrite(Cam_CLK, HIGH);
    delayMicroseconds(DT);
    digitalWrite(Cam_CLK, LOW);
    delayMicroseconds(DT);
  }

       bc=int(blacksum/blackcount); // this is the weighted average method which acts like a sensor noise filter, there might be less noise if we apply weighted MOVING average, try EWMA exponential weighted moving average
  
       tracking_algorithm(bc,blackcount); // PID convert line center error to PWM signal to drive motor float PID is slow, we use int PID, approximate blacklinecenterx 0<bc<127 
        // ISSUE bc=0 might be interpreted as a black line on the leftmost array OR a white line at the left or right, if bc==0 && blackcount==0, an open loop control is applied by turning right, this open loop control works perfectly if the white line is on the right, however, if its on the left it might cause trouble
                                      
}

void Chk_speedX(int *lL, int *rR) 
{

  int tmp_spdL = 0, tmp_spdR = 0;
  float diskslots=200.0

  tmp_spdL = TIMER2_BASE->CNT;
  tmp_spdR = TIMER1_BASE->CNT;
  delay(50);
  *lL = TIMER2_BASE->CNT - tmp_spdL ;
  *rR = TIMER1_BASE->CNT - tmp_spdR ;
  *lL=(*lL/diskslots)*60.00; //calculate RPM from encoder
  *rR=(*rR/diskslots)*60.00; 
}

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
    case 4:  // stop  STP mini brake subtle stop
      digitalWrite(IN1, 0);
      digitalWrite(IN2, 0);
      digitalWrite(IN3, 0);
      digitalWrite(IN4, 0);
      break;
    case 5:   // bake  BAK stronger brake longer time //enum MOT_dir {CW, CCW, FW, BW, STP, BAK, LLT, LRT, RRT, RLT};
      digitalWrite(IN1, 1);
      digitalWrite(IN2, 1);
      digitalWrite(IN3, 1);
      digitalWrite(IN4, 1);
      break;
    case 6:  //  LEFT turn BY L MTR LLT trash
      digitalWrite(IN1, 1);
      digitalWrite(IN2, 0);
      digitalWrite(IN3, 1);
      digitalWrite(IN4, 1);
      break;
    case 7:  //  right turn BY L MTR LRT trash
      digitalWrite(IN1, 0);
      digitalWrite(IN2, 1);
      digitalWrite(IN3, 1);
      digitalWrite(IN4, 1);
      break;
    case 8:  //  RIGHT turn BY R MTR RRT trash
      digitalWrite(IN1, 1);
      digitalWrite(IN2, 1);
      digitalWrite(IN3, 1);
      digitalWrite(IN4, 0);
      break;
    case 9:    //  LEFT turn BY R MTR RLT trash
      digitalWrite(IN1, 1);
      digitalWrite(IN2, 1);
      digitalWrite(IN3, 0);
      digitalWrite(IN4, 1);
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

void CAMrefresh() // 10 ms x 128 + 20 ms = 1300 micro s
{
  digitalWrite(Cam_CLK, LOW);
  digitalWrite(Cam_SI, HIGH);

  digitalWrite(Cam_CLK, HIGH);
  digitalWrite(Cam_SI, LOW);


  digitalWrite(Cam_CLK, LOW);

  for (uint16_t j = 0; j < ARR_SIZE; j++) // this is the CLK signal generator for 129 clock cycles
  {
    digitalWrite(Cam_CLK, HIGH);
    delayMicroseconds(DT); // 10 ms
    digitalWrite(Cam_CLK, LOW);
  }
  delayMicroseconds(DT); // 10 ms
  digitalWrite(Cam_SI, HIGH);
  digitalWrite(Cam_CLK, HIGH);
  delayMicroseconds(DT); // 10 ms
  digitalWrite(Cam_SI, LOW);
  digitalWrite(Cam_CLK, LOW);

}

int tracking_algorithm(int bc, int blackcount)
{

  int error=0;
  int previousError;
  int totalError=0;
  int power;

  uint8_t Kp=35; 
  uint8_t Ki=15;
  uint8_t Kd=1;  
  uint8_t PWM_Right;
  uint8_t PWM_Left;

  uint8_t n=0;
  uint8_t atotalError=0;
  uint8_t Ksr=10;
  
       previousError=error; //save previous error for differential -64<previouserror<64
       error=(bc - 64); // -64<error<63 center e=-1
       if(n==10) // count sum from n = 0 to 9 only
       {
        totalError=0;
        atotalError=0;
        n=0;   
       }
       totalError+=error;
       atotalError+=abs(error);
       power= int((Kp*error)/10) + Kd*(error-previousError)+ int(  (Ki*totalError) /  1024  ); 
       n++; // divide by 1024 yields similar result to divide by 1000 but faster since its power of 2
       
       if(power<0) 
       { // turn left
            if( power<-255 )
            { 
            power=-255;
            }
            PWM_Right=255- int(  (Ksr*atotalError)/1024 );
            PWM_Left=255-abs(power)- int(  (Ksr*atotalError)/1024 );                                                           
        }
         else //if(power>0) 
         { // turn right
            if( power>255 )
            {
            power=255;
            }
            PWM_Right=255-power- int(  (Ksr*atotalError)/1024 );
            PWM_Left=255- int(  (Ksr*atotalError)/1024 );                    
         } 
                              
          motorMOVE(  BW, int(PWM_Left/3) , int(PWM_Right/3) ); // dividing final speed by 3 to run at lower speed

      return power;
}

/******************************************************************/
