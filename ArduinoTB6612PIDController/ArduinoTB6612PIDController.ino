/**********************************************************************************************
 * PID DC Servo Motor controller for Arduino Nano and STM32 Blue Pill
 * by patlhy
 *
 * Use Arduino Nano or STM32 Blue Pill as a controller to replace stepper motors with DC motors.
 * This Arduino code accept STEP and DIRECTION input from driver sockets meant for A4988 or DRV8825 driver.
 * I used Miguel Sanchez's code in https://github.com/misan/dcservo as a starting point. 
 * Changed the motor driver to TB6612.Features added are dual motor control, step test,
 * motor stuck and runaway detection.
 * 
 * A companion PID tuning Python software can be found in 
 * https://github.com/patlhy/PID-DC-Motor-Servo-Controller/tree/main/PyPIDServoMotorTuner
 **********************************************************************************************/

#include <EEPROM.h>
#include "PID_v2.h"
#include "config.h"

signed char QEM[16] = {0,1,-1,2,-1,0,2,1,1,2,0,-1,2,-1,1,0}; //state machine for incrementing encoder count

boolean steptest=false, enable_active1 = false, enable_active2 = false, motor_err;
int  stepmillitime[ARRYSIZE], previoustime=0;

volatile long encoder1Pos = 0,OldPos1 =0;
unsigned long OldTime1;
double kp1=KP1,ki1=KI1,kd1=KD1;
double input1=0, output1=0, setpoint1=0;
PID myPID1(&input1, &output1, &setpoint1,kp1,ki1,kd1, DIRECT);
long target1=0;

static unsigned char old1=0;
static bool OldDir1;
int stepmax1=0, t0a=0, t1a=0, steppos1[ARRYSIZE], Out1[ARRYSIZE];



long stepstarttime;

#ifdef DOUBLEMOTOR
  volatile long encoder2Pos = 0, OldPos2 =0;
  unsigned long OldTime2;
  double kp2=KP2,ki2=KI2,kd2=KD2;
  double input2=0, output2=0, setpoint2=0;
  PID myPID2(&input2, &output2, &setpoint2,kp2,ki2,kd2, DIRECT);
  long target2=0;

  static unsigned char old2=0;
  static bool OldDir2;
  int stepmax2=0, t0b=0, t1b=0, steppos2[ARRYSIZE], Out2[ARRYSIZE] ;
#endif

#ifdef NANO
void pciSetup(byte pin) 
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group 
}
#endif

/**the setup routine runs once when you press reset:**/
void setup()  {
  bool eprmStatus;

  Serial.begin (115200);


#ifdef STM32
  attachInterrupt(STEP1, countStep1  , RISING);  // interrupt for STEP1 input pin
  attachInterrupt(Q1, encoderINT1 , CHANGE);     // interrupt for MOTOR1 encoder input pin
  attachInterrupt(Q2, encoderINT1 , CHANGE);


 #ifdef DOUBLEMOTOR
  attachInterrupt(STEP2, countStep2 , RISING);  // interrupt for STEP2 input pin
  attachInterrupt(Q5, encoderINT2 , CHANGE);    // interrupt for MOTOR2 encoder input pin
  attachInterrupt(Q6, encoderINT2 , CHANGE);
 #endif
#endif

#ifdef NANO
  pciSetup(Q1);
  pciSetup(Q2);
  attachInterrupt(1, countStep1  , RISING);  // step  input1 on interrupt 1 - pin 3

 #ifdef DOUBLEMOTOR
  pciSetup(Q5);
  pciSetup(Q6);
  attachInterrupt(0, countStep2 , RISING);  // step  input1 on interrupt 0 - pin 2
 #endif

  TCCR1B = TCCR1B & 0b11111000 | 1; // set 31Kh PWM 
#endif

  // Set up IO pin
  pinMode(M1, OUTPUT);
  pinMode(MA2, OUTPUT);
  pinMode(MA1, OUTPUT);     
  analogWrite(M1, 0);
  digitalWrite(MA2, LOW);
  digitalWrite(MA1, LOW);
  
  pinMode(M1_MAX ,OUTPUT);
  pinMode(STOP_PRINT ,OUTPUT);
  digitalWrite(STOP_PRINT, HIGH);

  pinMode(Q1,INPUT_PULLUP);
  pinMode(Q2,INPUT_PULLUP);

  pinMode(STEP1,INPUT_PULLUP);
  pinMode(DIR1,INPUT_PULLUP);
  pinMode(ENABLE1,INPUT_PULLUP);
  pinMode(MOTORENABLE,OUTPUT);

#ifdef ENABLE_OUT_LOW
  digitalWrite(MOTORENABLE, HIGH);
#else  
  digitalWrite(MOTORENABLE, LOW);
#endif

  myPID1.SetMode(AUTOMATIC);
  myPID1.SetSampleTime(SAMPLETIME);
  myPID1.SetOutputLimits(-MAXOUTPUT,MAXOUTPUT);

  if(eprmStatus=recoverPIDfromEEPROM())
      myPID1.SetControllerDirection(DIRECT);
  myPID1.SetTunings(kp1,ki1,kd1);
  pwmOut(0,M1,MA1,MA2);


#ifdef DOUBLEMOTOR  
  pinMode(M5, OUTPUT);
  pinMode(MB1, OUTPUT);
  pinMode(MB2, OUTPUT);   
  analogWrite(M5, 0);
  digitalWrite(MB1, LOW);
  digitalWrite(MB2, LOW);

  pinMode(M2_MAX ,OUTPUT);

  pinMode(Q5,INPUT_PULLUP);
  pinMode(Q6,INPUT_PULLUP);

  pinMode(STEP2,INPUT_PULLUP);
  pinMode(DIR2,INPUT_PULLUP);
  pinMode(ENABLE2,INPUT_PULLUP);

  myPID2.SetMode(AUTOMATIC);
  myPID2.SetSampleTime(SAMPLETIME);
  myPID2.SetOutputLimits(-MAXOUTPUT,MAXOUTPUT); 
  if(eprmStatus)
      myPID2.SetControllerDirection(REVERSE);
  myPID2.SetTunings(kp2,ki2,kd2);
  pwmOut(0,M5,MB1,MB2);
#endif
  Serial.println("DC Servo Motor controller Started. Ver July 2021");
  Serial.println("Press 'h' for help.\n");
  printPos();
}

/**the loop routine runs over and over again forever:**/
void loop()
{   
   int steptimenow;
   unsigned int errtime;
   static int i=0;

/**Check Enable PIN to enable and run Motor 1**/
#ifdef ENABLE_IN_INVERT       
   if(digitalRead(ENABLE1)==HIGH)//check Motor 1 Enable PIN. Activate motor only when it is active
#else    
   if(digitalRead(ENABLE1)==LOW)
#endif
   {
    if(!enable_active1)//Initialize variables when Enable PIN turn active
    {
       enable_active1 = true;
       motor_err = true;
       target1=0;
       encoder1Pos=0;
       OldTime1 = millis();     
       OldPos1 = 0;

#ifdef ENABLE_OUT_LOW
        digitalWrite(MOTORENABLE, LOW);
#else  
        digitalWrite(MOTORENABLE, HIGH);
#endif
       Serial.println("ENABLE_1 Active");   
    }
    
    input1 = encoder1Pos; 
    setpoint1=target1;
    myPID1.Compute();
  
    if(abs(output1) > MAXOUTALM){ //Motor 1 PID calculated output value exceed defined MAXOUTALM value
      digitalWrite(M1_MAX, HIGH); //Light up M1_MAX LED
      
      /**If output continue to be higher than MAXOUTALM for ALMTIME defined time and encoder have moved very little or too much.
      Pull STOP_PRINT pin low and stop the motor **/
      if((millis() > (OldTime1+ALMTIME))&&((abs(encoder1Pos-setpoint1)> MAXSTEP)||(abs(encoder1Pos-OldPos1)<MINSTEP))){
            pwmOut(0, M1,MA1, MA2);
            digitalWrite(STOP_PRINT, LOW);
            errtime = millis()-OldTime1;
            if(motor_err){
              motor_err = false;
              Serial.print("Error from setpoint:");
              Serial.print(abs(encoder1Pos-setpoint1));
              Serial.print("\tOutput:");
              Serial.print(output1);
              Serial.print("\tTime at Max:");
              Serial.print(millis()-OldTime1);
              Serial.println("\tM1:PRINT STOPPED!!!\r");
            }
            if(!(errtime%1500) )
              motor_err = true;
      }
      else
         pwmOut((int)output1, M1, MA1, MA2);
    }
    else{
        pwmOut((int)output1, M1, MA1, MA2);
        digitalWrite(M1_MAX, LOW);
        OldPos1 = encoder1Pos;
        OldTime1 = millis(); 
    }
  }
  else{ //Motor 1 Enable PIN not active
    if(enable_active1)
    {
      enable_active1 = false;
      Serial.println("ENABLE_1 off");

      digitalWrite(MA2, LOW);
      digitalWrite(MA1, LOW);
      if(enable_active2){ //If both Motor 1&2 Enable PIN are not active, disable TB6612 driver      
#ifdef ENABLE_OUT_LOW
        digitalWrite(MOTORENABLE, HIGH);
#else  
        digitalWrite(MOTORENABLE, LOW);
#endif   
        }
     }      
   }
   
/****Double Motor *********/
#ifdef DOUBLEMOTOR


/**Check Enable PIN to enable and run Motor 2**/
 #ifdef ENABLE_IN_INVERT 
   if(digitalRead(ENABLE2)==HIGH) //check Motor 2 Enable PIN. Activate motor only when it is active
 #else    
   if(digitalRead(ENABLE2)==LOW)
 #endif
   {
 
    if(!enable_active2) //Initialize variables when Enable PIN turn active
    {
       enable_active2 = true;
       motor_err = true;
       target2=0;
       encoder2Pos=0;
       OldTime2 = millis();     
       OldPos2 = 0;

 #ifdef ENABLE_OUT_LOW
        digitalWrite(MOTORENABLE, LOW);
 #else  
        digitalWrite(MOTORENABLE, HIGH);
 #endif  
       Serial.println("ENABLE_2 Active");   
    }
    
    input2 = encoder2Pos; 
    setpoint2=target2;
    myPID2.Compute();

    if(abs(output2) > MAXOUTALM){ //Motor 2 PID calculated output value exceed defined MAXOUTALM value
      digitalWrite(M2_MAX, HIGH); //Light up M2_MAX LED

       /**If output continue to be higher than MAXOUTALM for ALMTIME defined time and encoder have moved very little or too much.
       Pull STOP_PRINT pin low and stop the motor **/
      if((millis() > (OldTime2+ALMTIME))&&((abs(encoder2Pos-setpoint2)> MAXSTEP)||(abs(encoder2Pos-OldPos2)<MINSTEP))){
            pwmOut(0,M5, MB1, MB2);
            digitalWrite(STOP_PRINT, LOW);
            errtime = millis()-OldTime2;
            if(motor_err){
              motor_err = false;            
              Serial.print("Error from setpoint:");
              Serial.print(abs(encoder2Pos-setpoint2));
              Serial.print("\tOutput:");
              Serial.print(output2);
              Serial.print("\tTime at Max:");
              Serial.print(millis()-OldTime2);
              Serial.println("\tM2:PRINT STOPPED!!!\r"); 
            }
            if(!(errtime%1500))
              motor_err = true;
      }
      else{
        pwmOut((int)output2, M5, MB1, MB2);
      }
    }
    else{
        pwmOut((int)output2, M5, MB1, MB2);
        digitalWrite(M2_MAX, LOW);
        OldPos2 = encoder2Pos;
        OldTime2 = millis(); 
    }
  }
  else{ //Motor 1 Enable PIN not active
    if(enable_active2)
    {
      enable_active2 = false;
      Serial.println("ENABLE_2 off");

      digitalWrite(MB2, LOW);
      digitalWrite(MB1, LOW);
      
      if(enable_active1){ //If both Motor 1&2 Enable PIN are not active, disable TB6612 driver  
 #ifdef ENABLE_OUT_LOW
        digitalWrite(MOTORENABLE, HIGH);
 #else  
        digitalWrite(MOTORENABLE, LOW);
 #endif 
        }
      }      
  }
#endif 

/****Double Motor *********/
/****Record step test response in memory and print out to serial once completed ****************/
  if(steptest&&(enable_active1||enable_active2))
    {
      steptimenow = (int)(millis() -stepstarttime);
      
      if((encoder1Pos > target1)&& (t0a==0))
        t0a= steptimenow;

      if(((int)encoder1Pos < steppos1[i-1])&& (t0a!=0)&& (t1a ==0)&&(stepmax1==0))
        stepmax1 = steppos1[i-1]-(int)target1;  
      
      if((encoder1Pos < target1)&& (t0a!=0)&& (t1a ==0))
        t1a =steptimenow;  

    #ifdef DOUBLEMOTOR 
      if((encoder2Pos > target1)&& (t0b ==0))
        t0b = steptimenow;

      if(((int)encoder2Pos < steppos2[i-1])&& (t0b!=0)&& (t1b ==0)&&(stepmax2==0))
        stepmax2 =steppos2[i-1]-(int)target1;
      
      if((encoder2Pos < target1)&& (t0b !=0)&& (t1b ==0))
        t1b =steptimenow;  
    #endif
         
      if((i==0)||((steptimenow != previoustime)&&(i<=ARRYSIZE))){
        stepmillitime[i]= steptimenow;
        steppos1[i]= (int)encoder1Pos;
        Out1[i]= (int)output1;
 
    #ifdef DOUBLEMOTOR 
        steppos2[i]= (int)encoder2Pos;
        Out2[i]= (int)output2;   
    #endif    
        i++;
      }
      /**** Steps test recorded. Print out result through Serial port***/
      else if(i==ARRYSIZE){
        i=0;
        steptest =false;
        while(i< (HALFARRYSIZE)){
          Serial.print(stepmillitime[i]);
          Serial.print(":\t");
          Serial.print(steppos1[i] - (int)target1);
          Serial.print(",\t");

          Serial.print(Out1[i]);
          Serial.print(",\t\t| ");

  #ifdef DOUBLEMOTOR 
          Serial.print(steppos2[i]- (int)target2);
          Serial.print(",\t");
          Serial.print(Out2[i]);
          Serial.print(",\t\t| ");
  #endif
          Serial.print(stepmillitime[i+HALFARRYSIZE]);
          Serial.print(":\t");
          Serial.print(steppos1[i+HALFARRYSIZE] - (int)target1);
          Serial.print(",\t");

          Serial.print(Out1[i+HALFARRYSIZE]);
  #ifdef DOUBLEMOTOR 
          Serial.print(",\t\t| ");
          Serial.print(steppos2[i+HALFARRYSIZE] - (int)target2);
          Serial.print(",\t");

          Serial.print(Out2[i+HALFARRYSIZE]);
  #endif 
          Serial.println(" ");
          i++;
        }
                
         Serial.print("1st time to setpoint (A): ");
         Serial.print(t0a);
         Serial.print("\t\tTime to setpoint after overshoot (A): ");
         Serial.print(t1a);
         Serial.print("\t\tPeriod (A): ");
         Serial.print((t1a-t0a)*2);
         Serial.print("\t\tMax Overshoot(A): ");
         Serial.println(stepmax1);

#ifdef DOUBLEMOTOR 
         Serial.print("1st time to setpoint (B): ");
         Serial.print(t0b);
         Serial.print("\t\tTime to setpoint after overshoot (B): ");
         Serial.print(t1b);
         Serial.print("\t\tPeriod (B): ");
         Serial.print((t1b-t0b)*2);
         Serial.print("\t\tMax Overshoot(B): ");
         Serial.println(stepmax2);
#endif

       Serial.print("Target (A): ");
       Serial.print(target1);
       Serial.print("\t\tEncoder Pos (A): ");
       Serial.println(encoder1Pos);
#ifdef DOUBLEMOTOR 
       Serial.print("\t\t\tEncoder Pos (B)): ");
       Serial.println(encoder2Pos);
#endif       

         t0a=0;
         t1a=0;
         i=0;
         stepmax1=0;

#ifdef DOUBLEMOTOR
         t0b=0;
         t1b=0;
         stepmax2=0;
#endif
      }
       
      previoustime = steptimenow;
  }      
  
  if(Serial.available())
    process_line(); // Read from USB serial port and process commands entered
}

/********** Set PWM vaiue to TB6612 driver based on 'out' variable **************/
void pwmOut(int out, int motor1, int dir1, int dir2) {
   if(out<0) { analogWrite(motor1,abs(out)); digitalWrite(dir1,HIGH); digitalWrite(dir2,LOW); }
   else if(out>0){ analogWrite(motor1,abs(out)); digitalWrite(dir1,LOW); digitalWrite(dir2,HIGH); }
   else { analogWrite(motor1,0); digitalWrite(dir1,HIGH); digitalWrite(dir2,HIGH); }
  }
/********** count input step from STEP1 pin. triggered by interrupt ************/
void countStep1()
{ 
  #ifdef M1_DIR_REVERSE
    if(digitalRead(DIR1)==HIGH) 
         target1+= STEP_INPULSE;
    else 
         target1-= STEP_INPULSE;
  #else
    if(digitalRead(DIR1)==HIGH) 
         target1-= STEP_INPULSE;
    else 
         target1+= STEP_INPULSE;
  #endif
    
} 

#ifdef DOUBLEMOTOR 
/********** count input step from STEP2 pin. triggered by interrupt ************/
void countStep2()
{ 
  #ifdef M2_DIR_REVERSE
      if(digitalRead(DIR2)==HIGH) 
         target2+= STEP_INPULSE;
    else 
         target2-= STEP_INPULSE;
  #else
    if(digitalRead(DIR2)==HIGH) 
         target2-= STEP_INPULSE;
    else 
         target2+= STEP_INPULSE;
  #endif      
} 
#endif

/****Read and update encoder1 position. Triggered by interrupt ****************/
#ifdef STM32
void encoderINT1()
{
    int NewStep; 
    unsigned char New;

    New = (GPIOA-> IDR & M1ENCMASK)>>M1ENCSHIFT; //Read register for both encoder1 pulse
   
#endif

#ifdef NANO
ISR(PCINT0_vect)
{
    int NewStep; 
    unsigned char New;
    
    New = (PINB & M1ENCMASK)>>M1ENCSHIFT; //Read register for both encoder1 pulse
#endif

    NewStep = QEM[old1 * 4 + New]; //Use state machine to determine motor direction

    if(NewStep<0) //step in -ve direction
       OldDir1= true;   
    else if(NewStep == 1)
       OldDir1= false;    
    else if(NewStep == 2){ //error handling for a missed pulse
      Serial.print("m");
      if(OldDir1) 
       NewStep = -2;
    }         
    encoder1Pos+= NewStep;
    old1 = New;
}

#ifdef DOUBLEMOTOR
/****Read and update encoder2 position. Triggered by interrupt ****************/
#ifdef STM32
void encoderINT2()
{
    int NewStep; 
    unsigned char New;

    New = (GPIOB-> IDR & M2ENCMASK)>>M2ENCSHIFT;
#endif

#ifdef NANO
ISR(PCINT1_vect)
{
    int NewStep; 
    unsigned char New;

    New = PINC & M2ENCMASK;
#endif  

    NewStep = QEM[old2 * 4 + New];

    if(NewStep<0) //step in -ve direction
       OldDir2= true;   
    else if(NewStep == 1)
       OldDir2= false;    
    else if(NewStep == 2){ //error handling for a missed pulse
      Serial.print("M");
      if(OldDir2) 
       NewStep = -2;
    }         
    
    encoder2Pos+= NewStep;
    old2 = New;
}

#endif

/****Read from USB serial port and process commands entered **************/
void process_line() {

 int temp;
 long temptarget;
 char cmd = Serial.read();
 if(cmd>'Z') cmd-=32;
 switch(cmd) {
  case 'P':
        kp1=Serial.parseFloat(); 
        myPID1.SetTunings(kp1,ki1,kd1);
        printPos();
        break;
  case 'D':
        kd1=Serial.parseFloat();
        myPID1.SetTunings(kp1,ki1,kd1);
        printPos();
        break;
  case 'I': 
        ki1=Serial.parseFloat();
        myPID1.SetTunings(kp1,ki1,kd1);
        printPos();
        break;

#ifdef DOUBLEMOTOR
 case 'R':
        kp2=Serial.parseFloat(); 
        myPID2.SetTunings(kp2,ki2,kd2);
        printPos();
        break;
  case 'B':
        kd2=Serial.parseFloat();
        myPID2.SetTunings(kp2,ki2,kd2);
        printPos();
        break;
  case 'L': 
        ki2=Serial.parseFloat();
        myPID2.SetTunings(kp2,ki2,kd2);
        printPos();
        break;
#endif
        
  case 'T':
      temptarget = Serial.parseInt();
      target1= target1 + temptarget;
      Serial.print("\nStep test A(steps): ");
      Serial.println(temptarget);

      steptest = true;
      stepstarttime = millis();
      previoustime=0;
      break;
#ifdef DOUBLEMOTOR
  case 'U':
      temptarget = Serial.parseInt();
      target2= target2 + temptarget;
      Serial.print("\nStep test B(steps): ");
      Serial.println(temptarget);

      steptest = true;
      stepstarttime = millis();
      previoustime=0;
      break;
#endif
  case 'Z':
      target1= 0;
      encoder1Pos = 0;
#ifdef DOUBLEMOTOR
      target2= 0;
      encoder2Pos = 0;
#endif
      break;
  case 'X':
      temp = Serial.parseInt();
      if(temp==1){ 
        if(myPID1.GetDirection()){
          myPID1.SetControllerDirection(REVERSE);
          Serial.println("Encoder_1 direction: INVERT");
        }else{
          myPID1.SetControllerDirection(DIRECT);
          Serial.println("Encoder_1 direction: NON_INVERT");
        }
      }
      if(temp==2){ 
        if(myPID2.GetDirection()){
          myPID2.SetControllerDirection(REVERSE);
          Serial.println("Encoder_2 direction: NON_INVERT");
        }else{
          myPID2.SetControllerDirection(DIRECT);
          Serial.println("Encoder_2 direction: INVERT");
        }
      } 
      break;

  case 'S': printPos(); break;
  case 'H': help(); break;
  case 'W': writetoEEPROM(); break;  
 }
 while(Serial.read()!=10); // dump extra characters till LF is seen (you can use CRLF or just LF)
}

/****Print current position (with 's' input) ****************/
void printPos()
{
       Serial.print("Target 1: ");
       Serial.print(target1);
       Serial.print("\t\tEncoder Pos 1: ");
       Serial.print(encoder1Pos);
       Serial.print("\t\toutput1: ");
       Serial.println(output1);
       Serial.print("kp1: ");
       Serial.print(kp1);
       Serial.print("\t\tki1: ");
       Serial.print(ki1,5);
       Serial.print("\tkd1: ");
       Serial.println(kd1, 5);
       if(enable_active1)
          Serial.println("Enable_1 active");
       else
          Serial.println("Enable_1 off");
       if(myPID1.GetDirection()){
         Serial.println("Encoder_1 direction: NON_INVERT");
       }else{
         Serial.println("Encoder_1 direction: INVERT");
       } 

  #ifdef DOUBLEMOTOR
       Serial.print("Target 2: ");
       Serial.print(target2);
       Serial.print("\t\tEncoder Pos 2: ");
       Serial.print(encoder2Pos);
       Serial.print("\t\toutput2: ");
       Serial.println(output2);
       Serial.print("kp2: ");
       Serial.print(kp2);
       Serial.print("\t\tki2: ");
       Serial.print(ki2,5);
       Serial.print("\tkd2: ");
       Serial.println(kd2,5);
       
       if(enable_active2)
          Serial.println("Enable_2 active");
       else
          Serial.println("Enable_2 off");
       if(myPID2.GetDirection()){
         Serial.println("Encoder_2 direction: INVERT");
       }else{
         Serial.println("Encoder_2 direction: NON_INVERT");
       }    
  #endif           
 }
/****Print help messages (with 'h' input) ****************/
void help() {
 Serial.println(F("\nAvailable serial commands: (lines end with CRLF or LF)"));
 Serial.println(F("P123.34 sets proportional term to 123.34"));
 Serial.println(F("I123.34 sets integral term to 123.34"));
 Serial.println(F("D123.34 sets derivative term to 123.34"));

#ifdef DOUBLEMOTOR
 Serial.println(F("R123.34 sets proportional term for motor 2 to 123.34"));
 Serial.println(F("L123.34 sets integral term for motor 2 to 123.34"));
 Serial.println(F("B123.34 sets derivative term for motor 2 to 123.34"));
#endif
 Serial.println(F("S -- prints out current encoder, setpoint and PID values"));
 Serial.println(F("t20 -- step test, sets the target destination for motor A to 20 encoder pulses"));
#ifdef DOUBLEMOTOR
 Serial.println(F("u20 -- step test, sets the target destination for motor B to 20 encoder pulses"));
#endif
 Serial.println(F("Z -- reset target and encoder value to zero"));
 Serial.println(F("H -- will print this help message again"));
 Serial.println(F("W -- write PID values to EEPROM"));
}

/****Declaration for size of double variable ****************/
#ifdef STM32
 #define DBLSIZE 8
#endif
#ifdef NANO
 #define DBLSIZE 4
#endif

#ifdef DOUBLEMOTOR  
 #define LASTADDR DBLSIZE*6+1
 #define ENC_MEM 6
#else
 #define LASTADDR DBLSIZE*3+1
 #define ENC_MEM 3
#endif

/****Write PID values to EEPROM ****************/
void writetoEEPROM() { // keep PID set values in EEPROM so they are kept when arduino goes off
  char encoderdir;
  
  EEPROM.put(0, kp1);
  EEPROM.put(DBLSIZE, ki1);
  EEPROM.put(DBLSIZE*2, kd1);
  if(myPID1.GetDirection()){
    encoderdir = 0B00000001;
  }else{
    encoderdir = 0B00000000;
  }
  if(myPID2.GetDirection()){
    encoderdir = encoderdir|0B00000010;
  }
 #ifdef DOUBLEMOTOR  
  EEPROM.put(DBLSIZE*3, kp2);
  EEPROM.put(DBLSIZE*4, ki2);
  EEPROM.put(DBLSIZE*5, kd2);
 #endif

  EEPROM.put(DBLSIZE*ENC_MEM, encoderdir);
 
  double cks=0;
  for(int i=0; i<LASTADDR; i++) cks+=EEPROM.read(i);
  EEPROM.put(LASTADDR, cks);
  Serial.print("\nPID values stored to EEPROM. Checksum:");
  Serial.println(cks);
}

/****Read PID values from EEPROM ****************/
bool recoverPIDfromEEPROM() {
  double cks=0;
  double cksEE;
  char encoderdir;
  
  for(int i=0; i<LASTADDR; i++) cks+=EEPROM.read(i);
  EEPROM.get(LASTADDR, cksEE);
  
  if(cks==cksEE) {
    Serial.print("*** Found PID values on EEPROM: \t");
    EEPROM.get(0, kp1);
    EEPROM.get(DBLSIZE, ki1);
    EEPROM.get(DBLSIZE*2, kd1); 
#ifdef DOUBLEMOTOR  
    EEPROM.get(DBLSIZE*3, kp2);
    EEPROM.get(DBLSIZE*4, ki2);
    EEPROM.get(DBLSIZE*5, kd2);
#endif
    EEPROM.get(DBLSIZE*ENC_MEM, encoderdir);
    if(encoderdir & 0B00000001 ){
        myPID1.SetControllerDirection(DIRECT);
    }else{
        myPID1.SetControllerDirection(REVERSE);
    }
    if(encoderdir & 0B00000010 ){
        myPID2.SetControllerDirection(DIRECT);
    }else{
        myPID2.SetControllerDirection(REVERSE);
    }
      Serial.print("Checksum:");
      Serial.println(cksEE);
      return false;
  }
  else{ 
      Serial.print("**Bad checksum:");
      Serial.println(cksEE);
      return true;
  }
}
