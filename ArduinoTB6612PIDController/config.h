#define NANO                //STM32 for STM32 blue pill. NANO for Arduino NANO
#define DOUBLEMOTOR         //uncomment for 2 motors**
//#define MAXPULSE     1    //Provide a maximum pulse at a setpoint change** 
//#define ENABLE_IN_INVERT  //Default is Enable input pin LOW is active**
//#define ENABLE_OUT_LOW    //For other drivers. TB6612 need a active HIGH enable signal
#define SHARE_EN_PIN        //both motor share Motor 1 Enable pin**
//#define M1_DIR_REVERSE    //motor 1 direction reverse
//#define M2_DIR_REVERSE    //motor 2 direction reverse

#define KP1           1.38         //PID Proportion term for Motor1 //kp1=0.93,ki1=28,kd1=0.0028;
#define KI1           138          //PID Integral term for Motor1
#define KD1           0.004        //PID Differrential term for Motor1
#define KP2           0.6          //PID Proportion term for Motor2 //kp2=12,ki2=8,kd2=0.12;
#define KI2           35           //PID Integral term for Motor2
#define KD2           0.0035       //PID Differrential term for Motor2

#define STEP_INPULSE  5            //Number of steps per STEP input pulse

#ifdef STM32
 #define M1            PA3   // To TB6612 PWMA input for Motor 1
 #define MA1           PA1   // To TB6612 AIN1 input for Motor 1
 #define MA2           PA2   // To TB6612 AIN2 input for Motor 1
 #define Q1            PA6   // To Motor 1 encoder pulses. Tied to M1ENCMASK
 #define Q2            PA7   // To Motor 1 encoder pulses. Tied to M1ENCMASK
 #define M1ENCMASK     0B11000000  //Q1 &Q2 register
 #define M1ENCSHIFT    6           //shift Q1 &Q2 register

 #define DIR1          PB11   //Direction for input for Motor 1
 #define STEP1         PB10   //Step input for Motor 1, trigger on rising edge 
 #define ENABLE1       PB12   //Enable PIN for Motor 1
 #define M1_MAX        PB1    //Motor 1 MAX LED output

 #define MOTORENABLE   PC15   //Output to Standby pin on TB6612
 #define STOP_PRINT    PA4    //Motor Error eg. runaway or stuck LED output

 #ifdef DOUBLEMOTOR
   #define M5          PA0    // To TB6612 PWMB input for Motor 2
   #define MB1         PC13   // To TB6612 BIN2 input for Motor 2
   #define MB2         PC14   // To TB6612 BIN1 input for Motor 2
   #define Q5          PB4    // To Motor 2 encoder pulses. Tied to M2ENCMASK
   #define Q6          PB5    // To Motor 2 encoder pulses. Tied to M2ENCMASK
   #define M2ENCMASK   0B11000   //Q5 &Q6 register
   #define M2ENCSHIFT  3         //shift Q5 &Q6 register

   #define DIR2        PB14   //Direction for input for Motor 2
   #define STEP2       PB15   //Step input for Motor 2, trigger on rising edge
   #ifdef SHARE_EN_PIN
    #define ENABLE2     ENABLE1   //Shared Enable PIN
   #else 
    #define ENABLE2     PB13      //Enable PIN for Motor 2
   #endif
   #define M2_MAX      PA8        //Motor 2 MAX LED output 
   #define SAMPLETIME  100        // PID sample time 100us for single motor
 #else 
   #define SAMPLETIME  50         // PID sample time 50us for double motor
 #endif
 #define ARRYSIZE      120        //Array size to store step test values
 #define MAXOUTPUT     64989      //STM32 have an odd problem where -65535 or 65535 does not give max output
 #define MAXOUTALM     63800  //Alarm when PID output is at above this value. Will light up M*_MAX LED
 #define MINSTEP       500    //If motor move less than these step after ALMTIME, declare motor stuck
 #define MAXSTEP       10000  //If motor move more than these step after ALMTIME, declaring runaway motor
#endif

#ifdef NANO
 #define M1            11     // To TB6612 PWMB input for Motor 1
 #define MA1           A3     // To TB6612 BIN1 input for Motor 1
 #define MA2           A4     // To TB6612 BIN2 input for Motor 1
 #define Q1            9      // To Motor 1 encoder pulses. Tied to M1ENCMASK
 #define Q2            10     // To Motor 1 encoder pulses. Tied to M1ENCMASK
 #define M1ENCMASK   0B00000110   //Q1 &Q2 register
 #define M1ENCSHIFT    1          //shift Q1 &Q2 register

 #define DIR1          4      //Direction for input for Motor 1
 #define STEP1         3      //Step input for Motor 1, trigger on rising edge 
 #define ENABLE1       6      //Enable PIN for Motor 1
 #define M1_MAX        12     //Motor 1 MAX LED output

 #define MOTORENABLE   A2     //Output to Standby pin on TB6612
 #define STOP_PRINT    A7     //Motor Error eg. runaway or stuck output

 #ifdef DOUBLEMOTOR
   #define M5          5      // To TB6612 PWMA input for Motor 2
   #define MB1         8      // To TB6612 AIN2 input for Motor 2
   #define MB2         7      // To TB6612 AIN1 input for Motor 2
   #define Q5          A0     // To Motor 2 encoder pulses. Tied to M2ENCMASK
   #define Q6          A1     // To Motor 2 encoder pulses. Tied to M2ENCMASK
   #define M2ENCMASK   0B00000011   //Q5 &Q6 register

   #define DIR2        A5     //Direction for input for Motor 2
   #define STEP2       2      //Cannot change. Tied to PIN2 interrupt. Step input for Motor 2, trigger on rising edge
   #ifdef SHARE_EN_PIN
    #define ENABLE2    ENABLE1    //Shared Enable PIN
   #else 
    #define ENABLE2    A6         //Enable PIN for Motor 2
   #endif
   #define M2_MAX      13         //Motor 2 MAX LED output 
   
   #define SAMPLETIME  350        // PID sample time 350us for double motor
   #define ARRYSIZE    30         //Array size to store step test values
 #else 
   #define SAMPLETIME  150        // PID sample time 150us for single motor
   #define ARRYSIZE    100        //Array size to store step test values
 #endif
   #define MAXOUTPUT   255        //Max PWM output of Arduino Nano
   #define MAXOUTALM   250        //Alarm when PID output is at above this value. Will light up M*_MAX LED
   #define MINSTEP     200        //If motor move less than these step after ALMTIME, declare motor stuck
   #define MAXSTEP     500        //If motor move more than these step after ALMTIME, declaring runaway motor
#endif

#define ALMTIME       150    //Time at max output to trigger alarm 
#define HALFARRYSIZE  ARRYSIZE/2
