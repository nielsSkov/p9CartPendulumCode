// before upload:

// (1) remember to add libraries containing the below three .h files: 
//    Sketch - Import Library - Add Library
// the libraries will be placed in 
//  Documents\Arduino\libraries
// NB: when a file is changed the corresponding library has to been added again

// (2) Remember
// Tools - Board - Arduino Due (Programming Port)
// Tools - Port - COM4 (Arduino Due (Programming Port))
// check that  "Serial.begin(115200)" below agree with baud on terminal

// (3) remember: max voltage 50, max current 5

#include <Joint.h> // main file 
#include <Utility.h> // sat and sign function
#include <looptime.h> // set time in loop

// Arduino stuff generated by windows
//#include <address.h>
//#include <adk.h>
//#include <confdescparser.h>
//#include <hid.h>
//#include <hidboot.h>
//#include <hidusagestr.h>
//#include <KeyboardController.h>
//#include <MouseController.h>
//#include <parsetools.h>
//#include <Usb.h>
//#include <usb_ch9.h>

// Definitions
#define SAMPLINGTIME 0.005 // s
#define ENABLEPENDUL 50
#define ENABLESLED 48

// Create joint objects
Joint sled(1,SAMPLINGTIME);
Joint pend1(2,SAMPLINGTIME);
Joint pend2(3,SAMPLINGTIME);


// Setup system
void setup() {
  // HACK
  // Pin 34 needs to be input because of external clock (Remove when corrected)
  pinMode(34,INPUT);
  
  ////////////////////////
  // Setup Serial
  ///////////////////////
  //Serial.begin(9600);
  Serial.begin(115200); // to be able to do Serial.print and still have time in loop
  ////////////////////////
  // Setup pin modes
  ////////////////////////
  // Data bus
  pinMode(40,INPUT);
  pinMode(41,INPUT);
  pinMode(42,INPUT);
  pinMode(43,INPUT);
  pinMode(44,INPUT);
  pinMode(45,INPUT);
  pinMode(46,INPUT);
  pinMode(47,INPUT);
    
  ///////////////////////
  // Setup pendulum system
  //////////////////////
  // Initialize joints
  sled.init();
  pend1.init();
  pend2.init();
  // Reset positions
  sled.resetPos();
  pend1.resetPos();
  pend2.resetPos();
  // Set DAC resolution
  analogWriteResolution(12);
  analogReadResolution(12);
  // Set output to zero
  sled.setOutput(0);
  pend1.setOutput(0);
  pend2.setOutput(0);
  // setup Enable motor driver outputs (FOR SAFTY REASONS, DO NOT CHANGE THE BELOW 6 LINES)
  digitalWrite(ENABLEPENDUL,LOW);
  digitalWrite(ENABLESLED,LOW);
  pinMode(ENABLEPENDUL, OUTPUT); 
  pinMode(ENABLESLED, OUTPUT);
  digitalWrite(ENABLEPENDUL,LOW);
  digitalWrite(ENABLESLED,LOW);
}

// Global variables
int setOutSled = 0;
int setOutPend1 = 0;
float velSled = 0;
float velPend1 = 0;
unsigned long timeinloop;
float posSled;
float posPend1;
int setOut = 0;
int dist = 0;
int deg = 0;
float rho = 0;
float v =0;

float ap8=512; // to read of power consumption of motors
int analogPin8 = 8; // sled
float ap9=512;
int analogPin9 = 9; // pendulum1


// set gains for the various controllers
float gain1 = 10; // case 2 and 3
float gain2 = 1;//0.5; // case 4
float gainp1 = 5000; // case 5
float gainp2 = 300; // case 5

float epsilon = 1;//1; // for |angle|<0.1 in theory gain >= 10*epsilon (working in case 3, for case 4 use gain >=0.5*epsilon)
float beta0 = .001; // >0

float ampl = 500; // amplitude of disturbance
float ref = 180;

// Time counter for case 6
unsigned long ref_last_change = 0;
int ref_counter = 0;
float changing_ref[] = {0, 10, 0, -10};
unsigned long ref_change_interval = 10000;

float g = -9.81; // gravity
float l = 0.305; // length
float km = 0.0934; // motor constant
float a = g/l;
float E = km/(l*l); 

float nm = 0.1; // nominal mass
float minm = 0.95;//0.099; // min mass
float maxm = 0.105;//0.101; // max mass
float nalpha = 0.00175; // nominal (current = alpha * input)
float minalpha = 0.0017; // min
float maxalpha = 0.0018; // max
float nkf = 0.5; // nominal friction
float minkf = 0.4;//0.49; // min friction
float maxkf = 0.5;//0.51; // max friction
float nG = nalpha/nm; // nominal
float minG = minalpha/maxm;
float maxG = maxalpha/minm;
float nb = nkf/nm; //
float maxb = maxkf/minm;
float maxd = km*maxalpha*ampl; // max disturbance



void loop() {

  timeinloop = looptime(SAMPLINGTIME*1000); //=SAMPLINGTIME*1000-(time spend in loop)=waiting time
  //  Serial.print("loop time: ");
   // Serial.println(timeinloop); 
  
  ////////////////////////////////////////////
  // Input from user on serial interface
  ////////////////////////////////////////////

  int input;
  if(Serial.available() > 0) {
      input = Serial.read();
      // stop all
       if(input == '0') {  
         setOut = 0;
       }
       // activate sled
       else if(input == '1') {
         setOut = 1;
       }
       // activate pendulum
       else if(input == '2') {
         setOut = 2;
       }
       // activate sled and pendulum (without disturbance taken into account)
       else if(input == '3') {
         setOut = 3;
       }
       // activate sled and pendulum (with disturbance taken into account)
       else if(input == '4') {
         setOut = 4;
       }
       // p-control
       else if(input == '5') {
         setOut = 5;
       }
       // p-control p-control for sys id
       else if(input == '6') {
         setOut = 6;
       }
       // reset angle counter
        else if(input == 'r') {
         pend1.resetPos();
      }
   }
  
  ////////////////////////////////////////////
  // Read positions
  ////////////////////////////////////////////

  posSled = sled.readPos();
  posPend1 = pend1.readPos();
  velSled = sled.readVel();
  velPend1 = pend1.readVel();
  
  // print various stats of pendulum
  //  Serial.print("Position of sled is: ");
  //  Serial.println(posSled,DEC);
  //  Serial.print("Velocity of sled is: ");
  //  Serial.println(velSled,DEC);
  //   Serial.print("Angular velocity of pendulum is: ");
  //  Serial.println(velSled,DEC);
  //  Serial.print("Position of pendulum is: ");
    Serial.println(posPend1,DEC);
  
  
  ////////////////////////////////////////////
  // Read power
  ///////////////////////////////////////////
  
//  ap8 = analogRead(analogPin8); // sled
//  Serial.println(ap8,DEC);
//  Serial.print(',');
//  ap9 = analogRead(analogPin9); // pendulum1
//  Serial.println(ap9,DEC);
  
  ///////////////////////////////////////////
  
  
  ////////////////////////////////////////////
  // Apply control
  ////////////////////////////////////////////
  
  // stop all
  if (setOut == 0) { 
    digitalWrite(ENABLEPENDUL,LOW);
    digitalWrite(ENABLESLED,LOW);   
    setOutSled = 0;
    setOutPend1 = 0;
  }
  // activate sled
  else if(setOut == 1){
    digitalWrite(ENABLEPENDUL,HIGH);
    digitalWrite(ENABLESLED,HIGH);
    setOutSled = ampl*sin((float)dist*SAMPLINGTIME*10);
   // Serial.println(setOutSled,DEC);
    dist++;
   // Serial.println(dist,DEC);
  }
  // activate pendulum
  else if(setOut == 2){
    digitalWrite(ENABLEPENDUL,HIGH);
    digitalWrite(ENABLESLED,HIGH);
     dist++;
   // Serial.println(dist,DEC);
    velPend1 = velPend1*0.0175;
    posPend1 = (posPend1+ref)*0.0175;
    setOutPend1 = (nm*l*l/(km*nalpha))*(-gain1*velPend1-(-a*sin(posPend1)-nb*velPend1)-sat(velPend1+gain1*posPend1,epsilon));
    // Serial.println(setOutPend1,DEC);
  }
  // activate sled and pendulum (without disturbance taken into account) 
  else if(setOut == 3){
    digitalWrite(ENABLEPENDUL,HIGH);
    digitalWrite(ENABLESLED,HIGH);
    setOutSled = ampl*sin((float)dist*SAMPLINGTIME*10);
   // Serial.println(setOutSled,DEC);
    dist++;
   // Serial.println(dist,DEC);
    velPend1 = velPend1*0.0175;
    posPend1 = (posPend1+ref)*0.0175;
    setOutPend1 = (nm*l*l/(km*nalpha))*(-gain1*velPend1-(-a*sin(posPend1)-nb*velPend1)-sat(velPend1+gain1*posPend1,epsilon));
    // Serial.println(setOutPend1,DEC);
  }
  // activate sled and pendulum (with disturbance taken into account) 
  else if(setOut == 4){
    digitalWrite(ENABLEPENDUL,HIGH);
    digitalWrite(ENABLESLED,HIGH);
    setOutSled = ampl*sin((float)dist*SAMPLINGTIME*10);
   // Serial.println(setOutSled,DEC);
    dist++;
   // Serial.println(dist,DEC);
    velPend1 = velPend1*0.0175;
    posPend1 = (posPend1+ref)*0.0175;
//    rho = 1/minG*(abs(maxb)*abs(velPend1)+abs(maxd)+abs(a*(1-maxG/nG))+abs((1-maxG/nG)*gain2)*abs(velPend1));
//    rho = 1/minG*(abs(maxb)*abs(velPend1)+abs(maxd)+abs(a*(1-maxG/nG)*sin(posPend1))+abs((1-maxG/nG)*gain2)*abs(velPend1));
    rho = 1/minG*abs(maxb*velPend1+maxd+a*(1-maxG/nG)*sin(posPend1)+(1-maxG/nG)*gain2*velPend1);
    v = -(rho + beta0)*sat(velPend1+gain2*posPend1,epsilon);
    setOutPend1 = 1/E*(-1/nG*(-a*sin(posPend1)+gain2*velPend1)+v);
    // Serial.println(setOutPend1,DEC);
  }
   // p-control
  else if(setOut == 5){
    digitalWrite(ENABLEPENDUL,HIGH);
    digitalWrite(ENABLESLED,HIGH);
    setOutSled = ampl*sin((float)dist*SAMPLINGTIME*10);
   // Serial.println(setOutSled,DEC);
    dist++;
   // Serial.println(dist,DEC);
    velPend1 = velPend1*0.0175;
    posPend1 = (posPend1-ref)*0.0175;
    setOutPend1 = -gainp1*(posPend1)-gainp2*velPend1;
    // Serial.println(setOutPend1,DEC);
  }
  
   // p-control for sys id
  else if(setOut == 6){
    if ((millis() - ref_last_change) > ref_change_interval)
    {
      ref_last_change = millis();
      ref_counter = (ref_counter +1) % 4;
    }
    digitalWrite(ENABLEPENDUL,HIGH);
    digitalWrite(ENABLESLED,HIGH);
    setOutSled = ampl*sin((float)dist*SAMPLINGTIME*10);
   // Serial.println(setOutSled,DEC);
    dist++;
   // Serial.println(dist,DEC);
    velPend1 = velPend1*0.0175;
    posPend1 = (posPend1-changing_ref[ref_counter])*0.0175;
    setOutPend1 = -gainp1*(posPend1)-gainp2*velPend1;
    // Serial.println(setOutPend1,DEC);
  }
  
  ////////////////////////////////////////////
  // Set outputs
  ////////////////////////////////////////////

  sled.setOutput(setOutSled);
  //  Serial.print("Value of setOutSled: ");
  //  Serial.println(setOutSled,DEC);
  pend1.setOutput(setOutPend1);
  //  Serial.print("Value of setOutPend1: ");
  //  Serial.println(setOutPend1,DEC);
  
  ////////////////////////////////////////////
  // Safety
  ////////////////////////////////////////////
  
  //sled.safetyStop();
  //pend1.safetyStop();
  
}

