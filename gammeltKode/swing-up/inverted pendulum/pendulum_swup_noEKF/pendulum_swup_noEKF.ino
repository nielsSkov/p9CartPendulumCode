
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

#include <Joint1.h> // main file 
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
Joint sled(1, SAMPLINGTIME);
Joint pend1(2, SAMPLINGTIME);
Joint pend2(3, SAMPLINGTIME);


// Setup system
void setup() {
  // HACK
  // Pin 34 needs to be input because of external clock (Remove when corrected)
  pinMode(34, INPUT);

  ////////////////////////
  // Setup Serial
  ///////////////////////
  //Serial.begin(9600);
  Serial.begin(115200); // to be able to do Serial.print and still have time in loop
  ////////////////////////
  // Setup pin modes
  ////////////////////////
  // Data
  pinMode(40, INPUT);
  pinMode(41, INPUT);
  pinMode(42, INPUT);
  pinMode(43, INPUT);
  pinMode(44, INPUT);
  pinMode(45, INPUT);
  pinMode(46, INPUT);
  pinMode(47, INPUT);

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
  digitalWrite(ENABLEPENDUL, LOW);
  digitalWrite(ENABLESLED, LOW);
  pinMode(ENABLEPENDUL, OUTPUT);
  pinMode(ENABLESLED, OUTPUT);
  digitalWrite(ENABLEPENDUL, LOW);
  digitalWrite(ENABLESLED, LOW);
}

// Global variables
float setOutSled = 0;
int setOutPend1 = 0;
float velSled = 0;
float velPend1 = 0;
unsigned long timeinloop;
float posSled;
float posPend1;
//float posPend1_new;
float xI;
float xI_old = 0;
int setOut = 0;
int dist = 0;
int deg = 0;
float rho = 0;
float v = 0;

float ap8 = 512; // to read of power consumption of motors
int analogPin8 = 8; // sled
float ap9 = 512;
int analogPin9 = 9; // pendulum1

//------------------------------------------------------------------------------
//----Linear controller gains

float gain_x1 =  -30.61;  //-88.3;//-30.61;  // position
float gain_x2 =  -121.4;  //-193.3;//-121.4;  // angle
float gain_x3 =  -22.6;  //-50.98;// -22.6;  //velocity
float gain_x4 =  -17.13;  //  -32.4;//-17.13;  // angular velocity
float gain_xI =   0;  //-52.91;//     0;  // integral action
float convert_Fm_to_u = 55.47;//1/(km*beta/r)=1/(0.0934*0.00579/0.03)=1/0.01802, conversion factor Fm->u
float eta=1/1.44;
float eta_2=1/5.76;
float gama_r=0.00983;
float gamaL=0.64;
float gamaR=0.94;

//-------------------------------------------------------------------------------
//---- Swing up gains and variables
float K_d = 2;
float K_e = 1;
float K_v = 5.5;
float K_x = 14;
float P_off;
float En;
float swalpha1_R;
float swbeta1;
float swalpha1_L;
float v1;
float v1_max;

//Time counter for case 1
unsigned long ref_last_posChange = 0;
int ref_posCounter = 0;
//float changing_ref_sledPosition[] = {0, 150, 0, -150};
float changing_ref_sledPosition[] = {100, -100};
unsigned long ref_posChange_interval = 2000; //20000;//2000;
//----------------------------------------------------------------------------------
//
float gain1 = 10; // case 2 and 3
float gain2 = 1;//0.5; // case 4
float gainp1 = 5000; // case 5
float gainp2 = 300; // case 5

float epsilon = 1;//1; // for |angle|<0.1 in theory gain >= 10*epsilon (working in case 3, for case 4 use gain >=0.5*epsilon)
float beta0 = 0.001; // >0

float ampl = 1; //00 // amplitude of disturbance
float ref = 180; //180; // put 180 during swinging up

// Time counter for case 6
unsigned long ref_last_change = 0;
int ref_counter = 0;
float changing_ref[] = {0, 20, 0, -20};
float changing_ref_ol[] = {0, 160, 0, -160};
unsigned long ref_change_interval = 20000;
float gainps1 = 2000; // case 6
float gainps2 = 20; //500; // case 6

float m = 0.251; //0.251 %mass of pendulum
float M = 3.15; //mass of the cart
float gamma_rot = 0.00983; //rotational viscous friction
float g = 9.81; // gravity
float l = 0.305; // length
float km = 0.0934; // motor constant
float a = g / l;
float E = km / (l*l);

float nm = 0.1; // nominal mass
float minm = 0.95;//0.099; // min mass
float maxm = 0.105;//0.101; // max mass
float nalpha = 0.00175; // nominal (current = alpha * input)
float minalpha = 0.0017; // min
float maxalpha = 0.0018; // max
float nkf = 0.5; // nominal friction
float minkf = 0.4;//0.49; // min friction
float maxkf = 0.5;//0.51; // max friction
float nG = nalpha / nm; // nominal
float minG = minalpha / maxm;
float maxG = maxalpha / minm;
float nb = nkf / nm; //
float maxb = maxkf / minm;
float maxd = km * maxalpha * ampl; // max disturbance



void loop() {

  timeinloop = looptime(SAMPLINGTIME * 1000); //looptime(SAMPLINGTIME*1000); //=SAMPLINGTIME*1000-(time spend in loop)=waiting time
  //  Serial.print("loop time: ");
  // Serial.println(timeinloop);

  ////////////////////////////////////////////
  // Input from user on serial interface
  ////////////////////////////////////////////

  int input;
  if (Serial.available() > 0) {
    input = Serial.read();
    // stop all
    if (input == '0') {
      setOut = 0;
    }
    // activate sled
    else if (input == '1') {
      setOut = 1;
    }
    // activate pendulum
    else if (input == '2') {
      setOut = 2;
    }
    // activate sled and pendulum (without disturbance taken into account)
    else if (input == '3') {
      setOut = 3;
    }
    // activate sled and pendulum (with disturbance taken into account)
    else if (input == '4') {
      setOut = 4;
    }
    // p-control
    else if (input == '5') {
      setOut = 5;
    }
    // p-control p-control for sys id
    else if (input == '6') {
      setOut = 6;
    }
    // open loop
    else if (input == '7') {
      setOut = 7;
    }
    // reset angle counter
    else if (input == 'r') {
      pend1.resetPos();
      sled.resetPos();
      xI_old = 0;
    }
  }

  ////////////////////////////////////////////
  // Read positions
  ////////////////////////////////////////////

  posSled = -sled.readPos();
  posPend1 = pend1.readPos();
  velSled = -sled.readVel();
  velPend1 = pend1.readVel();
  //xI = xI_old + SAMPLINGTIME * ((posSled - changing_ref_sledPosition[ref_posCounter]) / 1000);
  xI = xI_old + SAMPLINGTIME * ((posSled) / 1000);
  xI_old = xI;

  // print various stats of pendulum
  //Serial.print("Position of sled is: ");
  Serial.print(posSled, DEC); //<--
  Serial.print(','); //<--

  //  Serial.print("Velocity of sled is: ");
  Serial.print(velSled, DEC); //<--
  Serial.print(','); //<--

  //   Serial.print("Angular velocity of pendulum is: ");
  //  Serial.println(velPend1,DEC);
  Serial.print(velPend1, DEC); //<--
  Serial.print(',');  //<--
  //Serial.print
  //  Serial.print("Position of pendulum is: ");
Serial.println(posPend1, DEC); //<--
   //Serial.print(posPend1,DEC);
  // Serial.print(',');



  ////////////////////////////////////////////
  // Read power
  ///////////////////////////////////////////

  //ap8 = analogRead(analogPin8); // sled
  //Serial.println(ap8,DEC);
  //-->Serial.print(','); //<--
  //-->ap9 = analogRead(analogPin9); // pendulum1 //<--
//  Serial.println(ap9,DEC);
//  -->Serial.print(ap9,DEC); //<--
//  -->Serial.print(','); //<--

  ///////////////////////////////////////////


  ////////////////////////////////////////////
  // Apply control
  ////////////////////////////////////////////

  // stop all
  if (setOut == 0) {
    digitalWrite(ENABLEPENDUL, LOW);
    digitalWrite(ENABLESLED, LOW);
    setOutSled = 0;
    setOutPend1 = 0;
  }
  // activate sled
  else if (setOut == 1) {

    // //   if ((millis() - ref_last_posChange) > ref_posChange_interval) //<--------comment this----------------------
// //   {                                                             //<------------comment this---------------------
// //     ref_last_posChange = millis();                              //<------------comment this---------------------
// //     ref_posCounter = (ref_posCounter + 1) % 4;                  //<------------comment this---------------------
// //     motor_value = - motor_value;                                //<--needed only to measure motor response -----
// //   }  

    //
    digitalWrite(ENABLEPENDUL, HIGH);
    digitalWrite(ENABLESLED, HIGH);
    velPend1 = velPend1 * 0.0175;
    posPend1 = (posPend1 + ref) * 0.0175;
    posSled = (posSled) * 0.001; //mm --> m
    //posSled = (posSled - changing_ref_sledPosition[ref_posCounter]) * 0.001; //mm --> m
    
//------------------motor time costant------------------------
//setOutSled = -convert_Fm_to_u * motor_value;


//--------- linear controller -----------
   
     if ((posPend1) <= (10*0.0175)){               
          setOutSled = - convert_Fm_to_u * ((posSled*gain_x1) + (posPend1*gain_x2) + (velSled*gain_x3) + (velPend1*gain_x4) + (xI * gain_xI));
          }
     else if((posPend1) >= (350*0.0175)) {
           setOutSled = - convert_Fm_to_u * ((posSled*gain_x1) + ((posPend1-6.28)*gain_x2) + (velSled*gain_x3) + (velPend1*gain_x4) + (xI * gain_xI));
      }
   
   else{
    //============================== swing up  controller   ==================================
  P_off = 3*m*g*l;    
 En = 0.5 * (M + m) * (velSled) * (velSled) + m * l * cos(posPend1) * (velSled) * (velPend1) + 0.5 * m * l * l * (velPend1) * (velPend1) +  m * g * l * (cos(posPend1) - 1) + P_off;
  swalpha1_R = m*sin(posPend1)* (l*(velPend1)*(velPend1)-g*cos(posPend1)) - gamaR*(velSled) + (1 / l) *gama_r * cos(posPend1) * (velPend1);
  swalpha1_L = m*sin(posPend1)* (l*(velPend1)*(velPend1)-g*cos(posPend1)) - gamaL*(velSled) + (1 / l) *gama_r * cos(posPend1) * (velPend1);
  swbeta1 = M + m*sin(posPend1)*(posPend1);
  v1 = -(gama_r * (velPend1) * (M + m)) / (m * l * cos(posPend1));
  v1_max = -(gama_r * (velPend1) * (M + m)) / (m * l * cos(82*0.0175));
 
if (velSled<=0) {                                   
  
   setOutSled = convert_Fm_to_u* (
              //(-K_d * (velSled) - K_x * (posSled) - K_v*swalpha1/swbeta1) / (K_e*(En-P_off) + K_v/swbeta1));                              // ----without v
          eta*((-K_d * (velSled) - K_x * (posSled) - K_v*swalpha1_R/swbeta1) / (K_e*(En-P_off) + K_v/swbeta1))  + eta_2* sat((v1/v1_max),(1/v1_max)));   // ------> with  term v
   }

 else{
            
       setOutSled = convert_Fm_to_u* (
              //(-K_d * (velSled) - K_x * (posSled) - K_v*swalpha1/swbeta1) / (K_e*(En-P_off) + K_v/swbeta1));                              // ----without v
          eta*((-K_d * (velSled) - K_x * (posSled) - K_v*swalpha1_L/swbeta1) / (K_e*(En-P_off) + K_v/swbeta1))  + eta_2* sat((v1/v1_max),(1/v1_max)));   // ------> with  term v
       
       }
   }


//===================================================================================================================================================================

    dist++;
    // Serial.println(dist,DEC);
  }
  // activate pendulum
  else if (setOut == 2) {
    digitalWrite(ENABLEPENDUL, HIGH);
    digitalWrite(ENABLESLED, HIGH);
    //dist++;
    // Serial.println(dist,DEC);
    velPend1 = velPend1 * 0.0175;
    posPend1 = (posPend1 + ref) * 0.0175;
    setOutPend1 = dist * 0.1; //0.88;//(nm*l*l/(km*nalpha))*(-gain1*velPend1-(-a*sin(posPend1)-nb*velPend1)-sat(velPend1+gain1*posPend1,epsilon));
    //Serial.print( setOutPend1,DEC);
    //Serial.print(',');
    // Serial.println(setOutPend1,DEC);
    dist++;
  }
  // activate sled and pendulum (without disturbance taken into account)
  else if (setOut == 3) {
    digitalWrite(ENABLEPENDUL, HIGH);
    digitalWrite(ENABLESLED, HIGH);
    setOutSled = ampl * sin((float)dist * SAMPLINGTIME * 10);
    // Serial.println(setOutSled,DEC);
    dist++;
    // Serial.println(dist,DEC);
    velPend1 = velPend1 * 0.0175;
    posPend1 = (posPend1 + ref) * 0.0175;
    setOutPend1 = (nm * l * l / (km * nalpha)) * (-gain1 * velPend1 - (-a * sin(posPend1) - nb * velPend1) - sat(velPend1 + gain1 * posPend1, epsilon));
    // Serial.println(setOutPend1,DEC);
  }
  // activate sled and pendulum (with disturbance taken into account)
  else if (setOut == 4) {
    digitalWrite(ENABLEPENDUL, HIGH);
    digitalWrite(ENABLESLED, HIGH);
    setOutSled = ampl * sin((float)dist * SAMPLINGTIME * 10);
    // Serial.println(setOutSled,DEC);
    dist++;
    // Serial.println(dist,DEC);
    velPend1 = velPend1 * 0.0175;
    posPend1 = (posPend1 + ref) * 0.0175;
    //    rho = 1/minG*(abs(maxb)*abs(velPend1)+abs(maxd)+abs(a*(1-maxG/nG))+abs((1-maxG/nG)*gain2)*abs(velPend1));
    //    rho = 1/minG*(abs(maxb)*abs(velPend1)+abs(maxd)+abs(a*(1-maxG/nG)*sin(posPend1))+abs((1-maxG/nG)*gain2)*abs(velPend1));
    //    rho = 1/minG*abs(maxb*velPend1+maxd+a*(1-maxG/nG)*sin(posPend1)+(1-maxG/nG)*gain2*velPend1);
    v = -(rho + beta0) * sat(velPend1 + gain2 * posPend1, epsilon);
    setOutPend1 = 1 / E * (-1 / nG * (-a * sin(posPend1) + gain2 * velPend1) + v);
    // Serial.println(setOutPend1,DEC);
  }
  // p-control
  else if (setOut == 5) {
    digitalWrite(ENABLEPENDUL, HIGH);
    digitalWrite(ENABLESLED, HIGH);
    setOutSled = ampl * sin((float)dist * SAMPLINGTIME * 10);
    // Serial.println(setOutSled,DEC);
    dist++;
    // Serial.println(dist,DEC);
    velPend1 = velPend1 * 0.0175;
    posPend1 = (posPend1 - ref) * 0.0175;
    setOutPend1 = -gainp1 * (posPend1) - gainp2 * velPend1;
    // Serial.println(setOutPend1,DEC);
  }

  // p-control for sys id
  else if (setOut == 6) {
    if ((millis() - ref_last_change) > ref_change_interval)
    {
      ref_last_change = millis();
      ref_counter = (ref_counter + 1) % 4;
    }
    digitalWrite(ENABLEPENDUL, HIGH);
    digitalWrite(ENABLESLED, HIGH);
    // setOutSled = ampl*sin((float)dist*SAMPLINGTIME*10);
    // Serial.println(setOutSled,DEC);
    dist++;
    // Serial.println(dist,DEC);
    velPend1 = velPend1 * 0.0175;
    posPend1 = (posPend1 - changing_ref[ref_counter]) * 0.0175;
    setOutPend1 = -gainps1 * (posPend1) - gainps2 * velPend1;
    // Serial.println(setOutPend1,DEC);
  }

  // open loop
  else if (setOut == 7) {
    if ((millis() - ref_last_change) > ref_change_interval)
    {
      ref_last_change = millis();
      ref_counter = (ref_counter + 1) % 4;
    }
    digitalWrite(ENABLEPENDUL, HIGH);
    digitalWrite(ENABLESLED, HIGH);
    // setOutSled = ampl*sin((float)dist*SAMPLINGTIME*10);
    // Serial.println(setOutSled,DEC);
    dist++;
    // Serial.println(dist,DEC);
    // velPend1 = velPend1*0.0175;
    // posPend1 = (posPend1-changing_ref[ref_counter])*0.0175;
    // setOutPend1 = -gainps1*(posPend1)-gainps2*velPend1;
    setOutPend1 = changing_ref_ol[ref_counter];
    // Serial.println(setOutPend1,DEC);
  }
  ////////////////////////////////////////////
  // Set outputs
  ////////////////////////////////////////////

  sled.setOutput(setOutSled);
  //  Serial.print("Value of setOutSled: ");
  //  Serial.println(setOutSled,DEC);
 // Serial.println(setOutSled,DEC); //------>comment if you are not printing the control law; it has to be devided by convert_Fm_to_u before to plot it!!!!
  pend1.setOutput(setOutPend1);
  //  Serial.print("Value of setOutPend1: ");
  //-->Serial.println(setOutPend1,DEC);

  ////////////////////////////////////////////
  // Safety
  ////////////////////////////////////////////

  //sled.safetyStop();
  //pend1.safetyStop();

}

