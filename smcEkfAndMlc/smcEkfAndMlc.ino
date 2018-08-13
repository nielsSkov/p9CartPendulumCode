// before upload:

// (1) remember to add libraries containing the below three .h files:
//    Sketch - Import Library - Add Library
// the libraries will be placed in
//  Documents\Arduino\libraries
// NB: when a file is changed the corresponding library has to been added again

// (2) Remember
// Tools - Board - Teensy 3.6 (Programming Port)
// Tools - Port - COM# (Teensy 3.6 (Programming Port))
// check that  "Serial.begin(115200)" below agree with baud on terminal

// (3) remember: max voltage 50, max current 5

#include <Joint_teensy.h> // main file
//#include <Utility.h> // sat and sign function
//#include <looptime.h> // set time in loop

// EKF libraries
#include "EKF.h"
#include "EKF_initialize.h"

// MLC libraries
#include "run_mlc.h"
#include "run_mlc_initialize.h"


// Definitions
#define SAMPLINGTIME 0.00667 // s
#define ENABLEPENDUL 11//50
#define ENABLESLED 12//48

// Create joint objects
Joint sled(1, SAMPLINGTIME);
Joint pend1(2, SAMPLINGTIME);

// Global variables
float setOutSled = 0;
float setOutPend1 = 0;
float velSled = 0;
float velPend1 = 0;
float posSled;
float posPend1;
int setOut = 0;

unsigned long current_time = 0;
unsigned long last_time = 0;
unsigned long  loop_time = 0;
unsigned long time_now = 0;

float I_error = 0;

// For initializing the EKF
bool first_run = true;
double x_est_correction[4];
double x_init[4];
float setOutSledNoComp = 0;

// Init control
bool wait_for_position = false;
#define init_ref 0.4
#define start_angle 0.15
#define start_pos 0.1
#define stop_angle 0.6

float cart_ref = init_ref; // initial reference
float cart_ref_goal = init_ref;

unsigned long start_3 = 0;
int i = 0;

// MLC init variables
int MLC_loop_count = 0;


// Setup system
void setup() {
  ////////////////////////
  // Setup Serial
  ///////////////////////
  //Serial.begin(9600);
  Serial.begin(115200); // to be able to do Serial.print and still have time in loop
  ////////////////////////
  // Setup pin modes
  ////////////////////////
  // Data bus
  pinMode(25, INPUT);
  pinMode(26, INPUT);
  pinMode(27, INPUT);
  pinMode(28, INPUT);
  pinMode(29, INPUT);
  pinMode(30, INPUT);
  pinMode(31, INPUT);
  pinMode(32, INPUT);

  ///////////////////////
  // Setup pendulum system
  //////////////////////
  // Initialize joints
  sled.init();
  pend1.init();
  // Reset positions
  sled.resetPos();
  pend1.resetPos();
  // Set DAC resolution
  analogWriteResolution(12);
  analogReadResolution(12);
  // Set output to zero
  sled.setOutput(0,1);
  pend1.setOutput(0,1);
  // setup Enable motor driver outputs (FOR SAFTY REASONS, DO NOT CHANGE THE BELOW 6 LINES)
  digitalWrite(ENABLEPENDUL, LOW);
  digitalWrite(ENABLESLED, LOW);
  pinMode(ENABLEPENDUL, OUTPUT);
  pinMode(ENABLESLED, OUTPUT);
  digitalWrite(ENABLEPENDUL, LOW);
  digitalWrite(ENABLESLED, LOW);

  current_time = micros();
  EKF_initialize();
  run_mlc_initialize();

}

void loop() {
  ////////////////////////////////////////////
  // Input from user on serial interface
  ////////////////////////////////////////////

  //int input;
  String input;
  if (Serial.available() > 0) {
    //input = Serial.read();
    input = Serial.readStringUntil('\n');
    // stop all
    if (input == "0") {
      // stop motor
      setOut = 0;
      cart_ref = init_ref;
      P_correction_EKF_not_empty_init(); // Reset EKF
      wait_for_position = false; // Reset control such that it waits for the correct position
      cart_ref_goal = init_ref;
      //Serial.println("Input = 0");
    } else if (input == "1") {
      // Sliding mode control with constant beta
      setOut = 1;
      time_now = micros();
      //Serial.println("Input = 1");
    } else if (input == "2") {
      // Sliding mode control with varying beta
      setOut = 2;
      //Serial.println("Input = 2");
    } else if (input == "3") {
      // Machine learning control
      setOut = 3;
      i = 0;
      //Serial.println("Input = 3");
    } else if (input == "r") {
      // Reset variables
      pend1.resetPos();
      sled.resetPos();

      P_correction_EKF_not_empty_init(); // Reset EKF
      wait_for_position = false; // Reset control such that it waits for the correct position
      cart_ref = init_ref;
      cart_ref_goal = init_ref;

      Serial.println("Input = r");
    } else if (input.startsWith("ref=")) {
      // change reference
      int ref_pos = input.indexOf('=');
      String ref_string = input.substring(ref_pos + 1);
      cart_ref_goal = ref_string.toFloat();
      //Serial.print("Cart reference changed to ref=");
      //Serial.println(ref_string);
    } else {
      Serial.println("ERROR: Wrong command selected.");
    }
    /*digitalWrite(ENABLESLED,HIGH);
      int tmp = input-'0';
      Serial.println(-tmp*100);
      setOutSled = -tmp*100;*/
  }

  ////////////////////////////////////////////
  // Read positions
  ////////////////////////////////////////////

  posSled = sled.readPos();
  posPend1 = pend1.readPos();
  posPend1 = (posPend1 + PI);
  velSled = sled.readVel();
  velPend1 = pend1.readVel();
  unsigned long time_stamp = micros();

  ////////////////////////////////////////////
  // Apply control
  ////////////////////////////////////////////

  if (fabs(cart_ref_goal - cart_ref) > 0.001) {
    if ((cart_ref_goal - cart_ref) > 0) {
      cart_ref += 0.1 / 150;
    } else {
      cart_ref -= 0.1 / 150;
    }
  } else {
    cart_ref = cart_ref_goal;
  }
  // ---------------Model parameters------------------------//
      float r_pulley = 0.028;
      float K_t = 0.0934;
      float B_c_pos = 3.0212 - 0.5;
      float B_c_neg = 2.7464;
      //float B_v_pos = 1.936;
      /*float B_v_neg = 1.422;
      float B_v_c = B_v_pos;
      float l = 0.3235;
      float m_c = 5.273;
      float m_b = 0.201;
      float g = 9.81;
      float B_v_p = 0.0004;
      float F_c_p = 0.004;
      float k_tanh = 250;*/

  // stop all
  if (setOut == 0) {
  //-----------FORCE TEST--------------
  //  setOutSled = 0;
  //  digitalWrite(ENABLESLED, HIGH);
  //  setOutSled = 1.3;
  //-----------------------------------
    digitalWrite(ENABLESLED, LOW);
    setOutSled = 0;
    setOutPend1 = 0;
    I_error = 0; // Reset integrator
  }
  // activate Sliding mode control
  else if (setOut == 1)
  {
    double y_meas[2];
    y_meas[0] = posSled;
    y_meas[1] = posPend1; // Has to be in radians

    if (first_run)
    {
      x_init[0] = posSled;
      x_init[1] = posPend1; // Has to be in radians
      x_init[2] = 0;
      x_init[3] = 0;
      first_run = false;
    }

    // EKF(y_meas,Ia,t_s,x_init)
    EKF(y_meas, setOutSledNoComp, SAMPLINGTIME, x_init, x_est_correction);

    // Should only print state estimates if the EKF is active
    //Serial.print("EKF: ");
    Serial.print(x_est_correction[0], 5);
    Serial.print(",");
    Serial.print(x_est_correction[1], 5);
    Serial.print(",");
    Serial.print(x_est_correction[2], 5);
    Serial.print(",");
    Serial.print(x_est_correction[3], 5);
    Serial.print(",");



    // ---------------Model parameters------------------------//
    float r_pulley = 0.028;
    float K_t = 0.0934;
    //float B_c_pos = 3.0212 - 0.5;
    float B_c_neg = 2.7464;
    float B_v_pos = 1.936;
    float B_v_neg = 1.422;
    float B_v_c = B_v_pos;
    float l = 0.3235;
    float m_c = 5.273;
    float m_b = 0.201;
    float g = 9.81;
    float B_v_p = 0.0004;
    float F_c_p = 0.004;
    float k_tanh = 250;

    // ------------------sliding mode parameters-----------------//
    float sat = 0;
    //For u=-inv(gamma)(beta_0*sat(s)):
    //float k1 = -4.59 - 25, k2 = 15.8 + 13, k3 = -1.46 - 0.5;
    
    //float k1 = -0.6110, k2 = 1.1586, k3 = 6.2939;
    //float k1 = -2.444, k2 = 2.7094, k3 = 9.9477;
    //float k1 = -6.4155, k2 = 5.1344, k3 = 14.8400;
    //float k1 = -9.165, k2 = 6.5257, k3 = 17.2965;
    //float k1 = -12.220, k2 = 8.1755, k3 = 20.362;    // works! sorta..
    //float k1 = -16.3728, k2 = 9.8187, k3 = 23.2948;  // works! sorta..
    //float k1 = -17.1079, k2 = 10.4411, k3 = 24.0570; // works! sorta..
    //float k1 = -21.3849, k2 = 11.0635, k3 = 25.505;  // works! sorta..
    //float k1 = -24.4399, k2 = 12.1979, k3 = 27.5572;   // contestant nr 1  aaand we have a winneeeer!
    float k1 = -27.4949, k2 = 13.3323, k3 = 29.6093;     // contestant nr 2
    //float k1 = -34.7982, k2 = 16.2121, k3 = 34.6289;   // works! sorta..
    //float k1 = -34.2159, k2 = 15.0875, k3 = 32.9048;    // lots on x pos
    
    float rho     = 8.5638;
    float beta_0  = .1;
    float beta    = rho + beta_0;
    float epsilon = 0.03;

    // ---------------------Friction determiner ------------------//
    if (((x_est_correction[2]) > 0) || (((x_est_correction[2]) == 0) && (setOutSled > 0))) {
      B_v_c = B_v_pos;
    }
    else if (((x_est_correction[2]) < 0) || (((x_est_correction[2]) == 0) && (setOutSled < 0))) {
      B_v_c = B_v_neg;
    }

    // Insert control here
    setOutSled = 0;
    digitalWrite(ENABLESLED, HIGH);
    if (posPend1 < stop_angle && posPend1 > -stop_angle) { //0.3 rad = 17 deg
      /*
          Sliding mode control here
          variables to use, cart_ref, x_est_correction,
      */
      // Sliding mode start
      
      //  CHANGE OF COOREDINATE CONVENSION
      //  x1 = x_est_correction[1]
      //  x2 = x_est_correction[0]
      //  x3 = x_est_correction[3]
      //  x4 = x_est_correction[2]
      
      //    s       = x3 + k1*x2 + k3*x1 + k2*(0.3348*x3 - x4*cos(x1))
      //    g_b_inv = (l*(M + m - m*cos(x1)^2))/cos(x1) 
      
      float g_b_inv = (l*(m_c + m_b - m_b*cos(x_est_correction[1])*cos(x_est_correction[1])))/cos(x_est_correction[1]);
      float s = x_est_correction[3] + k1*x_est_correction[0] + k3*x_est_correction[1] + k2*(0.3348*x_est_correction[3] - x_est_correction[2]*cos(x_est_correction[1]));
      float z = s / epsilon;
      if (z > 1) {
        sat = 1;
      } else if (z < -1) {
        sat = -1;
      } else {
        sat = z;
      }
      setOutSled = -(g_b_inv*beta * sat)*r_pulley/K_t;
      // Sliding mode end
      
    } else { // Safety trigger, if the angle gets too large, it cannot catch the pendulum again
      setOut = 0;
      setOutSled = 0;
      P_correction_EKF_not_empty_init(); // Reset EKF
      wait_for_position = false; // Reset control such that it waits for the correct position
      cart_ref = init_ref;
      cart_ref_goal = init_ref;
      first_run = true;
      Serial.println("Safety angle exceeded - Turning off controllers");
    }

    // ---------------------Compensation ------------------//
    setOutSledNoComp = setOutSled;
    if (((x_est_correction[2]) > 0) || (((x_est_correction[2]) == 0) && (setOutSled > 0))) {
      setOutSled = setOutSled + r_pulley / K_t * (B_c_neg); //tanh(250*x_est_correction[2]);
    }
    else if (((x_est_correction[2]) < 0) || (((x_est_correction[2]) == 0) && (setOutSled < 0))) {
      setOutSled = setOutSled + r_pulley / K_t * (B_c_neg) * (-1); //tanh(250*x_est_correction[2]);
    }
    else {
      setOutSled = 0;
    }
  }
  else if (setOut == 2) {

    if (posSled > cart_ref - start_pos && posSled < cart_ref + start_pos && posPend1 > -start_angle && posPend1 < start_angle) { // Check angle, 0.2 rad = 11.5
      wait_for_position = true;
    }
    else if (wait_for_position == false) {
      Serial.println(" ");
      Serial.println("Move the cart to the middle of the rail and lift the pendulum approximately straight up in the clockwise direction");
      Serial.println(" ");
    }
    if (wait_for_position == true) { // Pendulum should now be pointing straight up and start balancing around the middle of the rail

      double y_meas[2];
      y_meas[0] = posSled;
      y_meas[1] = posPend1; // Has to be in radians

      if (first_run) {
        x_init[0] = posSled;
        x_init[1] = posPend1; // Has to be in radians
        x_init[2] = 0;
        x_init[3] = 0;
        first_run = false;
      }

      // EKF(y_meas,Ia,t_s,x_init)
      EKF(y_meas, setOutSledNoComp, SAMPLINGTIME, x_init, x_est_correction);

      // Should only print state estimates if the EKF is active
      //Serial.print("EKF: ");
      Serial.print(x_est_correction[0], 5);
      Serial.print(",");
      Serial.print(x_est_correction[1], 5);
      Serial.print(",");
      Serial.print(x_est_correction[2], 5);
      Serial.print(",");
      Serial.print(x_est_correction[3], 5);
      Serial.print(",");



      // ---------------Model parameters------------------------//
      float r_pulley = 0.028;
      float K_t = 0.0934;
      //float B_c_pos = 3.0212;
      float B_c_neg = 2.7464;
      float B_v_pos = 1.936;
      float B_v_neg = 1.422;
      float B_v_c = B_v_pos;
      float l = 0.3235;
      float m_c = 5.273;
      float m_b = 0.201;
      /*float g = 9.81;
      float B_v_p = 0.0004;
      float F_c_p = 0.004;
      float k_tanh = 250;*/

      // ------------------sliding mode parameters-----------------//
      float sat = 0;
      //For u=-inv(gamma)(beta_0*sat(s)):
      //float k1 = -4.59 - 25, k2 = 15.8 + 13, k3 = -1.46 - 0.5;
      float k1 = -30, k2 = 29, k3 = -2.2;
      float beta_0 = 10;
      float epsilon = 0.035;

      // ---------------------Friction determiner ------------------//
      if (((x_est_correction[2]) > 0) || (((x_est_correction[2]) == 0) && (setOutSled > 0))) {
        B_v_c = B_v_pos;
      }
      else if (((x_est_correction[2]) < 0) || (((x_est_correction[2]) == 0) && (setOutSled < 0))) {
        B_v_c = B_v_neg;
      }

      // Insert control here
      setOutSled = 0;
      digitalWrite(ENABLESLED, HIGH);
      if (posPend1 < stop_angle && posPend1 > -stop_angle) { //0.3 rad = 17 deg
        /*
            Sliding mode control here
            variables to use, cart_ref, x_est_correction,
        */
        // Sliding mode start
        float ga4 = K_t / r_pulley * cos(x_est_correction[1]) / (l * (m_c + m_b * sin(x_est_correction[1]) * sin(x_est_correction[1])));
        float gamma = ga4;
        float s = x_est_correction[3] + k1 * (x_est_correction[0] - cart_ref) + k2 * x_est_correction[1] + k3 * (x_est_correction[2] * cos(x_est_correction[1]) / l - x_est_correction[3]);
        float z = s / epsilon;
        if (z > 1) {
          sat = 1;
        } else if (z < -1) {
          sat = -1;
        } else {
          sat = z;
        }
        setOutSled = -1 / (gamma) * ((beta_0) * sat);
        // Sliding mode end
        
      } else { // Safety trigger, if the angle gets too large, it cannot catch the pendulum again
        setOut = 0;
        setOutSled = 0;
        P_correction_EKF_not_empty_init(); // Reset EKF
        wait_for_position = false; // Reset control such that it waits for the correct position
        cart_ref = init_ref;
        cart_ref_goal = init_ref;
        first_run = true;
        Serial.println("Safety angle exceeded - Turning off controllers");
      }

      // ---------------------Compensation ------------------//
      setOutSledNoComp = setOutSled;
      if (((x_est_correction[2]) > 0) || (((x_est_correction[2]) == 0) && (setOutSled > 0))) {
        setOutSled = setOutSled + r_pulley / K_t * (B_c_neg); //tanh(250*x_est_correction[2]);
      }
      else if (((x_est_correction[2]) < 0) || (((x_est_correction[2]) == 0) && (setOutSled < 0))) {
        setOutSled = setOutSled + r_pulley / K_t * (B_c_neg) * (-1); //tanh(250*x_est_correction[2]);
      }
      else {
        setOutSled = 0;
      }
    }
  }
  // activate Sliding mode control
  else if (setOut == 3) {

    if (posSled > cart_ref - start_pos && posSled < cart_ref + start_pos && posPend1 > -start_angle && posPend1 < start_angle) { // Check angle, 0.2 rad = 11.5
      wait_for_position = true;
    }
    else if (wait_for_position == false) {
      Serial.println(" ");
      Serial.println("Move the cart to the middle of the rail and lift the pendulum approximately straight up in the clockwise direction");
      Serial.println(" ");
    }
    if (wait_for_position == true) { // Pendulum should now be pointing straight up and start balancing around the middle of the rail

      double y_meas[2];
      y_meas[0] = posSled;
      y_meas[1] = posPend1; // Has to be in radians

      if (first_run) {
        x_init[0] = posSled;
        x_init[1] = posPend1; // Has to be in radians
        x_init[2] = 0;
        x_init[3] = 0;
        first_run = false;
      }

      // EKF(y_meas,Ia,t_s,x_init)
      EKF(y_meas, setOutSledNoComp, SAMPLINGTIME, x_init, x_est_correction);

      // Should only print state estimates if the EKF is active
      //Serial.print("EKF: ");
      Serial.print(x_est_correction[0], 5);
      Serial.print(",");
      Serial.print(x_est_correction[1], 5);
      Serial.print(",");
      Serial.print(x_est_correction[2], 5);
      Serial.print(",");
      Serial.print(x_est_correction[3], 5);
      Serial.print(",");

      // ---------------Model parameters------------------------//
      float r_pulley = 0.028;
      float K_t = 0.0934;
      //float B_c_pos = 3.0212 - 0.5;
      float B_c_neg = 2.7464;
      float B_v_pos = 1.936;
      float B_v_neg = 1.422;
      float B_v_c = B_v_pos;
      float l = 0.3235;
      float m_c = 5.273;
      float m_b = 0.201;
      float g = 9.81;
      float B_v_p = 0.0004;
      float F_c_p = 0.004;
      float k_tanh = 250;

      // ------------------sliding mode parameters-----------------//
      float sat = 0;
      // For u = -inv(gamma)(omega+beta_0)*sat(s), with new manifold differentiation
      //float k1 = -4.59 - 25, k2 = 15.8 + 13, k3 = -1.46 - 0.5;
      float k1 = -30, k2 = 29, k3 = -2.2;
      float beta_0 = 9;
      float epsilon = 0.055;


      // ---------------------Friction determiner ------------------//
      if (((x_est_correction[2]) > 0) || (((x_est_correction[2]) == 0) && (setOutSled > 0))) {
        B_v_c = B_v_pos;
      }
      else if (((x_est_correction[2]) < 0) || (((x_est_correction[2]) == 0) && (setOutSled < 0))) {
        B_v_c = B_v_neg;
      }

      // Insert control here
      setOutSled = 0;
      digitalWrite(ENABLESLED, HIGH);
      if (posPend1 < stop_angle && posPend1 > -stop_angle) { //0.3 rad = 17 deg
        /*
            Sliding mode control here
            variables to use, cart_ref, x_est_correction,
        */
        // Sliding mode start
        float F4 = -(cos(x_est_correction[1]) * (B_v_c * x_est_correction[2] + m_b * l * sin(x_est_correction[1]) * x_est_correction[3] * x_est_correction[3])) / (l * (m_c + m_b * (sin(x_est_correction[1])) * (sin(x_est_correction[1])))) - ((m_c + m_b) * (B_v_p * x_est_correction[3] + F_c_p * sign(k_tanh * x_est_correction[3]) - m_b * l * g * sin(x_est_correction[1]))) / (m_b * l * l * (m_c + m_b * (sin(x_est_correction[1])) * (sin(x_est_correction[1]))));
        float ga4 = K_t / r_pulley * cos(x_est_correction[1]) / (l * (m_c + m_b * sin(x_est_correction[1]) * sin(x_est_correction[1])));
        float fc2 = (cos(x_est_correction[1]) * (B_v_p * x_est_correction[3] + F_c_p * sign(k_tanh * x_est_correction[3]) - m_b * l * g * sin(x_est_correction[1]))) / (l * (m_c + m_b * (sin(x_est_correction[1])) * (sin(x_est_correction[1]))));
        float omega = F4 + k1 * x_est_correction[2] + k2 * x_est_correction[3] + k3 * (-x_est_correction[2] * x_est_correction[3] * sin(x_est_correction[1]) / l + fc2 / (l) * (1 + m_c / m_b + cos(x_est_correction[1]) * cos(x_est_correction[1])) / cos(x_est_correction[1]));
        float gamma = ga4;
        float s = x_est_correction[3] + k1 * (x_est_correction[0] - cart_ref) + k2 * x_est_correction[1] + k3 * (x_est_correction[2] * cos(x_est_correction[1]) / l - x_est_correction[3]);
        float z = s / epsilon;
        if (z > 1) {
          sat = 1;
        } else if (z < -1) {
          sat = -1;
        } else {
          sat = z;
        }
        setOutSled = -1 / (gamma) * ((fabs(omega) / 3 + beta_0) * sat); // Output set from the sliding mode control
        
        // Sliding mode end
      } else { // Safety trigger, if the angle gets too large, it cannot catch the pendulum again
        setOut = 0;
        setOutSled = 0;
        P_correction_EKF_not_empty_init(); // Reset EKF
        wait_for_position = false; // Reset control such that it waits for the correct position
        cart_ref = init_ref;
        cart_ref_goal = init_ref;
        first_run = true;
        Serial.println("Safety angle exceeded - Turning off controllers");
      }

      // ---------------------Compensation ------------------//
      setOutSledNoComp = setOutSled;
      if (((x_est_correction[2]) > 0) || (((x_est_correction[2]) == 0) && (setOutSled > 0))) {
        setOutSled = setOutSled + r_pulley / K_t * (B_c_neg); //tanh(250*x_est_correction[2]);
      }
      else if (((x_est_correction[2]) < 0) || (((x_est_correction[2]) == 0) && (setOutSled < 0))) {
        setOutSled = setOutSled + r_pulley / K_t * (B_c_neg) * (-1); //tanh(250*x_est_correction[2]);
      }
      else {
        setOutSled = 0;
      }
    }
  }
  // activate Machine Learning control
  else if (setOut == 4) {
    digitalWrite(ENABLESLED, HIGH);
    if (posSled > cart_ref - start_pos && posSled < cart_ref + start_pos && posPend1 > -start_angle && posPend1 < start_angle && wait_for_position==false) { // Check angle, 0.2 rad = 11.5
      wait_for_position = true;
      MLC_loop_count = 1;
    }
    else if (wait_for_position == false) {
      Serial.println(" ");
      Serial.println("Move the cart to the middle of the rail and lift the pendulum approximately straight up in the clockwise direction");
      Serial.println(" ");
    }
    if (wait_for_position == true) { // Pendulum should now be pointing straight up and start balancing around the middle of the rail

      if (posPend1 < stop_angle && posPend1 > -stop_angle) { //0.3 rad = 17 deg
        /*
          Machine learning based control should be put into this case
        */

        // Kalman filter
        double y_meas[2];
        y_meas[0] = posSled;
        y_meas[1] = posPend1; // Has to be in radians

        if (first_run) {
          x_init[0] = posSled;
          x_init[1] = posPend1; // Has to be in radians
          x_init[2] = 0;
          x_init[3] = 0;
          first_run = false;
        }

      // ---------------------Inverse compensation ------------------//
      if (((x_est_correction[2]) > 0) || (((x_est_correction[2]) == 0) && (setOutSled > 0))) {
        setOutSledNoComp = setOutSled - r_pulley / K_t * (B_c_pos); //tanh(250*x_est_correction[2]);
      }
      else if (((x_est_correction[2]) < 0) || (((x_est_correction[2]) == 0) && (setOutSled < 0))) {
        setOutSledNoComp = setOutSled - r_pulley / K_t * (B_c_neg) * (-1); //tanh(250*x_est_correction[2]);
      }
      else {
        setOutSledNoComp = 0;
      }
        // EKF(y_meas,Ia,t_s,x_init)
        EKF(y_meas, setOutSledNoComp, SAMPLINGTIME, x_init, x_est_correction);

        // adjust sampling frequency to run at lower for MLC
        if (MLC_loop_count == 3){
          //x_est_correction[0]=x_est_correction[0]-0.4;
          setOutSled = run_mlc(x_est_correction);
          //x_est_correction[0]=x_est_correction[0]+0.4;
          MLC_loop_count = 1;
        }
        else {
          MLC_loop_count++;
        } 
      } else { // Safety trigger, if the angle gets too large, it cannot catch the pendulum again
        setOut = 0;
        setOutSled = 0;
        cart_ref = init_ref;
        cart_ref_goal = init_ref;
        P_correction_EKF_not_empty_init(); // Reset EKF
        wait_for_position = false;
        Serial.println("Safety angle exceeded - Turning off controllers");
      }
    }
  }

  ////////////////////////////////////////////
  // Set outputs
  ////////////////////////////////////////////

  sled.setOutput(setOutSled,1);
  
  Serial.print(posSled, 5);
  Serial.print(",");
  Serial.print(posPend1, 5);
  Serial.print(",");
  Serial.print(time_stamp, DEC);
  Serial.print(",");
  Serial.print(setOutSledNoComp, 5); // setOutSledNoComp control without compensation
  Serial.print(",");
  Serial.println(setOutSled, 5); // setOutSledNoComp control without compensation

  //if(posSled<0.05||posSled>0.72){
  //  setOut=0;}

  while (current_time + SAMPLINGTIME * 1000000 > micros()) { // Ensures correct sampling time
    // burn time
  }
  //Serial.println(micros()-current_time); //loop time
  if (micros() - current_time > SAMPLINGTIME * 1000000 + 2000 +1000000 ) {
    Serial.println("Loop Time not uphold");
    digitalWrite(ENABLESLED, LOW);
    delay(5000);
    setOutSled = 0;
    setOut = 0;
  }
  current_time = micros();
}

int sign(float angle) {
  if (angle > 0) {
    return 1;
  } else if (angle < 0) {
    return -1;
  } else {
    return 0;
  }

}
