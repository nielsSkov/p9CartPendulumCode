//////////////////////////////////////////////////////
//
//  Before Upload:
//
//  (1) Add libraries containing the used .h files
//      Sketch -> Import Library -> Add Library
//
//  NB: when a file is changed the corresponding
//      library has to been added again
//
//  (2) Remember
//  Tools -> Board -> teensy 3.6
//  Tools -> Port  -> choose com port
//  Set 'Serial Monitor' baud-rate to 115200
//
//  (3) Remember: Max Voltage 50, Max Current 5
//
//////////////////////////////////////////////////////

#include <Joint_teensy.h>    // main file 
//#include <Utility.h>       // sat and sign function
//#include <looptime.h>      // set time in loop

// Definitions
#define SAMPLINGTIME 0.00667 // s
#define ENABLEPENDUL 11      //50
#define ENABLESLED 12        //48

// Create joint objects
Joint sled(  1, SAMPLINGTIME );
Joint pend1( 2, SAMPLINGTIME );

// Global variables
int   setOutSled  = 0;
int   setOutPend1 = 0;
float velSled     = 0;
float velPend1    = 0;
float posSled;
float posPend1;
int   setOut      = 0;

unsigned long current_time = 0;
unsigned long last_time    = 0;
unsigned long loop_time    = 0;
unsigned long time_now     = 0;

float I_error = 0;

void setup()
{
  //setup serial
  Serial.begin(115200);  //to be able to do Serial.print
                         //and still have time in loop
  // Setup pin modes
  pinMode( 25, INPUT );
  pinMode( 26, INPUT );
  pinMode( 27, INPUT );
  pinMode( 28, INPUT );
  pinMode( 29, INPUT );
  pinMode( 30, INPUT );
  pinMode( 31, INPUT );
  pinMode( 32, INPUT );
  
  //initialize joints
  sled.init();
  pend1.init();

  //reset positions
  sled.resetPos();
  pend1.resetPos();

  //set DAC resolution
  analogWriteResolution(12);
  analogReadResolution(12);

  //set output to zero
  sled.setOutput(0);
  pend1.setOutput(0);

  //enable motor driver outputs
  //(FOR SAFTY REASONS, DO NOT CHANGE THE BELOW 6 LINES)
  digitalWrite( ENABLEPENDUL, LOW    );
  digitalWrite( ENABLESLED,   LOW    );
  pinMode(      ENABLEPENDUL, OUTPUT ); 
  pinMode(      ENABLESLED,   OUTPUT );
  digitalWrite( ENABLEPENDUL, LOW    );
  digitalWrite( ENABLESLED,   LOW    );

  current_time = micros();
}

void loop()
{
  ///////////////////////////////////////
  // Input from user on serial interface
  ///////////////////////////////////////

  int input;

  if(Serial.available() > 0)
  {
    input = Serial.read();
    
    // stop all
    switch(input)
    {
      case '0': // stop motor
        setOut = 0;
        break;
      case '1':
        setOut = 1;
        time_now = micros();
        break;
      case '2':
        setOut = 2;
        break;
      case '3':
        setOut = 3;
        break;
      case 'r':
        pend1.resetPos();
        sled.resetPos();
        break;
      default:
        Serial.println("ERROR: Wrong command selected.");
        break;
    }
  }
 
  ///////////////////////////////////////
  //sensor readings
  ///////////////////////////////////////
  posSled  = sled.readPos();
  posPend1 = pend1.readPos();
  velSled  = sled.readVel();
  velPend1 = pend1.readVel();
  
  unsigned long time_stamp = micros();
    
  ///////////////////////////////////////
  // Apply control
  ///////////////////////////////////////

  // stop all
  if (setOut == 0)
  { 
    digitalWrite( ENABLESLED, LOW );
    digitalWrite( ENABLEPENDUL, LOW);   
    setOutSled  = 0;
    setOutPend1 = 0;
    I_error     = 0; // Reset integrator
  }
  // activate
  else if( setOut == 1 )
  {
    digitalWrite( ENABLESLED, HIGH );
    //digitalWrite( ENABLEPENDUL, HIGH );

    if( (micros() - time_now) < 5500000000000 )
    {
      setOutSled = .1;
      setOutPend1 = 0;
    }
    //else if( (micros() - time_now) > 550000&&(micros() - time_now) < 550000 )
    //{
    //  setOutSled = 0;
    //  setOutPend1 = 0;
    //}
    //else if( (micros() - time_now) > 550000&&(micros() - time_now) < 1000000 )
    //{
    //  setOutSled = 0;
    //  setOutPend1 = 0;
    //}
    else
    {
      setOutSled = 0;
      setOutPend1 = 0;
      digitalWrite( ENABLESLED, LOW );
      //digitalWrite( ENABLEPENDUL, LOW );
    }
  }
  // activate 
  else if( setOut == 2 )
  {
    digitalWrite( ENABLESLED, HIGH );

    float vel_ref = 0.1;
    I_error      += vel_ref - velSled;
    float K_p     = 2500;
    float K_i     = 0;
    setOutSled    = (int)( (vel_ref - velSled)*K_p+I_error*K_i*SAMPLINGTIME );
  }
  else if(setOut == 3)
  {
    digitalWrite( ENABLESLED, HIGH );
    setOutSled = -5;
  }
  
  sled.setOutput(setOutSled);
  //pend1.setOutput(setOutPend1);


  //print serial outputs
  Serial.print(   posPend1,   5   );
  Serial.print(   ","             );
  Serial.print(   posSled,    5   );
  Serial.print(   ","             );
  Serial.print(   time_stamp, DEC );
  Serial.print(   ","             );
  Serial.print( setOutSled      );
  Serial.print(   ","             );
  Serial.println( setOutPend1     );

  //disable control near rail end-stops
  if( posSled < 0.05 || posSled > 0.72 )
  {
    setOut=0;
  }

  while( current_time + SAMPLINGTIME*1000000 > micros() )
  {
    //burn time to get fixed sample rate
  }
  //print loop time
  //Serial.println( micros()-current_time );


  if( micros() - current_time > SAMPLINGTIME*1000000 + 2000 )
  {
    Serial.println("Loop Time not uphold");
    
    digitalWrite( ENABLESLED, LOW);
    digitalWrite( ENABLEPENDUL, LOW);
    
    setOutSled = 0;
    setOut     = 0;
    
    delay(5000);
  }
  current_time = micros();
}
