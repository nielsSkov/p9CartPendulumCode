/******************************************************
 * looptime.h                                         *
 *                                                    *
 *  Created on: January 2014                          *
 *      Author: jdn                                   *
 *                                                    *
 ******************************************************
 *                                                    *
 *            (simple loop adj delay                  *
 *                                                    *
 *                                                    *
 *                                                    *
 * (C) 2012,2013                                      *
 *                                                    *
 * Jens Dalsgaard Nielsen <jdn@es.aau.dk>             *
 * http://www.control.aau.dk/~jdn                     *
 * Studentspace/Satlab                                *
 * Section of Automation & Control                    *
 * Aalborg University,                                *
 * Denmark                                            *
 *                                                    *
 * "THE BEER-WARE LICENSE" (frit efter PHK)           *
 * <jdn@es.aau.dk> wrote this file. As long as you    *
 * retain this notice you can do whatever you want    *
 * with this stuff. If we meet some day, and you think*
 * this stuff is worth it ...                         *
 *  you can buy me a beer in return :-)               *
 * or if you are real happy then ...                  *
 * single malt will be well received :-)              *
 *                                                    *
 * Use it at your own risk - no warranty              *
 *                                                    *
 * tested with duemilanove w/328, uno R3,             *
 * seeduino 1280 and mega2560                         *
 *****************************************************/

/*
looptime 

void loop()
{
if ( 0 == looptime(500))
  // ok
your stuff
}

Every loop will take 500 if there is time enough
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
 
#include <looptime.h>
unsigned long looptime(unsigned long t)
{
  static unsigned long tact,tn,td;
  static boolean firstTime = true;
 
  if ( 0 == t ) {
    firstTime = true;  // easy restart
    return 0;
  }
  
  if (firstTime) {
    firstTime = false;
    // first time only
    tn = millis();
    return (0);
  }
  else {
    tn = tn+t;
    tact = millis();
    td = tn -tact;
    if (tn < tact)
    {
      td = 0;
      return (0); // behind
     }
    delay(td);
    return (td); // ok
  }
}
  
 
