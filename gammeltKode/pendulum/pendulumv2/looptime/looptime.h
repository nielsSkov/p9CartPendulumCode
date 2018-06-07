 
 /******************************************************
 * looptime.h                                         *
 *                                                    *
 *  Created on: January 2014                          *
 *      Author: jdn                                   *
 *                                                    *
 ******************************************************
 *                                                    *
 *            (simple loop adj delay)                 *
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

/* USAGE
I want to have a loop time of 400 msec:

After 4000000000 milliseconds(45 days) it is unstable
void loop()
{
char res;

  res = looptime(400);
  if (res == 0)
  {
  // ok on time
  }
  else
  {
    // loop takes longer than your request
    // your code below takes to long time
    // if we come here looptime has not delayed at all
  }
  yourcode();
}
*/

#ifndef LOOPTIME
#define LOOPTIME

#ifdef __cplusplus
extern "C" {
#endif
 
unsigned long looptime(unsigned long t);

#ifdef __cplusplus
}
#endif
 
 #endif
