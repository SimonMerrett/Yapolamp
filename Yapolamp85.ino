/* Yapolamp is an experimental torch/flashlight intended to be safer for children
 Copyright 2008-2017 Simon Merrett, Nick Gammon and Martin Nawrath
 
 Current bootloader settings: ATtiny85 @8Mhz (internal oscillator; BOD disabled)
 Current version:  Yapolamp85_06 tidies up some unused lines and removes Arduino code where e.g. assembly has replaced it.
 Previous versions:
 Yapolamp85_05 uses the watchdog timer without sleep to time the ON period and "wind-down" period
 Yapolamp85_04 adds assembly code delays to ensure accurate pulse and delay timings at 8 MHz. 
 Also shifts from just interrupts to make the watchdog help with timing-out the ON and Transition modes
 Yapolamp85_03 has implemented port manipulation on the ATtiny85 and starts to introduce the control logic for different modes
 
     This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <avr/io.h> //ATtiny85 required library
#include <avr/sleep.h> //Needed for ATtiny85 sleep_mode
#include <avr/wdt.h> //Needed to enable/disable ATtiny85 watch dog timer

#define buttonPin PB0 //ATtiny85 pin assignment
#define driverPin PB1 //ATtiny85  pin assignment
#define LEDPin PB3 //ATtiny85  pin assignment
#define cPin PB4 //ATtiny85  pin assignment

byte oldButtonState = 0;  // no pull-up resistor
const unsigned long debounceTime = 20;  // milliseconds
unsigned long buttonPressTime;  // when the switch last changed state
volatile boolean buttonPressed = 0; // a flag variable
volatile boolean buttonFlag = 0; // interrupt flag variable
volatile byte MODE = 0; // the modes are 0: always on, 1: on, 2: wind-down, 3: charging (total sleep, not implemented on V1.0 hardware), 4: low battery (not implemented on V1.0 hardware)
volatile byte onFlag = 0; //this is triggered when the watchdog timer sees the end of the ON period 
volatile byte alwaysOnFlag = 0; //this is triggered when the watchdog timer sees the end of the "wind down" period 
volatile uint16_t counter = 0;
uint16_t oldCounter = 0;

void setup() {
  DDRB &= ~(1 << buttonPin);  // set up buttonPin as an input on ATtiny85
  DDRB |= (1 << driverPin);  // set up driverPin as an output on ATtiny85
  DDRB |= (1 << LEDPin);  // set up LEDPin as an output on ATtiny85
  DDRB &= ~(1 << cPin);  // set up buttonPin as an input on ATtiny85

  cli(); //stop interrupts
  // pin change interrupt
  PCMSK  |= bit (PCINT0);  // want pin PB0 / pin 5
  GIFR   |= bit (PCIF);    // clear any outstanding interrupts
  GIMSK  |= bit (PCIE);    // enable pin change interrupts 
  sei(); // start interrupts

  //Power down various bits of hardware to lower power usage
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); //Power down everything, wake up from WDT
  sleep_enable();
}

//This runs each time the watch dog wakes us up from sleep
ISR(WDT_vect) {
  cli();
  if (counter > 600)onFlag = 1; // duration of ON period, in seconds
  if (counter > 660) alwaysOnFlag = 1; // duration of ON period _PLUS_ "wind-down" period, in seconds
  if (MODE == 1) counter++; // only increment the counter in ON and "wind down" phases - not Always ON mode
  sei();
}

ISR (PCINT0_vect) // pin change interrupt service routine
{
  buttonFlag = 1; // flag a button state change
}

void buttonRead() {
  // Button reading with non-delay() debounce - thank you Nick Gammon! Based on http://www.gammon.com.au/switches
  byte buttonState = digitalRead (buttonPin);
  if (buttonState != oldButtonState) {
    if (millis () - buttonPressTime >= debounceTime) { // debounce
      buttonPressTime = millis ();  // when we closed the switch
      oldButtonState =  buttonState;  // remember for next time
      if (buttonState == HIGH) {
        buttonPressed = 1;
        buttonFlag = 0; // clear the flag in the ISR
      }
      else {
        buttonPressed = 0;
        buttonFlag = 0; // clear the flag in the ISR
      }
    }  // end if debounce time up
  } // end of state change
}

void loop() {
  tinyPulse(); // this is the pulse and associated delays to charge the inductor and allow discharge through the LEDs 
  // Then check if we've received a button input and act appropriately
  if (buttonFlag) {
    buttonRead();
    if (buttonPressed) {
      if (MODE == 0) {
        //digitalWrite(LEDPin,HIGH); // DEBUGGING - comment out for production sketch
        MODE = 1;
        counter = 0;
        onFlag = 0;
        alwaysOnFlag = 0;
        setup_watchdog(6);
      }
      else if (MODE == 1) {
        if(onFlag == 0) {
          //digitalWrite(LEDPin,LOW);  // DEBUGGING - comment out for production sketch
          MODE = 0;
        }
        else {
          counter = 0;
          onFlag = 0;
          alwaysOnFlag = 0;
        }
      }
      buttonPressed = 0;
    }
  }
  if (MODE == 0) {
    if   (buttonFlag == 0)    SLEEP();
  }
  if (MODE == 1) {
    if (onFlag) {
      if (counter != oldCounter) {
        //int pulse = 20*(counter %2);
        delay(20); 
        oldCounter = counter;
        setup_watchdog(6);
      }
    }
    if (alwaysOnFlag) {
      counter = 0;
      onFlag = 0;
      alwaysOnFlag = 0;
      MODE = 0;
    }
  }
}

void tinyPulse() { //ATtiny85 driver loop
  PORTB |= (1 << driverPin);  // turns ATtiny85 MOSFET on
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"); //1uS @ 8MHz
  PORTB &= ~(1 << driverPin); // turns ATtiny85 MOSFET off
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"); //1uS @ 8MHz
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"); //1uS @ 8MHz
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"); //1uS @ 8MHz
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"); //1uS @ 8MHz
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"); //1uS @ 8MHz
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"); //1uS @ 8MHz
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"); //1uS @ 8MHz
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"); //1uS @ 8MHz
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"); //1uS @ 8MHz
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"); //1uS @ 8MHz
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"); //1uS @ 8MHz
  __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"); //1uS @ 8MHz
}

void SLEEP() { //ATtiny85 driver loop
  ADCSRA &= ~(1 << ADEN); //Disable ADC, saves ~230uA
  setup_watchdog(1); //Setup watchdog to go off after 32ms
  sleep_mode(); //Go to sleep! Wake up 128ms later
  ADCSRA |= (1 << ADEN); //Enable ADC
}

//Sets the watchdog timer to wake us up, but not reset
//0=16ms, 1=32ms, 2=64ms, 3=128ms, 4=250ms, 5=500ms
//6=1sec, 7=2sec, 8=4sec, 9=8sec
//From: http://interface.khm.de/index.php/lab/experiments/sleep_watchdog_battery/
void setup_watchdog(int timerPrescaler) {
  if (timerPrescaler > 9 ) timerPrescaler = 9; //Limit incoming amount to legal settings
  byte bb = timerPrescaler & 7;
  if (timerPrescaler > 7) bb |= (1<<5); //Set the special 5th bit if necessary
  cli();
  //This order of commands is important and cannot be combined
  MCUSR &= ~(1<<WDRF); //Clear the watch dog reset
  WDTCR |= (1<<WDCE) | (1<<WDE); //Set WD_change enable, set WD enable
  WDTCR = bb; //Set new watchdog timeout value
  WDTCR |= _BV(WDIE); //Set the interrupt enable, this will keep unit from resetting after each int
  sei();
}











