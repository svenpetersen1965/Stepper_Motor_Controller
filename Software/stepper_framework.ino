#include <Wire.h>                   // import Wire library 
#include <LiquidCrystal_I2C.h>      // import LiquidCrystal_I2C library


// Pin definitions
#define step1 6                    // stepper 1 step
#define dir1 7                     // stepper 1 direction
#define step2 14                   // stepper 2 step
#define dir2 15                    // stepper 2 direction
#define nstpena 10                 // enable both steppers (active low) 

#define rot_sw 4                   // rotary encoder switch
#define rot_data 5                 // rotary encoder data
#define rot_clk 8                  // rotary encoder clock

#define buzzer 9                   // buzzer

#define t1_ticks_sec 2016          // the timer1 interrupt occures 2016 times per second
#define debounce 4                 // debouncing the rotary encoder
#define default_step 15            // one step every 10 timer1 interrupts
#define puls_width 30              // pulse width in µs of the step pulse

// directions
#define step_cw HIGH               // for clockwise, the direction pin is HIGH
#define step_ccw LOW               // for counter-clockwise, the direction pin is LOW
#define step_cha 0                      // stepper at channel A
#define step_chb 1                      // stepper at channel B

// state machine states
#define sm_stop 0                  // assigning a numbers to the state machine states
#define sm_set 1
#define sm_run 2

volatile boolean tick_1s = 0;                   // this flag is set every 1sec by the timer1 ISR
volatile boolean step_flag = 0;                 // this flag is set, after the time for the next step elapsed
volatile int step_ticks = 0;                    // the tick counter for the step
volatile int t1_ticks = 0;                      // counts the timer1 interrupts 
volatile int step_duration = default_step;      // full steps are used. This variables knows 

// the rotary encoder is processed by the timer1 ISR
volatile int rot_value = 0;                     // rotary encoder clockwise => value > 0; ccw => value < 0 
volatile boolean rot_button = false;            // true if button is pressed            

// variables for processing the rotary encoder
boolean rot_clk_status = HIGH;
boolean rot_clk_status_old = HIGH;
boolean rot_read;
int rot_clk_debounce = debounce;
int rot_sw_debounce = debounce;
boolean rot_sw_status = HIGH;
boolean rot_sw_status_old = HIGH;

boolean st_direction = step_cw;                // Default: clockwise
long num_steps = 0;                            // number of steps
int stepper=step_cha;                          // selects the stepper motor  

char recByte;
String recString = "";                        // receive command buffer
volatile boolean recComplete = false;
boolean syntaxError;

int state_machine;                             // this variable holds the state of the state machine 

LiquidCrystal_I2C lcd(0x27, 16, 2);            // define the lcd display (i²c address 0x27, 16 columns, 2 lines

// ****************************************
// * this is the heartbeat of the system! *
// ****************************************

ISR(TIMER1_COMPA_vect) { // function which will be called when an interrupt occurs at timer 1
  // every one second
  if (++t1_ticks == t1_ticks_sec) {   // set tick_1s every 2000 IRQ cycles (every 1 second)
    tick_1s = 1;                       
    t1_ticks = 0;                     // reset the tick counter
  }

  // count the time before te next steppper step
  if (++step_ticks == step_duration) {
    step_flag = 1;
    step_ticks = 0;
  }
  
  // ============== rotary encoder ==================
  rot_read = digitalRead( rot_clk ); 
  if ( rot_read != rot_clk_status) {              // is rot_clk different for <debounce> cycles?
    if (--rot_clk_debounce == 0) {              
      rot_clk_status = rot_read;                  // if yes: change status
    }
  }
  else {
    rot_clk_debounce = debounce;                  // if it is equal -> reset the debounce counter
  }
  if (rot_clk_status != rot_clk_status_old) {     // did a status change occur?
     if (rot_clk_status == LOW) {                 // is it a falling edge?
       if (digitalRead( rot_data ) == LOW) {      // yes: set the roraty value according to rot_data
         rot_value--;                             // LOW -> CCW
       }
       else {
         rot_value++;                             // HIGH -> CW
       }
     }
     rot_clk_status_old = rot_clk_status;         // update old status
  }
  rot_read = digitalRead( rot_sw );               // read rot switch
  if ( rot_read != rot_sw_status) {               // is rot_sw different for <debounce> cycles?
    if (--rot_sw_debounce == 0) {              
      rot_sw_status = rot_read;                   // if yes: change status
    }
  }
  else {
    rot_sw_debounce = debounce;                    // if it is equal -> reset the debounce counter
  }
  if (rot_sw_status != rot_sw_status_old) {       // did a status change occur?
     if (rot_sw_status == LOW) {                  // is it a falling edge?
       rot_button = true;                         // set switch semaphore
     }
     rot_sw_status_old = rot_sw_status;           // update old status
  }
}

// some functions
void start_screen(void) {                        // display the starte message on the LCD display
  lcd.clear();
  lcd.setCursor(0,0); 
  lcd.print("Stepper Cntr.");
}

void stop_screen(void) {
  lcd.clear();                        // clear the display
  lcd.setCursor(0,0);                 // position the cursor left, top
  lcd.print("Stop=>Channel ");        // Ausgabe: Stop
  if (stepper == step_cha) {
    lcd.print("A");
  }
  else {
    lcd.print("B");
  }
  lcd.setCursor(0,1);
  lcd.print("Steps: ");
  lcd.print( num_steps );
}

void run_screen(void) {
  lcd.clear();                                     // clear the LCD display
  lcd.setCursor(0,0);                                
  lcd.print("Run");                                // print state "Run" on screen
  lcd.noBlink();
  lcd.setCursor(0,1);
}

void makeStepA( void ) {                        // make a step on motor A
  digitalWrite( step1, HIGH );
  delayMicroseconds( puls_width );
  digitalWrite( step1, LOW );
}

void makeStepB( void ) {                        // make a step on motor B
  digitalWrite( step2, HIGH );
  delayMicroseconds( puls_width );
  digitalWrite( step2, LOW );
} 


// This function is called once at start up. It initializes the system
void setup() {
   /* ==================== TIMER 1 setup ======================= */
  // Interrupt every 0.000496 sec (= 2016.13Hz) 
  // prescaler = 256
  // Compare Match Register = 30

  cli(); // disable interrupts  
  // reset timer1 
  TCCR1A = 0;                          // set TCCR1A register to 0
  TCCR1B = 0;                          // set TCCR1B register to 0
  TCNT1  = 0;                          // reset counter value

  OCR1A = 30;                          // set compare match register of timer 1
  
  TCCR1B |= (1 << CS12);               // 1:256 prescaling for timer1  

  TCCR1B |= (1 << WGM12);              // turn on CTC mode
  TIMSK1 |= (1 << OCIE1A);             // enable timer compare interrupt

  sei();                               // allow interrupts
  
  lcd.begin(); //Im Setup wird der LCD gestartet (anders als beim einfachen LCD Modul ohne 16,2 in den Klammern denn das wurde vorher festgelegt
  start_screen();                      // initial display

  Serial1.begin(9600);                 // the RS232 interface (RX, TX on pin 0 and 1) is Serial1. 
  Serial1.println( "Hello RS-232" );   // say hello
  
  // setup stepper controller
  digitalWrite( step1, LOW );                // LOW -> Step Signal 
  digitalWrite( dir1,  st_direction  );      // default direction 
  digitalWrite( step2, LOW );                // LOW -> Step Signal 
  digitalWrite( dir2, st_direction );        // default direction 
  
  digitalWrite( nstpena, HIGH );             // disable stepper by setting the enable pin HIGH
  
  pinMode( step1, OUTPUT );                  // step1 is an output
  pinMode( dir1, OUTPUT );                   // dir1 is an output
  pinMode( step2, OUTPUT );                  // step1 is an output
  pinMode( dir2, OUTPUT );                   // dir1 is an output
  pinMode( nstpena, OUTPUT );                // nstpena is an output, that is the enable output for both stepper controlelrs

  pinMode(rot_sw, INPUT_PULLUP);             // rotary encoder: all signals input with pull-up
  pinMode(rot_data, INPUT_PULLUP);
  pinMode(rot_clk, INPUT_PULLUP);

  pinMode( buzzer, OUTPUT );                // the buzzer is an output

  recString.reserve(80);

  // beep "hello"
  tone( buzzer, 4000 );               // 1kz sound on buzzer
  delay( 1000 );                      // for 1 second
  noTone( buzzer ); 
  
  // state machine
  state_machine =  sm_run;           // first state is "run"
  // initialize the LCD display
  lcd.backlight();                    // backlight on
  switch (state_machine) {
    case sm_stop : {
      stop_screen();
      lcd.blink();
      break;   
    }
    case sm_run : {
      run_screen();
      lcd.noBlink();
      digitalWrite( dir1,  st_direction  ); 
      digitalWrite( dir2,  st_direction  ); 
      digitalWrite( nstpena, LOW );                    // enable stepper controller
      step_ticks = 0;                                  // reset flags
      step_flag = 0;
      break;   
    }
  }
  // reset rotary encoder variables (again) 
  rot_value = 0;                      // reset rot_value
  rot_button = false;                 // reset button semaphore
}

// this is being executed on incoming data over Serial1
void serialEvent1() {
  while (Serial1.available()) {       // bytes available on Serial1?
    recByte = (char)Serial1.read();   // read it
    if (recByte == (char)10) {        // is it "new line character"?
      recComplete = true;             // yes: the line is complete
    }
    else {
      recString += recByte;           // no: append character to input string
    }
  }
}

void loop() {
  // state machine
  switch (state_machine) {
    // state: stop =============================================================================================
    case sm_stop : {                                       // STOP
        lcd.setCursor(14,0);
        if (rot_button==true) {                            // rotary encoder: button pushed
          rot_button = false;                              // reset this flag  after detection                            
          Serial1.print("Button->set (Channel ");
          if (stepper == step_cha) {
            Serial1.println("A)");
          }
          else {
            Serial1.println("B)");
          }
          num_steps = 0;
          state_machine = sm_set;                          // change to state "set"
        }
        if (rot_value != 0) {
          if (rot_value < 0) {
            stepper=step_cha;
            lcd.print("A");
          }
          else {
            stepper=step_chb;
            lcd.print("B");
          }
          rot_value = 0;
        }
        break;
    } // sm_stop
    // state: set =============================================================================================
    case sm_set : {                                       // SET
        lcd.setCursor(7,1);
        if (rot_button==true) {                            // rotary encoder: button pushed
          rot_button = false;                              // reset this flag  after detection                            
          run_screen();
          Serial1.println("Button->run");
          if (num_steps<0) {
            num_steps = -num_steps;
            st_direction = step_ccw;
          }
          else {
            st_direction = step_cw; 
          }
          digitalWrite( dir1,  st_direction  ); 
          digitalWrite( dir2,  st_direction  ); 
          digitalWrite( nstpena, LOW );                    // enable stepper controller
          tone( buzzer, 4000 );                            // 1kz sound on buzzer
          delay( 300 );                                    // for 0.3 second
          noTone( buzzer );
          step_ticks = 0;                                  // reset flags
          step_flag = 0;
          state_machine = sm_run;                          // change to state "run"
        }
        if (rot_value != 0) {
           if (rot_value < 0) {
             num_steps = num_steps - 10;
             rot_value++;
           }
           else {
             num_steps = num_steps + 10;
             rot_value--;
           }
           lcd.setCursor(7,1);
           lcd.print( num_steps );
           lcd.print( "  " );
           Serial1.print( "Steps: " );
           Serial1.println( num_steps );
        }
        break;
    }
    // state: run =============================================================================================
    case sm_run : {
        if (step_flag == 1) {
          step_flag = 0;
          if (num_steps > 0) {
            if (stepper == step_cha) {
               makeStepA();  
            }
            else {
              makeStepB();
            }
            num_steps--;
          }
        }
        if (rot_button==true) {
          rot_button = false;                 
          stop_screen();
          lcd.blink();
          Serial1.println("Button->stop");
          digitalWrite( nstpena, HIGH );                   // disable stepper controller
          tone( buzzer, 4000 );                            // 1kz sound on buzzer
          delay( 300 );                                    // for 0.3 second
          noTone( buzzer );
          state_machine = sm_stop;
        }
        if (recComplete) {                                 // a complete command line is received on RS-232 
            lcd.setCursor(0,1);                            // position the cursor on the second line
            lcd.print( recString );                        // print the received string
            lcd.print( "               " );                // clear the line after that string
            
            // now parsing the command line
            recString.toUpperCase();                       // convert the line to upper case (it is not case sensitive)
            syntaxError = false;                           // reset the Syntax Error flag
            
            if (recString[0] == 'A') {                     // is the first character an A?
              stepper = step_cha;                          // yes: select Stepper#1 (channel A)
            }
            else  if (recString[0] == 'B') {               // is the first character a B?
              stepper = step_chb;                          // yes: select Stepper#2 (channel B)
            }
            else {
              syntaxError = true;                          // else, the syntax is not correct.
            }
            if (!syntaxError) {                            // syntax still correct?
              int pos = (int) recString.indexOf(",");      // find the first comma ","
              if (pos < 1) {                               // if no comma is found
                syntaxError = true;                        // set the Syntax Error Flag
              }
              else {                                       // a comma is found:
                recString.remove(0,pos+1);                 // remove the beginning, the comma included
                num_steps = (int) recString.toInt();       // convert the string to integer -> num_steps
                if (num_steps<0) {                         // is it a negative number?
                   num_steps = -num_steps;                 // yes: make it positive
                   st_direction = step_ccw;                //      the direction is counterclockwise
                }
                else {                                     // no:
                  st_direction = step_cw;                  // the direction is clockwise
                }
                digitalWrite( dir1,  st_direction  );      // set the direction of both stepper motors
                digitalWrite( dir2,  st_direction  ); 
              }
            }
            if (syntaxError) {
              Serial1.println("? SYNTAX ERROR");           // report the syntax error
            }
            else {
              Serial1.print("Execute: Channel ");          // report the execution
              switch (stepper) {
                case step_cha: {
                  Serial1.print("A, ");                    // Channel A
                  break;
                }
                case step_chb: {
                  Serial1.print("B, ");                    // Channel B
                  break;
                }
              }
              Serial1.print( num_steps );                  // and the number of steps
              if (st_direction == step_cw) {
                Serial1.println(" steps, CW");             // and the direction
              }
              else {
                Serial1.println(" steps, CCW");
              }
            }
            recComplete = false;                           // reset the semaphore
            recString = "";                                // reset the recString (receive buffer)
        }
        break;
    } // sm_run
  } // switch (state_machine)
} // loop                        
