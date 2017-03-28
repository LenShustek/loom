/******************************************************************************************************

   automation program for the Glimakra "Julia" floor loom

   The hardware added to the loom consists of:
    - up to 8 stepper motors, 23HS22-1504S, NEMA 23, 5.4V, 1.5A, 164 oz-in holding torque
    - up to 8 DRV8825 stepper motor drivers (Pololu or equiv.)
    - 1 24VDC 200W power supply
    - 1 4-row x 20-character LCD display (Adafruit 198)
    - 1 4-digit 7-segment I2C big digit numerical display (Adafruit 1268)
    - 2 rotary encoders with RGB LEDs and pushbuttons (Sparkfun COM-10982 and BOB-11722)
    - 4 general-purpose pushbuttons (MPJA 32730 or equiv.)
    - 1 foot pedal (MPJA 18150 or equiv.)
    - 1 Teensy 3.5 microprocessor, with 512KB flash, 192KB RAM, 4KB EEPROM, and 40 I/O pins

   The software allows you to graphically configure the treadle tie-ups and the treadle sequence,
   and then weave by throwing the shuttle and just pushing the single foot pedeal to get the
   next configuration of the shafts.  The warping, though, must still be done manually!

   ------------------------------------------------------------------------------------------------------
   Copyright (c) 2017, Len Shustek

   The MIT License (MIT)
   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
   associated documentation files (the "Software"), to deal in the Software without restriction,
   including without limitation the rights to use, copy, modify, merge, publish, distribute,
   sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all copies or
   substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
   NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
   DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
   ------------------------------------------------------------------------------------------------------

******************************************************************************************************/
/*
   23 Jan 2017, L. Shustek, V1.0, - first version
   31 Jan 2017, L. Shustek, V1.1, - fix treadle sequence changes during weaving,
                                  - clean up filename handling and display
    4 Feb 2017, L. Shustek, V1.2  - add motor fault detection
                                  - fix filename bug
                                  - change schematic to match driver physical layout;
                                  - clean up compiler warnings
   10 Feb 2017, L. Shustek, V1.3  - separate motor hardware config, and don't write it to memory
                                  - better debugging output
                                  - parametrize CW/CCW motor initialization
                                  - highlight the treadle while weaving, not just the line
   12 Feb 2017, L. Shustek, V1.4  - more interesting light show during weaving
                                  - make the vertical button during weaving be "undo" to the previous shed
                                  - allow abort from load/save/erase menu
   23 Mar 2017, L. Shustek, V1.5  - increase max treadle sequence to 99
                                  - fix bug when editing the filename extends its length
                                  - temporarily disable motors during weaving if there is no activity for a few minutes.
   27 Mar 2017, L. Shustek, V1.6  - display filename on top-level screen, or "unsaved" if pgmg changes are made
                                  - default to current file in load/save menu
*/
#define VERSION "1.6"

/* TODO:
    - Invent a more programmatic way to configure treadle sequences with repeat loops.
      Since weaving is limited to one treadle at a time we can use decimal rather than unary
      notation on the narrow display. It doesn't match weaving books, though, so weavers would
      consider that sacriligious!

      Idea #1: a programming mode for "treadle loops", in addition to the traditional sequence.
      The notation might be like this, using "Irish Meadows blanket" as an example:
          4x: 5 6
          6x: 6 5
          8x: 5 6
          xx: 1 2 3 4
          8x: 5 6
          6x: 6 5
          4x: 5 6
      When weaving, the "xx" line would be repeated indefinitely until a button is pushed to advance.
      Either this "treadle loop" notation or a reconstructed "treadle sequence" notation could be displayed.
      The big-digit display could show how far along in a repeat sequence we are.
      The file format would have to be elaborated to be able to save and load these looped treadle sequences.

    - Save space by making the tieup[][] matrix be a true bitmap. That would allow more files in the EEPROM.
*/

#define DEBUG true
#define HW_TEST false

#include <Encoder.h>
#include <LiquidCrystalFast.h>
#include <EEPROM.h>
#include <i2c_t3.h>
#include <string.h>

//**** global parameters

#define NUM_TREADLES 16 // max 17 will fit on the display currently
#define NUM_SHAFTS 8
#define MAX_SEQUENCE 99

#define MSEC_PER_ROTATION 1000      // how long we should take for one rotation
#define uSTEPS_PER_STEP 4           // how many microsteps per step the drivers are configured for (MODE1 high)
#define STEPS_PER_ROTATION (200 * uSTEPS_PER_STEP)  // the stepper motors have 1.8 degree "full" steps; 360/1.8=200
#define MIN_STEP_uSEC (1000UL * MSEC_PER_ROTATION / STEPS_PER_ROTATION)
#define DEBOUNCE_MSEC 50
#define ROTATION_PERCENT (112/2) // what part of a full rotation goes from CENTER to UP or DOWN, nominally
#define STEPS_PER_LIGHT_CHANGE 64
#define MINS_BEFORE_MOTOR_DISABLE 10

//**** hardware pin configuration

#define LCD_D4 11  // Hitachi HD44780 LCD controller
#define LCD_D5 10
#define LCD_D6 9
#define LCD_D7 8
#define LCD_ENB 12
#define LCD_RS 24
#define LCD_RW 7
#define PB1 3  // pushbuttons: active low (blue)
#define PB2 2  //   (yellow)
#define PB3 1  //   (green)
#define PB4 0  //   (black)
#define PEDAL 6
#define MOTOR_DIR 21    // direction control for all motors
#define MOTOR_ENB 22    // active low: enable motors
#define MOTOR_FAULT 23  // active low input: motor fault
#define H_ENC_RED 31    // horizontal and vertical rotary encoder lights
#define H_ENC_GREEN 32  //   (active low)
#define H_ENC_BLUE 29
#define V_ENC_RED 28
#define V_ENC_GREEN 27
#define V_ENC_BLUE 25
#define LED_ON LOW
#define LED_OFF HIGH
#define H_ENC_PB 30     // encoder pushbuttons: active high
#define V_ENC_PB 26
#define H_ENC_A 34      // encoder step outputs
#define H_ENC_B 33
#define V_ENC_A 36
#define V_ENC_B 35
#define ENCODER_INCR 4  // number of encoder steps per click
#define LED_SCL 37      // SCL1 for the big 4-digit 7-segment LED display (Wire1)
#define LED_SDA 38      // SDA1 for the big 4-digit 7-segment LED display (Wire1)
#define I2C_ADDR 0x70   // default I2C slave address
// unused Teensy 3.5 pins: 4, 5, 39

enum events { // input events
   eNone,
   ePB1, ePB2, ePB3, ePB4,  // big pushbuttons
   ePBH, ePBV,              // horizontal and vertical encoder pushbuttons
   ePEDAL,                  // foot pedal
   eHrotL, eHrotR,          // horizontal encoder rotation
   eVrotL, eVrotR           // vertical encoder rotation
};
static struct {   // input pins
   byte pin;
   enum events event;
   byte active_state; }
inpins[] = {
   {PB1, ePB1, LOW },
   {PB2, ePB2, LOW },
   {PB3, ePB3, LOW },
   {PB4, ePB4, LOW },
   { H_ENC_PB, ePBH, HIGH },
   { V_ENC_PB, ePBV, HIGH },
   {PEDAL, ePEDAL, LOW },
   {0xff } };

static byte lightpins[] = { // output pins for encoder LEDs
   H_ENC_RED, H_ENC_GREEN, H_ENC_BLUE,
   V_ENC_RED, V_ENC_GREEN, V_ENC_BLUE, 0xff };

static struct  { // output pins for the stepper motors
   byte pin_step;       // what pin starts a step for this motor
   bool clockwise;      // is clockwise up?
#define FRONT_CW false  // (these depend on which way the chains are
#define BACK_CW true    //  looped around the sprockets)
} shaft_hardware[NUM_SHAFTS] = {
   {16, FRONT_CW }, // top front motor
   {15, FRONT_CW },
   {14, FRONT_CW },
   {13, FRONT_CW }, // bottom front motor
   {17, BACK_CW },  // bottom back motor
   {18, BACK_CW },
   {19, BACK_CW },
   {20, BACK_CW }   // top back motor
};

enum shaft_positions {SHAFT_CENTER, SHAFT_UP, SHAFT_DOWN };
#define NOMINAL_STEPS ((ROTATION_PERCENT * STEPS_PER_ROTATION) / 100)
static struct {
   enum shaft_positions shaft_position;  // where is the shaft currently?
   int steps_to_down;   // how many steps move from center to down
   int steps_to_up;     // how many steps move from center to up
   int steps_to_move;   // how many steps up (pos) or down (neg) are left to move
}
shaft_status[NUM_SHAFTS] = {
   {SHAFT_CENTER,  NOMINAL_STEPS,  NOMINAL_STEPS, 0 },  // top front motor
   {SHAFT_CENTER,  NOMINAL_STEPS,  NOMINAL_STEPS, 0 },
   {SHAFT_CENTER,  NOMINAL_STEPS,  NOMINAL_STEPS, 0 },
   {SHAFT_CENTER,  NOMINAL_STEPS,  NOMINAL_STEPS, 0 },  // bottom front motor
   {SHAFT_CENTER,  NOMINAL_STEPS,  NOMINAL_STEPS, 0 },  // bottom back motor
   {SHAFT_CENTER,  NOMINAL_STEPS,  NOMINAL_STEPS, 0 },
   {SHAFT_CENTER,  NOMINAL_STEPS,  NOMINAL_STEPS, 0 },
   {SHAFT_CENTER,  NOMINAL_STEPS,  NOMINAL_STEPS, 0 }   // top back motor
};

// weaving configuration

static bool tieup [NUM_SHAFTS] [NUM_TREADLES] =  { // does shaft i go down with treadle j?
   // initialization: a line is all treadles connected to a particular shaft
   {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // initialized to basic weave
   {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
   /* everything else is 0 */ };
static byte treadle_sequence[MAX_SEQUENCE]; // treadle sequence for this pattern; values are 0..NUM_TREADLES-1
#define UNUSED_SEQ 0xff                     // or 0xff signifying "this position is unused"
static int sequence_size;

//**** EEPROM format for persistent data

#define EEPROM_SIZE 4096
/* layout is:
     config_hdr
     shaft_status []
     repeat {
        file_hdr
        tieup []
        treadle_sequence []
        }
*/
struct {
   char id[4];       // "cfg" for identification
   int size;         // CONFIG_SIZE: size of the whole config record, including this header
   byte numshafts;   // compile-time config parameters for consistency checking
   byte numtreadles;
   byte maxsequence;
   byte lastfilenum;  // the last file we loaded or saved
   byte tr_seq_pos;   // the current index into the treadle sequence for that file
   byte unused[15];
   // after this we write the "shaft_status" array, which contains the current shaft configuration data
} config_hdr = {0 };
#define CONFIG_SIZE ((int)sizeof(config_hdr) + (int)sizeof(shaft_status))

#define MAX_FILENAME 14
struct {
   char id[4]; // "fil" for identification
   int size;   // size of the whole file, including this header
   byte checksum;
   char filename[MAX_FILENAME + 1]; // null-terminated string
   // after this we write the tieup array, followed by the treadle_sequence vector
} file_hdr;
#define FILE_SIZE ((int)sizeof(file_hdr) + (int)sizeof(tieup) + (int)sizeof(treadle_sequence))
#define MAX_FILES ((EEPROM_SIZE - CONFIG_SIZE) / FILE_SIZE)
// The current implementation assumes all files are padded to the fixed maximum size.
// All stored files will break if NUM_SHAFTS, NUM_TREADLES, or MAX_SEQUENCE is changed!

//**** global variables

LiquidCrystalFast lcd (LCD_RS, LCD_RW, LCD_ENB, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
Encoder H_ENC (H_ENC_A, H_ENC_B);
Encoder V_ENC (V_ENC_A, V_ENC_B);
long Hold, Vold;  // last encoder values
#define ENC_STEPS_PER_CLICK 4
char last_filename[MAX_FILENAME + 1] = {0};
bool unsaved_file = false;

//**** utility routines

void assert (boolean test, const char *msg) {
   if (!test) {
      lcd.clear();
      lcd.print("** INTERNAL ERROR **");
      lcd.setCursor(0, 1);
      lcd.print("Assertion failed:");
      lcd.setCursor(0, 2);
      lcd.print(msg);
      while (true) ; } }

void center_message (byte row, const char *msg) {
   int len, nblanks;
   len = strlen(msg);
   assert (len <= 20, "bad center_message");
   nblanks = (20 - len) >> 1;
   lcd.setCursor(0, row);
   for (byte i = 0; i < nblanks; ++i) lcd.print(" ");
   lcd.print(msg);
   nblanks = (20 - nblanks) - len;
   for (byte i = 0; i < nblanks; ++i) lcd.print(" "); }

void info_message(const char *msg1, char *msg2) {
   lcd.clear();
   center_message(1, msg1);
   if (msg2) center_message(2, msg2);
   delay(3000);
   lcd.clear(); }

void wait_for_button(byte pin) {
   while (digitalRead(pin) == HIGH) ;
   delay(DEBOUNCE_MSEC);
   while (digitalRead(pin) == LOW) ;
   delay(DEBOUNCE_MSEC); }

void testbutton (byte pin, bool active) {
   if (digitalRead(pin) == active) {
      Serial.print("pushed: "); Serial.println(pin);
      delay(DEBOUNCE_MSEC);
      while (digitalRead(pin) == active) ;
      Serial.print("release: "); Serial.println(pin);
      delay(DEBOUNCE_MSEC); } }

void H_LED (int led) {
   digitalWrite(H_ENC_RED, led == H_ENC_RED ? LED_ON : LED_OFF);
   digitalWrite(H_ENC_GREEN, led == H_ENC_GREEN ? LED_ON : LED_OFF);
   digitalWrite(H_ENC_BLUE, led == H_ENC_BLUE ? LED_ON : LED_OFF); }
void V_LED (int led) {
   digitalWrite(V_ENC_RED, led == V_ENC_RED ? LED_ON : LED_OFF);
   digitalWrite(V_ENC_GREEN, led == V_ENC_GREEN ? LED_ON : LED_OFF);
   digitalWrite(V_ENC_BLUE, led == V_ENC_BLUE ? LED_ON : LED_OFF); }

//**** big 7-sgement LED display routines

static const uint8_t numbertable[] = {
   0x3F, /* 0 */  0x06, /* 1 */  0x5B, /* 2 */  0x4F, /* 3 */  0x66, /* 4 */
   0x6D, /* 5 */  0x7D, /* 6 */  0x07, /* 7 */  0x7F, /* 8 */  0x6F, /* 9 */
   0x77, /* a */  0x7C, /* b */  0x39, /* C */  0x5E, /* d */  0x79, /* E */  0x71, /* F */
};
uint16_t displaybuffer[8];

void led7_turnoff(void) {
   Wire1.beginTransmission(I2C_ADDR);
   Wire1.write(0x80);  // turn off the display
   Wire1.endTransmission(); }

void led7_start(void) {
   Wire1.begin();    // start LED display
   Wire1.beginTransmission(I2C_ADDR);
   Wire1.write(0x21);  // turn on oscillator
   Wire1.endTransmission();
   Wire1.beginTransmission(I2C_ADDR);
   Wire1.write(0xef);  // set maximum brightness
   Wire1.endTransmission();
   led7_turnoff(); }

void led7_write_digit(byte pos, byte num, bool dot) {
   if (pos > 4) pos = 4;
   if (num > 15) num = 15;
   displaybuffer[pos] = numbertable[num] | (dot << 7); }

void led7_shownum(byte num, bool left) {
   led7_write_digit(left ? 0 : 3, (num / 10) % 10, false);
   led7_write_digit(left ? 1 : 4, num % 10, false);
   Wire1.beginTransmission(I2C_ADDR);
   Wire1.write((uint8_t)0x00); // start at address $00
   for (uint8_t i = 0; i < 8; i++) {
      Wire1.write(displaybuffer[i] & 0xFF);
      Wire1.write(displaybuffer[i] >> 8); }
   Wire1.endTransmission();
   Wire1.beginTransmission(I2C_ADDR);
   Wire1.write(0x81);  // turn on the display
   Wire1.endTransmission(); }

//****** more utility routines

void test_hardware(void) {
   for (int i = 0; lightpins[i] != 0xff; ++i) { // test all the lights
      digitalWrite(lightpins[i], LOW);
      delay(750);
      digitalWrite(lightpins[i], HIGH);
      delay(250); }
   for (int row = 0; row < 4; ++row) {  // test the LCD display
      char buf[21];
      sprintf(buf, "row %d", row);
      center_message(row, buf); }
   for (int leftright = 0; leftright < 2; ++leftright) {
      for (int i = 0; i < 10; ++i) { // test the LED display
         led7_shownum(i, leftright);
         delay(500); }
      for (int i = 0; i < 100; i += 10) {
         led7_shownum(i, leftright);
         delay(500); } }
   long Vold = -999, Hold = -999, Vnew, Hnew;
   while (1) {
      testbutton(PB1, LOW);
      testbutton(PB2, LOW);
      testbutton(PB3, LOW);
      testbutton(PB4, LOW);
      testbutton(PEDAL, LOW);
      testbutton(H_ENC_PB, HIGH);
      testbutton(V_ENC_PB, HIGH);
      Hnew = H_ENC.read();
      if (Hnew != Hold) {
         Serial.print("H pos: "); Serial.println(Hnew);
         Hold = Hnew; }
      Vnew = V_ENC.read();
      if (Vnew != Vold) {
         Serial.print("V pos: "); Serial.println(Vnew);
         Vold = Vnew; } } }

enum events check_event(void) {
   // check for a switch or button event; return eNone if none
   for (int i = 0; inpins[i].pin != 0xff; ++i) { //test all inputs
      // we should be cleverer here: trigger on press, but record to ignore until release
      if (digitalRead(inpins[i].pin) == inpins[i].active_state) { // button pushed
         delay(DEBOUNCE_MSEC);
         while (digitalRead(inpins[i].pin) == inpins[i].active_state) ;  // wait for release
         delay(DEBOUNCE_MSEC);
         return inpins[i].event; } }
   long Hnew = H_ENC.read();
   if (Hnew - Hold >= ENC_STEPS_PER_CLICK) {
      Hold = Hnew;
      return eHrotL; }
   if (Hold - Hnew >= ENC_STEPS_PER_CLICK) {
      Hold = Hnew;
      return eHrotR; }
   long Vnew = V_ENC.read();
   if (Vnew - Vold >= ENC_STEPS_PER_CLICK) {
      Vold = Vnew;
      return eVrotL; }
   if (Vold - Vnew >= ENC_STEPS_PER_CLICK) {
      Vold = Vnew;
      return eVrotR; }
   return eNone; }

bool wait_yesno(const char *msg1, const char *msg2) {
   enum events event;
   bool answer;
   lcd.clear();
   if (msg1) center_message(0, msg1);
   if (msg2) center_message(1, msg2);
   center_message(3, "  yes       no  ");
   H_LED(H_ENC_GREEN);
   V_LED(V_ENC_RED);
   while (1) {
      event = check_event();
      if (event == ePBH) {
         answer = true;
         break; }
      if (event == ePBV) {
         answer =  false;
         break; } }
   H_LED(0);
   V_LED(0);
   lcd.clear();
   return answer; }

int ask_choice(const char *msg1, const char *msg2, const char *msg3, const char *msg4) {
   // return 0 if abort, else choice 1..4
   enum events event;
   int row = 0, numchoices = 2, answer = -1;
   lcd.clear();
   center_message(0, msg1);
   center_message(1, msg2);
   if (msg3) {
      center_message(2, msg3);
      ++numchoices;
      if (msg4) {
         center_message(3, msg4);
         ++numchoices; } }
   V_LED(V_ENC_GREEN);
   H_LED(H_ENC_RED);
   while (answer < 0) {
      lcd.setCursor(10, row); // put cursor in the center of the line
      lcd.blink();
      event = check_event();
      switch (event) {
         case eVrotL: // up
            if (row > 0) --row;;
            break;
         case eVrotR: // down
            if (row < numchoices - 1) ++row;
            break;
         case ePBV:  // green button selects a choice
            answer = row + 1;
            break;
         default:
            if (event != eNone) answer = 0;  // any other button aborts
            break; } }
   lcd.noBlink();
   V_LED(0);
   H_LED(0);
   return answer; }

void lcd_write_dd (int number) {
   lcd.write(number < 10 ? ' ' : '0' + number / 10);
   lcd.write('0' + number % 10);
   lcd.write(':'); }


//**** loom programming routines

void display_tieups(int /*starting*/ shaft, bool title) {
   int row = 3; // start from bottom to match weaving pattern books
   lcd.clear();
   if (title) {
      center_message(row, "Treadle tie-ups");
      --row; }
   for (; row >= 0 && shaft < NUM_SHAFTS; --row, ++shaft) {
      lcd.setCursor(0, row);
      lcd.write('S');
      lcd.write('1' + shaft);
      lcd.write(':');
      for (int treadle = 0; treadle < NUM_TREADLES; ++treadle)
         lcd.write(tieup[shaft] [treadle] ? 'X' : ' '); } }

void display_treadle_sequence(int /*starting*/ sequence_pos, bool title) {
   int row = 0; // start from top to match weaving pattern books
   lcd.clear();
   if (title) {
      center_message(row, "Treadle sequence");
      ++row; }
   for (; row <= 3 && sequence_pos < MAX_SEQUENCE; ++row, ++sequence_pos) {
      byte treadle = treadle_sequence[sequence_pos];
      center_message(row, "");
      lcd.setCursor(0, row);
      lcd_write_dd(sequence_pos + 1);
      if (treadle != UNUSED_SEQ) {
         lcd.setCursor(treadle + 3, row);
         lcd.write('X'); } } }

void program_tieups(void) {
   int shaft_num_on_bottom = 0;
   int shaft_num = 0, treadle_num = 0;
   bool show_title = true;
   enum events event;
   H_LED(H_ENC_GREEN);
   V_LED(V_ENC_RED);
   while (1) {
      display_tieups(shaft_num_on_bottom, show_title);
      lcd.setCursor(treadle_num + 3, 3 - (shaft_num - shaft_num_on_bottom) - (show_title ? 1 : 0));
      lcd.cursor();
      do {
         event = check_event(); }
      while (event == eNone);
      switch (event) {
         case eHrotL:  // move left
            if (treadle_num > 0) --treadle_num;
            break;
         case eHrotR:   // move right
move_right:
            if (treadle_num < NUM_TREADLES - 1) ++treadle_num;
            break;
         case eVrotL:  // move up
            if (shaft_num < NUM_SHAFTS - 1) {
               ++shaft_num;
               treadle_num = 0;
               if (3 - (shaft_num - shaft_num_on_bottom) - (show_title ? 1 : 0) < 0) { // scroll down
                  if (!show_title) ++shaft_num_on_bottom;
                  show_title = false; } }
            break;
         case eVrotR:  // move down
            if (shaft_num > 0) {
               --shaft_num;
               treadle_num = 0;
               if (shaft_num < shaft_num_on_bottom) {// scroll up
                  --shaft_num_on_bottom; } }
            break;
         case ePBH: // green (left, horizontal) button
            unsaved_file = true;
            tieup[shaft_num][treadle_num] = true;
            goto move_right;
         case ePBV: // red (right, vertical) button
            unsaved_file = true;
            tieup[shaft_num][treadle_num] = false;
            goto move_right;
         case ePB1:  // top button: stop programming
            H_LED(0);
            V_LED(0);
            lcd.noCursor();
            return;
         default:; } } }

void cleanup_treadle_sequence(void) {
   // compact the treadle sequence by removing unused lines,
   // and compute the useful size
   int rpos, wpos;
   sequence_size = 0;
   for (rpos = 0, wpos = 0; rpos < MAX_SEQUENCE; ++rpos)
      if (treadle_sequence[rpos] != UNUSED_SEQ) {
         treadle_sequence[wpos++] = treadle_sequence[rpos];
         ++sequence_size; }
   if (DEBUG) {
      Serial.print("sequence size: "); Serial.println(sequence_size); } }

void insert_treadle_sequence(int position) {
   byte saved_treadle = treadle_sequence[position], temp_treadle;;
   treadle_sequence[position++] = UNUSED_SEQ;  // insert unused row
   while (position < MAX_SEQUENCE) {
      temp_treadle = treadle_sequence[position];
      treadle_sequence[position++] = saved_treadle;
      saved_treadle = temp_treadle; } }

void delete_treadle_sequence(int position) {
   for (; position < MAX_SEQUENCE - 1; ++position)
      treadle_sequence[position] = treadle_sequence[position + 1];
   treadle_sequence[position] = UNUSED_SEQ; // new last row is "unused"
}

void program_treadle_sequence(void) {
   int sequence_pos_on_top = 0;
   int sequence_pos = 0, treadle_num = 0;
   bool show_title = true;
   bool row_select = false;  // are we selecting the whole row by having the cursor on the far left?
   enum events event;
   H_LED(H_ENC_GREEN);
   V_LED(V_ENC_RED);
   while (1) {
      display_treadle_sequence(sequence_pos_on_top, show_title);
      lcd.setCursor(row_select ? 0 : treadle_num + 3,
                    sequence_pos - sequence_pos_on_top + (show_title ? 1 : 0));
      if (row_select) {
         lcd.noCursor();
         lcd.blink(); }
      else {
         lcd.noBlink();
         lcd.cursor(); }
      do {
         event = check_event(); }
      while (event == eNone);
      switch (event) {
         case eHrotL:  // move left
            if (treadle_num == 0) {
               row_select = true;
               lcd.noCursor();
               lcd.blink();
               H_LED(H_ENC_BLUE);
               V_LED(V_ENC_RED); }
            else --treadle_num;
            break;
         case eHrotR:   // move right
            if (row_select) {
               row_select = false;
               lcd.noBlink();
               lcd.cursor();
               H_LED(H_ENC_GREEN);
               V_LED(V_ENC_RED); }
            else if (treadle_num < NUM_TREADLES - 1) ++treadle_num;
            break;
         case eVrotR:  // move down
move_down:
            if (sequence_pos < MAX_SEQUENCE - 1) {
               ++sequence_pos;
               if (sequence_pos - sequence_pos_on_top + (show_title ? 1 : 0) > 3) { // scroll up
                  if (!show_title) ++sequence_pos_on_top;
                  show_title = false; } }
            break;
         case eVrotL:  // move up
            if (sequence_pos > 0) {
               --sequence_pos;
               if (sequence_pos < sequence_pos_on_top) {// scroll down
                  --sequence_pos_on_top; } }
            break;
         case ePBH: // green or blue (left, horizontal) button
            unsaved_file = true;
            if (row_select) insert_treadle_sequence(sequence_pos);
            else {
               treadle_sequence[sequence_pos] = treadle_num;  // assign the treadle to this position
               goto move_down; }
            break;
         case ePBV:  // red (right, vertical) button
            unsaved_file = true;
            if (row_select) delete_treadle_sequence(sequence_pos);
            else {
               if (treadle_sequence[sequence_pos] == treadle_num)  // if we're on this treadle
                  treadle_sequence[sequence_pos] = UNUSED_SEQ; // remove it
            }
            break;
         case ePB1:  // program button: stop programming
            H_LED(0);
            V_LED(0);
            lcd.noBlink();
            lcd.noCursor();
            cleanup_treadle_sequence();
            return;
         default:; } } }

// file routines

void dump_file (const char *msg) {
   Serial.println(msg);
   Serial.print(file_hdr.id); Serial.print(", ");
   Serial.print(file_hdr.size); Serial.print(" bytes, checksum "); Serial.println(" checksum");
   Serial.print("name:"); Serial.println(file_hdr.filename);
   Serial.println("tieups:");
   for (int sh = 0; sh < NUM_SHAFTS; ++sh) {
      for (int tr = 0; tr < NUM_TREADLES; ++tr) {
         Serial.print(tieup[sh][tr]); Serial.print(' '); }
      Serial.println(); }
   Serial.print("treadle sequence: ");
   for (int tr = 0; tr < sequence_size; ++tr) {
      Serial.print(treadle_sequence[tr]); Serial.print(' '); }
   Serial.println(); }

void dump_shaft_state(void) {
   static const char *position_name[3] = {  " CT ", " UP ", " DN " };
   Serial.println("shaft state:");
   for (int sh = 0; sh < NUM_SHAFTS; ++sh) {
      Serial.print(sh);  Serial.print(position_name[shaft_status[sh].shaft_position]);
      Serial.print(shaft_status[sh].steps_to_down); Serial.print(", ");
      Serial.print(shaft_status[sh].steps_to_up); Serial.print(", ");
      Serial.print(shaft_status[sh].steps_to_move); Serial.println(); } }

void read_EEPROM (int *fromloc, char * ptr, int count) {
   while (count--)
      *ptr++ = EEPROM.read((*fromloc)++); }

void write_EEPROM(int *toloc, char * ptr, int count) {
   while (count--) {
      EEPROM.write((*toloc)++, *ptr++); } }

void erase_EEPROM(void) {
   if (wait_yesno(0, "delete all files?")) {
      for (int loc = 0; loc < EEPROM_SIZE; ++loc)
         EEPROM.update(loc, 0); } }

void write_config(void) {
   int toloc = 0;
   strcpy(config_hdr.id, "cfg");
   config_hdr.size = CONFIG_SIZE;
   config_hdr.numshafts = NUM_SHAFTS;
   config_hdr.numtreadles = NUM_TREADLES;
   config_hdr.maxsequence = MAX_SEQUENCE;
   // we expect lastfilenum and tr_seq_ndx to have been set
   memset(config_hdr.unused, 0, sizeof(config_hdr.unused));
   write_EEPROM(&toloc, (char *) &config_hdr, sizeof(config_hdr));
   write_EEPROM(&toloc, (char *) shaft_status, sizeof(shaft_status));
   if (DEBUG) Serial.println("***config written");
   if (DEBUG) dump_shaft_state(); }

void read_config(void) {
   int fromloc = 0;
   read_EEPROM(&fromloc, (char*) &config_hdr, sizeof(config_hdr));
   if (strcmp(config_hdr.id, "cfg") == 0) { // valid config in EEPROM
      Serial.print("EEPROM config size:"); Serial.println(config_hdr.size);
      assert (config_hdr.size == CONFIG_SIZE, "bad config size");
      assert (config_hdr.numshafts == NUM_SHAFTS && config_hdr.numtreadles == NUM_TREADLES
              && config_hdr.maxsequence == MAX_SEQUENCE, "inconsistent config");
      read_EEPROM(&fromloc, (char *) shaft_status, sizeof(shaft_status)); // read shaft data
      if (DEBUG) Serial.println("***config read");
      if (DEBUG) dump_shaft_state(); }
   else {
      write_config(); // bad config in EEPROM: rewrite it
      write_file(0); // write the "plain weave" file
   } }

void cleanup_filename (char *name) {
   int i;
   name[MAX_FILENAME] = '\0';  // ensure a null ending
   for (i = MAX_FILENAME - 1; i >= 0; --i) {
      if (name[i] == ' ') name[i] = '\0';  // remove trailing blanks
      else if (name[i] != '\0') break;  // stop at first non-blank, non-null character
   }
   for ( ; i >= 0; --i)
      if (name[i] == '\0') name[i] = ' '; // convert any other embedded nulls to blanks.
}

void write_file (int filenum) {
   int toloc = CONFIG_SIZE + filenum * FILE_SIZE;
   int checksum = 0;
   assert(filenum < MAX_FILES, "bad write_file");
   strcpy (file_hdr.id, "fil");
   file_hdr.size = FILE_SIZE;
   file_hdr.checksum = 0;
   for (unsigned int i = 0; i < sizeof(file_hdr); ++i)
      checksum += ((char *)&file_hdr) [ i];
   file_hdr.checksum = -checksum;
   write_EEPROM(&toloc, (char*) &file_hdr, sizeof(file_hdr));
   write_EEPROM(&toloc, (char*) tieup, sizeof(tieup));
   write_EEPROM(&toloc, (char*) treadle_sequence, sizeof(treadle_sequence));
   config_hdr.lastfilenum = filenum;
   strcpy(last_filename, file_hdr.filename);
   unsaved_file = false;
   if (DEBUG) dump_file("file saved:"); }

void read_file_hdr (int filenum) {
   int fromloc = CONFIG_SIZE + filenum * FILE_SIZE;
   assert (filenum < MAX_FILES, "bad read_file_hdr");
   read_EEPROM(&fromloc, (char*) &file_hdr, sizeof(file_hdr));
   if (strcmp(file_hdr.id, "fil") == 0) { // it's a file
      byte checksum = 0;
      assert (file_hdr.size == FILE_SIZE, "bad file size");
      for (unsigned int i = 0; i < sizeof(file_hdr); ++i)
         checksum += ((char *)&file_hdr) [ i];
      assert(checksum == 0, "bad file checksum");
      cleanup_filename(file_hdr.filename); }
   else { // damaged or never-used header: make it look good
      strcpy (file_hdr.id, "fil");
      memset(file_hdr.filename, '\0', MAX_FILENAME + 1);
      file_hdr.size = FILE_SIZE; } }

void read_file_data (int filenum) {
   int fromloc = CONFIG_SIZE + filenum * FILE_SIZE + (int)sizeof(file_hdr);
   assert (filenum < MAX_FILES, "bad read_file_data");
   read_EEPROM(&fromloc, (char*) tieup, sizeof(tieup));
   read_EEPROM(&fromloc, (char*) treadle_sequence, sizeof(treadle_sequence));
   cleanup_treadle_sequence();
   config_hdr.lastfilenum = filenum;
   strcpy(last_filename, file_hdr.filename);
   unsaved_file = false;
   if (DEBUG) dump_file("file read:"); }

void display_files (int filenum) {
   lcd.clear();
   for (int row = 0; row <= 3 && filenum < MAX_FILES; ++row, ++filenum) {
      center_message(row, "");
      lcd.setCursor(0, row);   //nn:nnnnnnnnnnnnnnn
      lcd_write_dd(filenum + 1);
      read_file_hdr(filenum);
      lcd.print(file_hdr.filename); } }

char changeletter(char letter, bool forward) {
   static char filenamechars[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789!@#$%^&*()_+-,.<>?/'";
   int ndx = 0; char *letterptr;
   if (letter != 0) { // 0 gets treated like blank
      assert ((letterptr = strchr(filenamechars, letter )) != NULL, "bad filename char");
      ndx = letterptr - filenamechars; }
   if (forward) { // forwards in lexical order
      if (++ndx >= (int)sizeof(filenamechars) - 1) ndx = 0; }
   else { // backwards in lexical order
      if (--ndx < 0) ndx = (int)sizeof(filenamechars) - 2; }
   return filenamechars[ndx]; }

void loadsave_files (void) {
   int filenum = 0, filenum_on_top = 0, charnum = 0, choice;
   bool row_select = true; // are we selecting the whole row by having the cursor on the far left?
   bool save;
   enum events event;
   char string[25];

   choice = ask_choice("load file?", "save file?", "erase memory?", 0);
   if (choice == 0)
      return;
   if (choice == 3) {
      erase_EEPROM();
      return; }
   save = choice == 2;  // choice is either 1 for load, or 2 for save

   // highlight the last file we loaded or saved on line 2, maybe
   filenum = config_hdr.lastfilenum < MAX_FILES ? config_hdr.lastfilenum : 0;
   filenum_on_top = filenum <= 2 ? 0 : filenum - 2;

   row_select = true;
   display_files(filenum_on_top);
   read_file_hdr(filenum);
   while (1) {
      H_LED(save && row_select ? H_ENC_GREEN : 0);
      V_LED(save ? (row_select ? V_ENC_RED : V_ENC_BLUE) : V_ENC_GREEN);
      lcd.setCursor(row_select ? 0 : 3 + charnum, filenum - filenum_on_top);
      if (row_select) {
         lcd.noCursor();
         lcd.blink(); }
      else {
         lcd.noBlink();
         lcd.cursor(); }
      do {
         event = check_event(); }
      while (event == eNone);
      switch (event) {
         case eHrotL:  // move left
            if (save) {
               if (charnum == 0) {
                  row_select = true;
                  lcd.noBlink();
                  lcd.cursor(); }
               else --charnum; }
            break;
         case eHrotR:   // move right
            if (save) {
               if (row_select) {
                  row_select = false;
                  lcd.noCursor();
                  lcd.blink(); }
               else if (charnum < MAX_FILENAME) {
                  ++charnum;
                  if (file_hdr.filename[charnum] == '\0') { // if we're going beyond the end
                     file_hdr.filename[charnum] = ' ';        // change NUL to a blank
                     file_hdr.filename[charnum + 1] = '\0';   // and make a new NUL end
                  } } }
            break;
         case eVrotR:  // move down
            if (!save || row_select) {
               if (filenum < MAX_FILES - 1) {
                  ++filenum;
                  if (filenum - filenum_on_top > 3)  // scroll up
                     ++filenum_on_top;
                  display_files(filenum_on_top);
                  read_file_hdr(filenum);
                  charnum = 0; } }
            else {  // save, in name: change filename char
               file_hdr.filename[charnum] = changeletter(file_hdr.filename[charnum], true);
               goto update_letter; }
            break;
         case eVrotL:  // move up
            if (!save || row_select) {
               if (filenum > 0) {
                  --filenum;
                  if (filenum < filenum_on_top) // scroll down
                     --filenum_on_top;
                  display_files(filenum_on_top);
                  read_file_hdr(filenum);
                  charnum = 0; } }
            else { // save, in name: change filename char
               file_hdr.filename[charnum] = changeletter(file_hdr.filename[charnum], false);
update_letter:
               lcd.write(file_hdr.filename[charnum]);
               lcd.setCursor(3 + charnum, filenum - filenum_on_top); // restore cursor position
            }
            break;
         case ePBH: // left (horizontal) button
            if (save) {
               if (row_select) { // write this file
                  cleanup_filename(file_hdr.filename);
                  write_file(filenum);
                  sprintf (string, "file %d saved", filenum + 1);
                  info_message(string, file_hdr.filename);
                  write_config(); // record this file number in EEPROM
                  goto exit_loadsave; } }
            break;
         case ePBV:  // right (vertical) button
            if (save) {
               if (!row_select) { // name entry: go to next char
                  if (charnum < MAX_FILENAME) ++charnum; }
               else { // abort writing
                  cleanup_filename(file_hdr.filename);
                  info_message("file save aborted", 0);
                  goto exit_loadsave; } }
            else { // load file
               read_file_data(filenum);
               sprintf (string, "file %d loaded", filenum + 1);
               info_message(string, file_hdr.filename);
               write_config(); // record this file number in EEPROM
               goto exit_loadsave; }
            break;
         case ePB1:  // other buttons: exit
         case ePB2:
         case ePB3:
         case ePB4:
exit_loadsave:
            H_LED(0);
            V_LED(0);
            lcd.noBlink();
            lcd.noCursor();
            return;
         default:; } } }

//**** weaving routines

bool using_shaft(int shaft) {  // does any used treadle tie up to this shaft?
   for (int pos = 0; pos < sequence_size; ++pos)
      if (treadle_sequence[pos] != UNUSED_SEQ && tieup[shaft][treadle_sequence[pos]])
         return true;
   return false; }

void do_steps() {  // do queued-up steps for all motors
   bool step_done;
   unsigned int light_counter = STEPS_PER_LIGHT_CHANGE, light_index = 0; // for an amusing light display while weaving
   static byte light_sequence[] = {H_ENC_RED, V_ENC_GREEN, H_ENC_BLUE, V_ENC_RED, H_ENC_GREEN, V_ENC_BLUE };

   assert(digitalRead(MOTOR_FAULT) == HIGH, "motor failure");
   do {
      step_done = false;
      for (int shaft = 0; shaft < NUM_SHAFTS; ++shaft) { // figure out which shafts to step
         if (shaft_status[shaft].steps_to_move < 0) { // moving down
            digitalWrite(MOTOR_DIR, 1 - shaft_hardware[shaft].clockwise); // set direction to "down"
            ++shaft_status[shaft].steps_to_move; // one fewer to do
            goto do_step; }
         if (shaft_status[shaft].steps_to_move > 0) { // moving up
            digitalWrite(MOTOR_DIR, shaft_hardware[shaft].clockwise); // set direction to "up"
            --shaft_status[shaft].steps_to_move;  // one fewer to do
do_step:
            digitalWrite(shaft_hardware[shaft].pin_step, HIGH); // do a step on this shaft
            delayMicroseconds(5);
            digitalWrite(shaft_hardware[shaft].pin_step, LOW);
            delayMicroseconds(5);
            step_done = true; } }
      if (step_done) {
         if (++light_counter >= STEPS_PER_LIGHT_CHANGE) {
            light_counter = 0;
            digitalWrite(light_sequence[light_index], LED_OFF);
            if (++light_index >= sizeof(light_sequence)) light_index = 0;
            digitalWrite(light_sequence[light_index], LED_ON); }
         delayMicroseconds(MIN_STEP_uSEC);
         assert(digitalRead(MOTOR_FAULT) == HIGH, "motor failure"); } }
   while (step_done);
   H_LED(0);
   V_LED(0); }

void do_treadle(int treadle) { // queue up and then do the shaft motion for a treadle push
   #if DEBUG
   dump_shaft_state();
   Serial.print("treadle "); Serial.print(treadle); Serial.print(": ");
   for (int shaft = 0; shaft < NUM_SHAFTS; ++shaft) {
      Serial.print(shaft); Serial.print(tieup[shaft][treadle] ? " dn, " : " up, "); }
   Serial.println();
   #endif
   for (int shaft = 0; shaft < NUM_SHAFTS; ++shaft) {
      if (using_shaft(shaft)) {  // if we ever use this shaft in our pattern, then move it up or down
         if ( tieup[shaft] [treadle]) { // this shaft should go down
            if (shaft_status[shaft].shaft_position == SHAFT_UP)
               shaft_status[shaft].steps_to_move = -shaft_status[shaft].steps_to_down - shaft_status[shaft].steps_to_up;
            if (shaft_status[shaft].shaft_position == SHAFT_CENTER)
               shaft_status[shaft].steps_to_move = -shaft_status[shaft].steps_to_down; }
         else { // this shaft should go up
            if (shaft_status[shaft].shaft_position == SHAFT_DOWN)
               shaft_status[shaft].steps_to_move = shaft_status[shaft].steps_to_down + shaft_status[shaft].steps_to_up;
            if (shaft_status[shaft].shaft_position == SHAFT_CENTER)
               shaft_status[shaft].steps_to_move = shaft_status[shaft].steps_to_up;  }
         if (DEBUG) {
            Serial.print("shaft "); Serial.print(shaft);
            Serial.print(" moves "); Serial.println(shaft_status[shaft].steps_to_move); } } }
   do_steps();
   for (int shaft = 0; shaft < NUM_SHAFTS; ++shaft) // record the ending position of the shafts
      shaft_status[shaft].shaft_position = tieup[shaft] [treadle] ? SHAFT_DOWN : SHAFT_UP; }

int get_calibration(int shaft, const char *msg, bool allow_abort) {  // calibrate the center, bottom, or top position of a shaft
   // returns the number of steps down (negative) or up (positive) we went, or 999 if aborted
   enum events event;
   int steps = 0, stepadj;
   char str[25];
   lcd.clear();
   sprintf(str, "move shaft %d", shaft + 1);
   center_message(0, str);
   sprintf(str, "%s, push green", msg);
   center_message(1, str);
   if (allow_abort) {
      center_message(3, "red button skips");
      H_LED(H_ENC_RED); }
   V_LED(V_ENC_GREEN);
   while (1) {
      event = check_event();
      if (event == eVrotR) {
         digitalWrite(MOTOR_DIR, 1 - shaft_hardware[shaft].clockwise);  // set direction to "down"
         stepadj = -1;
         goto do_move; }
      if (event == eVrotL) {
         digitalWrite(MOTOR_DIR, shaft_hardware[shaft].clockwise); // set direction to "up"
         stepadj = +1;
do_move:
         for (int num_moves = 0; num_moves < uSTEPS_PER_STEP; ++num_moves) {  // do all microsteps of a full original step
            delayMicroseconds(5);
            digitalWrite(shaft_hardware[shaft].pin_step, HIGH); // do one step
            delayMicroseconds(5);
            digitalWrite(shaft_hardware[shaft].pin_step, LOW);
            delayMicroseconds(MIN_STEP_uSEC);
            steps += stepadj; } }
      if (allow_abort && event == ePBH) {
         steps = 999;
         goto do_exit; }
      if (event == ePBV) {
do_exit:
         V_LED(0);
         H_LED(0);
         if (DEBUG) {
            Serial.print("shaft "); Serial.print(shaft); Serial.print(' ');
            Serial.print(msg); Serial.print(", steps:"); Serial.println(steps); }
         return steps; } } }

void do_calibration(void) {  // calibrate the center, bottom, and top positions of all shafts in use
   int shaft;
   for (shaft = 0; shaft < NUM_SHAFTS; ++shaft) {  // initialize all shafts as centered and nominal
      shaft_status[shaft].shaft_position = SHAFT_CENTER;
      shaft_status[shaft].steps_to_down = shaft_status[shaft].steps_to_up = NOMINAL_STEPS;
      shaft_status[shaft].steps_to_move = 0; }
   for (shaft = 0; shaft < NUM_SHAFTS; ++shaft) {  // now ask to recalibrate
      if (using_shaft(shaft)) {                    //  the shafts in use for this pattern
         if (get_calibration(shaft, "center", true) != 999) { // not calibrating this shaft
            shaft_status[shaft].steps_to_down = -get_calibration(shaft, "down", false);
            shaft_status[shaft].steps_to_move = shaft_status[shaft].steps_to_down; // return it to the center
            do_steps();  // (it will be the only motor to move)
            shaft_status[shaft].steps_to_up = get_calibration(shaft, "up", false);
            shaft_status[shaft].steps_to_move = -shaft_status[shaft].steps_to_up; // return it to the center
            do_steps(); // (it will be the only motor to move)
         } } }
   write_config(); // write the shaft calibration data
   info_message("calibration saved", 0); }

void center_all_shafts(void) {
   for (int shaft = 0; shaft < NUM_SHAFTS; ++shaft) {
      if (using_shaft(shaft)) switch (shaft_status[shaft].shaft_position) {
            case SHAFT_UP:
               shaft_status[shaft].steps_to_move = -shaft_status[shaft].steps_to_up;
               break;
            case SHAFT_DOWN:
               shaft_status[shaft].steps_to_move = shaft_status[shaft].steps_to_down;
               break;
            case SHAFT_CENTER:
               shaft_status[shaft].steps_to_move = 0;
               break; }
      shaft_status[shaft].shaft_position = SHAFT_CENTER; }
   do_steps(); }

void weave (void) {  // enter weaving mode
   int shuttle_count = 0;
   enum events event;
   bool before_movement = true; // we haven't yet moved to the position shown on the screen
   unsigned long last_weave_time;
   bool motors_on;

   digitalWrite(MOTOR_ENB, LOW);  // enable the motors
   motors_on = true;
   last_weave_time = millis();
   assert(digitalRead(MOTOR_FAULT) == HIGH, "motor failure");
   if (wait_yesno("shed must be closed", "do calibration?"))
      do_calibration();
   Serial.println("weaving");
   lcd.clear();
   lcd.blink();
   display_treadle_sequence(config_hdr.tr_seq_pos, false);  // show the position we are about to move to
   lcd.setCursor(0, 0);  // indicate that by putting the cursor before the sequence number
   led7_shownum(shuttle_count, true);
   while (1) {// do the treadle sequence forever
      V_LED(V_ENC_BLUE); // signal "ok to change rows"
      event = check_event();
      switch (event) {
         case ePB1:  // any other button stops the weaving
         case ePB2:
         case ePB3:
            lcd.noBlink();
            V_LED(0);
            led7_turnoff();       // turn off the number display
            center_all_shafts();  // close the shed
            digitalWrite(MOTOR_ENB, HIGH);  // disable the motors
            write_config(); // record the updated treadle sequence position
            return;
         case ePEDAL:  // next in treadle sequence
do_pedal:
            if (!motors_on) {
               digitalWrite(MOTOR_ENB, LOW);  // enable the motors again
               delay(500);  // delay for power supply ramp
               motors_on = true; }
            last_weave_time = millis();
            if (!before_movement) {  // we need to move to a new position
               if (++config_hdr.tr_seq_pos >= sequence_size) config_hdr.tr_seq_pos = 0;
               display_treadle_sequence(config_hdr.tr_seq_pos, false);
               led7_shownum(++shuttle_count, true); }
            if (treadle_sequence[config_hdr.tr_seq_pos] != UNUSED_SEQ) {
               lcd.setCursor(3 + treadle_sequence[config_hdr.tr_seq_pos], 0); // indicate that we will have moved to that position
               // by highlighting the treadle itself
               do_treadle(treadle_sequence[config_hdr.tr_seq_pos]); }
            before_movement = false;
            break;
         case ePB4: // clear shuttle counter
            shuttle_count = 0;
            led7_shownum(shuttle_count, true);
            break;
         case ePBV:
         case eVrotL:   // go up (backward) in treadle sequence
            if (--config_hdr.tr_seq_pos < 0) config_hdr.tr_seq_pos = sequence_size - 1;
            if (shuttle_count > 0) --shuttle_count;
            goto new_position;
         case eVrotR:  // go down (forward) in treadle sequence
            if (++config_hdr.tr_seq_pos >= sequence_size) config_hdr.tr_seq_pos = 0;
            ++shuttle_count;
new_position:
            before_movement = true;
            display_treadle_sequence(config_hdr.tr_seq_pos, false);
            lcd.setCursor(0, 0); // indicate that we are displaying a position we aren't at
            led7_shownum(shuttle_count, true);
            if (event == ePBV) goto do_pedal;
            break;
         default:
            if (motors_on && (millis() - last_weave_time) > MINS_BEFORE_MOTOR_DISABLE * 1000L * 60L) {
               digitalWrite(MOTOR_ENB, HIGH);  // disable the motors because of inactivity
               motors_on = false;; } } } }

//**** initialization

void setup(void) {
   char string[25];
   #if DEBUG
   Serial.begin(115200);
   //while (!Serial) ; // hangs if serial not connected
   delay(1000);
   Serial.println("loom program");
   #endif

   for (int shaft = 0; shaft < NUM_SHAFTS; ++shaft)
      pinMode(shaft_hardware[shaft].pin_step, OUTPUT);
   pinMode(MOTOR_DIR, OUTPUT);
   pinMode(MOTOR_ENB, OUTPUT);
   pinMode(MOTOR_FAULT, INPUT_PULLUP);
   digitalWrite(MOTOR_ENB, HIGH); // disable motors
   for (int i = 0; inpins[i].pin != 0xff; ++i) { // configure all user inputs
      pinMode (inpins[i].pin, INPUT_PULLUP); }
   Vold = V_ENC.read();
   Hold = H_ENC.read();
   for (int i = 0; lightpins[i] != 0xff; ++i) { // configure the lights
      pinMode(lightpins[i], OUTPUT);
      digitalWrite(lightpins[i], HIGH); // turn off (active low)
   }
   lcd.begin(20, 4); // start LCD display
   if (DEBUG) Serial.println("LCD started");
   led7_start();     // start LED display
   if (DEBUG) Serial.println("LED started");

   treadle_sequence[0] = 0;// initialize treadle sequence to "tabby"
   treadle_sequence[1] = 1;
   sequence_size = 2;
   for (int i = 2; i < MAX_SEQUENCE; ++i)
      treadle_sequence[i] = UNUSED_SEQ;;

   #if HW_TEST
   Serial.println("pin config done");
   test_hardware();
   #endif
   if (DEBUG) {
      Serial.print("config size: "); Serial.println(CONFIG_SIZE);
      Serial.print("file size: "); Serial.println(FILE_SIZE);
      Serial.print("max files: "); Serial.println(MAX_FILES);
      if (digitalRead(PB4) == LOW) // magic: if black button is down during powerup
         erase_EEPROM();            // then offer to erase the memory
      dump_file( "initial state");
      Serial.print("config size:"); Serial.println(CONFIG_SIZE);
      Serial.print("file size:"); Serial.println(FILE_SIZE); }
   read_config();
   if (config_hdr.lastfilenum < MAX_FILES) {
      read_file_hdr(config_hdr.lastfilenum);
      read_file_data(config_hdr.lastfilenum);
      sprintf (string, "file %d reloaded", config_hdr.lastfilenum + 1);
      info_message(string, file_hdr.filename); } }

//**** the main loop

void loop(void) {
   enum events event;
   char string[22];
   lcd.clear();
   sprintf(string, "eLoom V%s", VERSION);
   center_message(0, string);
   center_message(3, unsaved_file ? "<unsaved file>" : last_filename);
   do {
      event = check_event(); }
   while (event == eNone);
   switch (event) {
      case ePB1:
         program_tieups();
         program_treadle_sequence();
         break;
      case ePB2:
         weave();
         break;
      case ePB3:
         loadsave_files();
         break;
      default: ; } }
//*


