/*
* Full-board test for the Spartronics Experimenter board
*
* Written by Eric B. Wertz 2009/10/18
* Last modified 2010/01/27
*
* Edushield pin assignments
* AnalogIn Pin 0 - temp sensor
* AnalogIn Pin 1 - potentiometer
* AnalogIn Pin 2 - LDR/photocell
* AnalogIn Pin 3-5 - unused
* Digital Pin 0 - reserved for UART
* Digital Pin 1 - reserved for UART
* Digital Pin 2 - unused (INT0)
* Digital Pin 3 - LED3/RGB_red (INT1, pwm, 3+6+9+11 jumperable)
* Digital Pin 4 - SW0 (I2C SCL, T0)
* Digital Pin 5 - speaker (I2C SDA, pwm, jumMiperable)
* Digital Pin 6 - LED2/RGB_green (AIN0, pwm, 3+6+9+11 jumperable)
* Digital Pin 7 - SW3 (AIN1)
* Digital Pin 8 - SW2 (ICP)
* Digital Pin 9 - LED1/RGB_blue (OC1, pwm, 3+6+9+11 jumperable)
* Digital Pin 10 - servo (SPI_SS, pwm)
* Digital Pin 11 - LED0 (SPI_MOSI, pwm, 3+6+9+11 jumperable)
* Digital Pin 12 - SW1 (SPI_MISO)
* Digital Pin 13 - unused (SPI_SCK, built-in LED)
* Digital Pin 22-29 - unused
* Digital Pin 30-33 - LED4-7
* Digital Pin 34-37 - SW4-7
* Digital Pin 38-44 - LEDdigit1a-g
* Digital Pin 45 - LEDdigit1dp
* Digital Pin 46-52 - LEDdigit2a-g
* Digital Pin 53 - LEDdigit2dp
*
* This program demonstrates the following:
* - flashing of LEDs
* - fade up/down each of the RGB channels
* - scanning for button presses
* - determine full range of potentiometer values
* - react to LDR/photocell transitionsi
* - react to temp sensor transitions
* - "record" and play tones on the speaker
* - manipulate servo
* - demonstrate use of C-preprocessor based on hardware configuration
*/
//#define ARDUINO_MEGA
//#define DEBUG
#include <Servo.h>
const int MSECS_PER_SEC = 1000;
const long USECS_PER_SEC = 1000000L;
const int ANALOG_IN_MIN = 0, ANALOG_IN_MAX = 1023; // 10-bit ADCs for input
const int ANALOG_OUT_MIN = 0, ANALOG_OUT_MAX = 255; // 8-bit resolution PWM for output
// ANALOG pin assignments
const int pinTemp = 0;
const int pinPot = 1;
const int pinPhotocell = 2;
// DIGITAL pin assignments
//const int pinLED0 = 11;
//const int pinLED1_B = 9;
//const int pinLED2_G = 6;
//const int pinLED3_R = 3;
//const int pinLED4 = 30;
//const int pinLED5 = 31;
//const int pinLED6 = 32;
//const int pinLED7 = 33;
const int pinSpeaker = 5;
const int pinServo = 10;
//const int pinButton0 = 12;
//const int pinButton1 = 8;
//const int pinButton2 = 7;
//const int pinButton3 = 4;
//const int pinButton4 = 34;
//const int pinButton5 = 35;
//const int pinButton6 = 36;
//const int pinButton7 = 37;
//const int pin7Seg1a = 38;
//const int pin7Seg1b = 39;
//const int pin7Seg1c = 40;
//const int pin7Seg1d = 41;
//const int pin7Seg1e = 42;
//const int pin7Seg1f = 43;
//const int pin7Seg1g = 44;
//const int pin7Seg1dp = 45;
//const int pin7Seg2a = 46;
//const int pin7Seg2b = 47;
//const int pin7Seg2c = 48;
//const int pin7Seg2d = 49;
//const int pin7Seg2e = 50;
//const int pin7Seg2f = 51;
//const int pin7Seg2g = 52;
//const int pin7Seg2dp = 53;
const int PIN_UNDEFINED = -1;
const int BUTTON_NONE = -1;
/*
* Number of milliseconds to wait for a switch to stop bouncing before checking again.
*/
const unsigned int BUTTON_DEBOUNCE_DELAY = 50;
/*
* Number of milliseconds to light at one intensity level before changing.
* Must be less than or equal to 655 to not overflow an "int" when multiplied
* by 100 (percent).
*/
const unsigned int LED_STEP_PERIOD = 25;
const unsigned int POT_SAMPLE_DELAY = 250;
/*
* For choosing between printing base-10 and base-16 values, either on the
* COM port or on the 7-segment LED.
*/
const unsigned int RADIX_BASE10 = 10;
const unsigned int RADIX_BASE16 = 16;
const byte MAX_NAMELEN = 16;
#ifdef ARDUINO_MEGA
const byte buttons[] = { 12, 8, 7, 4, 34, 35, 36, 37 };
const byte leds_red[] = { 11, 9, 6, 3, 30, 31, 32, 33 };
#else
const byte buttons[] = { 12, 8, 7, 4 };
const byte leds_red[] = { 11, 9, 6, 3 };
#endif
const unsigned int numButtons = sizeof(buttons) / sizeof(buttons[0]);
const unsigned int numLEDs = sizeof(leds_red) / sizeof(leds_red[0]);
const byte leds_RGB[] = { 3, 9, 6 };
const char *names_RGB[] = { "red", "green", "blue" };
#if 0 //(sizeof(buttons) != sizeof(leds_red))
# error "we are counting on numButtons==numLEDs"
#endif
#if 0 //((sizeof(buttons)/sizeof(buttons[0])) > 16)
# error "too many buttons for masking in lightLEDsWithButtons()"
#endif
#ifdef ARDUINO_MEGA
const byte seg7digit1[] = {38, 39, 40, 41, 42, 43, 44, 45};
const byte seg7digit2[] = {46, 47, 48, 49, 50, 51, 52, 53};
#endif
const unsigned int LED_SCAN_DELAY = 1000;
const unsigned int BUTTON_SCAN_DELAY = 100;
const unsigned int PHOTOCELL_POLL_DELAY = 250;
const unsigned int TEMP_POLL_DELAY = 1000;
const unsigned int SERVO_SAMPLE_DELAY = 250;
const byte HEATLEVEL_AMBIENT = 0;
const byte HEATLEVEL_WARM = 1;
const byte HEATLEVEL_HOT = 2;
/* warm,hot delta temp thresholds in degrees C */
const byte HEATDIFF_WARM = 2;
const byte HEATDIFF_HOT = 4;
const unsigned int SERVO_DEGREES_MIN = 0, SERVO_DEGREES_MAX = 180;
const unsigned int FREQ_MIN = 600, FREQ_MAX = 2400; // use two octaves for output
const unsigned int TONES_MAX = 30; // maximum number of tones to record
const unsigned int TONE_PLAYBACK_PERIOD = 500; // determines speed of playback
/* Detect (software debounced) button press */
boolean isButtonDown(int button)
{
  int pin = buttons[button];
  if (digitalRead(pin) == HIGH)
    return false;
  delay(BUTTON_DEBOUNCE_DELAY);
  if (digitalRead(pin) == LOW) {
# ifdef DEBUG
    Serial.print("[S"); Serial.print(button); Serial.println("]");
# endif
    return true;
  }
  else return false;
}
/* Detect (software debounced) button release */
boolean isButtonUp(int button)
{
  int pin = buttons[button];
  if (digitalRead(pin) == LOW)
    return false;
  delay(BUTTON_DEBOUNCE_DELAY);
  return (digitalRead(pin) == HIGH);
}
void waitForPressAndRelease(int button)
{
  while (isButtonUp(button))
    ;
  while (isButtonDown(button))
    ;
}
int whichButtonDown()
{
  int i;
  for (i = 0; i < numButtons; i++)
    if (isButtonDown(i)) {
# ifdef DEBUG
      Serial.print("<S"); Serial.print(i); Serial.println(">");
# endif
      return i;
    }
  return BUTTON_NONE;
}
boolean isAnyButtonDown()
{
  return (whichButtonDown() != BUTTON_NONE);
}
int waitForAnyPress(boolean waitForRelease)
{
  int index;
  do {
    index = whichButtonDown();
  } while (index == BUTTON_NONE);
# ifdef DEBUG
  Serial.print("<<S"); Serial.print(index); Serial.println(">>");
# endif
  if (waitForRelease) {
    while (isButtonDown(index))
      ;
  }
  return index;
}
#ifdef ARDUINO_MEGA
/*
* Display 0..(radix-1) on a 7-segment LED for radix<=16.
*/
void display7SegmentDigit(int n, const byte *segmentPins)
{
  boolean a, b, c, d, e, f, g;
  a = ((n == 0) || (n == 2) || (n == 3) || (n == 5) || (n == 6) || (n == 7) || (n == 8) || (n == 9) || (n == 10) || (n == 12) || (n == 14) ||
       (n == 15));
  b = ((n == 0) || (n == 1) || (n == 2) || (n == 3) || (n == 4) || (n == 7) || (n == 8) || (n == 9) || (n == 10) || (n == 13));
  c = ((n == 0) || (n == 1) || (n == 3) || (n == 4) || (n == 5) || (n == 6) || (n == 7) || (n == 8) || (n == 9) || (n == 10) || (n == 11) ||
       (n == 13));
  d = ((n == 0) || (n == 2) || (n == 3) || (n == 5) || (n == 6) || (n == 8) || (n == 11) || (n == 12) || (n == 13) || (n == 14));
  e = ((n == 0) || (n == 2) || (n == 6) || (n == 8) || (n == 10) || (n == 11) || (n == 12) || (n == 13) || (n == 14) || (n == 15));
  f = ((n == 0) || (n == 4) || (n == 5) || (n == 6) || (n == 8) || (n == 9) || (n == 10) || (n == 11) || (n == 12) || (n == 14) || (n == 15));
  g = ((n == 2) || (n == 3) || (n == 4) || (n == 5) || (n == 6) || (n == 8) || (n == 9) || (n == 10) || (n == 11) || (n == 13) || (n == 14) ||
       (n == 15));
  digitalWrite(segmentPins[0], a ? HIGH : LOW);
  digitalWrite(segmentPins[1], b ? HIGH : LOW);
  digitalWrite(segmentPins[2], c ? HIGH : LOW);
  digitalWrite(segmentPins[3], d ? HIGH : LOW);
  digitalWrite(segmentPins[4], e ? HIGH : LOW);
  digitalWrite(segmentPins[5], f ? HIGH : LOW);
  digitalWrite(segmentPins[6], g ? HIGH : LOW);
}
void display7SegmentDash(const byte *segmentPins)
{
  display7SegmentOff(segmentPins);
  digitalWrite(segmentPins[6], HIGH);
}
void display7SegmentOff(const byte *segmentPins)
{
  for (int i = 0; i < 8; i++)
    digitalWrite(segmentPins[i], LOW);
}
/*
* Display 0..99 or 0..FF on dual 7-segment LEDs for radix<=16, respectively.
*/
void display7SegmentPair(unsigned int n, unsigned int radix,
                         const byte *pinsDigit1, const byte *pinsDigit2)
{
  unsigned int rangesize = radix * radix;
  if (n >= rangesize) {
    display7SegmentDash(pinsDigit1);
    display7SegmentDash(pinsDigit2);
  }
  else {
    n %= rangesize;
    display7SegmentDigit(n / radix, pinsDigit1);
    display7SegmentDigit(n % radix, pinsDigit2);
  }
}
/*
* Blank both 7-segment LEDs.
*/
void display7SegmentPairOff(const byte *pinsDigit1, const byte *pinsDigit2)
{
  display7SegmentOff(pinsDigit1);
  display7SegmentOff(pinsDigit2);
}
#endif
void waitForKeyboardInput()
{
  while (!Serial.available())
    ;
}
void displayInt(int n, byte radix, boolean addNewline)
{
# ifdef ARDUINO_MEGA
  display7SegmentPair(n, radix, seg7digit1, seg7digit2);
# endif
  switch (radix) {
    case RADIX_BASE10: Serial.print(n, DEC); break;
    case RADIX_BASE16: Serial.print(n, HEX); break;
    default: Serial.print(n); break;
  }
  if (addNewline)
    Serial.println();
}
void displayOff()
{
# ifdef ARDUINO_MEGA
  display7SegmentPairOff(seg7digit1, seg7digit2);
# endif
}
/*
* Fade through specified LED set until a key is pressed.
* Check to see if the button is depressed at every intensity step to be more responsive.
*/
void doLEDFades()
{
  boolean done = false;
  Serial.print ("Press any button after setting the RED LED jumper, and again to end test.");
  waitForAnyPress(true);
  while (!done) {
    for (int i = 0; i < numLEDs; i++) {
      int ledpin = leds_red[i];
      for (int pct = 10 ; !done && (pct <= 100) ; pct += 10) {
        analogWrite(ledpin, (pct * ANALOG_OUT_MAX) / 100);
        delay(LED_STEP_PERIOD);
        done = isAnyButtonDown();
      }
      for (int pct = 100 ; !done && (pct >= 0) ; pct -= 10) {
        analogWrite(ledpin, (pct * ANALOG_OUT_MAX) / 100);
        delay(LED_STEP_PERIOD);
        done = isAnyButtonDown();
      }
    }
  }
  while (isAnyButtonDown())
    ; // wait for button to get released before continuing
}
/*
* Fade through specified LED set until a key is pressed.
* Check to see if the button is depressed at every intensity step to be more responsive.
*/
void doRGBMixing(int potMin, int potMax)
{
  int level, lastLevel = 0, button, editing = BUTTON_NONE;
  Serial.println("Set jumper to use the RGB LED now, then use buttons 0,1,2");
  Serial.println(" to select R,G,B, then use the pot to dial the desired intensity.");
  Serial.println("Press button 3 to end test.");
  do {
    button = whichButtonDown();
    if ((button == BUTTON_NONE) && (editing == BUTTON_NONE))
      continue; // still waiting for first button press
    if (button == BUTTON_NONE) {
      button = editing; // no button press means continue editing same
    }
    else {
      while (isButtonDown(button))
        ;
    }
    if (button != 3) {
      if (button != editing) {
        Serial.println(names_RGB[button]);
        editing = button;
      }
      level = map(analogRead(pinPot), potMin, potMax, 0, 99);
      if (level != lastLevel) {
        analogWrite(leds_RGB[editing], level);
        displayInt(level, RADIX_BASE16, true);
        lastLevel = level;
      }
      delay(100);
    }
  } while (button != 3);
  while (isButtonDown(3))
    ; // wait for button#3 release
  displayOff(); // save power
}
/*
* Light LED corresponding to its button when pressed. Repeatedly poll all buttons and
* display values until the last button is pressed.
* The only real trick here is to ensure that every button gets pressed at least once.
*/
void lightLEDsWithButtons()
{
  unsigned int leftToPressMask = 0, prevMask = 0;
  Serial.println("Light each LED by pressing its button.");
  for (int i = 0; i < numButtons; i++) {
    leftToPressMask = (leftToPressMask << 1) | 0x1;
    digitalWrite(leds_red[i], LOW);
  }
  do {
    prevMask = leftToPressMask;
    for (int i = 0; i < numButtons; i++) {
      if (isButtonDown(i)) {
        unsigned int theBit = 1 << i;
        if ((leftToPressMask & theBit)) {
          leftToPressMask &= ~theBit;
          while (isButtonDown(i))
            ;
          digitalWrite(leds_red[i], HIGH);
        }
      }
    }
  } while (leftToPressMask != 0);
  delay(500); // FIXME
  for (int i = 0; i < numButtons; i++)
    digitalWrite(leds_red[i], LOW);
}
/*
* Cycle through LEDs and buttons. Freeze LED when its button is pressed until more than
* until more than one button is pressed simultaneously.
*/
void scanButtons()
{
  boolean done = false;
  int i, heldButton = PIN_UNDEFINED;
  Serial.println("Press buttons to stop the scanning of LEDs.");
  Serial.println("Holding two buttons down simultaneously ends the test.");
  while (!done) {
    for (i = 0; !done && (i < numButtons); i++) {
      if (isButtonDown(i)) {
        if ((heldButton != PIN_UNDEFINED) && (heldButton != i)) {
          // additional button pressed, so we're done
          digitalWrite(leds_red[heldButton], LOW);
          done = true;
          break;
        }
        heldButton = i;
      } else {
          if (i == heldButton) {
            digitalWrite(leds_red[heldButton], LOW); // this button *was* down, but not anymore
            heldButton = PIN_UNDEFINED;
          }
        }
        if (heldButton == PIN_UNDEFINED)
          digitalWrite(leds_red[i], HIGH);
        else digitalWrite(leds_red[heldButton], HIGH);
        digitalWrite(leds_red[(i == 0) ? (numLEDs - 1) : i - 1], LOW); // turn off any previous LED
        delay(BUTTON_SCAN_DELAY);
      }
    }
    while (isAnyButtonDown())
      ;
    Serial.println();
  }
  void findPotRange(int pin, unsigned int *pValMin, unsigned int *pValMax)
  {
    int valMin = ANALOG_IN_MAX, valMax = ANALOG_IN_MIN, value;
    boolean changed;
    Serial.println("Sweep the pot over its full range. Press any button when done.");
    while (!isAnyButtonDown()) {
      changed = false;
      value = analogRead(pin);
      if (value > valMax) {
        valMax = value;
        changed = true;
      }
      if (value < valMin) {
        valMin = value;
        changed = true;
      }
      if (changed) {
        Serial.print ("min=");
        Serial.print (valMin);
        Serial.print (", max=");
        Serial.println(valMax);
      }
      delay(POT_SAMPLE_DELAY);
    }
    *pValMin = valMin;
    *pValMax = valMax;
    while (isAnyButtonDown())
      ;
  }
  void doPhotocell()
  {
    int valueAmbient = ANALOG_IN_MAX, valuePrevious;
    Serial.print("Sensing highest ambient light level... ");
    for (int i = 0; i < 10; i++) {
      int val = analogRead(pinPhotocell);
      if (val < valueAmbient)
        valueAmbient = val;
      delay(500);
    }
    Serial.println(valueAmbient);
    valuePrevious = valueAmbient;
    Serial.println();
    Serial.println("Cover the photocell to turn on the light. Press any button when done.");
    Serial.print ("Current light value=");
    Serial.println(valueAmbient);
    for (int i = 0; i < 3; i++)
      digitalWrite(leds_RGB[i], LOW);
    while (!isAnyButtonDown()) {
      int value = analogRead(pinPhotocell);
      if (abs(value - valuePrevious) > 50) {
        Serial.println(value);
        valuePrevious = value;
      }
      for (int i = 0; i < 3; i++)
        digitalWrite(leds_RGB[i], (abs(value - valueAmbient) > 200) ? HIGH : LOW);
      delay(PHOTOCELL_POLL_DELAY);
    }
    while (isAnyButtonDown())
      ;
  }
  /*
  * 4883 (5000000uV/1024) uV/ADCtick, 10000 mV/deg
  */
  void convertTemp(int adcvalue, int *pTempC, int *pTempF)
  {
    unsigned long degreesC = ((unsigned long)adcvalue * 4883) / 10000;
    unsigned long degreesF = ((degreesC * 180) / 100) + 32;
    *pTempC = (int)degreesC;
    *pTempF = (int)degreesF;
  }
  void doTempSensing()
  {
    int tempCAmbient, tempCPrevious = 0, tempC, tempF;
    int heatLevel = 0;
    Serial.println("Warm the temp sensor by (gently) holding it. Press any button when done.");
    convertTemp(analogRead(pinTemp), &tempC, &tempF);
    tempCAmbient = tempC;
    Serial.print ("Ambient temp = ");
    Serial.println(tempF);
    while (!isAnyButtonDown()) {
      convertTemp(analogRead(pinTemp), &tempC, &tempF);
      if (tempC != tempCPrevious) {
        displayInt(tempF, RADIX_BASE10, true); // NB: only works for temp<100
        if (tempC > tempCAmbient + HEATDIFF_HOT) {
          if (heatLevel < HEATLEVEL_HOT)
            Serial.println(" Ugh, it's definitely hot in here.");
          heatLevel = HEATLEVEL_HOT;
        }
        else if (tempC > tempCAmbient + HEATDIFF_WARM) {
          if (heatLevel < HEATLEVEL_WARM)
            Serial.println(" Is it getting hot in here, or is it just me?");
          heatLevel = HEATLEVEL_WARM;
        }
        else {
          if (heatLevel > HEATLEVEL_AMBIENT)
            Serial.println(" Ah, that's better...");
          heatLevel = HEATLEVEL_AMBIENT;
        }
        tempCPrevious = tempC;
      }
      delay(TEMP_POLL_DELAY);
    }
    while (isAnyButtonDown())
      ;
  }
  void doServo(unsigned int potMin, unsigned int potMax)
  {
    Servo theServo;
    int angle, angleLast = (-1);
    Serial.println("Servo test, use pot to steer. Press any button when done.");
    theServo.attach(pinServo);
    while (!isAnyButtonDown()) {
      int potVal = analogRead(pinPot),
          angle = map(potVal, potMin, potMax, SERVO_DEGREES_MIN, SERVO_DEGREES_MAX);
      if (angle != angleLast)
        displayInt(angle, RADIX_BASE16, true);
      theServo.write(angle);
      angleLast = angle;
    }
    while (isAnyButtonDown())
      ;
    theServo.detach();
    displayOff();
  }
  void playTone(int pin, int freq, int durationMSecs)
  {
    long duration = (long)durationMSecs * MSECS_PER_SEC,
         fullPeriod = USECS_PER_SEC / freq,
         halfPeriod = fullPeriod / 2, i;
    for (i = 0; i < duration; i += fullPeriod) {
      digitalWrite(pin, HIGH);
      delayMicroseconds(halfPeriod);
      digitalWrite(pin, LOW);
      delayMicroseconds(halfPeriod);
    }
  }
  unsigned int recordTones(unsigned int *tones, unsigned int potMin, unsigned int potMax)
  {
    unsigned int tonenum = 0;
    Serial.println("Record a sequence of tones using the pot and button-0. Hold button-0 for");
    Serial.println("two seconds to playback the recorded tones. Press button-0 to get started.");
    waitForPressAndRelease(0);
    while (true) {
      unsigned int freq = map(analogRead(pinPot), potMin, potMax, FREQ_MIN, FREQ_MAX);
      playTone(pinSpeaker, freq, 100/*msec*/);
      if (isButtonDown(0)) {
        unsigned long pressStart = millis();
        delay(BUTTON_DEBOUNCE_DELAY);
        while (isButtonDown(0))
          ;
        if ((millis() - pressStart) > 2000) { // 2000=two seconds of mSecs
          Serial.println();
          break;
        }
        else {
          displayInt(tonenum, RADIX_BASE10, false);
          Serial.print (" - ");
          Serial.print (freq);
          Serial.println("Hz ");
          tones[tonenum++] = freq;
        }
      }
    }
    return tonenum;
  }
  /*
  * Play back array of tones, half-second on (at duty cycle 50%) then half-second off.
  */
  void doPlayback(unsigned int numtones, unsigned int *tones)
  {
    unsigned int i;
    for (i = 0; i < numtones; i++) {
      displayInt(i, RADIX_BASE10, true);
      playTone(pinSpeaker, tones[i], TONE_PLAYBACK_PERIOD / 2);
      delay(TONE_PLAYBACK_PERIOD / 2);
    }
    Serial.println();
  }
  /*
  * one-time initializations at start of program
  */
  void setup() {
    int i;
    Serial.begin(9600); // init serial connection to 9600bps
    for (i = 0; i < numLEDs; i++)
      pinMode(leds_red[i], OUTPUT);
    for (i = 0; i < numButtons; i++) {
      pinMode (buttons[i], INPUT);
      digitalWrite(buttons[i], HIGH); // enable internal pull-ups
    }
#ifdef ARDUINO_MEGA
    for (i = 0; i < 8; i++) {
      pinMode(seg7digit1[i], OUTPUT);
      pinMode(seg7digit2[i], OUTPUT);
    }
#endif
    pinMode(pinSpeaker, OUTPUT);
    pinMode(pinServo, OUTPUT);
  }
  /*
  * loop() contains the code that gets called repeatedly from the main() function
  * provided by the Arduino framework.
  */
  void loop() {
    unsigned int i, potMin, potMax, numtones, tones[TONES_MAX];
    char username[MAX_NAMELEN];
    Serial.println("Welcome to the SJSU Mechatronics Edushield full-board demo.");
    Serial.println();
    doLEDFades();
    lightLEDsWithButtons();
    scanButtons();
    findPotRange(pinPot, &potMin, &potMax);
    doRGBMixing (potMin, potMax);
    doPhotocell ();
    doTempSensing();
    doServo (potMin, potMax);
    numtones = recordTones(tones, potMin, potMax);
    doPlayback(numtones, tones);
    Serial.println("Press any button to run again.");
    waitForAnyPress(true);
  }

