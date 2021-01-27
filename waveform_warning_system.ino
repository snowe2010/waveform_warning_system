#define FASTLED_ALLOW_INTERRUPTS 0
#include <FastLED.h>
#include <MemoryFree.h>
/***************************
 ********* FastLED *********
****************************/

//#define LED_PIN     22
#define NUM_LEDS    162
#define LEDS_PER_ROW 54   // LEDs per row, should be same as LEDS_PER_ROW
#define BRIGHTNESS  128
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
//CRGB leds1[NUM_LEDS];
//CRGB leds2[NUM_LEDS];
//CRGB leds3[NUM_LEDS];
CRGB leds4[NUM_LEDS];
CRGB leds5[NUM_LEDS];
CRGB leds6[NUM_LEDS ];
//CRGB leds7[NUM_LEDS];
//CRGB leds8[NUM_LEDS];
//CRGB leds9[NUM_LEDS];

#define yres 13
int Intensity[LEDS_PER_ROW] = { }; // initialize Frequency Intensity to zero
int Displacement = 1;

/***************************
 *********** FHT ***********
****************************/

// FHT, http://wiki.openmusiclabs.com/wiki/ArduinoFHT

#define LOG_OUT 1 // use the log output function
#define LIN_OUT8 1 // use the linear byte output function
#define OCTAVE 1
#define OCT_NORM 0
#define SCALE 64
#define FHT_N 64 // set to 256 point fht
#include <FHT.h> // include the library

// pins
#define MicPin A0 // used with analogRead mode only

// consts
#define AmpMax (1024 / 2)
#define MicSamples (1024*3) // Three of these time-weightings have been internationally standardised, 'S' (1 s) originally called Slow, 'F' (125 ms) originally called Fast and 'I' (35 ms) originally called Impulse.

// modes
#define Use33 // use 3.3 voltage. the 5v voltage from usb is not regulated. this is much more stable.
#define ADCReClock // switch to higher clock, not needed if we are ok with freq between 0 and 4Khz.
#define ADCFlow // read data from adc with free-run (not interupt). much better data, dc low. hardcoded for A0.

//#define Octave // options Octave, FreqLog, Linear
//#define FreqLog // options Octave, FreqLog, Linear
#define Linear // options Octave, FreqLog, Linear

// use octave scale for FHT frequencies, closer to human perception
#ifdef Octave
#define FreqOutData fht_oct_out
#define FreqGainFactorBits 0
#endif

// use log scale for FHT frequencies
#ifdef FreqLog
#define FreqOutData fht_log_out
#define FreqGainFactorBits 0
#endif

#ifdef Linear
#define FreqOutData fht_lin_out8
#define FreqGainFactorBits 2
#endif

//#define FreqSerialBinary

#define VolumeGainFactorBits 0

// macros
// http://yaab-arduino.blogspot.co.il/2015/02/fast-sampling-from-analog-input.html
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

void setup()
{
    delay( 3000 ); // power-up safety delay
//    TIMSK0 = 0; // turn this off else you can't use FastLED

  //pinMode(MicPin, INPUT); // relevant for digital pins. not relevant for analog. however, don't put into digital OUTPUT mode if going to read analog values.

#ifdef ADCFlow
  // set the adc to free running mode
  // register explanation: http://maxembedded.com/2011/06/the-adc-of-the-avr/
  // 5 => div 32. sample rate 38.4
  // 7 => switch to divider=128, default 9.6khz sampling
  ADCSRA = 0xe0 + 5; // "ADC Enable", "ADC Start Conversion", "ADC Auto Trigger Enable" and divider.
  ADMUX = 0x0; // use adc0 (hardcoded, doesn't use MicPin). Use ARef pin for analog reference (same as analogReference(EXTERNAL)).
#ifndef Use33
  ADMUX |= 0x40; // Use Vcc for analog reference.
#endif
  DIDR0 = 0x01; // turn off the digital input for adc0
#else
#ifdef Use33
  analogReference(EXTERNAL); // 3.3V to AREF
#endif
#endif

#ifdef ADCReClock // change ADC freq divider. default is div 128 9.6khz (bits 111)
  // http://yaab-arduino.blogspot.co.il/2015/02/fast-sampling-from-analog-input.html
  // 1 0 0 = mode 4 = divider 16 = 76.8khz
  //sbi(ADCSRA, ADPS2);
  //cbi(ADCSRA, ADPS1);
  //cbi(ADCSRA, ADPS0);
  // 1 0 1 = mode 5 = divider 32 = 38.4Khz
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);
#endif



  // serial
  Serial.begin(115200);
  while (!Serial); // Wait untilSerial is ready - Leonardo

  Serial.println("R");
//  FastLED.addLeds<LED_TYPE, 22, COLOR_ORDER>(leds1, NUM_LEDS).setCorrection( TypicalLEDStrip );
//  FastLED.addLeds<LED_TYPE, 23, COLOR_ORDER>(leds2, NUM_LEDS).setCorrection( TypicalLEDStrip );
//  FastLED.addLeds<LED_TYPE, 24, COLOR_ORDER>(leds3, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<LED_TYPE, 25, COLOR_ORDER>(leds4, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<LED_TYPE, 26, COLOR_ORDER>(leds5, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<LED_TYPE, 27, COLOR_ORDER>(leds6, NUM_LEDS).setCorrection( TypicalLEDStrip );
//  FastLED.addLeds<LED_TYPE, 28, COLOR_ORDER>(leds7, NUM_LEDS).setCorrection( TypicalLEDStrip );
//  FastLED.addLeds<LED_TYPE, 29, COLOR_ORDER>(leds8, NUM_LEDS).setCorrection( TypicalLEDStrip );
//  FastLED.addLeds<LED_TYPE, 30, COLOR_ORDER>(leds9, NUM_LEDS).setCorrection( TypicalLEDStrip );

  FastLED.setBrightness(  BRIGHTNESS );

//  fill_solid(leds1, NUM_LEDS, CRGB(0, 0, 0));
//  fill_solid(leds2, NUM_LEDS, CRGB(0, 0, 0));
//  fill_solid(leds3, NUM_LEDS, CRGB(0, 0, 0));
//  fill_solid(leds4, NUM_LEDS, CRGB(0, 0, 0));
//  fill_solid(leds5, NUM_LEDS, CRGB(0, 0, 0));
//  fill_solid(leds6, NUM_LEDS, CRGB(0, 0, 0));
//  fill_solid(leds7, NUM_LEDS, CRGB(0, 0, 0));
//  fill_solid(leds8, NUM_LEDS, CRGB(0, 0, 0));
//  fill_solid(leds9, NUM_LEDS, CRGB(0, 0, 0));
  
  int color = 255;
  for (int i = 54; i < 54*2; i++) {
    leds5[i] = CHSV(color, 255, BRIGHTNESS);
    color -= 255 / LEDS_PER_ROW;
  }

  FastLED.show();
}

void loop()
{
  
    Serial.print(F("freeMemory()="));
    Serial.println(freeMemory());
  Serial.println(F("l"));
  MeasureFHT();
  //  renderWaveform();
  //  EVERY_N_MILLISECONDS( 300 ) { renderWaveform(); }
  calculateIntensity();
  displayUpdate();
  Serial.println(F("making it here?"));
  
  Serial.print(F("freeMemory()="));
  Serial.println(freeMemory());
  FastLED.show();
  
    Serial.print(F("freeMemory()="));
    Serial.println(freeMemory());
  
  Serial.println(F("did it crash?"));
  FastLED.delay(200);
}

void calculateIntensity() {
  for (int i = 2; i < (32 * Displacement) + 2; i += Displacement) {
    FreqOutData[i] = constrain(FreqOutData[i], 0 , 1024);          // set max value for input data
    FreqOutData[i] = map(FreqOutData[i], 0, 1024, 0, yres);        // map data to fit our display
    Intensity[(i / Displacement) - 2] --;                          // Decrease displayed value
    if (FreqOutData[i] > Intensity[(i / Displacement) - 2]) {      // Match displayed value to measured value
      Intensity[(i / Displacement) - 2] = FreqOutData[i];
    }
  }
}

void displayUpdate() {
  Serial.println(F("made it here..."));
  
    Serial.print(F("freeMemory()="));
    Serial.println(freeMemory());
  int color = 0;
  for (int i = 0; i < LEDS_PER_ROW; i++) {
    for (int j = 0; j < yres; j++) {
//      Serial.println("yres");
 
      if (j <= Intensity[i]) {                                // Light everything within the intensity range
//        Serial.println(F("intense"));
        lightLedBasedOnRow(i, CRGB::White, j);
        
//        FastLED.show();
        //        if (j%2 == 0) {
        //          Serial.println("J%2==0");
//                  leds5[(LEDS_PER_ROW*(j+1))-i-1] = CRGB::White;//CHSV(color, 255, BRIGHTNESS);
        //        } else {
        //          Serial.println("J%2!=0");
        //          leds1[(LEDS_PER_ROW*j)+i] = CHSV(color, 255, BRIGHTNESS);
        //        }
      } else {                                                  // Everything outside the range goes dark
//        Serial.print("darken: x"); 
//Serial.println(F("not intense"));
        lightLedBasedOnRow(i, CRGB::Black, j);
        //        if (j%2 == 0) {
        //          Serial.println("J%2==0");
//                  leds5[(LEDS_PER_ROW*(j+1))-i-1] = CRGB::Black; //CHSV(color, 255, 0);
        //        } else {
        //          Serial.println("J%2!=0");
        //          leds1[(LEDS_PER_ROW*j)+i] = CHSV(color, 255, 0);
        //        }
      }
    }
    color += 255 / LEDS_PER_ROW;                                    // Increment the Hue to get the Rainbow
  }
}


void lightLedBasedOnRow(int led, CRGB color, int row) {
  switch (row)  {
    case 0:
      lightLedFromArrayAndRow(leds5, LEDS_PER_ROW*2, led, color, row);
      lightLedFromArrayAndRow(leds5, LEDS_PER_ROW*0, led, color, row);
      break;
//    case 1:
//      lightLedFromArrayAndRow(leds6, LEDS_PER_ROW*0, led, color, row);
//      lightLedFromArrayAndRow(leds4, LEDS_PER_ROW*2, led, color, row);
//      break;
//    case 2:
//      lightLedFromArrayAndRow(leds6, LEDS_PER_ROW*1, led, color, row);
//      lightLedFromArrayAndRow(leds4, LEDS_PER_ROW*1, led, color, row);
//      break;
//    case 3:
//      lightLedFromArrayAndRow(leds6, LEDS_PER_ROW*2, led, color, row);
//      lightLedFromArrayAndRow(leds4, LEDS_PER_ROW*0, led, color, row);
//    case 4:
//      lightLedFromArrayAndRow(leds7, LEDS_PER_ROW*0, led, color, row);
//      lightLedFromArrayAndRow(leds3, LEDS_PER_ROW*2, led, color, row);
//    case 5:
//      lightLedFromArrayAndRow(leds7, LEDS_PER_ROW*1, led, color, row);
//      lightLedFromArrayAndRow(leds3, LEDS_PER_ROW*1, led, color, row);
//    case 6:
//      lightLedFromArrayAndRow(leds7, LEDS_PER_ROW*2, led, color, row);
//      lightLedFromArrayAndRow(leds3, LEDS_PER_ROW*0, led, color, row);
//    case 7:
//      lightLedFromArrayAndRow(leds8, LEDS_PER_ROW*0, led, color, row);
//      lightLedFromArrayAndRow(leds2, LEDS_PER_ROW*2, led, color, row);
//    case 8:
//      lightLedFromArrayAndRow(leds8, LEDS_PER_ROW*1, led, color, row);
//      lightLedFromArrayAndRow(leds2, LEDS_PER_ROW*1, led, color, row);
//    case 9:
//      lightLedFromArrayAndRow(leds8, LEDS_PER_ROW*2, led, color, row);
//      lightLedFromArrayAndRow(leds2, LEDS_PER_ROW*0, led, color, row);
//    case 10:
//      lightLedFromArrayAndRow(leds9, LEDS_PER_ROW*0, led, color, row);
//      lightLedFromArrayAndRow(leds1, LEDS_PER_ROW*2, led, color, row);
//    case 11:
//      lightLedFromArrayAndRow(leds9, LEDS_PER_ROW*1, led, color, row);
//      lightLedFromArrayAndRow(leds1, LEDS_PER_ROW*1, led, color, row);
//    case 12:
//      lightLedFromArrayAndRow(leds9, LEDS_PER_ROW*2, led, color, row);
//      lightLedFromArrayAndRow(leds1, LEDS_PER_ROW*0, led, color, row);
//      break;
  }
}

void lightLedFromArrayAndRow(CRGB arr[], int offset, int column, CRGB color, int row) {
  if (row % 3 == 0) {
    arr[offset + (LEDS_PER_ROW * (row + 1)) - column - 1] = color;
  } else {
    arr[offset + column] = color;
  }
}

// calculate frequencies in the signal and print to serial
void MeasureFHT()
{
//  long t0 = micros();
#ifdef ADCFlow
  //  cli();  // UDRE interrupt slows this way down on arduino1.0
#endif
  for (int i = 0; i < FHT_N; i++) { // save 256 samples
#ifdef ADCFlow
    while (!(ADCSRA & /*0x10*/_BV(ADIF))); // wait for adc to be ready (ADIF)
    sbi(ADCSRA, ADIF); // restart adc
    byte m = ADCL; // fetch adc data
    byte j = ADCH;
    int k = ((int)j << 8) | m; // form into an int
#else
    int k = analogRead(MicPin);
#endif
    k -= 0x0200; // form into a signed int
    k <<= 6; // form into a 16b signed int
    k <<= FreqGainFactorBits;
    fht_input[i] = k; // put real data into bins
  }
#ifdef ADCFlow
  //  sei();
#endif
//  long dt = micros() - t0;
  fht_window(); // window the data for better frequency response
  fht_reorder(); // reorder the data before doing the fht
  fht_run(); // process the data in the fht

#ifdef FreqLog
  fht_mag_log();
#endif

#ifdef Octave
  fht_mag_octave();
#endif

#ifdef Linear
  fht_mag_lin8();
#endif

//#ifdef FreqSerialBinary
//  // print as binary
//  Serial.write(255); // send a start byte
//  Serial.write(FreqOutData, FHT_N / 2); // send out the data
//#else
  // print as text
//  for (int i = 0; i < FHT_N / 2; i++)
//  {
//    Serial.print(FreqOutData[i]);
//    Serial.print(',');
//  }
//  long sample_rate = FHT_N * 1000000l / dt;
//  Serial.print(dt);
//  Serial.print(',');
//  Serial.println(sample_rate);
//#endif
}
