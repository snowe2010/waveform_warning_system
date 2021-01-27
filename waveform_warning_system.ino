//#define FASTLED_ALLOW_INTERRUPTS 0
#include <FastLED.h>

/***************************
 ********* FastLED *********
****************************/

//#define LED_PIN     22
#define NUM_LEDS    54*3
#define LEDS_PER_ROW 54   // LEDs per row, should be same as xres
#define BRIGHTNESS  128
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
CRGB leds1[NUM_LEDS];
CRGB leds2[NUM_LEDS];
CRGB leds3[NUM_LEDS];
CRGB leds4[NUM_LEDS];
CRGB leds5[NUM_LEDS];
CRGB leds6[NUM_LEDS];
CRGB leds7[NUM_LEDS];
CRGB leds8[NUM_LEDS];
CRGB leds9[NUM_LEDS];

#define xres 54
#define yres 14
int Intensity[LEDS_PER_ROW/2] = { }; // initialize Frequency Intensity to zero
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
#define FreqGainFactorBits 6
#endif

//#define FreqSerialBinary

#define VolumeGainFactorBits 0

// macros
// http://yaab-arduino.blogspot.co.il/2015/02/fast-sampling-from-analog-input.html
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

void setup()
{
  //  delay( 1000 ); // power-up safety delay
  //  TIMSK0 = 0; // turn this off else you can't use FastLED

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

  Serial.println("Running setup");
  FastLED.addLeds<LED_TYPE, 22, COLOR_ORDER>(leds1, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<LED_TYPE, 23, COLOR_ORDER>(leds2, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<LED_TYPE, 24, COLOR_ORDER>(leds3, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<LED_TYPE, 25, COLOR_ORDER>(leds4, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<LED_TYPE, 26, COLOR_ORDER>(leds5, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<LED_TYPE, 27, COLOR_ORDER>(leds6, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<LED_TYPE, 28, COLOR_ORDER>(leds7, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<LED_TYPE, 29, COLOR_ORDER>(leds8, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<LED_TYPE, 30, COLOR_ORDER>(leds9, NUM_LEDS).setCorrection( TypicalLEDStrip );

  FastLED.setBrightness(  BRIGHTNESS );

  fill_solid(leds1, NUM_LEDS, CRGB(0, 0, 0));
  fill_solid(leds2, NUM_LEDS, CRGB(0, 0, 0));
  fill_solid(leds3, NUM_LEDS, CRGB(0, 0, 0));
  fill_solid(leds4, NUM_LEDS, CRGB(0, 0, 0));
  fill_solid(leds5, NUM_LEDS, CRGB(0, 0, 0));
  fill_solid(leds6, NUM_LEDS, CRGB(0, 0, 0));
  fill_solid(leds7, NUM_LEDS, CRGB(0, 0, 0));
  fill_solid(leds8, NUM_LEDS, CRGB(0, 0, 0));
  fill_solid(leds9, NUM_LEDS, CRGB(0, 0, 0));
  
//  int color = 255;
//  for (int i = 54; i < 54*2; i++) {
//    leds5[i] = CHSV(color, 255, BRIGHTNESS);
//    color -= 255 / xres;
//  }

  FastLED.show();
}

void loop()
{
//  Serial.println("loop");
  MeasureFHT();
  //  renderWaveform();
  //  EVERY_N_MILLISECONDS( 300 ) { renderWaveform(); }
//  delay(1000);
  calculateIntensity();
  displayUpdate();
  
  FastLED.show();
//  while(1);
}

void calculateIntensity() {
//  Serial.print(F("Calculate Intensity: "));
  for (int i = 0; i < ((LEDS_PER_ROW/2) * Displacement); i += Displacement) {
//    Serial.print(FreqOutData[i]);
    
    FreqOutData[i] = constrain(FreqOutData[i], 0 , 1024);          // set max value for input data
    FreqOutData[i] = map(FreqOutData[i], 0, 1024, 0, yres);        // map data to fit our display
    Intensity[(i / Displacement)] --;                          // Decrease displayed value
    if (FreqOutData[i] > Intensity[(i / Displacement)]) {      // Match displayed value to measured value
      Intensity[(i / Displacement)] = FreqOutData[i];
    }
  }
}

/**
  void renderWaveform() {
  //  Serial.println("render waveform");
  for (int i = 0; i < FHT_N / 2 - 6; i++) {
    int binAmp = FreqOutData[i];
    //    if (binAmp > 1) {
    if (0 < binAmp && binAmp <= 10) {
  //      Serial.println("below 20");
      mirrorWaveform(i, CRGB::Black);
      //      leds[i+31] = CRGB::Black;
    } else if (10 < binAmp && binAmp <= 50) {
      Serial.println("20-50");
      mirrorWaveform(i, CRGB::Amethyst);
      //      leds[i+31] = CRGB::Amethyst;
    } else if (50 < binAmp && binAmp <= 60) {
      Serial.println("50-60");
      mirrorWaveform(i, CRGB::Aqua);
      //      leds[i+31] = CRGB::Aqua;
    } else if (15 < binAmp && binAmp <= 20) {
      Serial.println("15-20");
      mirrorWaveform(i, CRGB::Aquamarine);
      //      leds[i+31] = CRGB::Aquamarine;
    } else if (20 < binAmp && binAmp <= 25) {
      Serial.println("20-25");
      mirrorWaveform(i, CRGB::Azure);
      //      leds[i+31] = CRGB::Azure;
    } else if (25 < binAmp && binAmp <= 30) {
      Serial.println("25-30");
      mirrorWaveform(i, CRGB::DarkBlue);
      //      leds[i+31] = CRGB::DarkBlue;
    } else if (30 < binAmp) {
      Serial.println("above 30");
      mirrorWaveform(i, CRGB::Red);
      //      leds[i+31] = CRGB::Red;
    } else {
      mirrorWaveform(i, CRGB::Black);
      //      leds[i+31] = CRGB::Black;
    }
  }
  FastLED.show();
  }
**/


void displayUpdate() {
  int color = 0;
  for (int i = 0; i < LEDS_PER_ROW / 2; i++) {
    for (int j = 0; j < yres; j++) {
      if (j <= Intensity[i]) {                                // Light everything within the intensity range
//Serial.print("intensity: ");
//Serial.print(Intensity[i]);
//Serial.print(" i: ");
//Serial.print(i);
//Serial.print(" j: ");
//Serial.println(j);

        lightLedBasedOnRow(i, CHSV(color, 255, BRIGHTNESS), j);
      } else {                                                  // Everything outside the range goes dark
        lightLedBasedOnRow(i, CHSV(color, 255, 0), j);
      }      
    }
    color += 255 / xres;                                    // Increment the Hue to get the Rainbow
  }
}


void lightLedBasedOnRow(int led, CHSV color, int row) {
  switch (row)  {
    case 0:
      lightLedFromArrayAndRow(leds5, LEDS_PER_ROW, led, color, row);
      break;
    case 1:
      lightLedFromArrayAndRow(leds5, LEDS_PER_ROW*2, led, color, row);
      lightLedFromArrayAndRow(leds5, LEDS_PER_ROW*0, led, color, row);
      break;
    case 2:
      lightLedFromArrayAndRow(leds6, LEDS_PER_ROW*0, led, color, row);
      lightLedFromArrayAndRow(leds4, LEDS_PER_ROW*2, led, color, row);
      break;
    case 3:
      lightLedFromArrayAndRow(leds6, LEDS_PER_ROW*1, led, color, row);
      lightLedFromArrayAndRow(leds4, LEDS_PER_ROW*1, led, color, row);
      break;
    case 4:
      lightLedFromArrayAndRow(leds6, LEDS_PER_ROW*2, led, color, row);
      lightLedFromArrayAndRow(leds4, LEDS_PER_ROW*0, led, color, row);
    case 5:
      lightLedFromArrayAndRow(leds7, LEDS_PER_ROW*0, led, color, row);
      lightLedFromArrayAndRow(leds3, LEDS_PER_ROW*2, led, color, row);
    case 6:
      lightLedFromArrayAndRow(leds7, LEDS_PER_ROW*1, led, color, row);
      lightLedFromArrayAndRow(leds3, LEDS_PER_ROW*1, led, color, row);
    case 7:
      lightLedFromArrayAndRow(leds7, LEDS_PER_ROW*2, led, color, row);
      lightLedFromArrayAndRow(leds3, LEDS_PER_ROW*0, led, color, row);
    case 8:
      lightLedFromArrayAndRow(leds8, LEDS_PER_ROW*0, led, color, row);
      lightLedFromArrayAndRow(leds2, LEDS_PER_ROW*2, led, color, row);
    case 9:
      lightLedFromArrayAndRow(leds8, LEDS_PER_ROW*1, led, color, row);
      lightLedFromArrayAndRow(leds2, LEDS_PER_ROW*1, led, color, row);
    case 10:
      lightLedFromArrayAndRow(leds8, LEDS_PER_ROW*2, led, color, row);
      lightLedFromArrayAndRow(leds2, LEDS_PER_ROW*0, led, color, row);
    case  11:
      lightLedFromArrayAndRow(leds9, LEDS_PER_ROW*0, led, color, row);
      lightLedFromArrayAndRow(leds1, LEDS_PER_ROW*2, led, color, row);
    case  12:
      lightLedFromArrayAndRow(leds9, LEDS_PER_ROW*1, led, color, row);
      lightLedFromArrayAndRow(leds1, LEDS_PER_ROW*1, led, color, row);
    case  13:
      lightLedFromArrayAndRow(leds9, LEDS_PER_ROW*2, led, color, row);
      lightLedFromArrayAndRow(leds1, LEDS_PER_ROW*0, led, color, row);
      break;
  }
}


void lightLedFromArrayAndRow(CRGB arr[], int offset, int column, CHSV color, int row) {
    arr[offset + 27 - column - 1] = color;
    arr[offset + 28 + column - 1] = color;
}

void mirrorWaveform(uint8_t colorIndex, CRGB color) {
  leds1[27 - colorIndex] = color;
  leds1[28 + colorIndex] = color;
}

// measure basic properties of the input signal
// determine if analog or digital, determine range and average.
void MeasureAnalog()
{
  long signalAvg = 0, signalMax = 0, signalMin = 1024, t0 = millis();
  //cli();  // UDRE interrupt slows this way down on arduino1.0
  for (int i = 0; i < MicSamples; i++)
  {
#ifdef ADCFlow
    while (!(ADCSRA & /*0x10*/_BV(ADIF))); // wait for adc to be ready (ADIF)
    sbi(ADCSRA, ADIF); // restart adc
    byte m = ADCL; // fetch adc data
    byte j = ADCH;
    int k = ((int)j << 8) | m; // form into an int
#else
    int k = analogRead(MicPin);
#endif
    signalMin = min(signalMin, k);
    signalMax = max(signalMax, k);
    signalAvg += k;
  }
  signalAvg /= MicSamples;
  //sei();

  // print
  Serial.print("Time: " + String(millis() - t0));
  Serial.print(" Min: " + String(signalMin));
  Serial.print(" Max: " + String(signalMax));
  Serial.print(" Avg: " + String(signalAvg));
  Serial.print(" Span: " + String(signalMax - signalMin));
  Serial.print(", " + String(signalMax - signalAvg));
  Serial.print(", " + String(signalAvg - signalMin));
  Serial.println("");

}

// calculate volume level of the signal and print to serial and LCD
void MeasureVolume()
{
  long soundVolAvg = 0, soundVolMax = 0, soundVolRMS = 0, t0 = millis();
  //cli();  // UDRE interrupt slows this way down on arduino1.0
  for (int i = 0; i < MicSamples; i++)
  {
#ifdef ADCFlow
    while (!(ADCSRA & /*0x10*/_BV(ADIF))); // wait for adc to be ready (ADIF)
    sbi(ADCSRA, ADIF); // restart adc
    byte m = ADCL; // fetch adc data
    byte j = ADCH;
    int k = ((int)j << 8) | m; // form into an int
#else
    int k = analogRead(MicPin);
#endif
    int amp = abs(k - AmpMax);
    amp <<= VolumeGainFactorBits;
    soundVolMax = max(soundVolMax, amp);
    soundVolAvg += amp;
    soundVolRMS += ((long)amp * amp);
  }
  soundVolAvg /= MicSamples;
  soundVolRMS /= MicSamples;
  float soundVolRMSflt = sqrt(soundVolRMS);
  //sei();

  float dB = 20.0 * log10(soundVolRMSflt / AmpMax);

  // convert from 0 to 100
  soundVolAvg = 100 * soundVolAvg / AmpMax;
  soundVolMax = 100 * soundVolMax / AmpMax;
  soundVolRMSflt = 100 * soundVolRMSflt / AmpMax;
  soundVolRMS = 10 * soundVolRMSflt / 7; // RMS to estimate peak (RMS is 0.7 of the peak in sin)

  // print
  Serial.print("Time: " + String(millis() - t0));
  Serial.print(" Amp: Max: " + String(soundVolMax));
  Serial.print("% Avg: " + String(soundVolAvg));
  Serial.print("% RMS: " + String(soundVolRMS));
  Serial.println("% dB: " + String(dB, 3));

}

// calculate frequencies in the signal and print to serial
void MeasureFHT()
{
  long t0 = micros();
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
  long dt = micros() - t0;
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

#ifdef FreqSerialBinary
  // print as binary
  Serial.write(255); // send a start byte
  Serial.write(FreqOutData, FHT_N / 2); // send out the data
#else
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
#endif
}
