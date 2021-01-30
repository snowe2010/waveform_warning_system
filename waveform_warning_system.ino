//#define FASTLED_ALLOW_INTERRUPTS 0
#include <FastLED.h>

/***************************
 ********* FastLED *********
****************************/

#define NUM_OF_STRIPS 9
#define NUM_LEDS    54*3
#define LEDS_PER_ROW 54
#define BRIGHTNESS  128
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB

CRGB leds[NUM_OF_STRIPS][NUM_LEDS];
//CRGB leds1[NUM_LEDS];
//CRGB leds2[NUM_LEDS];
//CRGB leds3[NUM_LEDS];
//CRGB leds4[NUM_LEDS];
//CRGB leds5[NUM_LEDS];
//CRGB leds6[NUM_LEDS];
//CRGB leds7[NUM_LEDS];
//CRGB leds8[NUM_LEDS];
//CRGB leds9[NUM_LEDS];

#define xres 54
#define yres 14
int Intensity[LEDS_PER_ROW / 2] = { }; // initialize Frequency Intensity to zero
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

#define VolumeGainFactorBits 2

// macros
// http://yaab-arduino.blogspot.co.il/2015/02/fast-sampling-from-analog-input.html
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))


int noise[] = {230, 200, 40, 50, 40, 40, 40, 20, 20, 20, 30, 80, 50, 20, 20, 80, 105, 70, 20, 20, 35, 35, 30, 20, 25, 25, 15, 20, 20};

float noise_fact[] = {15, 7, 1.5, 1, 1.2, 1.4, 1.7, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}; // noise level determined by playing pink noise and seeing levels [trial and error]{204,188,68,73,150,98,88,68}
float noise_fact_adj[] = {15, 7, 1.5, 1, 1.2, 1.4, 1.7, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}; // noise level determined by playing pink noise and seeing levels [trial and error]{204,188,68,73,150,98,88,68}
float volumeMultiplier = 0.0;

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
//  FastLED.addLeds<LED_TYPE, 22, COLOR_ORDER>(leds1, NUM_LEDS).setCorrection( TypicalLEDStrip );
//  FastLED.addLeds<LED_TYPE, 23, COLOR_ORDER>(leds2, NUM_LEDS).setCorrection( TypicalLEDStrip );
//  FastLED.addLeds<LED_TYPE, 24, COLOR_ORDER>(leds3, NUM_LEDS).setCorrection( TypicalLEDStrip );
//  FastLED.addLeds<LED_TYPE, 25, COLOR_ORDER>(leds4, NUM_LEDS).setCorrection( TypicalLEDStrip );
//  FastLED.addLeds<LED_TYPE, 26, COLOR_ORDER>(leds5, NUM_LEDS).setCorrection( TypicalLEDStrip );
//  FastLED.addLeds<LED_TYPE, 27, COLOR_ORDER>(leds6, NUM_LEDS).setCorrection( TypicalLEDStrip );
//  FastLED.addLeds<LED_TYPE, 28, COLOR_ORDER>(leds7, NUM_LEDS).setCorrection( TypicalLEDStrip );
//  FastLED.addLeds<LED_TYPE, 29, COLOR_ORDER>(leds8, NUM_LEDS).setCorrection( TypicalLEDStrip );
//  FastLED.addLeds<LED_TYPE, 30, COLOR_ORDER>(leds9, NUM_LEDS).setCorrection( TypicalLEDStrip );


  FastLED.addLeds<LED_TYPE, 22, COLOR_ORDER>(leds[0], NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<LED_TYPE, 23, COLOR_ORDER>(leds[1], NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<LED_TYPE, 24, COLOR_ORDER>(leds[2], NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<LED_TYPE, 25, COLOR_ORDER>(leds[3], NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<LED_TYPE, 26, COLOR_ORDER>(leds[4], NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<LED_TYPE, 27, COLOR_ORDER>(leds[5], NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<LED_TYPE, 28, COLOR_ORDER>(leds[6], NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<LED_TYPE, 29, COLOR_ORDER>(leds[7], NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.addLeds<LED_TYPE, 30, COLOR_ORDER>(leds[8], NUM_LEDS).setCorrection( TypicalLEDStrip );
  
  FastLED.setBrightness(  BRIGHTNESS );

  for (int i = 0; i< NUM_OF_STRIPS; i++) {
    fill_solid(leds[i], NUM_LEDS, CRGB(0, 0, 0));
  }

  FastLED.show();
}

// Serial input
const byte numChars = 32;
char receivedChars[numChars];
boolean newData = false;
int dataNumber = 0;


int prev_j[28];
int beat = 0;
int prev_oct_j;
int counter = 0;
int prev_beat = 0;
int led_index = 0;
int saturation = 0;
int saturation_prev = 0;
int brightness = 0;
int brightness_prev = 0;

int counter2 = 0;

void loop()
{
  recvWithStartEndMarkers();
  showNewNumber();
  MeasureFHT();
//  MeasureVolume();
//  transform();
    calculateIntensity();
  displayUpdate();
  FastLED.show();
}

void transform() {

  for (int i = 0; i < 28; i++) {
    int j;
    j = (FreqOutData[i] - noise[i]); // take the pink noise average level out, take the asbolute value to avoid negative numbers
    if (j < 10) {
      j = 0;
    }
    j = j * noise_fact_adj[i];

    if (j < 10) {
      j = 0;
    } else {
      j = j * noise_fact_adj[i];
      if (j > 180) {
        if (i >= 27) {
          beat += 2;
        } else {
          beat += 1;
        }
      }
      j = j / 30;
      j = j * 30; // (force it to more discrete values)
    }


    for (uint8_t y = 0; y < 13; y++) {
      lightLedBasedOnRow(led_index, CHSV(j + y * 30, saturation, brightness), y);
      if (i > 2) {
        prev_oct_j = (j + prev_j[i - 1]) / 2;
        lightLedBasedOnRow(led_index, CHSV(prev_oct_j + y * 30, saturation_prev, brightness_prev), y);
      }
    }
  }
}

void calculateIntensity() {
  for (int i = 2; i < ((LEDS_PER_ROW / 2) * Displacement) + 2; i += Displacement) {
//    Serial.print("Freq: ");
//    Serial.print(FreqOutData[i]);
    FreqOutData[i] = constrain(FreqOutData[i], 0 , 1024);          // set max value for input data
    
    long j = 0;
    j = ((long) FreqOutData[i] - (long) noise[i]); // take the pink noise average level out, take the absolute value to avoid negative numbers
//    Serial.print(" j: ");
//    Serial.print(j);
    if (j < 50) {
      FreqOutData[i] = 0;
    } else {
      FreqOutData[i] = (int) j;
    }
    Serial.print("/");
    Serial.print(FreqOutData[i]);

    
    FreqOutData[i] = log(FreqOutData[i]+1)/log(1024)*1024;
    
    FreqOutData[i] = map(FreqOutData[i], 0, 1024, 0, yres);        // map data to fit our display
        Serial.print("/");
    Serial.print(FreqOutData[i]);
        Serial.print(", ");

//    Serial.print("volume multiplier: ");
//    Serial.println(volumeMultiplier);
//    Serial.print(FreqOutData[i]);
//    Serial.print(",  ");
    Intensity[(i / Displacement) -2] --;                          // Decrease displayed value
    if (FreqOutData[i] > Intensity[(i / Displacement)-2]) {      // Match displayed value to measured value
      Intensity[(i / Displacement)-2] = FreqOutData[i];
    }
  }
  Serial.println(" ");
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
  int color = 200;
  for (int i = 0; i < LEDS_PER_ROW / 2; i++) {
    for (int j = 0; j < yres; j++) {
      if (j <= Intensity[i]) {                                // Light everything within the intensity range
        lightLedBasedOnRow(i, CHSV(color, 255, BRIGHTNESS), j);
      } else {                                                  // Everything outside the range goes dark
        lightLedBasedOnRow(i, CHSV(color, 255, 0), j);
      }
    }
    color -= 255 / xres;                                    // Increment the Hue to get the Rainbow
  }
}


void lightLedBasedOnRow(int led, CHSV color, int row) {
  switch (row)  {
    case 0:
      lightLedFromArrayAndRow(leds[4], LEDS_PER_ROW, led, color, row);
      break;
    case 1:
      lightLedFromArrayAndRow(leds[4], LEDS_PER_ROW * 2, led, color, row);
      lightLedFromArrayAndRow(leds[4], LEDS_PER_ROW * 0, led, color, row);
      break;
    case 2:
      lightLedFromArrayAndRow(leds[5], LEDS_PER_ROW * 0, led, color, row);
      lightLedFromArrayAndRow(leds[3], LEDS_PER_ROW * 2, led, color, row);
      break;
    case 3:
      lightLedFromArrayAndRow(leds[5], LEDS_PER_ROW * 1, led, color, row);
      lightLedFromArrayAndRow(leds[3], LEDS_PER_ROW * 1, led, color, row);
      break;
    case 4:
      lightLedFromArrayAndRow(leds[5], LEDS_PER_ROW * 2, led, color, row);
      lightLedFromArrayAndRow(leds[3], LEDS_PER_ROW * 0, led, color, row);
    case 5:
      lightLedFromArrayAndRow(leds[6], LEDS_PER_ROW * 0, led, color, row);
      lightLedFromArrayAndRow(leds[2], LEDS_PER_ROW * 2, led, color, row);
    case 6:
      lightLedFromArrayAndRow(leds[6], LEDS_PER_ROW * 1, led, color, row);
      lightLedFromArrayAndRow(leds[2], LEDS_PER_ROW * 1, led, color, row);
    case 7:
      lightLedFromArrayAndRow(leds[6], LEDS_PER_ROW * 2, led, color, row);
      lightLedFromArrayAndRow(leds[2], LEDS_PER_ROW * 0, led, color, row);
    case 8:
      lightLedFromArrayAndRow(leds[7], LEDS_PER_ROW * 0, led, color, row);
      lightLedFromArrayAndRow(leds[1], LEDS_PER_ROW * 2, led, color, row);
    case 9:
      lightLedFromArrayAndRow(leds[7], LEDS_PER_ROW * 1, led, color, row);
      lightLedFromArrayAndRow(leds[1], LEDS_PER_ROW * 1, led, color, row);
    case 10:
      lightLedFromArrayAndRow(leds[7], LEDS_PER_ROW * 2, led, color, row);
      lightLedFromArrayAndRow(leds[1], LEDS_PER_ROW * 0, led, color, row);
    case  11:
      lightLedFromArrayAndRow(leds[8], LEDS_PER_ROW * 0, led, color, row);
      lightLedFromArrayAndRow(leds[0], LEDS_PER_ROW * 2, led, color, row);
    case  12:
      lightLedFromArrayAndRow(leds[8], LEDS_PER_ROW * 1, led, color, row);
      lightLedFromArrayAndRow(leds[0], LEDS_PER_ROW * 1, led, color, row);
    case  13:
      lightLedFromArrayAndRow(leds[8], LEDS_PER_ROW * 2, led, color, row);
      lightLedFromArrayAndRow(leds[0], LEDS_PER_ROW * 0, led, color, row);
      break;
  }
}


void lightLedFromArrayAndRow(CRGB arr[], int offset, int column, CHSV color, int row) {
  if (volumeMultiplier < 1.5) {
    arr[offset + 27 - column - 1] = color;
    arr[offset + 28 + column - 1] = color;
  } else {
    if (color.val != 0) {
      arr[offset + 27 - column - 1] = CRGB::Red;
      arr[offset + 28 + column - 1] = CRGB::Red;
    } else {
      arr[offset + 27 - column - 1] = color;
      arr[offset + 28 + column - 1] = color;
    }
    
  }
}

// measure basic properties of the input signal
// determine if analog or digital, determine range and average.
void MeasureAnalog() {
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
void MeasureVolume() {
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
void MeasureFHT() {

  long soundVolAvg = 0, soundVolMax = 0, soundVolRMS = 0;
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

    int amp = abs(k - AmpMax);
    amp <<= VolumeGainFactorBits;
    soundVolMax = max(soundVolMax, amp);
    soundVolAvg += amp;
    soundVolRMS += ((long)amp * amp);
    
    k -= 0x0200; // form into a signed int
    k <<= 6; // form into a 16b signed int
    k <<= FreqGainFactorBits;
    fht_input[i] = k; // put real data into bins
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

  volumeMultiplier = 1.0 + ((float) soundVolMax/100.0);
//  Serial.print("Volume: " + String(volumeMultiplier, 3));
//  Serial.print(" Amp: Max: " + String(soundVolMax,3));
//  Serial.print("% Avg: " + String(soundVolAvg));
//  Serial.print("% RMS: " + String(soundVolRMS));
//  Serial.println("% dB: " + String(dB, 3));
//  
  

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

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}


void showNewNumber() {
  if (newData == true) {
    dataNumber = 0;             // new for this version
    dataNumber = atoi(receivedChars);   // new for this version
    Serial.print("This just in ... ");
    Serial.println(receivedChars);
    Serial.print("Data as Number ... ");    // new for this version
    Serial.println(dataNumber);     // new for this version
    noise[0] = dataNumber;
    newData = false;
  }
}


// Param for different pixel layouts
const bool    kMatrixSerpentineLayout = false;
// Set 'kMatrixSerpentineLayout' to false if your pixels are
// laid out all running the same way, like this:

// Set 'kMatrixSerpentineLayout' to true if your pixels are
// laid out back-and-forth, like this:

uint16_t getXFromXY( uint8_t x, uint8_t y)
{
  uint16_t i;

  if ( kMatrixSerpentineLayout == false) {
    i = (y * xres) + x;
  }

  if ( kMatrixSerpentineLayout == true) {
    if ( y & 0x01) {
      // Odd rows run backwards
      uint8_t reverseX = (xres - 1) - x;
      i = (y * xres) + reverseX;

    } else {
      // Even rows run forwards
      i = (y * xres) + x;

    }
  }

  i = (i + counter2) % NUM_LEDS;
  return i;
}
