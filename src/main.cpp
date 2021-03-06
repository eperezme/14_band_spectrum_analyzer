#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_NeoPixel.h"
#include "si5351mcu.h"    //Si5351mcu library
Si5351mcu Si;             //Si5351mcu Board
#define PULSE_PIN     13
#define NOISE         50
#define ROWS          20  //num of row MAX=20
#define COLUMNS       14  //num of column
#define DATA_PIN      9   //led data pin
#define STROBE_PIN    6   //MSGEQ7 strobe pin
#define RESET_PIN     7   //MSGEQ7 reset pin
#define NUMPIXELS    ROWS * COLUMNS
int sensor=A3;
int pot_peak_r= A4;
int pot_peak_g= A5;
int pot_peak_b= A6;
int pot_col_r= A7;
int pot_col_g= A8;
int pot_col_b= A9;
int pot_selector_rainbow= A10;
int pot_selector_custom = A11;

struct Point{
char x, y;
char  r,g,b;
bool active;
};
struct TopPoint{
int position;
int peakpause;
};
Point spectrum[ROWS][COLUMNS];
TopPoint peakhold[COLUMNS];
int spectrumValue[COLUMNS];
long int counter = 0;
int long pwmpulse = 0;
bool toggle = false;
int long time_change = 0;
int effect = 0;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, DATA_PIN, NEO_GRB + NEO_KHZ800);

void clearspectrum();
void flushMatrix();
void topSinking();
int sensorPotent;
int outputBright;
int peak_r_read;
int peak_g_read;
int peak_b_read;
int col_r_read;
int col_g_read;
int col_b_read;
int peak_r;
int peak_g;
int peak_b;
int col_r;
int col_g;
int col_b;
int rainbow;
int custom;

void setup()
 {
 Si.init(25000000L);
 Si.setFreq(0, 104570);
 Si.setFreq(1, 166280);
 Si.setPower(0, SIOUT_8mA);
 Si.setPower(1, SIOUT_8mA);
 Si.enable(0);
 Si.enable(1);
 pinMode      (STROBE_PIN,    OUTPUT);
 pinMode      (RESET_PIN,     OUTPUT);
 pinMode      (DATA_PIN,      OUTPUT);
 pinMode      (PULSE_PIN,     OUTPUT);
 digitalWrite(PULSE_PIN, HIGH);
 delay(100);
 digitalWrite(PULSE_PIN, LOW);
 delay(100);
 digitalWrite(PULSE_PIN, HIGH);
 delay(100);
 digitalWrite(PULSE_PIN, LOW);
 delay(100);
 digitalWrite(PULSE_PIN, HIGH);
 delay(100);
 pixels.setBrightness(20); //set Brightness
 pixels.begin();
 pixels.show();
 pinMode      (STROBE_PIN, OUTPUT);
 pinMode      (RESET_PIN,  OUTPUT);
 digitalWrite (RESET_PIN,  LOW);
 digitalWrite (STROBE_PIN, LOW);
 delay        (1);
 digitalWrite (RESET_PIN,  HIGH);
 delay        (1);
 digitalWrite (RESET_PIN,  LOW);
 digitalWrite (STROBE_PIN, HIGH);
 delay        (1);
 }

void loop()
  {
  counter++;
  clearspectrum();
  peak_r_read = analogRead(pot_peak_r); //Reads the value of RGB for peak from potentiometer
  peak_g_read = analogRead(pot_peak_g);
  peak_b_read = analogRead(pot_peak_b);
  col_r_read = analogRead(pot_col_r); //Reads the value of RGB for colum from potentiometer
  col_g_read = analogRead(pot_col_g);
  col_b_read = analogRead(pot_col_b);

  peak_r = map(peak_r_read, 0, 1023, 0, 255); //Maps value to led intensity
  peak_g = map(peak_g_read, 0, 1023, 0, 255);
  peak_b = map(peak_b_read, 0, 1023, 0, 255);
  col_r = map(col_r_read, 0, 1023, 0, 255);
  col_g = map(col_g_read, 0, 1023, 0, 255);
  col_b = map(col_b_read, 0, 1023, 0, 255);

  if (millis() - pwmpulse > 3000){
  toggle = !toggle;
  digitalWrite(PULSE_PIN, toggle);
  pwmpulse = millis();
  }
  digitalWrite(RESET_PIN, HIGH);
  delayMicroseconds(3000);
  digitalWrite(RESET_PIN, LOW);
  for(int i=0; i < COLUMNS; i++){

  digitalWrite(STROBE_PIN, LOW);
  delayMicroseconds(1000);

  spectrumValue[i] = analogRead(0);
  if(spectrumValue[i] < 120)spectrumValue[i] = 0;
  spectrumValue[i] = constrain(spectrumValue[i], 0, 1023);
  spectrumValue[i] = map(spectrumValue[i], 0, 1023, 0, ROWS);i++;
  spectrumValue[i] = analogRead(1);
  if(spectrumValue[i] < 120)spectrumValue[i] = 0;
  spectrumValue[i] = constrain(spectrumValue[i], 0, 1023);
  spectrumValue[i] = map(spectrumValue[i], 0, 1023, 0, ROWS);
  digitalWrite(STROBE_PIN, HIGH);
  }

  rainbow = analogRead(pot_selector_rainbow);
  custom = analogRead(pot_selector_custom);

  for(int j = 0; j < COLUMNS; j++) {
      for (int i = 0; i < spectrumValue[j]; i++) {
          if( rainbow != 0) {
              spectrum[i][COLUMNS - 1 - j].active = 1;
              if (i <= (ROWS / 4)) {
                  spectrum[i][COLUMNS - 1 - j].r = 0;
                  spectrum[i][COLUMNS - 1 - j].g = map(i, 1, (ROWS / 4), 0, 255);
                  spectrum[i][COLUMNS - 1 - j].b = 255;
              }
              if (i <= 2 * (ROWS / 4) && i > (ROWS / 4)) {
                  spectrum[i][COLUMNS - 1 - j].r = 0;
                  spectrum[i][COLUMNS - 1 - j].g = 255;
                  spectrum[i][COLUMNS - 1 - j].b = map(i, (ROWS / 4), 2 * (ROWS / 4), 255, 0);
              }
              if (i <= 3 * (ROWS / 4) && i > 2 * (ROWS / 4)) {
                  spectrum[i][COLUMNS - 1 - j].r = map(i, 2 * (ROWS / 4), 3 * (ROWS / 4), 0, 255);
                  spectrum[i][COLUMNS - 1 - j].g = 255;
                  spectrum[i][COLUMNS - 1 - j].b = 0;
              }
              if (i <= ROWS && i > 3 * (ROWS / 4)) {
                  spectrum[i][COLUMNS - 1 - j].r = 255;
                  spectrum[i][COLUMNS - 1 - j].g = map(i, 3 * (ROWS / 4), ROWS, 0, 255);
                  spectrum[i][COLUMNS - 1 - j].b = 0;
              }
          }
          if( custom != 0) {
              spectrum[i][COLUMNS - 1 - j].active = 1;
              spectrum[i][COLUMNS - 1 - j].r =col_r;           //COLUMN Color red
              spectrum[i][COLUMNS - 1 - j].g =col_g;         //COLUMN Color green
              spectrum[i][COLUMNS - 1 - j].b =col_b;           //COLUMN Color blue
          }
      }

  if(spectrumValue[j] - 1 > peakhold[j].position)
  {
  spectrum[spectrumValue[j] - 1][COLUMNS - 1 - j].r = 0;
  spectrum[spectrumValue[j] - 1][COLUMNS - 1 - j].g = 0;
  spectrum[spectrumValue[j] - 1][COLUMNS - 1 - j].b = 0;
  peakhold[j].position = spectrumValue[j] - 1;
  peakhold[j].peakpause = 1; //set peakpause
  }
  else
  {
  spectrum[peakhold[j].position][COLUMNS - 1 - j].active = 1;
  spectrum[peakhold[j].position][COLUMNS - 1 - j].r = peak_r;  //Peak Color red
  spectrum[peakhold[j].position][COLUMNS - 1 - j].g = peak_g;  //Peak Color green
  spectrum[peakhold[j].position][COLUMNS - 1 - j].b = peak_b;    //Peak Color blue
  }
  }
  flushMatrix();
  if(counter % 3 ==0)topSinking(); //peak delay
  }
  void topSinking() {
  for(int j = 0; j < ROWS; j++)
  {
  if(peakhold[j].position > 0 && peakhold[j].peakpause <= 0) peakhold[j].position--;
  else if(peakhold[j].peakpause > 0) peakhold[j].peakpause--;
  }
  }
  void clearspectrum() {
  for(int i = 0; i < ROWS; i++)
  {
  for(int j = 0; j < COLUMNS; j++)
  {
  spectrum[i][j].active = false;
  }
  }
  }
  void flushMatrix() {
  for(int j = 0; j < COLUMNS; j++)
  {
  if( j % 2 != 0)
  {
  for(int i = 0; i < ROWS; i++)
  {
  if(spectrum[ROWS - 1 - i][j].active)
  {
  pixels.setPixelColor(j * ROWS + i, pixels.Color(
  spectrum[ROWS - 1 - i][j].r,
  spectrum[ROWS - 1 - i][j].g,
  spectrum[ROWS - 1 - i][j].b));
  }
  else
  {
  pixels.setPixelColor( j * ROWS + i, 0, 0, 0);
  }
  }
  }
  else
  {
  for(int i = 0; i < ROWS; i++)
  {
  if(spectrum[i][j].active)
  {
  pixels.setPixelColor(j * ROWS + i, pixels.Color(
  spectrum[i][j].r,
  spectrum[i][j].g,
  spectrum[i][j].b));
  }
  else
  {
  pixels.setPixelColor( j * ROWS + i, 0, 0, 0);
  }
  }
  }
  }
  sensorPotent = analogRead(sensor);
  outputBright = map(sensorPotent, 0, 1023, 0, 255);
  pixels.setBrightness(outputBright);
  pixels.show();
  }
