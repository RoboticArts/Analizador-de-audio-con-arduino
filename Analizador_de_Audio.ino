/*
 * Ejemplo en el que se muestra un espectro de frecuencias a traves de la pantalla
 * TFT, los valores se obtienen desde un random()
 */

#include <Adafruit_GFX.h>    // Libreria de graficos
#include <Adafruit_TFTLCD.h> // Libreria de LCD 
#include <TouchScreen.h>     // Libreria del panel tactil

#define YP A1  // Pin analogico A1 para ADC
#define XM A2  // Pin analogico A2 para ADC
#define YM 7 
#define XP 6 

short TS_MINX = 150; // Coordenadas del panel tactil para delimitar
short TS_MINY = 120; // el tamaño de la zona donde podemos presionar
short TS_MAXX = 850; // y que coincida con el tamaño del LCD
short TS_MAXY = 891; 


// Definimos la presion máxima y minima que podemos realizar sobre el panel
#define MINPRESSURE 1
#define MAXPRESSURE 1000

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 364); 

#define BLACK   ~0x0000 //Define colors
#define WHITE   ~0xFFFF
#define RED     ~0xF800
#define GREEN   ~0x07E0
#define BLUE    ~0x001F
#define CYAN    ~0x07FF
#define MAGENTA ~0xF81F
#define YELLOW  ~0xFFE0
#define GREY    ~0x2108

#define LCD_CS A3   // Definimos los pines del LCD
#define LCD_CD A2   // para poder visualizar elementos graficos
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET); // Instancia LCD 

#define BOXSIZE 40 // Tamaño de los cajetines de colores
#define BOXSIZEY 30 // Tamaño de los cajetines de colores
#define PENRADIUS 2 // Tamaño del cursor a la hora de pintar 

int oldcolor, currentcolor; // Colores del cursor

/////////////////////////////////////////////////

// FHT, http://wiki.openmusiclabs.com/wiki/ArduinoFHT
#define LOG_OUT 1 // use the log output function
#define LIN_OUT8 1 // use the linear byte output function
#define FHT_N 256 // set to 256 point fht
#include <FHT.h> // include the library

// pins
//#define MicPin A0 // used with analogRead mode only

// consts
#define AmpMax (1024 / 2)
#define MicSamples (1024*2) // Three of these time-weightings have been internationally standardised, 'S' (1 s) originally called Slow, 'F' (125 ms) originally called Fast and 'I' (35 ms) originally called Impulse.

// modes
#define Use3.3 // use 3.3 voltage. the 5v voltage from usb is not regulated. this is much more stable.
#define ADCReClock // switch to higher clock, not needed if we are ok with freq between 0 and 4Khz.
#define ADCFlow // read data from adc with free-run (not interupt). much better data, dc low. hardcoded for A0.

#define FreqLog // use log scale for FHT frequencies
#ifdef FreqLog
#define FreqOutData fht_log_out
#define FreqGainFactorBits 0
#else
#define FreqOutData fht_lin_out8
#define FreqGainFactorBits 3
#endif
#define FreqSerialBinary

#define VolumeGainFactorBits 0

// macros
// http://yaab-arduino.blogspot.co.il/2015/02/fast-sampling-from-analog-input.html
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))



 
/////////////////////////////////////////////

// Meter colour schemes
#define RED2RED 0
#define GREEN2GREEN 1
#define BLUE2BLUE 2
#define BLUE2RED 3
#define GREEN2RED 4
#define RED2GREEN 5

///////////////////////7
    int randomNumber25  = 0;
    int randomNumber = 0;
    int offsetX = 50;
    int offsetY = 10;

    int decibelios = 0;

void setup() {
  
  tft.begin(0x9341); // Iniciamos el LCD especificando el controlador ILI9341. 
  tft.fillScreen(BLACK); // Pintamos el fondo de negro
   
  Serial.begin(9600);
/*
    tft.setCursor(-20 + offsetX , 12 + offsetY );  // Situamos el cursor en la posicion del LCD deseada,
                            // (X, Y) siendo X el ancho (240 px max.) e Y el alto (320 px max.) 

      tft.setTextSize(1); // Definimos tamaño del texto. (Probado tamaños del 1 al 10)
    
    tft.setTextColor(WHITE); // Definimos el color del texto 
    
    tft.println("25"); // Escribimos nuestro texto en el LCD. Realizará un salto de linea 
                          // automatico si el texto es mayor que el tamaño del LCD

    
    tft.setCursor(-20 + offsetX, 32 + offsetY);
    tft.println("50");

    tft.setCursor(-26 + offsetX, 52 + offsetY);
    tft.println("100");

    tft.setCursor(-26 + offsetX, 72 + offsetY);
    tft.println("200");

    tft.setCursor(-26 + offsetX, 91 + offsetY);
    tft.println("500");
    
    tft.setCursor(-26 + offsetX, 111 + offsetY);
    tft.println("800");
    
    tft.setCursor(-30 + offsetX, 132 + offsetY);
    tft.println("1000");
    
    tft.setCursor(-30 + offsetX, 152 + offsetY);
    tft.println("1500");

    tft.setCursor(-30 + offsetX, 172 + offsetY);
    tft.println("2000");

    tft.setCursor(-30 + offsetX, 191 + offsetY);
    tft.println("2500");

    tft.setCursor(-30 + offsetX, 211 + offsetY);
    tft.println("3000");

    tft.setCursor(-30 + offsetX, 231 + offsetY);
    tft.println("5000");

    tft.setCursor(-35 + offsetX, 251 + offsetY);
    tft.println("10000");

    tft.setCursor(-35 + offsetX, 271 + offsetY);
    tft.println("20000");
*/
////////////////////////////////

#ifdef ADCFlow
  // set the adc to free running mode
  // register explanation: http://maxembedded.com/2011/06/the-adc-of-the-avr/
  // 5 => div 32. sample rate 38.4
  // 7 => switch to divider=128, default 9.6khz sampling

  //ADCSRA = 0xe0+7; // "ADC Enable", "ADC Start Conversion", "ADC Auto Trigger Enable" and divider.
  ADCSRA = 0xE5;
  //ADMUX = 0x0; // use adc0 (hardcoded, doesn't use MicPin). Use ARef pin for analog reference (same as analogReference(EXTERNAL)).
  ADMUX = 0x5; // ADC 5 sin configurar Vref
  
#ifndef Use3.3
  ADMUX |= 0x40; // Use Vcc for analog reference.
#endif
  //DIDR0 = 0x01; // turn off the digital input for adc0
    DIDR0 = 0x20;
#else
#ifdef Use3.3
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

////////////////////////////////77


}

void loop() {
  tft.setRotation(0);
/*

    //25Hz
    randomNumber =  random(0,100);
    tft.fillRect(0 + offsetX, 10 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 10 + offsetY, randomNumber , 10, WHITE); // Dibujamos un cuadrado/rectangulo relleno de color 
                                       //(Punto inicial X, Punto inicial Y, Longitud X,Longitud Y, Color)

    //50Hz
    randomNumber = random(0,100);
    tft.fillRect(0  + offsetX, 30 + offsetY , 200, 10, BLACK);
    tft.fillRect(0  + offsetX, 30 + offsetY , randomNumber, 10, WHITE ); 

    //100Hz
    randomNumber = random(0,100);
    tft.fillRect(0 + offsetX, 50 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 50 + offsetY, randomNumber, 10, WHITE ); 
    
    
    //200HZ
    randomNumber = random(0,100);
    tft.fillRect(0 + offsetX,70 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 70 + offsetY, randomNumber, 10, WHITE ); 
    

    //500hz
    randomNumber = random(0,100);
    tft.fillRect(0 + offsetX,90 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 90 + offsetY, randomNumber, 10, WHITE );   

    //800Hz
    randomNumber = random(0,100);
    tft.fillRect(0 + offsetX,110 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 110 + offsetY, randomNumber, 10, WHITE ); 
   

    //1KHz
    randomNumber = random(0,100);
    tft.fillRect(0 + offsetX,130 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 130 + offsetY, randomNumber, 10, WHITE ); 
    

    //1.5 Khz
    randomNumber = random(0,100);
    tft.fillRect(0 + offsetX,150 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 150 + offsetY, randomNumber, 10, WHITE ); 
     

    //2kHz
    randomNumber = random(0,100);
    tft.fillRect(0 + offsetX,170 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 170 + offsetY, randomNumber, 10, WHITE ); 
  

    //2.5 KHz
    randomNumber = random(0,100);
    tft.fillRect(0 + offsetX,190 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 190 + offsetY, randomNumber, 10, WHITE );  

    //3Khz
    randomNumber = random(0,100);
    tft.fillRect(0 + offsetX,210 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 210 + offsetY, randomNumber, 10, WHITE ); 
   

    //5KHz
    randomNumber = random(0,100);
    tft.fillRect(0 + offsetX,230 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 230 + offsetY, randomNumber, 10 , WHITE );   

    //10Khz
    randomNumber = random(0,100);
    tft.fillRect(0 + offsetX,250 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 250 + offsetY, randomNumber, 10 , WHITE ); 
  

    //20Khz
    randomNumber = random(0,100);
    tft.fillRect(0 + offsetX,270 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 270 + offsetY, randomNumber, 10 , WHITE ); 
*/
     //MeasureAnalog();
     MeasureVolume();

    tft.setCursor(-30 + offsetX, 152 + offsetY);
    tft.setTextSize(5); 
    tft.setTextColor(WHITE); 
    tft.fillRect(-30 + offsetX,152 + offsetY, 200, 50, BLACK);
    tft.println(decibelios);
     
/*
    int xpos = 0, ypos = 5, gap = 4, radius = 52;
    // Draw a large meter
    xpos = 320/2 - 160, ypos = 0, gap = 100, radius = 105;
    ringMeter(decibelios,0,160, xpos,ypos,radius,"dB",RED2RED); // Draw analogue meter
  */  
} 



void MeasureAnalog()
{  
  long signalAvg = 0, signalMax = 0, signalMin = 1024, t0 = millis();
  //cli();  // UDRE interrupt slows this way down on arduino1.0
  for (int i = 0; i < MicSamples; i++)
  { 

#ifdef ADCFlow

      while (!(ADCSRA & _BV(ADIF))); // wait for adc to be ready (ADIF)
      sbi(ADCSRA, ADIF); // restart adc
      byte m = ADCL; // fetch adc data
      byte j = ADCH;
      int k = ((int)j << 8) | m; // form into an int

#endif  
    signalMin = min(signalMin, k);
    signalMax = max(signalMax, k);
    signalAvg += k;


  }
  signalAvg /= MicSamples;
  //sei();


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
    //int k = analogRead(MicPin);
#endif
    int amp = abs(k - AmpMax);
    amp <<= VolumeGainFactorBits;
    soundVolMax = max(soundVolMax, amp);
    soundVolAvg += amp;
    soundVolRMS += ((long)amp*amp);
  }
  soundVolAvg /= MicSamples;
  soundVolRMS /= MicSamples;
  float soundVolRMSflt = sqrt(soundVolRMS);
  //sei();

  float dB = 20.0*log10(soundVolRMSflt/AmpMax);

  // convert from 0 to 100
  soundVolAvg = 100 * soundVolAvg / AmpMax; 
  soundVolMax = 100 * soundVolMax / AmpMax; 
  soundVolRMSflt = 100 * soundVolRMSflt / AmpMax;
  soundVolRMS = 10 * soundVolRMSflt / 7; // RMS to estimate peak (RMS is 0.7 of the peak in sin)

  // print
  //Serial.print("Time: " + String(millis() - t0));
  //Serial.print(" Amp: Max: " + String(soundVolMax));
  //Serial.print("% Avg: " + String(soundVolAvg));
  //Serial.print("% RMS: " + String(soundVolRMS));
  dB = 200 - abs(dB*10);
  Serial.println("% dB: " + String(dB,3));

  decibelios = int(dB);

}

// #########################################################################
//  Draw the meter on the screen, returns x coord of righthand side
// #########################################################################
int ringMeter(int value, int vmin, int vmax, int x, int y, int r, char *units, byte scheme)
{
  // Minimum value of r is about 52 before value text intrudes on ring
  // drawing the text first is an option
  /*
  x += r; y += r;   // Calculate coords of centre of ring
  int w = r / 3;    // Width of outer ring is 1/4 of radius 
  int angle = 150;  // Half the sweep angle of meter (300 degrees)
  int v = map(value, vmin, vmax, -angle, angle); // Map the value to an angle v
  byte seg = 3; // Segments are 3 degrees wide = 100 segments for 300 degrees
  byte inc = 6; // Draw segments every 3 degrees, increase to 6 for segmented ring
  // Variable to save "value" text colour from scheme and set default
  int colour = BLUE;
 
  // Draw colour blocks every inc degrees
  for (int i = -angle+inc/2; i < angle-inc/2; i += inc) {
    // Calculate pair of coordinates for segment start
    float sx = cos((i - 90) * 0.0174532925);
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (r - w) + x;
    uint16_t y0 = sy * (r - w) + y;
    uint16_t x1 = sx * r + x;
    uint16_t y1 = sy * r + y;

    // Calculate pair of coordinates for segment end
    float sx2 = cos((i + seg - 90) * 0.0174532925);
    float sy2 = sin((i + seg - 90) * 0.0174532925);
    int x2 = sx2 * (r - w) + x;
    int y2 = sy2 * (r - w) + y;
    int x3 = sx2 * r + x;
    int y3 = sy2 * r + y;

    if (i < v) { // Fill in coloured segments with 2 triangles
      switch (scheme) {
        case 0: colour = RED; break; // Fixed colour
        case 1: colour = GREEN; break; // Fixed colour
        case 2: colour = BLUE; break; // Fixed colour
        //case 3: colour = rainbow(map(i, -angle, angle, 0, 127));  break; // Full spectrum blue to red
        //case 4: colour = rainbow(map(i, -angle, angle, 70, 127)); break; // Green to red (high temperature etc)
        //case 5: colour = rainbow(map(i, -angle, angle, 127, 63));  break; // Red to green (low battery etc)
        default: colour = BLUE; break; // Fixed colour
      }
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, colour);
     // tft.fillTriangle(x1, y1, x2, y2, x3, y3, colour);
//      text_colour = colour; // Save the last colour drawn
    }
    else // Fill in blank segments
    {
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, GREY);
      //tft.fillTriangle(x1, y1, x2, y2, x3, y3, GREY);
    }
  }
  
  // Convert value to a string
  char buf[10];
  byte len = 2; if (value > 999) len = 4;
  dtostrf(value, len, 0, buf);
  buf[len] = ' '; buf[len] = 0; // Add blanking space and terminator, helps to centre text too!
  // Set the text colour to default
  tft.setTextSize(1);

  if(value>9){
  tft.setTextColor(colour,BLACK);
  tft.setCursor(x-25,y-10);tft.setTextSize(5);
  tft.print(buf);}
  if(value<10){
  tft.setTextColor(colour,BLACK);
  tft.setCursor(x-25,y-10);tft.setTextSize(5);
  tft.print(buf);
  }

  tft.setTextColor(WHITE,BLACK);
  
  tft.setCursor(x-39,y+75);tft.setTextSize(2);
  tft.print(units); // Units display
  */
  // Calculate and return right hand side x coordinate
  return x + r;
}



