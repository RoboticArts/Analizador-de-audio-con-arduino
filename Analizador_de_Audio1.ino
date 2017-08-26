#include <Adafruit_TFTLCD.h>
#include <pin_magic.h>
#include <registers.h>


#include <Adafruit_GFX.h>    // Libreria de graficos
#include <Adafruit_TFTLCD.h> // Libreria de LCD 
#include <TouchScreen.h>     // Libreria del panel tactil


/************************** CONFIGURACÓN DE LA PANTALLA TFT *****************/
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
//Es RGB 565: http://www.barth-dev.de/online/rgb565-color-picker/
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

/******************* CONFIGURACIÓN DEL ADC POR REGISTROS *******************/

// FHT, http://wiki.openmusiclabs.com/wiki/ArduinoFHT
#define LOG_OUT 1 // use the log output function
#define FHT_N 256 // set to 256 point fht
#include <FHT.h> // include the library

// consts
#define AmpMax (1024 / 2)
#define MicSamples (1024*2) // Three of these time-weightings have been internationally standardised, 'S' (1 s) originally called Slow, 'F' (125 ms) originally called Fast and 'I' (35 ms) originally called Impulse.




// macros
// http://yaab-arduino.blogspot.co.il/2015/02/fast-sampling-from-analog-input.html
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))



/********************** VARIABLES DEL CÓDIGO ******************/

    int randomNumber = 0;
    int last_randomNumber = 0;
    int offsetX = 50;
    int offsetY = 10;
  
    //Para la aguja del tacómetro
    double circleX = 160;
    double circleY = 100;
    double radio = 120;

void setup() {
  
  tft.begin(0x9341); // Iniciamos el LCD especificando el controlador ILI9341. 
  tft.fillScreen(BLACK); // Pintamos el fondo de negro
   
  Serial.begin(9600);

  init_fast_ADC();

}

void loop() {

  /*
    tft.setRotation(0);

    //25Hz
    randomNumber = map( fht_log_out [0], 0 , 256, 0 ,100); 
    tft.fillRect(0 + offsetX, 10 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 10 + offsetY, randomNumber , 10, WHITE); // Dibujamos un cuadrado/rectangulo relleno de color 
                                       //(Punto inicial X, Punto inicial Y, Longitud X,Longitud Y, Color)
    //50Hz
    randomNumber = map( fht_log_out [1], 0 , 256, 0 ,100); 
    tft.fillRect(0  + offsetX, 30 + offsetY , 200, 10, BLACK);
    tft.fillRect(0  + offsetX, 30 + offsetY , randomNumber, 10, WHITE ); 
    
    //100Hz
    randomNumber = map( fht_log_out [2], 0 , 256, 0 ,100); 
    tft.fillRect(0 + offsetX, 50 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 50 + offsetY, randomNumber, 10, WHITE ); 
    
    
    //200HZ
    randomNumber = map( fht_log_out [3], 0 , 256, 0 ,100); 
    tft.fillRect(0 + offsetX,70 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 70 + offsetY, randomNumber, 10, WHITE ); 
    
    //500hz
    randomNumber = map( fht_log_out [4], 0 , 256, 0 ,100); 
    tft.fillRect(0 + offsetX,90 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 90 + offsetY, randomNumber, 10, WHITE );   
    
    //800Hz
    randomNumber = map( fht_log_out [5], 0 , 256, 0 ,100); 
    tft.fillRect(0 + offsetX,110 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 110 + offsetY, randomNumber, 10, WHITE ); 
   
    //1KHz
    randomNumber = map( fht_log_out [6], 0 , 256, 0 ,100); 
    tft.fillRect(0 + offsetX,130 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 130 + offsetY, randomNumber, 10, WHITE ); 
    
    //1.5 Khz
    randomNumber = map( fht_log_out [7], 0 , 256, 0 ,100); 
    tft.fillRect(0 + offsetX,150 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 150 + offsetY, randomNumber, 10, WHITE ); 
     
    //2kHz
    //randomNumber = map( fht_log_out [8], 0 , 256, 0 ,100); 
    tft.fillRect(0 + offsetX,170 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 170 + offsetY, randomNumber, 10, WHITE ); 
  
    //2.5 KHz
    randomNumber = map( fht_log_out [9], 0 , 256, 0 ,100); 
    tft.fillRect(0 + offsetX,190 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 190 + offsetY, randomNumber, 10, WHITE );  
    
    //3Khz
    randomNumber = map( fht_log_out [10], 0 , 256, 0 ,100); 
    tft.fillRect(0 + offsetX,210 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 210 + offsetY, randomNumber, 10, WHITE ); 
   
    //5KHz
    randomNumber = map( fht_log_out [11], 0 , 256, 0 ,100); 
    tft.fillRect(0 + offsetX,230 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 230 + offsetY, randomNumber, 10 , WHITE );   
    
    //10Khz
    //randomNumber = map( fht_log_out [12], 0 , 256, 0 ,100); 
    tft.fillRect(0 + offsetX,250 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 250 + offsetY, randomNumber, 10 , WHITE ); 
  
    //20Khz
    randomNumber = map( fht_log_out [13], 0 , 256, 0 ,100); 
    tft.fillRect(0 + offsetX,270 + offsetY, 200, 10, BLACK);
    tft.fillRect(0 + offsetX, 270 + offsetY, randomNumber, 10 , WHITE ); 
  */


/*
    tft.setCursor(-30 + offsetX, 152 + offsetY);
    tft.setTextSize(5); 
    tft.setTextColor(WHITE); 
    tft.fillRect(-30 + offsetX,152 + offsetY, 200, 50, BLACK);
    tft.println(decibelios);
  */   


  /* 
   for(int i = 157; i>=0; i--){
   float alpha = float(i)/100; // Primer cuadrante
   tft.fillTriangle(int(100*cos(alpha)+160),int(240-(100*sin(alpha)+120)),160,120,int(100*cos(alpha)+160),120,RED);
   
   //Cuarto cuadrante
   tft.fillTriangle(int(100*cos(alpha)+160),int((100*sin(alpha)+120)),160,120,int(100*cos(alpha)+160),120,BLUE);
   
   float beta = 1.57+(1.57-alpha); //Segundo cuadrante
   tft.fillTriangle(int(100*cos(beta)+160),int(240-(100*sin(beta)+120)),160,120,int(100*cos(beta)+160),120,YELLOW);
   
   //Tercer cuadrante
   tft.fillTriangle(int(100*cos(beta)+160),int((100*sin(beta)+120)),160,120,int(100*cos(beta)+160),120,GREEN);
   }
   tft.fillCircle(160,120, 65, BLACK);
   delay(10000);
   delay(10000);
   delay(10000);
   */
 /*  
    //show_SoundMeter(3); 
    // MODOS
    // 1 -> Amplitud en %
    // 2 -> RMS en %
    // 3 -> dB
*/
     show_SpectrumFFT(1);
    //MODOS
    // 1 -> Modo ancho de banda
    // 2 -> Modo música
    
} 




float MeasureVolume(int mode)
{
  float soundVolAvg = 0, soundVolMax = 0, soundVolRMS = 0;

  for (int i = 0; i < MicSamples; i++)
  {
    

    while (!(ADCSRA & _BV(ADIF))); // wait for adc to be ready (ADIF)
    sbi(ADCSRA, ADIF); // restart adc
    byte m = ADCL; // fetch adc data
    byte j = ADCH;
    int k = ((int)j << 8) | m; // form into an int


    int amp = abs(k - AmpMax);
    //amp <<= VolumeGainFactorBits;
    soundVolMax = max(soundVolMax, amp);
    soundVolRMS += ((float)amp*amp);
  }

  soundVolRMS /= MicSamples;
  float soundVolRMSflt = sqrt(soundVolRMS);


  float dB = 20.0*log10(soundVolRMSflt/AmpMax);

  // convert from 0 to 100
  soundVolMax = 100 * soundVolMax / AmpMax; 
  soundVolRMSflt = 100 * soundVolRMSflt / AmpMax;
  soundVolRMS = 10 * soundVolRMSflt / 7; // RMS to estimate peak (RMS is 0.7 of the peak in sin)


  if(mode == 1)
     return soundVolMax;
  if(mode == 2)
    return soundVolRMS;
  if(mode == 3)
    return dB;
  
 return -1; 

}

void init_fast_ADC()
{
  
  ADCSRA = 0xE5;
  ADMUX = 0x5; // ADC 5 sin configurar Vref
  DIDR0 = 0x20;
  analogReference(EXTERNAL); // 3.3V to AREF
  // http://yaab-arduino.blogspot.co.il/2015/02/fast-sampling-from-analog-input.html
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);

}

void init_SoundMeter(double circleX, double circleY, double radio){

    double lados = 32;
    double arista= (2*PI)/ lados;
    int color;
  
      tft.setRotation(1);
   //Se crea una figura geometrica de 32 lados
    
    for(double i = 1; i<=lados; i++){
      
      if(i>=0 && i <= 6){color = BLUE;}
      if(i>6 && i <= 20){color = ~0x4583;} //GREEN
      if(i>20 && i<=28){color = BLACK;}
      if(i>28 && i<=32){color = RED;}
      tft.fillTriangle(circleX,240- circleY, int(radio*cos(arista*i)+circleX),int(240-(radio*sin(arista*i)+circleY)),int(radio*cos((arista*i)-0.2)+circleX),int(240-(radio*sin((arista*i)-0.2)+circleY)),color);  
    }
    
    //Se dibujan las marcas de divison de segmentos
    for(double i = 2; i <= lados; i+=2)
    {
     if(i<=20 || i >28)
          { 
            tft.drawLine(circleX,240- circleY,int(radio*cos(arista*i)+circleX),int(240-(radio*sin(arista*i)+circleY)), WHITE); 
          }     
     }

    //Se borra el centro de la figura gemoetrica
    tft.fillCircle(circleX,240-circleY,radio-20, BLACK);
    
    //Se alisa la figura geometrica
    tft.drawCircle(circleX,240-circleY,radio,BLACK);
    tft.drawCircle(circleX,240-circleY,radio+1,BLACK);
    tft.drawCircle(circleX,240-circleY,radio+2,BLACK);

    //Se añaden detalles
    tft.drawCircle(circleX,240-circleY,radio+8,YELLOW);
  }

void init_SpectrumFFT(int mode){
   
    tft.setRotation(0);
    tft.setTextSize(1); // Definimos tamaño del texto. (Probado tamaños del 1 al 10) 
    tft.setTextColor(WHITE); // Definimos el color del texto
    
    float sample = 38000.00/256.00;

    if(mode == 2){

    tft.setCursor(-26 + offsetX , 12 + offsetY );                       
    tft.println(int(sample*2));   
    tft.setCursor(-26 + offsetX, 32 + offsetY);
    tft.println(int(sample*4));
    tft.setCursor(-26 + offsetX, 52 + offsetY);
    tft.println(int(sample*6));
    tft.setCursor(-30 + offsetX, 72 + offsetY);
    tft.println(int(sample*8));
    tft.setCursor(-30 + offsetX, 92 + offsetY);
    tft.println(int(sample*10));
    tft.setCursor(-30 + offsetX, 112 + offsetY);
    tft.println(int(sample*12));  
    tft.setCursor(-30 + offsetX, 132 + offsetY);
    tft.println(int(sample*14));   
    tft.setCursor(-30 + offsetX, 152 + offsetY);
    tft.println(int(sample*16));
    tft.setCursor(-30 + offsetX, 172 + offsetY);
    tft.println(int(sample*18));
    tft.setCursor(-30 + offsetX, 192 + offsetY);
    tft.println(int(sample*20));
    tft.setCursor(-30 + offsetX, 212 + offsetY);
    tft.println(int(sample*22));
    tft.setCursor(-30 + offsetX, 232 + offsetY);
    tft.println(int(sample*24));
    tft.setCursor(-35 + offsetX, 252 + offsetY);
    tft.println(int(sample*26));
    tft.setCursor(-35 + offsetX, 272 + offsetY);
    tft.println(int(sample*28));
    }

    if(mode == 1){
         
    
    tft.setCursor(-30 + offsetX , 12 + offsetY );                       
    tft.println(int(sample*9*1));   
    tft.setCursor(-30 + offsetX, 32 + offsetY);
    tft.println(int(sample*9*2));
    tft.setCursor(-30 + offsetX, 52 + offsetY);
    tft.println(int(sample*9*3));
    tft.setCursor(-30 + offsetX, 72 + offsetY);
    tft.println(int(sample*9*4));
    tft.setCursor(-30 + offsetX, 92 + offsetY);
    tft.println(int(sample*9*5));
    tft.setCursor(-30 + offsetX, 112 + offsetY);
    tft.println(int(sample*9*6));  
    tft.setCursor(-30 + offsetX, 132 + offsetY);
    tft.println(int(sample*9*7));   
    tft.setCursor(-35 + offsetX, 152 + offsetY);
    tft.println(int(sample*9*8));
    tft.setCursor(-35 + offsetX, 172 + offsetY);
    tft.println(int(sample*9*9));
    tft.setCursor(-35 + offsetX, 192 + offsetY);
    tft.println(int(sample*9*10));
    tft.setCursor(-35 + offsetX, 212 + offsetY);
    tft.println(int(sample*9*11));
    tft.setCursor(-35 + offsetX, 232 + offsetY);
    tft.println(int(sample*9*12));
    tft.setCursor(-35 + offsetX, 252 + offsetY);
    tft.println(int(sample*9*13));
    tft.setCursor(-35 + offsetX, 272 + offsetY);
    tft.println(int(sample*9*14));
    }
     
  }

void show_SoundMeter(int mode){
  
     static bool key_sound = 1; //Se inicia una vez
     if(key_sound == 1){
      key_sound = 0; 
      init_SoundMeter(160,100,120); // Posicion X, Posicion Y, Radio /// Probado con R:100  X: 160 Y: 120
     }
 
    float medida = MeasureVolume(mode); // 1: Amplitud Max 2: RMS 3: Decibelios
    static float ultima_medida = 0;

    tft.setTextSize(2);
    tft.setTextColor(WHITE);
    tft.setCursor(circleX - 35 , 240 - circleY + 50);  
    tft.fillRect(circleX - 35 , 240 - circleY + 50, 80, 35, BLACK);                      
    tft.println(medida);
  

      tft.setTextSize(2); 
      tft.setTextColor(WHITE);
      tft.setCursor(circleX,circleY);
     
        switch(mode)
        {          
          case 1: tft.println("Amp %"); break;
          case 2: tft.println("RMS %"); break;
          case 3: tft.println("dB"); break;  
          
        }           

    
    //Tacometer
    float MIN_t =  (((2*PI)/32)*20)*100; // Min
    float MAX_t = -(((2*PI)/32)*4)*100; // Max
    
    medida = medida*100;
        
    if(mode == 3)
    {
      
      //Signal in dB
      float MIN_dB = -1300; //Modificar para calibrar el sonometro
      float MAX_dB =  0;
       
      medida = map(int(medida), int(MIN_dB), int(MAX_dB), int (MIN_t), int(MAX_t));
    }

    if(mode == 2 || mode == 1)
    {
      float MIN_100 = 0*100; // Modificar para calibrar la amplitud maxima
      float MAX_100 = 100*100; 

      if(medida > 100*100){ medida = 100*100; }
       
      medida = map(int(medida), int(MIN_100), int(MAX_100), int(MIN_t), int (MAX_t));
    }

    medida = medida/100; // Se pasa a decimal el dato otra vez
    tft.drawLine(circleX,240- circleY,int((radio-30)*cos(ultima_medida)+circleX),int(240-((radio-30)*sin(ultima_medida)+circleY)), BLACK); //Se borra el dato anterior 
    ultima_medida = medida;
    tft.drawLine(circleX,240- circleY,int((radio-30)*cos(medida)+circleX),int(240-((radio-30)*sin(medida)+circleY)), YELLOW); // Se muestra en nuevo dato
    
  }

void MeasureFHT(){
  
  for (int i = 0; i < FHT_N; i++) { // save 256 samples

    while (!(ADCSRA & /*0x10*/_BV(ADIF))); // wait for adc to be ready (ADIF)
    sbi(ADCSRA, ADIF); // restart adc
    byte m = ADCL; // fetch adc data
    byte j = ADCH;
    int k = ((int)j << 8) | m; // form into an int

    k -= 0x0200; // form into a signed int
    k <<= 6; // form into a 16b signed int
    //k <<= FreqGainFactorBits;
    
    fht_input[i] = k; // put real data into bins
    
  }
  fht_window(); // window the data for better frequency response
  fht_reorder(); // reorder the data before doing the fht
  fht_run(); // process the data in the fht
  fht_mag_log();  //process amplitude fht in dB
}

void show_SpectrumFFT(int mode){
  
  static bool key_fft = 1; //Se inicializa una unica vez
  if(key_fft == 1){
    init_SpectrumFFT(mode);
    key_fft = 0;
  }
  MeasureFHT();

 
  if(mode == 1)
  { 
    //De 0 a 1900 Hz
    for(int i = 2; i<=127; i++){ //126 bins
      int separacion = 2;
      tft.drawLine(0 + offsetX, 10+separacion*i + offsetY, 200 + offsetX , 10+separacion*i+offsetY, BLACK); //Se borra el anterior valor
      randomNumber = fht_log_out[i]-90; //Filtro digital
      if(randomNumber <= 0){randomNumber = 0;} 
      tft.drawLine(0 + offsetX, 10+separacion*i + offsetY, randomNumber + offsetX , 10 + separacion*i + offsetY, WHITE);  //Se muestra el nuevo valor
      }
  }

  if(mode == 2)
  {
     //De 0 a 4453 Hz //Musica
    for(int i = 2; i<=29; i++){ //28 bins
      int separacion = 9;
      tft.drawLine(0 + offsetX, 10+separacion*i + offsetY, 200 + offsetX , 10+separacion*i+offsetY, BLACK); //Se borra el anterior valor
      randomNumber = fht_log_out[i]-90; //Filtro digital
      if(randomNumber <= 0){randomNumber = 0;} 
      tft.drawLine(0 + offsetX, 10+separacion*i + offsetY, randomNumber + offsetX , 10 + separacion*i + offsetY, WHITE);  //Se muestra el nuevo valor
      }
  }
  
  
}
  

