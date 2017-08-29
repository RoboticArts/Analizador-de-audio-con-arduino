
#include <Adafruit_GFX.h>    // Libreria de graficos
#include <Adafruit_TFTLCD.h> // Libreria de LCD 


/************************** CONFIGURACÓN DE LA PANTALLA TFT *****************/
#define YP A1  // Pin analogico A1 para ADC
#define XM A2  // Pin analogico A2 para ADC
#define YM 7 
#define XP 6 


//Es RGB 565: http://www.barth-dev.de/online/rgb565-color-picker/
#define BLACK   ~0x0000 
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
#define LOG_OUT 1 // Se usa la amplitud en dB
#define FHT_N 256 // Configura la FHT en 256 bins
#include <FHT.h> // Se incluye la librera (debe estar definda después de LOG_OUT y FHT_N

// Consante
#define AmpMax (1024 / 2)
#define MicSamples (1024*2) 



// Se limpian los registros del ADC
// http://yaab-arduino.blogspot.co.il/2015/02/fast-sampling-from-analog-input.html
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))



/********************** VARIABLES DEL CÓDIGO ******************/

    int button_A = 12;
    int button_B = 11;

    int offsetX = 50;
    int offsetY = 10;
    int FFT_Number = 0;
  
    //Para la aguja del tacómetro
    double circleX = 160;
    double circleY = 100;
    double radio = 120;

    int modos = 0;
    int last_modos = 3;
    bool key_1 = 0;
    bool key_2 = 0;

void setup() {
  
  tft.begin(0x9341); // Iniciamos el LCD especificando el controlador ILI9341. 
  tft.fillScreen(BLACK); // Pintamos el fondo de negro

  pinMode(button_A, INPUT_PULLUP);
  pinMode(button_B, INPUT_PULLUP);
   
  Serial.begin(9600);

  init_fast_ADC();


}

void loop() {

 
    //show_SoundMeter(3); 
    // MODOS
    // 1 -> Amplitud en %
    // 2 -> RMS en %
    // 3 -> dB

    //show_SpectrumFFT(1);
    //MODOS
    // 1 -> Modo ancho de banda
    // 2 -> Modo música

   
    bool next = 1 - digitalRead(button_A); //Como la entrada es PULL UP se invierte la lectura
    bool prev = 1 - digitalRead(button_B);
    

    if(next == 1 && key_1 == 0)
    {
      if(modos < 4)
      {
        modos++;       
      }
      key_1 = 1;
    }
    if(next == 0)
    {
      key_1 = 0;
    }


    if(prev == 1 && key_2 == 0)
    {
      if(modos > 0)
      {
        modos--;
      }
      key_2 = 1;
    }
    if(prev == 0)
    {
      key_2 = 0;
    }


    //Inicio
    if(modos != last_modos)
    {
      switch(modos)
      {
      case 0: tft.fillScreen(BLACK); init_SoundMeter(160,100,120);  break;
      case 1: tft.fillScreen(BLACK); init_SoundMeter(160,100,120);  break;
      case 2: tft.fillScreen(BLACK); init_SoundMeter(160,100,120);  break;
      case 3: tft.fillScreen(BLACK); init_SpectrumFFT(1); break;
      case 4: tft.fillScreen(BLACK); init_SpectrumFFT(2); break;         
      }  
    }
    last_modos = modos;

    
    //Ejecucion
    switch(modos)
    {
      case 0: show_SoundMeter(1);  break;
      case 1: show_SoundMeter(2);  break;
      case 2: show_SoundMeter(3);  break;
      case 3: show_SpectrumFFT(1); break;
      case 4: show_SpectrumFFT(2); break;
    }
    
} 


float MeasureVolume(int mode)
{
  float soundVolAvg = 0, soundVolMax = 0, soundVolRMS = 0;

  for (int i = 0; i < MicSamples; i++)
  {
    
    while (!(ADCSRA & _BV(ADIF))); // Espera a que el ADC esté listo (ADIF)
    sbi(ADCSRA, ADIF); // Se resetea el ADC
    byte m = ADCL; 
    byte j = ADCH;
    int k = ((int)j << 8) | m; // Se convierte en int


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
  soundVolRMS = 10 * soundVolRMSflt / 7; // Calculo del RMS aproximado


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
  
  ADCSRA = 0xE5; //Prescaler de 32  // 11100101
  ADMUX = 0x5; // ADC 5 sin configurar Vref // 00000101
  DIDR0 = 0x20; // Se activa el ahorro de energia para el ADC 5
  analogReference(EXTERNAL); // 3.3V de AREF
  // http://yaab-arduino.blogspot.co.il/2015/02/fast-sampling-from-analog-input.html
  // http://maxembedded.com/2011/06/the-adc-of-the-avr/
  //https://sites.google.com/site/qeewiki/books/avr-guide/analog-input
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

    float medida = MeasureVolume(mode); // 1: Amplitud Max 2: RMS 3: Decibelios
    static float ultima_medida = 0;
    static int last_mode = 0;

    tft.setTextSize(2);
    tft.setTextColor(WHITE);
    tft.setCursor(circleX - 35 , 240 - circleY + 50);  
    tft.fillRect(circleX - 35 , 240 - circleY + 50, 80, 35, BLACK);                      
    tft.println(medida);
  
    if(last_mode != mode)
    {
      tft.fillRect(circleX, circleY, 50,25, BLACK); //Borra el texto
      tft.setTextSize(2); 
      tft.setTextColor(WHITE);
      tft.setCursor(circleX,circleY);
     
        switch(mode)
        {          
          case 1: tft.println("Amp %"); break;
          case 2: tft.println("RMS %"); break;
          case 3: tft.println("dB"); break;  
        }           
        
    }
    if(mode != 3)
      last_mode = mode;
    else
      last_mode = 12; //Se le da un numero diferente de 1,2 y 3 
      
    //Tacometer
    float MIN_t =  (((2*PI)/32)*20)*100; // Min
    float MAX_t = -(((2*PI)/32)*4)*100; // Max
    
    medida = medida*100;
        
    if(mode == 3)
    {
      
      //Signal in dB
      float MIN_dB = -4000; //Modificar para calibrar el sonometro
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

    while (!(ADCSRA & /*0x10*/_BV(ADIF))); // Espera a que esté preparado el ADC (ADIF)
    sbi(ADCSRA, ADIF); // Resetea el ADC
    byte m = ADCL; // Se obtienen los valores
    byte j = ADCH;
    int k = ((int)j << 8) | m; 

    k -= 0x0200; 
    k <<= 6; // Se convierte a int con signo
   
    fht_input[i] = k; // La lectura se almacena en el vector FHT
    
  }
  fht_window(); // Se aplica una ventana
  fht_reorder(); // Se reordena los datos
  fht_run(); // Se procesa la fht
  fht_mag_log();  //Convierte la amplitud en dB
}

void show_SpectrumFFT(int mode){

  MeasureFHT();

 
  if(mode == 1)
  { 
    //De 0 a 1900 Hz
    for(int i = 2; i<=127; i++){ //126 bins
      int separacion = 2;
      tft.drawLine(0 + offsetX, 10+separacion*i + offsetY, 200 + offsetX , 10+separacion*i+offsetY, BLACK); //Se borra el anterior valor
      FFT_Number = fht_log_out[i]-90; //Filtro digital
      if(FFT_Number <= 0){FFT_Number = 0;} 
      tft.drawLine(0 + offsetX, 10+separacion*i + offsetY, FFT_Number + offsetX , 10 + separacion*i + offsetY, WHITE);  //Se muestra el nuevo valor
      }
  }

  if(mode == 2)
  {
     //De 0 a 4453 Hz //Musica
    for(int i = 2; i<=29; i++){ //28 bins
      int separacion = 9;
      tft.drawLine(0 + offsetX, 10+separacion*i + offsetY, 200 + offsetX , 10+separacion*i+offsetY, BLACK); //Se borra el anterior valor
      FFT_Number = fht_log_out[i]-90; //Filtro digital
      if(FFT_Number <= 0){FFT_Number = 0;} 
      tft.drawLine(0 + offsetX, 10+separacion*i + offsetY, FFT_Number + offsetX , 10 + separacion*i + offsetY, WHITE);  //Se muestra el nuevo valor
      }
  }
  
  
}

