 //  SABSUNG Company 

#include <avr/io.h>
#include <avr/interrupt.h>


#define SinDiv (300)// Sub divisions of sisusoidal wave.
#define HIGH_THRESHOLD  1.4
#define LOW_THRESHOLD   1.3

// These constants won't change
static int microMHz = 16; // Micro clock frequency
static int freq     = 50; // Sinusoidal frequency
static long int period;   // Period of PWM in clock cycles.


// variables
//static long int TimerTop;   // Period of PWM in clock cycles -1.
volatile unsigned int StartCycle20ms=0;
volatile int PWM_Power;


static unsigned int LUH[SinDiv];
static unsigned int LUL[SinDiv];



const int analogAC_FBInPin = A0; // Analog input pin 23 PC0
const int analogBatteryInPin = A2; // Analog input pin 25 PC2

int   FBV = 0;        // value read from the pot
float Vo = 0.0;
float VBat = 0.01;
float ei = 0.0;

void setup()
{
   //Serial.begin(9600);
   float aoH = 0.96;
   float aoL = 0.30;
   double temp; 
   period = microMHz*1e6/freq/SinDiv;// Period of PWM in clock cycles    TimerTop=period-1
   // TimerTop=period-1;
 
   // Generating the look up table of A leg.
   for(int i = 0; i < SinDiv; i++)
   { 
     //temp = 0.5*aoH*(4000-2)*sin(i*2*M_PI/SinDiv)+2000;
     temp = 0.5*aoH*(period-2)*sin(i*2*M_PI/SinDiv)+0.5*period;
     LUH[i] = (unsigned int)(temp+0.5);       // Round to integer. 
     temp = 0.5*aoL*(period-2)*sin(i*2*M_PI/SinDiv)+0.5*period;
     //temp = 0.5*aoL*(4000-2)*sin(i*2*M_PI/SinDiv)+2000;
     LUL[i] = (unsigned int)(temp+0.5);
   }

   // Initialising the Hysteris automaton
   FBV= analogRead (analogAC_FBInPin);
   float Vprevious = FBV * (5.0 / 1023.0);
   delay(1);
   FBV= analogRead (analogAC_FBInPin);
   Vo = FBV * (5.0 / 1023.0);
   if (Vo <= LOW_THRESHOLD)  
   {
     PWM_Power=1;
   }
   else if (Vo >= HIGH_THRESHOLD)
   {
     PWM_Power=0;
   }
   else
   {
     if (Vo >= Vprevious)
     {
       PWM_Power=1;
     }
     else
     {
        PWM_Power=0;
     }
   }
   //DDRB = 0b00000110; // Set PB1 and PB2 as outputs.
   // Register initilisation, see datasheet for more detail.
   TCCR1A = 0b10100010;
   //TCCR1A = 0b10110010;
   TCCR1B = 0b00011001;
   TIMSK1 = 0b00000001;
       
   ICR1   = period;     // Period for 16MHz crystal, it is better to load  ICR1H = 0x03; ICR1l = 0xe7.
   // Set the TIMER1 Counter TOP value on ICR1H and ICR1L
   //ICR1H = (period >> 8 ) & 0x00FF;
   //ICR1L =  period; 
   //ICR1H = highByte(period);
   //ICR1L = lowByte(period); 
  
   sei();             // Enable global interrupts.
   DDRB = 0b00000110; // Set PB1 and PB2 as outputs.
   pinMode(13,OUTPUT);
} // End setup

ISR(TIMER1_OVF_vect){
   static unsigned int num;
   static char trig;
   
   // change duty-cycle every period.
   if (PWM_Power==1)
   {
     OCR1A = LUH[num];
     OCR1B = LUH[SinDiv-1-num];
     /* OCR1AH=(LUH[num] >> 8 ) & 0x00FF;
     OCR1AL=LUH[num];
     OCR1BH=(LUH[SinDiv-1-num] >> 8 ) & 0x00FF;
     OCR1BL=LUH[SinDiv-1-num];*/
      
     //OCR1AH=highByte(LUH[num]);
    // OCR1AL=lowByte(LUH[num]);
     //OCR1BH=highByte(LUH[SinDiv-1-num]);
     //OCR1BL=lowByte(LUH[SinDiv-1-num]);
   }
   else //if (PWM_Power==0) 
   {
     OCR1A = LUL[num];
     OCR1B = LUL[SinDiv-1-num];
     /* OCR1AH=(LUL[num] >> 8 ) & 0x00FF;
     OCR1AL=LUL[num];
     OCR1BH=(LUL[SinDiv-1-num] >> 8 ) & 0x00FF;
     OCR1BL=LUL[SinDiv-1-num];*/
     //OCR1AH=highByte(LUL[num]);
     //OCR1AL=lowByte(LUL[num]);
     //OCR1BH=highByte(LUL[SinDiv-1-num]);
     //OCR1BL=lowByte(LUL[SinDiv-1-num]);
   }
   if(++num >= SinDiv)
   {  
     // Pre-increment num then check it's above or equal SinDiv.
      num = 0;       // Reset num.
      StartCycle20ms=1;
      trig = trig^0b00000001;
      digitalWrite(13,trig);
   }
}


void loop()
{ 
  if (StartCycle20ms==1)
  {
     
     FBV= analogRead (analogAC_FBInPin);
     Vo = FBV * (5.0 / 1023.0);
     //ei = Vref-Vo;
     //Serial.println(ei);   
     //int intGain = (int)(CtrlGain*ei/Vref);       
     //byte oldSREG = SREG;   // remember if interrupts are on or off
     //noInterrupts ();
     StartCycle20ms=0;
     if ( (PWM_Power == 1) && (Vo > HIGH_THRESHOLD) ) 
     {
       PWM_Power=0;
     }
     else if ( (PWM_Power == 0) && (Vo < LOW_THRESHOLD) )
     {
       PWM_Power=1;
     }
     //interrupts (); or 
     //SREG = oldSREG;
     //delay(40);        // delay in between reads for stability
     //Serial.print(Vo);
     //Serial.print(" ");
     //Serial.println(PWM_Power);
  }

     
     /* else
     { 
       for(int i = 0; i < SinDiv; i++)
       { 
         LU1[i] = LU1ref[i];      
       }
       for(int i = 0; i < SinDiv; i++)
       { 
        LU2[i] = LU2ref[i];       //   
       }
     }*/
 
 // int VBat= analogRead(analogBatteryInPin);
 // VBat = round( VBat * (5.0 / 1023.0));
}

// End of program End of program End of program





/* Examples
 pin 12 Buzzer  AINO
 pin 28 SCL Micro is working


 if VBat <10.7 then buzzer
 if VBat <10.5 Stop Inverting an Alarm that
 if VBat >14.6 Stop Charging an Alarm that !!!!
 

  uint16_t value = analogRead(ANALOG_INPUT);
*/
