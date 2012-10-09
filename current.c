//***************************************************************************

// Im Hauptprogramm einfügen:
 
// current
//volatile uint16_t                   currentcount0=0;
volatile static uint32_t             currentcount=0;
volatile uint16_t                      impulscount=0;


volatile static uint32_t            impulszeit=0;
volatile static float               impulszeitsumme=0;
volatile static float               impulsmittelwert=0;


volatile uint8_t                    currentstatus=0;
volatile uint8_t                    webstatus =0;
volatile uint8_t                    anzimpulse =0;
volatile uint8_t                    anzwertefuermittelwert =4;


// Endwert fuer Compare-Match von Timer2
#define TIMER2_ENDWERT					125; // 10 us

#define IMPULSBIT                   4 // gesetzt wenn interrupt. Nach Auswertung im Hauptprogramm wieder zurueckgesetzt

#define ANZAHLWERTE                 4

#define SPI_BUFSIZE							32

#define OSZIPORT		PORTC
#define OSZIPORTDDR	DDRC
#define OSZIPORTPIN	PINC
#define PULS			1

#define OSZILO OSZIPORT &= ~(1<<PULS)
#define OSZIHI OSZIPORT |= (1<<PULS)
#define OSZITOGG OSZIPORT ^= (1<<PULS)

#define CURRENTSEND                 0     // Bit fuer: Daten an Server senden
#define CURRENTSTOP                 1     // Bit fuer: Impulse ignorieren
#define CURRENTWAIT                 2     // Bit fuer: Impulse wieder bearbeiten

#define	IMPULSPIN                  0    // Pin fuer Anzeige Impuls


//volatile uint8_t timer2startwert=TIMER2_ENDWERT;

// SPI

//**************************************************************************
#include <avr/io.h>
#include "lcd.h"





/*
 TIMSK2=(1<<OCIE2A); // compare match on OCR2A
 TCNT2=0;  // init counter
 OCR2A=244; // value to compare against
 TCCR2A=(1<<WGM21); // do not change any output pin, clear at compare match

 */

// Timer2 fuer Atmega328
void timer2(void) 
{ 
   //lcd_gotoxy(10,1);
	//lcd_puts("Tim2 ini\0");
   PRR&=~(1<<PRTIM2); // write power reduction register to zero
  
   TIMSK2 |= (1<<OCIE2A);                 // CTC Interrupt En

   TCCR2A |= (1<<WGM21);                  // Toggle OC2A
    
   TCCR2A |= (1<<COM2A0);                  // CTC
   
   /*
    CS22	CS21	CS20	Description
    0    0     0     No clock source
    0    0     1     clk/1
    0    1     0     clk/8
    0    1     1     clk/32
    1    0     0     clk/64
    1    0     1     clk/128
    1    1     0     clk/256
    1    1     1     clk/1024
    */

   //TCCR2B |= (1<<CS22); // 
   //TCCR2B |= (1<<CS21);//							
	TCCR2B |= (1<<CS20);


   
	TIFR2 |= (1<<TOV2);							//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	
   //TIMSK2 |= (1<<TOIE2);						//Overflow Interrupt aktivieren
   TCNT2 = 0;                             //Rücksetzen des Timers
	//OSZILO;
   OCR2A = TIMER2_ENDWERT;

    
}





// ISR fuer Atmega328
/*
ISR (TIMER2_OVF_vect)
{	
	//OSZITOGG ;
//	TCCR2A=0;
	//lcd_clr_line(1);
   currentcount0 ++;
   if (currentcount0 == 0x0200)
   {
      lcd_gotoxy(0,1);
      lcd_puts("Tim2OVF \0");
      lcd_putint(currentcount);

      currentcount++;
      currentcount0=0;
   }

}
*/

ISR(TIMER2_COMPA_vect) // CTC Timer2
{
   //OSZITOGG;
   // Anzahl Zaehlimpulse increment
   
   currentcount++;

}



// Interrupt Routine Slave Mode (interrupt controlled)
// Aufgerufen bei fallender Flanke an INT0

ISR( INT0_vect )
{
   //lcd_gotoxy(10,1);
	//lcd_puts("I0:\0");
   //lcd_putint(impulscount);
   
   /*
   if (webstatus & (1<<CURRENTSTOP)) // Webevent im Gang, Impulse ignorieren > Nutzlos
   {
      lcd_gotoxy(10,1);
      lcd_puts("I0:sp\0");
      return;
   }
   */
   
   // Zaehlerstand abnehmen
   impulszeit = currentcount;
   // Zaehler reset
   currentcount =0;
   OSZILO;
   
   if (webstatus & (1<<CURRENTWAIT)) // Webevent fertig, neue Serie starten
   {
      //lcd_gotoxy(10,1);
      //lcd_puts("I0:wt\0");
      anzimpulse=0;
      
      webstatus &= ~(1<<CURRENTWAIT);
      
      TCCR2B |= (1<<CS20); // Timer wieder starten,
      
      //     return;   //Impuls ist Startimpuls, nicht auswerten > auskomm 121009.
   }
   
   impulscount++; // fortlaufende Add
   currentstatus |= (1<< IMPULSBIT); // Impuls bearbeiten in WebServer. Bit wird dort reset.
   
   
   
   
   
   
}	// ISR



void InitCurrent(void) 
{ 
      
	// interrupt on INT0 pin falling edge (sensor triggered) 
	EICRA = (1<<ISC01) | (0<<ISC00);
	// turn on interrupts!
	EIMSK  |= (1<<INT0);


	//lcd_gotoxy(0,0);
	//lcd_puts("C0 Ini\0");
   
	sei(); // Enable global interrupts
   
} 


