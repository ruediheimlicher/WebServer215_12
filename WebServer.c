/*********************************************
 * vim:sw=8:ts=8:si:et
 * To use the above modeline in vim you must have "set modeline" in your .vimrc
 * Author: Guido Socher
 * Copyright: GPL V2
 * See http://www.gnu.org/licenses/gpl.html
 *
 * Chip type           : Atmega88 or Atmega168 or Atmega328 with ENC28J60
 * Note: there is a version number in the text. Search for tuxgraphics
 *********************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <avr/pgmspace.h>
#include "ip_arp_udp_tcp.c"
#include "websrv_help_functions.c"
#include "enc28j60.h"
//#include "timeout.h"
#include "net.h"

#include <avr/wdt.h>
#include "lcd.c"
#include "adc.c"
//#include "websr.c"
#include "current.c"
#include "web_SPI.c"

//#include "out_slave.c"
#include "datum.c"
#include "version.c"
#include "homedata.c"
//# include "twimaster.c"
//***********************************
//									*
//									*
//***********************************
//
//									*
//***********************************


#define SDAPIN		4
#define SCLPIN		5

/*
 #define TASTE1		38
 #define TASTE2		46
 #define TASTE3		54
 #define TASTE4		72
 #define TASTE5		95
 #define TASTE6		115
 #define TASTE7		155
 #define TASTE8		186
 #define TASTE9		205
 #define TASTEL		225
 #define TASTE0		235
 #define TASTER		245
 */

#define	LOOPLEDPORT		PORTB
#define	LOOPLEDPORTPIN	DDRB
#define	LOOPLED			1
#define	TWIPIN			0



#define STARTDELAYBIT		0
#define HICOUNTBIT			1
#define CLIENTBIT			3
#define WDTBIT				7
#define TASTATURPIN			0           //	Eingang fuer Tastatur
#define THERMOMETERPIN		1           //	Eingang fuer Thermometer
#define RELAISPIN          5           //	Ausgang fuer Reset-Relais

#define MASTERCONTROLPIN	4           // Eingang fuer MasterControl: Meldung MasterReset

#define INT0PIN            2           // Pin Int0
#define INT1PIN            3           // Pin Int1

volatile uint16_t LoopCounter=0;

volatile uint8_t rxdata =0;
volatile uint16_t EventCounter=0;
static char baseurl[]="http://ruediheimlicher.dyndns.org/";



/* *************************************************************************  */
/* Eigene Deklarationen                                                       */
/* *************************************************************************  */
#define NULLTASK					0xB0	// Nichts tun
#define ERRTASK					0xA0	// F

#define STATUSTASK				0xB1	// Status des TWI aendern
#define STATUSCONFIRMTASK		0xB2	// Statusaenderung des TWI bestaetigen
#define EEPROMREADTASK			0xB8	// von EEPROM lesen
#define EEPROMSENDTASK			0xB9	// Daten vom HomeServer an HomeCentral senden
#define EEPROMRECEIVETASK		0xB6	// Adresse fuer EEPROM-Write empfangen
#define EEPROMWRITETASK			0xB7	// auf EEPROM schreiben
#define EEPROMCONFIRMTASK		0xB5	// Quittung an HomeCentral senden
#define EEPROMREPORTTASK		0xB4	// Daten vom EEPROM an HomeServer senden

#define EEPROMREADWOCHEATASK	0xBA
#define EEPROMREADWOCHEBTASK	0xBB

#define RESETTASK					0xBF	// HomeCentral reseten

#define DATATASK					0xC0	// Normale Loop im Webserver
#define CURRENTTASK					0xC1	// Daten von solar

#define MASTERERRTASK			0xC7	// Fehlermeldung vom Master senden


#define STATUSTIMEOUT			0x0080
//
// Eventuell kritische Werte
#define START_BYTE_DELAY		2				// Timerwert fuer Start-Byte
#define BYTE_DELAY				2				// Timerwert fuer Data-Byte

volatile uint16_t					timer2_counter=0;

//enum webtaskflag{IDLE, TWISTATUS,EEPROMREAD, EEPROMWRITE};

static uint8_t  webtaskflag =0;

static uint8_t monitoredhost[4] = {10,0,0,7};

//#define STR_BUFFER_SIZE 24
//static char strbuf_A[STR_BUFFER_SIZE+1];


#define TAGPLANBREITE		0x40	// 64 Bytes, 2 page im EEPROM
#define RAUMPLANBREITE		0x200	// 512 Bytes
#define twi_buffer_size		8
#define buffer_size			8 
#define page_size				32
#define eeprom_buffer_size 8


volatile uint8_t	TWI_Pause=1;


volatile uint8_t StartDaten;


static volatile uint8_t Temperatur;
/*Der Sendebuffer, der vom Master ausgelesen werden kann.*/
//volatile uint8_t txbuffer[twi_buffer_size];
volatile uint8_t txstartbuffer;

static char HeizungDataString[64];
static char SolarDataString[64];
static char CurrentDataString[64];
//static char EEPROM_String[96];

//static  char d[4]={};
//static char* key1;
//static char *sstr;

//char HeizungVarString[64];

//static char AlarmDataString[64];

//static char ErrDataString[32];


volatile uint8_t oldErrCounter=0;



static volatile uint8_t stepcounter=0;

// Prototypes
void lcdinit(void);
void r_itoa16(int16_t zahl, char* string);
void tempbis99(uint16_t temperatur,char*tempbuffer);


// the password string (only the first 5 char checked), (only a-z,0-9,_ characters):
static char password[10]="ideur00"; // must not be longer than 9 char
static char resetpassword[10]="ideur!00!"; // must not be longer than 9 char


uint8_t TastenStatus=0;
uint16_t Tastencount=0;
uint16_t Tastenprellen=0x01F;


static volatile uint8_t pingnow=1; // 1 means time has run out send a ping now
static volatile uint8_t resetnow=0; 
static volatile uint8_t reinitmac=0; 
//static uint8_t sendping=1; // 1 we send ping (and receive ping), 0 we receive ping only
static volatile uint8_t pingtimer=1; // > 0 means wd running
//static uint8_t pinginterval=30; // after how many seconds to send or receive a ping (value range: 2 - 250)
static char *errmsg; // error text

unsigned char TWI_Transceiver_Busy( void );
//static volatile uint8_t twibuffer[twi_buffer_size+1]; // Buffer fuer Data aus/fuer EEPROM
static volatile char twiadresse[4]; // EEPROM-Adresse
//static volatile uint8_t hbyte[4];
//static volatile uint8_t lbyte[4];
extern volatile uint8_t twiflag; 
static uint8_t aktuelleDatenbreite=8;
static volatile uint8_t send_cmd=0;


#define TIMERIMPULSDAUER            10    // us

#define CURRENTSEND                 0     // Daten an server senden

volatile uint16_t                   wattstunden=0;
volatile uint16_t                   kilowattstunden=0;

volatile float leistung =0;
float lastleistung =0;
uint8_t lastcounter=0;

// Wert fuer Anzeige und pwm-timer 0
volatile uint8_t  anzeigewert =0;

char stromstring[10];

volatile uint8_t  paketcounter =0;


void timer0(void);
uint8_t WochentagLesen(unsigned char ADRESSE, uint8_t hByte, uint8_t lByte, uint8_t *Daten);
uint8_t SlavedatenLesen(const unsigned char ADRESSE, uint8_t *Daten);
void lcd_put_tempAbMinus20(uint16_t temperatur);

/* ************************************************************************ */
/* Ende Eigene Deklarationen																 */
/* ************************************************************************ */


// Note: This software implements a web server and a web browser.
// The web server is at "myip" and the browser tries to access "websrvip".
//
// Please modify the following lines. mac and ip have to be unique
// in your local area network. You can not have the same numbers in
// two devices:
//static uint8_t mymac[6] = {0x54,0x55,0x58,0x10,0x00,0x29};

//RH4702 52 48 34 37 30 33
static uint8_t mymac[6] = {0x52,0x48,0x34,0x37,0x30,0x35};

// how did I get the mac addr? Translate the first 3 numbers into ascii is: TUX
// This web server's own IP.
//static uint8_t myip[4] = {10,0,0,29};
//static uint8_t myip[4] = {192,168,255,100};

// IP des Webservers 
static uint8_t myip[4] = {192,168,1,215};

// IP address of the web server to contact (IP of the first portion of the URL):
//static uint8_t websrvip[4] = {77,37,2,152};


// ruediheimlicher
static uint8_t websrvip[4] = {193,17,85,42}; // ruediheimlicher 193.17.85.42

// The name of the virtual host which you want to contact at websrvip (hostname of the first portion of the URL):


#define WEBSERVER_VHOST "www.ruediheimlicher.ch"


// Default gateway. The ip address of your DSL router. It can be set to the same as
// websrvip the case where there is no default GW to access the 
// web server (=web server is on the same lan as this host) 

// ************************************************
// IP der Basisstation !!!!!
// Runde Basisstation : 
//static uint8_t gwip[4] = {192,168,1,5};// Rueti

// Viereckige Basisstation:
static uint8_t gwip[4] = {192,168,1,1};// Rueti

// ************************************************

static char urlvarstr[21];
// listen port for tcp/www:
#define MYWWWPORT 81 
//

#define BUFFER_SIZE 800


static uint8_t buf[BUFFER_SIZE+1];
static uint8_t pingsrcip[4];
static uint8_t start_web_client=0;
static uint8_t web_client_attempts=0;
static uint8_t web_client_sendok=0;
static volatile uint8_t sec=0;
static volatile uint8_t cnt2step=0;



#define tag_start_adresse 0

#define lab_data_size 8


#define CURRENTSEND                 0     // Bit fuer: Daten an Server senden
#define CURRENTSTOP                 1     // Bit fuer: Impulse ignorieren
#define CURRENTWAIT                 2     // Bit fuer: Impulse wieder bearbeiten
#define CURRENTMESSUNG              3     // Bit fuer: Messen
#define DATASEND                    4     // Bit fuer: Daten enden
#define DATAOK                      5

void str_cpy(char *ziel,char *quelle)
{
	uint8_t lz=strlen(ziel); //startpos fuer cat
	//printf("Quelle: %s Ziellaenge: %d\n",quelle,lz);
	uint8_t lq=strlen(quelle);
	//printf("Quelle: %s Quelllaenge: %d\n",quelle,lq);
	uint8_t i;
	for(i=0;i<lq;i++)
	{
		//printf("i: %d quelle[i]: %c\n",i,quelle[i]);
		ziel[i]=quelle[i];
	}
	lz=strlen(ziel);
	ziel[lz]='\0';
}

void str_cat(char *ziel,char *quelle)
{
	uint8_t lz=strlen(ziel); //startpos fuer cat
	//printf("Quelle: %s Ziellaenge: %d\n",quelle,lz);
	uint8_t lq=strlen(quelle);
	//printf("Quelle: %s Quelllaenge: %d\n",quelle,lq);
	uint8_t i;
	for(i=0;i<lq;i++)
	{
		//printf("i: %d quelle[i]: %c\n",i,quelle[i]);
		ziel[lz+i]=quelle[i];
		
	}
	//printf("ziel: %s\n",ziel);
	lz=strlen(ziel);
	ziel[lz]='\0';
}

// http://stackoverflow.com/questions/122616/how-do-i-trim-leading-trailing-whitespace-in-a-standard-way
char *trimwhitespace(char *str)
{
   char *end;
   
   // Trim leading space
   while(isspace(*str)) str++;
   
   if(*str == 0)  // All spaces?
      return str;
   
   // Trim trailing space
   end = str + strlen(str) - 1;
   while(end > str && isspace(*end)) end--;
   
   // Write new null terminator
   *(end+1) = 0;
   
   return str;
}


void timer0()
{
	//----------------------------------------------------
	// Set up timer 0 
	//----------------------------------------------------
   /*
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS00) | _BV(CS02);
	OCR0A = 0x2;
	TIMSK0 = _BV(OCIE0A);
    */
   
   DDRD |= (1<< PORTD6);   // OC0A Output

   TCCR0A |= (1<<WGM00);   // fast PWM  top = 0xff
   TCCR0A |= (1<<WGM01);   // PWM
   //TCCR0A |= (1<<WGM02);   // PWM
   
   TCCR0A |= (1<<COM0A1);   // set OC0A at bottom, clear OC0A on compare match
   TCCR0B |= 1<<CS02;
   TCCR0B |= 1<<CS00;
   
   OCR0A=10;
   TIMSK0 |= (1<<OCIE0A);
   
   
}

ISR(TIMER0_COMPA_vect)
{
   
  //OCR0A = anzeigewert;
   //OCR0A++;
    
}

uint16_t http200ok(void)
{
	return(fill_tcp_data_p(buf,0,PSTR("HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nPragma: no-cache\r\n\r\n")));
}

// we were ping-ed by somebody, store the ip of the ping sender
// and trigger an upload to http://tuxgraphics.org/cgi-bin/upld
// This is just for testing and demonstration purpose
void ping_callback(uint8_t *ip)
{
    uint8_t i=0;
    // trigger only first time in case we get many ping in a row:
    if (start_web_client==0)
    {
        start_web_client=1;
        //			lcd_gotoxy(12,0);
        //			lcd_puts("ping\0");
        // save IP from where the ping came:
        while(i<4)
        {
            pingsrcip[i]=ip[i];
            i++;
        }
        
    }
}

/*
void strom_browserresult_callback(uint8_t statuscode,uint16_t datapos)
{
   // datapos is not used in this example
   if (statuscode==0)
   {
      
     // lcd_gotoxy(0,1);
     // lcd_puts("        \0");
    //  lcd_gotoxy(0,1);
     // lcd_puts("s cb OK\0");
      
      lcd_gotoxy(19,0);
      lcd_putc(' ');
      lcd_gotoxy(19,0);
      lcd_putc('+');
      // Messungen wieder starten
      webstatus &= ~(1<<CURRENTSTOP);
      webstatus |= (1<<CURRENTWAIT); // Beim naechsten Impuls Messungen wieder starten
      //webstatus |= (1<< CURRENTMESSUNG);
      //webstatus &= ~(1<<DATASEND);
      sei();
      
      //lcd_gotoxy(13,1);
      //lcd_puts("  \0");

      
      web_client_sendok++;
      //				sei();
      
   }
   else
   {
      lcd_gotoxy(0,1);
      lcd_puts("        \0");
      lcd_gotoxy(0,1);
      lcd_puts("s cb err\0");
      lcd_puthex(statuscode);
      
   }
}
*/
void strom_browserresult_callback(uint8_t statuscode,uint16_t datapos) // von 101_2
{
   // datapos is not used in this example
   if (statuscode==0)
   {
      /*
       lcd_gotoxy(12,0);
       lcd_puts("        \0");
       lcd_gotoxy(12,0);
       lcd_puts("s cb OK\0");
       */
      lcd_gotoxy(19,0);
      lcd_putc(' ');
      lcd_gotoxy(19,0);
      lcd_putc('+');
      
      
      // Messungen wieder starten
      webstatus &= ~(1<<CURRENTSTOP);
      webstatus |= (1<<CURRENTWAIT); // Beim naechsten Impuls Messungen wieder starten
      //sei();
      
      webstatus |= (1<< CURRENTMESSUNG);
      webstatus &= ~(1<<DATAOK);
      webstatus &= ~(1<<DATASEND);

      web_client_sendok++;
      //				sei();
      
   }
   else
   {
      /*
       lcd_gotoxy(0,1);
       lcd_puts("        \0");
       lcd_gotoxy(0,1);
       lcd_puts("s cb err\0");
       lcd_puthex(statuscode);
       */
      lcd_gotoxy(19,0);
      lcd_putc(' ');
      lcd_gotoxy(19,0);
      lcd_putc('-');
      
   }
}



void home_browserresult_callback(uint8_t statuscode,uint16_t datapos)
{
    // datapos is not used in this example
    if (statuscode==0)
    {
        
        lcd_gotoxy(0,0);
        lcd_puts("        \0");
        lcd_gotoxy(0,0);
        lcd_puts("h cb OK\0");
        
        web_client_sendok++;
        //				sei();
        
    }
    else
    {
        lcd_gotoxy(0,0);
        lcd_puts("        \0");
        lcd_gotoxy(0,0);
        lcd_puts("h cb err\0");
        lcd_puthex(statuscode);
        
    }
}



void alarm_browserresult_callback(uint8_t statuscode,uint16_t datapos)
{
    // datapos is not used in this example
    if (statuscode==0)
    {
        
        lcd_gotoxy(0,0);
        lcd_puts("        \0");
        lcd_gotoxy(0,0);
        lcd_puts("a cb OK\0");
        
        web_client_sendok++;
        //				sei();
        
    }
    else
    {
        lcd_gotoxy(0,0);
        lcd_puts("         \0");
        lcd_gotoxy(0,0);
        lcd_puts("a cb err\0");
        lcd_puthex(statuscode);
        
    }
}

/* ************************************************************************ */
/* Eigene Funktionen														*/
/* ************************************************************************ */

uint8_t verify_password(char *str)
{
	// the first characters of the received string are
	// a simple password/cookie:
	if (strncmp(password,str,7)==0)
   {
		return(1);                 // PW OK
	}
	return(0);                    //PW falsch
}


uint8_t verify_reset_password(char *str)
{
	// the first characters of the received string are
	// a simple password/cookie:
	if (strncmp(resetpassword,str,7)==0)
   {
		return(1); // Reset-PW OK
	}
	return(0); //Reset-PW falsch
}


void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
}

uint8_t Hex2Int(char *s) 
{ 
    long res; 
    char *Chars = "0123456789ABCDEF", *p; 
    
    if (strlen(s) > 8) 
    /* Error ... */ ; 
    
    for (res = 0L; *s; s++) { 
        if ((p = strchr(Chars, toupper(*s))) == NULL) 
        /* Error ... */ ; 
        res = (res << 4) + (p-Chars); 
    } 
    
    return res; 
} 

void tempbis99(uint16_t temperatur,char*tempbuffer)
{
	char buffer[8]={};
	//uint16_t temp=(temperatur-127)*5;
	uint16_t temp=temperatur*5;
	
	//itoa(temp, buffer,10);
	
	r_itoa16(temp,buffer);
	
	//lcd_puts(buffer);
	//lcd_putc('*');
	
	//char outstring[7]={};
	
	tempbuffer[6]='\0';
	tempbuffer[5]=' ';
	tempbuffer[4]=buffer[6];
	tempbuffer[3]='.';
	tempbuffer[2]=buffer[5];
	if (abs(temp)<100)
	{
		tempbuffer[1]=' ';
		
	}
	else 
	{
		tempbuffer[1]=buffer[4];
		
	}		
	tempbuffer[0]=buffer[0];
	
	
}

void tempAbMinus20(uint16_t temperatur,char*tempbuffer)
{
    
    char buffer[8]={};
    int16_t temp=(temperatur)*5;
    temp -=200;
    char Vorzeichen=' ';
    if (temp < 0)
    {
        Vorzeichen='-';
    }
    
    r_itoa16(temp,buffer);
    //		lcd_puts(buffer);
    //		lcd_putc(' * ');
    
    //		char outstring[7]={};
    
    tempbuffer[6]='\0';
    //outstring[5]=0xDF; // Grad-Zeichen
    tempbuffer[5]=' ';
    tempbuffer[4]=buffer[6];
    tempbuffer[3]='.';
    tempbuffer[2]=buffer[5];
    if (abs(temp)<100)
    {
		tempbuffer[1]=Vorzeichen;
		tempbuffer[0]=' ';
    }
    else
    {
		tempbuffer[1]=buffer[4];
		tempbuffer[0]=Vorzeichen;
    }
    //		lcd_puts(outstring);
}


// search for a string of the form key=value in
// a string that looks like q?xyz=abc&uvw=defgh HTTP/1.1\r\n
//
// The returned value is stored in the global var strbuf_A

// Andere Version in Webserver help funktions
/*
 uint8_t find_key_val(char *str,char *key)
 {
 uint8_t found=0;
 uint8_t i=0;
 char *kp;
 kp=key;
 while(*str &&  *str!=' ' && found==0){
 if (*str == *kp)
 {
 kp++;
 if (*kp == '\0')
 {
 str++;
 kp=key;
 if (*str == '=')
 {
 found=1;
 }
 }
 }else
 {
 kp=key;
 }
 str++;
 }
 if (found==1){
 // copy the value to a buffer and terminate it with '\0'
 while(*str &&  *str!=' ' && *str!='&' && i<STR_BUFFER_SIZE)
 {
 strbuf_A[i]=*str;
 i++;
 str++;
 }
 strbuf_A[i]='\0';
 }
 // return the length of the value
 return(i);
 }
 */


#pragma mark analye_get_url

// takes a string of the form ack?pw=xxx&rst=1 and analyse it
// return values:  0 error
//                 1 resetpage and password OK
//                 4 stop wd
//                 5 start wd
//                 2 /mod page
//                 3 /now page 
//                 6 /cnf page 

uint8_t analyse_get_url(char *str)	// codesnippet von Watchdog
{
	char actionbuf[32];
	errmsg="inv.pw";
	
    
	webtaskflag =0; //webtaskflag zuruecksetzen. webtaskflag bestimmt aktion, die ausgefuehrt werden soll. Wird an Master weitergegeben.
	// ack 
	
	// str: ../ack?pw=ideur00&rst=1
	if (strncmp("ack",str,3)==0)
	{
		lcd_clr_line(1);
		lcd_gotoxy(0,1);
		lcd_puts("ack\0");
        
		// Wert des Passwortes eruieren
		if (find_key_val(str,actionbuf,10,"pw"))
		{
			urldecode(actionbuf);
			
			// Reset-PW?
			if (verify_reset_password(actionbuf))
			{
				return 15;
			}
            
			// Passwort kontrollieren
			if (verify_password(actionbuf))
			{
				if (find_key_val(str,actionbuf,10,"tst"))
				{
					return(1);
				}
			}
		}
        
		return(0);
		
	}//ack
	
	if (strncmp("twi",str,3)==0)										//	Daten von HC beginnen mit "twi"	
	{
		//lcd_clr_line(1);
		//lcd_gotoxy(17,0);
		//lcd_puts("twi\0");
		
		// Wert des Passwortes eruieren
		if (find_key_val(str,actionbuf,10,"pw"))					//	Passwort kommt an zweiter Stelle
		{		
			urldecode(actionbuf);
			webtaskflag=0;
			//lcd_puts(actionbuf);
			// Passwort kontrollieren
			
			
			if (verify_password(actionbuf))							// Passwort ist OK
			{
				//OSZILO;
				if (find_key_val(str,actionbuf,10,"status"))		// Status soll umgeschaltet werden
				{
					
					webtaskflag =STATUSTASK;							// Task setzen
                    
					out_startdaten=STATUSTASK; // B1
                    
					//lcd_gotoxy(6,0);
					//lcd_puts(actionbuf);
					outbuffer[0]=atoi(actionbuf);
					out_lbdaten=0x00;
					out_lbdaten=0x00;
                    
					if (actionbuf[0]=='0') // twi aus
					{
                        
						//WebTxDaten[1]='0';
						outbuffer[1]=0;
						out_hbdaten=0x00;
						out_lbdaten=0x00;
						return (2);
					}
					if (actionbuf[0]=='1') // twi ein
					{
                        
						//WebTxDaten[1]='0';
						outbuffer[1]=1;
						out_hbdaten=0x01;
						out_lbdaten=0x00;
						return (3);				// Status da, sendet Bestaetigung an Homeserver
					}
                    
				}//st
				
				
				
				
				
				// Daten fuer EEPROM von Homeserver empfangen
				
				if (find_key_val(str,actionbuf,10,"wadr"))			// EEPROM-Daten werden von Homeserver gesendet
				{
					//webspistatus |= (1<< TWI_WAIT_BIT);				// Daten nicht an HomeCentral senden
                    
					// Nur erste Stelle der EEPROM-Adresse, default 0xA0 wird im Master zugefuegt
					//WebTxDaten[0]=atoi(actionbuf);
					outbuffer[0]=atoi(actionbuf);
					
					//				lcd_gotoxy(17,1);
					//				lcd_puthex(txbuffer[0]);
					//				lcd_gotoxy(17,2);
					//				lcd_puts(actionbuf);
					
					uint8_t dataOK=0;
					webtaskflag = EEPROMRECEIVETASK; // B6
                    
					//out_startdaten=EEPROMRECEIVETASK;
					
					aktuelleDatenbreite = buffer_size;
					
					if (find_key_val(str,actionbuf,10,"hbyte"))		//hbyte der Adresse
					{
						dataOK ++;
						//strcpy((char*)hbyte,actionbuf);
						//outbuffer[1]=atoi(actionbuf);
						out_hbdaten=atoi(actionbuf);
					}
					
					if (find_key_val(str,actionbuf,10,"lbyte"))		// lbyte der Adresse
					{
						dataOK ++;
						//strcpy((char*)lbyte,actionbuf);
						//outbuffer[2]=atoi(actionbuf);
						out_lbdaten=atoi(actionbuf);
					}
					
					if (find_key_val(str,actionbuf,28,"data"))		// Datenstring mit '+' - Trennzeichen
					{
						//lcd_gotoxy(0,0);
						//lcd_puthex(strlen(actionbuf));
						//lcd_putc(' ');
						//lcd_puts(actionbuf);
						//lcd_puts("    \0");
						//lcd_gotoxy(14,1);
						//lcd_putc('A');
						
						webtaskflag = EEPROMWRITETASK;					// Task setzen
						//EEPROMTxStartDaten=webtaskflag;
						
						out_startdaten=EEPROMWRITETASK;
						
						// Test
						/*
                         EEPROMTxDaten[1]=1;
                         EEPROMTxDaten[2]=2;
                         EEPROMTxDaten[3]=3;
                         */
						//lcd_putc('B');
						
						dataOK ++;
						char* buffer= malloc(32);
						//lcd_putc('C');
						
						strcpy(buffer, actionbuf);
						
						//lcd_putc('D');
						
						uint8_t index=0;
						char* linePtr = malloc(32);
						
						linePtr = strtok(buffer,"+");
						
						while (linePtr !=NULL)								// Datenstring: Bei '+' trennen
						{
							//EEPROMTxDaten[index++] = strtol(linePtr,NULL,16); //http://www.mkssoftware.com/docs/man3/strtol.3.asp
							outbuffer[index++] = strtol(linePtr,NULL,16); //http://www.mkssoftware.com/docs/man3/strtol.3.asp
							linePtr = strtok(NULL,"+");
						}
						free(linePtr);
						free(buffer);
					} // if data
					
                    //				if (dataOK==2) // alle Daten da
					{
						
						return (9);												// Empfang bestätigen
					}
				} // wadr
				
				if (find_key_val(str,actionbuf,10,"iswriteok"))		// Anfrage ob writeok
				{
                    
					return (7);
				}
                
				if (find_key_val(str,actionbuf,12,"isstat0ok"))		// Anfrage ob statusok isstat0ok
				{
					lcd_gotoxy(7,0);
					lcd_putc('*');
					return (10);
				}
				
                if (find_key_val(str,actionbuf,10,"reset")) // HomeCentral reseten
                {
                    
                }	
				
			}//verify pw
		}//find_key pw
		return(0);
	}//twi
    
	
    
	errmsg="inv.url";
	return(0);
}





uint16_t print_webpage_ok(uint8_t *buf,uint8_t *okcode)
{
	// Schickt den okcode als Bestaetigung fuer den Empfang des Requests
	uint16_t plen;
	plen=fill_tcp_data_p(buf,0,PSTR("HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nPragma: no-cache\r\n\r\n"));
	plen=fill_tcp_data_p(buf,plen,PSTR("<p>okcode="));
	plen=fill_tcp_data(buf,plen,(void*)okcode);	
	return plen;
}



uint16_t print_webpage_confirm(uint8_t *buf)
{
	uint16_t plen;
	plen=fill_tcp_data_p(buf,0,PSTR("HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nPragma: no-cache\r\n\r\n"));
	plen=fill_tcp_data_p(buf,plen,PSTR("<h2>Bearbeiten</h2><p>Passwort OK.</p>\n"));
	
	//
	/*
     plen=fill_tcp_data_p(buf,plen,PSTR("<form action=/ram method=get>"));
     plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=radio name=gto value=0 checked> Heizung <br></p>\n"));
     plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=radio name=gto value=1> Werkstatt<br></p>\n"));
     plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=radio name=gto value=3> Buero<br></p>\n"));
     plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=radio name=gto value=4> Labor<br></p><input type=submit value=\"Gehen\"></form>\n"));
     */
	
	plen=fill_tcp_data_p(buf,plen,PSTR("<form action=/cde method=get>"));										// Code fuer Tagbalken eingeben
	plen=fill_tcp_data_p(buf,plen,PSTR("<p>Raum: <input type=text name=raum size=2><br>"));				// Raum
	plen=fill_tcp_data_p(buf,plen,PSTR("Wochentag: <input type=text name=wochentag  size=2><br>"));		// Wochentag
	plen=fill_tcp_data_p(buf,plen,PSTR("Objekt: <input type=text name=objekt  size=2><br>"));				// Objekt
	plen=fill_tcp_data_p(buf,plen,PSTR("Stunde: <input type=text name=stunde  size=2><br>"));				// Stunde
	plen=fill_tcp_data_p(buf,plen,PSTR("<input type=submit value=\"Lesen\"></p>"));
	
	/*
     plen=fill_tcp_data_p(buf,plen,PSTR("<p><input type=radio name=std value=0 checked> OFF <br>"));
     plen=fill_tcp_data_p(buf,plen,PSTR("<input type=radio name=std value=2> erste halbe Stunde<br>"));
     plen=fill_tcp_data_p(buf,plen,PSTR("<input type=radio name=std value=1> zweite halbe Stunde<br>"));
     plen=fill_tcp_data_p(buf,plen,PSTR("<input type=radio name=std value=3> ganze Stunde<br></p>"));
     */	
    
	plen=fill_tcp_data_p(buf,plen,PSTR("Stunde: <input type=text name=code  size=2><br>"));				// code
    
	plen=fill_tcp_data_p(buf,plen,PSTR("<input type=submit value=\"Setzen\"></form>"));
    
	
	
	
	plen=fill_tcp_data_p(buf,plen,PSTR("<form action=/twi method=get>"));
	plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=hidden name=pw value=\"ideur00\"></p>"));
	if (PIND & (1<<TWIPIN)) // TWI ist ON
	{
		plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=radio name=status value=0> OFF<br></p>\n")); // st: Statusflag
		plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=radio name=status value=1 checked> ON <br></p>\n"));
		
	}
	else
	{
		plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=radio name=status value=0 checked> OFF<br></p>\n"));
		plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=radio name=status value=1> ON <br></p>\n"));
		
	}
	//	plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=hidden name=pw value=\"ideur00\"><input type=radio name=st value=0> OFF<br></p>\n"));
	//	plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=radio name=st value=1 checked> ON <br></p>\n"));
	plen=fill_tcp_data_p(buf,plen,PSTR("<p>\n<input type=submit value=\"OK\"></form>\n"));
	
	//
	plen=fill_tcp_data_p(buf,plen,PSTR("<a href=\"/\">&lt;&lt;zur&uuml;ck zu Status</a></p>\n"));
	
	
	return(plen);
}

#pragma mark Webpage_status

// prepare the webpage by writing the data to the tcp send buffer
uint16_t print_webpage_status(uint8_t *buf)
{
	
	uint16_t plen=0;
	//char vstr[5];
	plen=http200ok();
	
	plen=fill_tcp_data_p(buf,plen,PSTR("<h1>HomeCurrent</h1>"));
	//
	
	//
	plen=fill_tcp_data_p(buf,plen,PSTR("<p>  HomeCurrent<br>  Falkenstrasse 20<br>  8630 Rueti"));
	plen=fill_tcp_data_p(buf,plen,PSTR("<hr><h4><font color=\"#00FF00\">Status</h4></font></p>"));
	
	
	//return(plen);
	
	
	plen=fill_tcp_data_p(buf,plen,PSTR("<p>Leistung: "));
	Temperatur=0;
	
	//Temperatur=WebRxDaten[2];
	//Temperatur=inbuffer[2]; // Vorlauf
	//Temperatur=Vorlauf;
	//tempbis99(Temperatur,TemperaturString);
	
	//r_itoa(Temperatur,TemperaturStringV);
	
   
   plen=fill_tcp_data(buf,plen,stromstring);
   plen=fill_tcp_data_p(buf,plen,PSTR(" Watt</p>"));

	   
   return(plen);
   
	// Taste und Eingabe fuer Passwort
	plen=fill_tcp_data_p(buf,plen,PSTR("<form action=/ack method=get>"));
	plen=fill_tcp_data_p(buf,plen,PSTR("<p>\nPasswort: <input type=password size=10 name=pw ><input type=hidden name=tst value=1>  <input type=submit value=\"Bearbeiten\"></p></form>"));
	
	plen=fill_tcp_data_p(buf,plen,PSTR("<p><hr>"));
	plen=fill_tcp_data(buf,plen,DATUM);
	plen=fill_tcp_data_p(buf,plen,PSTR("  Ruedi Heimlicher"));
	plen=fill_tcp_data_p(buf,plen,PSTR("<br>Version :"));
	plen=fill_tcp_data(buf,plen,VERSION);
	plen=fill_tcp_data_p(buf,plen,PSTR("\n<hr></p>"));
	
	//
	
	/*
	 // Tux
	 plen=fill_tcp_data_p(buf,plen,PSTR("<h2>web client status</h2>\n<pre>\n"));
	 
	 char teststring[24];
	 strcpy(teststring,"Data");
	 strcat(teststring,"-\0");
	 strcat(teststring,"Uploads \0");
	 strcat(teststring,"mit \0");
	 strcat(teststring,"ping : \0");
	 plen=fill_tcp_data(buf,plen,teststring);
	 
	 plen=fill_tcp_data_p(buf,plen,PSTR("Data-Uploads mit ping: "));
	 // convert number to string:
	 itoa(web_client_attempts,vstr,10);
	 plen=fill_tcp_data(buf,plen,vstr);
	 plen=fill_tcp_data_p(buf,plen,PSTR("\nData-Uploads aufs Web: "));
	 // convert number to string:
	 itoa(web_client_sendok,vstr,10);
	 plen=fill_tcp_data(buf,plen,vstr);
	 plen=fill_tcp_data_p(buf,plen,PSTR("\n</pre><br><hr>"));
	 */
	return(plen);
}

void master_init(void)
{
	
	DDRB |= (1<<PORTB1);	//Bit 1 von PORT B als Ausgang für Kontroll-LED
	PORTB |= (1<<PORTB1);	//Pull-up
	DDRB |= (1<<PORTB0);	//Bit 0 von PORT B als Ausgang für Kontroll-LED
	PORTB |= (1<<PORTB0);	//Pull-up
	
	DDRD |=(1<<RELAISPIN); //Pin 5 von Port D als Ausgang fuer Reset-Relais
	PORTD |=(1<<RELAISPIN); //HI
	// Eventuell: PORTD5 verwenden, Relais auf Platine 
	
   /*
   if (INTERRUPTQUELLE)
   {
      DDRD &=~(1<<INT1PIN); //Pin 3 von Port D als Eingang fuer Interrupt Impuls
      PORTD |=(1<<INT1PIN); //HI

   }
   else
   {
      DDRD &=~(1<<INT0PIN); //Pin 2 von Port D als Eingang fuer Interrupt Impuls
      PORTD |=(1<<INT0PIN); //HI
   }
    */
	DDRD &= ~(1<<MASTERCONTROLPIN); // Pin 4 von PORT D als Eingang fuer MasterControl
	PORTD |= (1<<MASTERCONTROLPIN);	// HI
	
	pendenzstatus=0;
   
   //DDRB |= (1<<PORTB3); // OC2A
	
}	

void initOSZI(void)
{
	OSZIPORTDDR |= (1<<PULS);
	OSZIPORT |= (1<<PULS); // HI
}

void lcdinit()
{
	//*********************************************************************
	//	Definitionen LCD im Hauptprogramm
	//	Definitionen in lcd.h
	//*********************************************************************
	
	LCD_DDR |= (1<<LCD_RSDS_PIN); //PIN 3 von PORT D als Ausgang fuer LCD_RSDS_PIN
	LCD_DDR |= (1<<LCD_ENABLE_PIN); //PIN 4 von PORT D als Ausgang fuer LCD_ENABLE_PIN
	LCD_DDR |= (1<<LCD_CLOCK_PIN); //PIN 5 von PORT D als Ausgang fuer LCD_CLOCK_PIN
	/* initialize the LCD */
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
	lcd_puts("LCD Init\0");
	delay_ms(300);
	lcd_cls();
	
}

void WDT_off(void)
{
    cli();
    wdt_reset();
    /* Clear WDRF in MCUSR */
    MCUSR &= ~(1<<WDRF);
    /* Write logical one to WDCE and WDE */
    /* Keep old prescaler setting to prevent unintentional time-out
     */
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    /* Turn off WDT */
    WDTCSR = 0x00;
    sei();
}

uint8_t i=0;



/* ************************************************************************ */
/* Ende Eigene Funktionen														*/
/* ************************************************************************ */

#pragma mark main
int main(void)
{
	
	/* ************************************************************************ */
	/* Eigene Main														*/
	/* ************************************************************************ */
	//JTAG deaktivieren (datasheet 231)
    //	MCUCSR |=(1<<7);
    //	MCUCSR |=(1<<7);
    
	MCUSR = 0;
	wdt_disable();
	Temperatur=0;
	//SLAVE
	//uint16_t Tastenprellen=0x0fff;
	uint16_t loopcount0=0;
	uint16_t loopcount1=0;
    
	//uint16_t plen;
	uint8_t i=0;
	int8_t cmd;
	
	
	// set the clock speed to "no pre-scaler" (8MHz with internal osc or 
	// full external speed)
	// set the clock prescaler. First write CLKPCE to enable setting of clock the
	// next four instructions.
	CLKPR=(1<<CLKPCE); // change enable
	CLKPR=0; // "no pre-scaler"
	delay_ms(1);
	
	
	i=1;
	//	WDT_off();
	//init the ethernet/ip layer:
    //	init_ip_arp_udp_tcp(mymac,myip,MYWWWPORT);
	// timer_init();
	
	//     sei(); // interrupt enable
	master_init();
	
	lcdinit();
	lcd_puts("Guten Tag \0");
	lcd_gotoxy(11,0);
	lcd_puts("V:\0");
	//lcd_puts(VERSION);
   char versionnummer[7];
   strcpy(versionnummer,&VERSION[13]);
   versionnummer[6] = '\0';
   lcd_puts(versionnummer);
   //lcd_putc(VERSION[13]);
   
    
	delay_ms(1600);
	//lcd_cls();
	
	TWBR =0;
    
		
	txstartbuffer = 0x00;
	uint8_t sendWebCount=0;	// Zahler fuer Anzahl TWI-Events, nach denen Daten des Clients gesendet werden sollen
	webspistatus=0;
	
	//Init_SPI_Master();
	//initOSZI();
	/* ************************************************************************ */
	/* Ende Eigene Main														*/
	/* ************************************************************************ */
		
	uint16_t dat_p;
	char str[30]; 
	
	// set the clock speed to "no pre-scaler" (8MHz with internal osc or 
	// full external speed)
	// set the clock prescaler. First write CLKPCE to enable setting of clock the
	// next four instructions.
	CLKPR=(1<<CLKPCE); // change enable
	CLKPR=0; // "no pre-scaler"
	_delay_loop_1(0); // 60us
	
	/*initialize enc28j60*/
	enc28j60Init(mymac);
	enc28j60clkout(2); // change clkout from 6.25MHz to 12.5MHz
	_delay_loop_1(0); // 60us
	
	sei();
	
	/* Magjack leds configuration, see enc28j60 datasheet, page 11 */
	// LEDB=yellow LEDA=green
	//
	// 0x476 is PHLCON LEDA=links status, LEDB=receive/transmit
	// enc28j60PhyWrite(PHLCON,0b0000 0100 0111 01 10);
	enc28j60PhyWrite(PHLCON,0x476);
	
	//DDRB|= (1<<DDB1); // LED, enable PB1, LED as output
	//PORTD &=~(1<<PD0);;
	
	//init the web server ethernet/ip layer:
	init_ip_arp_udp_tcp(mymac,myip,MYWWWPORT);

	// init the web client:
	client_set_gwip(gwip);  // e.g internal IP of dsl router
	client_set_wwwip(websrvip);
	register_ping_rec_callback(&ping_callback);
	
    //		SPI
    for (i=0;i<out_BUFSIZE;i++)
	{
        outbuffer[i]='-';
	}
    
    //end SPI
    
    
	timer0();
  
    //	
    lcd_clr_line(1);
   lcd_gotoxy(0,0);
   lcd_puts("         \0");
   //OSZIHI;
   
   InitCurrent();
   timer2();
   //webstatus |= ( 1<<CURRENTMESSUNG);
   webstatus |= ( 1<<DATASEND);
   webstatus |= ( 1<<DATAOK);
   uint8_t messungcounter = 0;
   uint8_t paketcounter = 0;
   uint16_t sendcounter = 0;
   
#pragma  mark "while"
	while(1)
	{
		sei();
		//Blinkanzeige
      
		loopcount0++;
		if (loopcount0>=0x2FFF)
		{
			loopcount0=0;
			// *** SPI senden
			//waitspi();
			//StartTransfer(loopcount1,1);
			//sendcounter++;
			if (loopcount1 >= 0xFFFE)
			{
				
				loopcount1 = 0;
				//OSZITOGG;
				//LOOPLEDPORT |= (1<<TWILED);           // TWILED setzen, Warnung
				//TWBR=0;
				//lcdinit();
			}
			else
			{
				loopcount1++;
				
			}
			if (LOOPLEDPORTPIN &(1<<LOOPLED))
			{
				sec++;
			}
			LOOPLEDPORT ^=(1<<LOOPLED);
			
		}
		
		
		
		//**	Ende Start-Routinen	***********************
		
		
		//**	Beginn Current-Routinen	***********************
      if (webstatus & ( 1<<CURRENTMESSUNG))
		{
         if (currentstatus & (1<<IMPULSBIT)) // neuer Impuls angekommen
         {
            messungcounter++;
            currentstatus++; // ein Wert mehr gemessen
            impulszeitsumme += impulszeit/ANZAHLWERTE;     // Wert aufsummieren
            
            if ((currentstatus & 0x0F) == ANZAHLWERTE)   // genuegend Werte
            {
               paketcounter++;
               lcd_gotoxy(16,1);
               lcd_puts("    \0");

//              lcd_gotoxy(0,1);
//              lcd_puts("          \0");
               //delay_ms(10);
               //lcd_gotoxy(14,0);
               //lcd_putint((currentstatus & 0x0C));
               //lcd_putc('*');
               impulsmittelwert = impulszeitsumme;
               impulszeitsumme = 0;
               
               //currentstatus = 0x00;
               
               //uint8_t lb= impulsmittelwert & 0xFF;
               //uint8_t hb = (impulsmittelwert>>8) & 0xFF;
               
               //                lcd_gotoxy(0,1);
               //               lcd_putc('I');
               //lcd_puts("INT0 \0");
               
               //lcd_puthex(hb);
               //lcd_puthex(lb);
               //                lcd_putc(':');
               //char impstring[12];
               //dtostrf(impulsmittelwert,10,2,impstring);
               //lcd_gotoxy(0,0);
               //lcd_puts(impstring);
               //lcd_putc(':');
               //lcd_putint16(impulsmittelwert);
               
               /*
                Impulsdauer: impulsmittelwert * TIMERIMPULSDAUER (10us)
                Umrechnung auf ms: /1000
                Energie pro Zählerimpuls: 360 Ws
                Leistung: (Energie pro Zählerimpuls)/Impulsabstand
                Umrechnung auf Sekunden: *1000
                Faktor: *100000
                */
               
               // leistung = 0xFFFF/impulsmittelwert;
               
               leistung = 360.0/impulsmittelwert*100000.0;
               
               // leistung = 36000000.0/impulsmittelwert;
               // Stromzaehler
               
               //lcd_gotoxy(8,0);
               //lcd_putint16(leistung);
               //lcd_putc('*');
               
               wattstunden = impulscount/10;
               dtostrf(leistung,5,0,stromstring);
               char* newstromstring = trimwhitespace(stromstring);
               
               lcd_gotoxy(0,1);
               lcd_puts("L:\0");
               lcd_puts(newstromstring);
               lcd_putc('W');
               lcd_putc(' ');
               lcd_putint(messungcounter);
               lcd_putc(' ');
               lcd_putint(paketcounter);
               currentstatus &= 0xF0;
               /*
                lcd_gotoxy(10,1);
                lcd_putint(wattstunden/1000);
                lcd_putc('.');
                lcd_putint3(wattstunden);
                lcd_putc('W');
                lcd_putc('h');
                //lcd_putc(':');
                */
               
               //lcd_putc('*');
               //lcd_puts(stromstring);
               //lcd_putc('*');
               //lcd_putc(' ');
               //lcd_putint16(leistung);
               //lcd_putc(' ');
               /*
                if (abs(leistung-lastleistung) > 10)
                {
                lastcounter++;
                
                if (lastcounter>3)
                {
                char diff[10];
                dtostrf(leistung-lastleistung,7,2,diff);
                lcd_gotoxy(10,1);
                lcd_putc('D');
                lcd_putc(':');
                lcd_puts(diff);
                lastleistung = leistung;
                }
                }
                else
                {
                lastcounter=0;
                }
                */
               
               //anzeigewert = 0xFF/0x8000*leistung; // 0x8000/0x255 = 0x81
               anzeigewert = leistung/0x20;
               //lcd_putint(anzeigewert);
               //lcd_putc('*');
               
               OCR0A = anzeigewert;
                
               //lcd_putint(anzeigewert);
               
               // webstatus |= (1<<CURRENTSEND);
               
               webstatus |= (1<<CURRENTSTOP); // Impulse blockieren
               
               // sendstring vorbereiten
               
               char key1[]="pw=";
               char sstr[]="Pong";
               
               
               strcpy(CurrentDataString,key1);
               strcat(CurrentDataString,sstr);
               
               strcat(CurrentDataString,"&strom=");
               char webstromstring[10]={};
               urlencode(newstromstring,webstromstring);
               //strcat(CurrentDataString,stromstring);
               strcat(CurrentDataString,webstromstring);
               
               /*
                char d[5]={};
                //char dd[4]={};
                strcat(CurrentDataString,"&c0=");
                itoa(hb++,d,16);
                strcat(CurrentDataString,d);
                
                strcat(CurrentDataString,"&c1=");
                itoa(lb++,d,16);
                strcat(CurrentDataString,d);
                */
               
               
               //lcd_gotoxy(0,1);
               //lcd_puts(CurrentDataString);
               webstatus &= ~(1<<CURRENTSEND);
               
               // Zaehler fuer senden incr
               sendWebCount++;
 //              lcd_gotoxy(9,1);
  //             lcd_putint2(sendWebCount);
               
            	//if (sendWebCount == 4)
               if (paketcounter && paketcounter % 8 ==0)
               {
                  //start_web_client=1;
                  sendWebCount=0;
                  //webstatus &= ~(1<< CURRENTMESSUNG);
                  //webstatus |= (1<< DATASEND);
                  //webstatus |= (1<< DATAOK);
                  
                  //
                  
                  //
               }

            
            } // genuegend Werte
            else
            {
               //lcd_gotoxy(8,1);
               //lcd_puts("    \0");
               
            }
            
            impulszeit=0;
            currentstatus &= ~(1<<IMPULSBIT);
            
         } // end IMPULSBIT
         
      }
		//**    End Current-Routinen*************************

      // strom busy?
      
		if (webstatus & (1<< DATASEND))
      {
#pragma mark packetloop
			
			// handle ping and wait for a tcp packet
			
         
         // **	Beginn Ethernet-Routinen	***********************
         
         
         cli();
			sendcounter++;
         if (sendcounter >= 0x0400)
         {
            sendcounter =0;
         }
			dat_p=packetloop_icmp_tcp(buf,enc28j60PacketReceive(BUFFER_SIZE, buf));
			//dat_p=1;
			
			if(dat_p==0) // Kein Aufruf, eigene Daten senden an Homeserver
			{
            lcd_gotoxy(10,1);
            //lcd_puts(" TCP \0");
            //lcd_puthex(start_web_client);
            lcd_putint16(sendcounter);
            
            if ((start_web_client==1)) // In Ping_Calback gesetzt: Ping erhalten
            {
               //OSZILO;
               sec=0;
               //lcd_gotoxy(0,0);
               //lcd_puts("    \0");
               lcd_gotoxy(12,0);
               lcd_puts("ping ok\0");
               lcd_clr_line(1);
               delay_ms(100);
               start_web_client=2; // nur erstes ping beantworten. start_web_client wird in pl-send auf 0 gesetzt
               web_client_attempts++;
               
               mk_net_str(str,pingsrcip,4,'.',10);
               char* pingsstr="ideur01\0";
               
               urlencode(pingsstr,urlvarstr);
               //lcd_gotoxy(0,1);
               //lcd_puts(urlvarstr);
               //delay_ms(1000);
               
            }
            
            if (webstatus & (1<<DATAOK) && sendcounter == 1) // StromDaten an HomeServer schicken
            {
               lcd_gotoxy(0,0);
              // lcd_puts(CurrentDataString);
               //lcd_puts(WEBSERVER_VHOST);
               lcd_putc('*');
               lcd_putint16(sendcounter);
               
               start_web_client=2;
               web_client_attempts++;
               char* teststring = "pw=Pong&strom=366\0";
               //strcat(SolarVarString,SolarDataString);
               start_web_client=0;
               
               // Daten an strom.pl schicken
               //client_browse_url(PSTR("/cgi-bin/strom.pl?"),CurrentDataString,PSTR(WEBSERVER_VHOST),&strom_browserresult_callback);
               client_browse_url(PSTR("/cgi-bin/strom.pl?"),teststring,PSTR(WEBSERVER_VHOST),&strom_browserresult_callback);
               
 
              
               // webstatus &= ~(1<< DATASEND);
               //webstatus |= (1<< CURRENTMESSUNG);

               //sendWebCount++;
               
               // Daten senden
               //www_server_reply(buf,dat_p); // send data
               
               //if (sendWebCount>4)
               {
               //webstatus &= ~(1<<DATAOK);
               
               }
               
           //   webstatus |= (1<< CURRENTMESSUNG);
               //webstatus &= ~(1<<DATAOK);
           // webstatus &= ~(1<<DATASEND);
             
            }

            //continue;
         } // dat_p=0
			
         sei();
         
			/*
			 if (strncmp("GET ",(char *)&(buf[dat_p]),4)!=0)
			 {
			 // head, post and other methods:
			 //
			 // for possible status codes see:
			 
			 // http://www.w3.org/Protocols/rfc2616/rfc2616-sec10.html
			 lcd_gotoxy(0,0);
			 lcd_puts("*GET*\0");
			 dat_p=http200ok();
			 dat_p=fill_tcp_data_p(buf,dat_p,PSTR("<h1>HomeCentral 200 OK</h1>"));
			 goto SENDTCP;
			 }
			 */
			
			
			if (strncmp("/ ",(char *)&(buf[dat_p+4]),2)==0) // Slash am Ende der URL, Status-Seite senden
			{
				lcd_gotoxy(12,1);
				lcd_puts("+/+\0");
				dat_p=http200ok(); // Header setzen
				dat_p=fill_tcp_data_p(buf,0,PSTR("HTTP/1.0 200 OK\r\nContent-Type: text/html\r\n\r\n<h1>200 OK</h1>"));
            dat_p=fill_tcp_data_p(buf,dat_p,PSTR("<h1>HomeCurrent 200 OK</h1>"));
				dat_p=print_webpage_status(buf);
				goto SENDTCP;
			}
			else
			{
				// Teil der URL mit Form xyz?uv=... analysieren
				
#pragma mark cmd
				
				//out_startdaten=DATATASK;	// default
				
				// out_daten setzen
				cmd=analyse_get_url((char *)&(buf[dat_p+5]));
				
				//lcd_gotoxy(5,0);
				//lcd_puts("cmd:\0");
				//lcd_putint(cmd);
				//lcd_putc(' ');
				if (cmd == 1)
				{
					dat_p = print_webpage_confirm(buf);
				}
				else if (cmd == 2)	// TWI > OFF
				{
#pragma mark cmd 2
				}
				
				else
				{
					dat_p=fill_tcp_data_p(buf,0,PSTR("HTTP/1.0 401 Unauthorized\r\nContent-Type: text/html\r\n\r\n<h1>401 Zugriff verweigert</h1>"));
				}
				cmd=0;
				// Eingangsdaten reseten, sofern nicht ein Status0-Wait im Gang ist:
				//if ((pendenzstatus & (1<<SEND_STATUS0_BIT)))
				{
					
				}
				
				goto SENDTCP;
			}
			//
         
		SENDTCP:
         
         
              OSZIHI;
         //www_server_reply(buf,dat_p); // send data
			
			
		} // strom not busy
	}
	return (0);
}
