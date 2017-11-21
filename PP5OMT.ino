/* +--------------------------------------------+
   | PP5OMT: 80m CW WX Beacon              V3.5 |
   |--------------------------------------------|
   |*A1 ( A1 ): Small Photocell (Day/Night)     |
   | A4 ( A4 ): BME-280 ( SDA )                 |
   | A5 ( A5 ): BME-280 ( SCL )                 |
   |--------------------------------------------|
   | Note: BME280 Sensor Module is only 3.3V !  |
   |--------------------------------------------|
   | 02 ( 02 ): B1 Button                       |
   | 03 ( 03 ): B2 Button                       |
   | 04 ( 04 ): B3 Button                       |
   | 05 ( 05 ): B4 Button                       |
   |*11 ( 11 ): PTT Output (with a 2N7000)      |
   | 12 ( 12 ): Sidetone Out by a Trimpot 10kA  |
   |*13 ( 13 ): Key Output (with a 2N7000)      |
   |--------------------------------------------|
   | *: not implemented here                    
   |--------------------------------------------|
   |                            (c)2017 - PP5VX |
   | Parts: (c)2009 - K6HX                      |
   |        (c)2015 - BA5AG/KC2WAG              |
   +--------------------------------------------+
   |  Additional Library: BME280 (Bosch) here:  | 
   |https://github.com/Seeed-Studio/Grove_BME280|
   +--------------------------------------------+
   | And the "real thing" is here ( enjoy ! ):  |
   | www.qrz.com/db/pp5omt !                    | 
   +--------------------------------------------+
*/
#include "Seeed_BME280.h"
#include <Wire.h>
BME280 bme280;
#define B1        2 // B1 Button
#define B2        3 // B2 Button
#define B3        4 // B3 Button
#define B4        5 // B4 Button
#define PTT_Pin  11 // PTT Output: Pin 11
#define ST_Pin   12 // Sidetone Output: Pin 12
#define key_Pin  13 // Transmitter Key: Pin 13

#define DefaWPM  16 // Default Speed: 18 wpm
#define DefaHZ  500 // Default Tone: 500 HZ
#define DefaRTT  60 // Default Repeat Timer: 60s = 1 minute
//                 123456789012345678901234567890123456789012345678901234567890123456789012345678901234567890  === Maximum: 100 caracters
//                 |        |         |         |         |         |         |         |         |         |
#define _msgDay   "PP5OMT BEACON GG53RS - PWR 50W RMS - ANT INV VEE WITH 120 DEGREES OPEN LEGS AND 18M APEX +" // == Fixed MSG Day   (Long)
#define _msgNight "PP5OMT BEACON GG53RS - WX QTC FOLLOWS = "                                                   // == Fixed MSG Night (Full)
#define _msgError "VVV VVV DE PP5OMT PP5OMT GG53RS 73 +"                                                       // == Fixed MSG Error (Short)
  
float  pressao, temperatura, umidade, altitude;
int    tempo = 0;
byte   key_speed, ditTime;
char   p[8];
String _WXmsg, _BCNmsg;

#define N_MORSE  (sizeof(morsetab)/sizeof(morsetab[0]))  // Tabela !
struct t_mtab { char c, pat; };
// ======================================= Begin of ASCII Code Table by: (c)2009 - K6HX
struct t_mtab morsetab[] =
{ { '+',  42 },  // 00101010b = 2Ah =  42d ( AR )
  { '-',  97 },  // 01100001b = 61h =  97d
  { '=',  49 },  // 00110001b = 31h =  49d
  { '.',  10 },  // "R" as separator or 01101010b = 6Ah = 106d
  { ',', 115 },  // 01110011b = 73h = 115d
  { '?',  76 },  // 01001100b = 4Ch =  76d
  { '/',  41 },  // 00101001b = 29h =  41d
  { 'A',   6 },  // 00000110b = 06h =   6d
  { 'B',  17 },  // 00010001b = 11h =  17d
  { 'C',  21 },  // 00010101b = 15h =  21d
  { 'D',   9 },  // 00001001b = 09h =   9d
  { 'E',   2 },  // 00000010b = 02h =   2d
  { 'F',  20 },  // 00010100b = 14h =  20d
  { 'G',  11 },  // 00001011b = 0Bh =  11d
  { 'H',  16 },  // 00010000b = 10h =  16d
  { 'I',   4 },  // 00000100b = 04h =   4d
  { 'J',  30 },  // 00011110b = 1Eh =  30d
  { 'K',  13 },  // 00001101b = 0Dh =  13d
  { 'L',  18 },  // 00010010b = 12h =  18d
  { 'M',   7 },  // 00000111b = 07h =   7d
  { 'N',   5 },  // 00000101b = 05h =   5d
  { 'O',  15 },  // 00001111b = 0Fh =  15d
  { 'P',  22 },  // 00010110b = 16h =  22d
  { 'Q',  27 },  // 00011011b = 1Bh =  27d
  { 'R',  10 },  // 00001010b = 0Ah =  10d
  { 'S',   8 },  // 00001000b = 08h =   8d
  { 'T',   3 },  // 00000011b = 03h =   3d
  { 'U',  12 },  // 00001100b = 0Ch =  12d
  { 'V',  24 },  // 00011000b = 18h =  24d
  { 'W',  14 },  // 00001110b = 0Eh =  14d
  { 'X',  25 },  // 00011001b = 19h =  25d
  { 'Y',  29 },  // 00011101b = 1Dh =  29d
  { 'Z',  19 },  // 00010011b = 13h =  19d
  { '1',  62 },  // 00111110b = 3Eh =  62d
  { '2',  60 },  // 00111100b = 3Ch =  60d
  { '3',  56 },  // 00111000b = 38h =  56d
  { '4',  48 },  // 00110000b = 30h =  48d
  { '5',  32 },  // 00100000b = 20h =  32d
  { '6',  33 },  // 00100001b = 21h =  33d
  { '7',  35 },  // 00100011b = 23h =  35d
  { '8',  39 },  // 00100111b = 27h =  39d
  { '9',  47 },  // 00101111b = 2Fh =  47d
  { '0',  63 }   // 00111111b = 3Fh =  63d
};               // don't forget this ";" !
// ======================================= End of ASCII Code Table by: (c)2009 - K6HX

void setup() 
{ pinMode(ST_Pin,       OUTPUT);        // Sets the Sidetone Digital Pin as Output
  pinMode(key_Pin,      OUTPUT);        // Sets the Keying Pin as Output
  digitalWrite(key_Pin, LOW);           // KEY OUT OFF ( Disabled )
  pinMode(B1,           INPUT);         // MSG 1: Physical Button
  digitalWrite(B1,      INPUT_PULLUP);  // Pullup for Button 1
  digitalWrite(B1,      HIGH);          // Button S1 tate OFF
  pinMode(B2,           INPUT);         // MSG 2: Physical Button
  digitalWrite(B2,      INPUT_PULLUP);  // Pullup for Button 2
  digitalWrite(B2,      HIGH);          // Button 2 State OFF
  pinMode(B3,           INPUT);         // MSG 3: Physical Button
  digitalWrite(B3,      INPUT_PULLUP);  // Pullup for Button 3
  digitalWrite(B3,      HIGH);          // Button 3 State OFF
  pinMode(B4,           INPUT);         // MSG 4: Physical Button
  digitalWrite(B4,      INPUT_PULLUP);  // Pullup for Button 4
  digitalWrite(B4,      HIGH);          // Button 4 State OFF
  readWX();
  Serial.begin(115200);
  Serial.println(F("WX CW Beacon - wxCWb V3.5"));
  Serial.println(F("========================================================================================================================================"));
  Serial.print(F("[1] - Speed........: ")); Serial.print(DefaWPM); Serial.print(F(" wpm ( Range: 10wpm to 30 wpm - Factory Default: ")); Serial.print(DefaWPM); Serial.println(F(" wpm )"));
  Serial.print(F("[2] - Tone.........: ")); Serial.print(DefaHZ);  Serial.print(F(" HZ ( Range: 400HZ to 700 HZ - Factory Default: "));  Serial.print(DefaHZ);  Serial.println(F(" HZ )"));
  Serial.print(F("[3] - Timer........: ")); Serial.print(DefaRTT); Serial.print(F(" s ( Range: 60s to 1800s- Factory Default: "));       Serial.print(DefaRTT); Serial.println(F(" s )"));
  Serial.print(F("[4] - Message Day..: ")); Serial.println(_msgDay);
  Serial.print(F("[5] - Message Error: ")); Serial.println(_msgError);
  Serial.print(F("[6] - Message Night: ")); Serial.print(_msgNight);  Serial.println(_WXmsg);
  Serial.print(F("                     ")); Serial.print(F("( ")); Serial.print(_BCNmsg); Serial.println(F(" )")); 
  Serial.println(F("[A] - Begin Beacon Full Cycle"));
  Serial.println(F("========================================================================================================================================"));
  Serial.println(F("* The International License (except in Brazil), is by: Creative Commons CC BY-NC-ND 4.0 ( Non Commercial and No Derivatives )"));
  Serial.println(F("* The Brazilian **only** License, is: (c)1980, 2017 - PP5VX ( Todos os Direitos Reservados - All Rights Reserved ), ou seja, seu 'Autor'"));
  Serial.println(F("========================================================================================================================================"));
  Serial.println(F("* NOTA BRASILEIRA: A República Federativa do Brasil **não protege ou reconhece** o dito 'Codigo-Livre' (Open Source) na legislaçao atual"));  
  Serial.println(F("  Baseados nisto, e com o animus de salvaguadar os nossos interesses, esteja muito bem ciente de que este Material e protegido em todo o"));
  Serial.println(F("  Territorio Brasileiro pela: Lei 9.609/98 - A Lei do Software (especialmente pelo Art.2 e Art.12),bem como pela Lei 9.610/98 - A Lei do"));
  Serial.println(F("  Direito Autoral - indepedentemente de qualquer outro tipo de protecao juridica **suposta** ou **alegada**, em possivel caso de litigio"));
  Serial.println(F("  Logo, os eventuais infratores, estarão sujeitos ao **máximo** rigor que a Lei nos permite, de modo a defender nossos interesses."));
  Serial.println(F("========================================================================================================================================"));
  Serial.println(F("> Este software 'gratuito',e de utilizacao apenas a Radioamadores ou Estacoes-Piloto, desde que ( ambos ) sejam *legalmente licenciados*"));
  Serial.println(F("  A sua comercializacao ( direta ou indireta ) fica **EXPRESSAMENTE PROIBIDA**, sem uma anuencia **POR ESCRITO** do Autor, acima citado."));
  Serial.println(F("========================================================================================================================================"));
  Serial.println(F("> QRV !"));
  Serial.end();
  CWSettings();
  sendmsg("QRV");
}

// === Only for BME-280 Sensor
void readWX()
{ if (!bme280.init())
  { Serial.begin(115200);
    Serial.println(F("> Sorry ! BME-280 Sensor, has some BAD error !"));
    Serial.end();
    sendmsg("E E E ?");
  } 
  else
  { pressao     = (bme280.getPressure())/100;
    temperatura = bme280.getTemperature();
    umidade     = bme280.getHumidity();
    altitude    = bme280.calcAltitude(pressao);
    _BCNmsg = "Atmospheric Pressure is ";
    _BCNmsg = _BCNmsg + dtostrf(pressao, 4, 0, p);
    _BCNmsg = _BCNmsg + "HPA - ";
    _WXmsg  = dtostrf(pressao, 4, 0, p);
    _WXmsg  = _WXmsg + " HPA , ";
        
    _BCNmsg = _BCNmsg + "Temperature is ";
    _BCNmsg = _BCNmsg + dtostrf(temperatura, 3, 1, p);
    _BCNmsg = _BCNmsg + "C - ";
    _WXmsg  = _WXmsg  + dtostrf(temperatura, 3, 1, p);
    _WXmsg  = _WXmsg  + " C , ";
    
    _BCNmsg = _BCNmsg + "Humidity is ";
    _BCNmsg = _BCNmsg + dtostrf(umidade, 2, 0, p);
    _BCNmsg = _BCNmsg + "% - ";
    _BCNmsg = _BCNmsg + "Altitude is ";
    _BCNmsg = _BCNmsg + dtostrf(altitude,4, 1, p);
    _BCNmsg = _BCNmsg + "m";
    
    _WXmsg  = _WXmsg  + dtostrf(umidade, 2, 0, p);
    _WXmsg  = _WXmsg  + " +"; }
}  

// === Only for BME-280 Sensor ===========================================
void sendWX()
{ readWX(); dtostrf(pressao, 4, 0, p);     sendmsg(p); sendmsg(" HPA , ");
            dtostrf(temperatura, 3, 1, p); sendmsg(p); sendmsg(" C , ");
            dtostrf(umidade, 2, 0, p);     sendmsg(p); sendmsg(" +"); }

// =====================================================================
void sendDOT()                        // Send unique DOT + Weigth @ TONE
{ tone(ST_Pin,DefaHZ); delay(ditTime); noTone(ST_Pin); delay(ditTime); }

// =====================================================================
void sendDASH()                      // Send unique DASH + Weigth @ TONE
{ tone(ST_Pin,DefaHZ);delay(3*ditTime); noTone(ST_Pin);delay(ditTime); }

// =================================================================
void send(char c)
{ int a1;
  if (c == ' ') { delay(7*ditTime); return; }
  for (a1=0; a1<N_MORSE; a1++)
  { if (morsetab[a1].c == c)
    { unsigned char p = morsetab[a1].pat;
      while (p != 1) { if (p&1) sendDASH(); else sendDOT(); p=p/2; }
      delay(3*ditTime);
      return;
    }
  }
}

// ===================================================================
void sendmsg(char *str)          // or: void sendmsg(const char str[])
{ while (*str) { send(*str++);} }

void CWSettings()
{ ditTime = 1200/DefaWPM; }

// ===============================================================================================================================
void loop()
{ CWSettings();
  Serial.begin(115200);
  Serial.print(F("> Now "));

  // == Message for Day ( Short )
  if ( digitalRead(B1)==LOW )
  { Serial.print(F("Day TX Message: ")); Serial.println(_msgDay); sendmsg(_msgDay);
    Serial.println(F("> Ready !")); }
  else
  // == Message for Night ( Full )
  if ( digitalRead(B2)==LOW )
  { Serial.print(F("Night TX Message: "));
    Serial.print(_msgNight); Serial.println(_WXmsg);
    sendmsg(_msgNight); sendWX();
    Serial.println(F("> Ready !")); }
  else
  // == Message Internal Error
  if ( digitalRead(B3)==LOW )
  { Serial.print(F("Internal Error TX Message: ")); 
    Serial.println(_msgError); sendmsg(_msgError);
    Serial.println(F("> Ready !")); }
  else
  // == Message for Testing
  if ( digitalRead(B4)==LOW )
  { Serial.print(F("Testing ( no TX ) Message: "));
    Serial.println(_BCNmsg); delay(500); }
  else
  { tempo++; Serial.print(tempo); Serial.print(F(". TXing...")); delay(1000*DefaRTT); }
  Serial.end();
}
