/* An arduino uno trip computer for 2005 Toyota Camry (K-line ISO9141 protocol 
using MC33290 ISO K line chip to level shift the 12v logic from the car computer to work with arduino
*/

#undef int
#include <stdio.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <Wire.h>

///////////for 0.96" OLED display ssd1306 clone from ebay
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
 
#define OLED_RESET 0  // GPIO0
Adafruit_SSD1306 lcd(OLED_RESET);

// Memory prototypes
void params_load(void);
void params_save(void);

// Others prototypes
byte menu_select_yes_no(byte p);
void long_to_dec_str(long value, char *decs, byte prec);
int memoryTest(void);
void test_buttons(void);
bool first_attempt = true;


/* PID codes */
unsigned long  pid01to20_support;  // this one always initialized at setup()
unsigned long  pid21to40_support=0;
unsigned long  pid41to60_support=0;
#define PID_SUPPORT00 0x00
#define MIL_CODE      0x01
#define FREEZE_DTC    0x02
#define FUEL_STATUS   0x03
#define LOAD_VALUE    0x04
#define COOLANT_TEMP  0x05
#define STFT_BANK1     0x06
#define LTFT_BANK1     0x07
#define STFT_BANK2     0x08
#define LTFT_BANK2     0x09
#define FUEL_PRESSURE 0x0A
#define MAN_PRESSURE  0x0B
#define ENGINE_RPM    0x0C
#define VEHICLE_SPEED 0x0D
#define TIMING_ADV    0x0E
#define INT_AIR_TEMP  0x0F
#define MAF_AIR_FLOW  0x10
#define THROTTLE_POS  0x11
#define SEC_AIR_STAT  0x12
#define OXY_SENSORS1  0x13
#define B1S1_O2_V     0x14
#define B1S2_O2_V     0x15
#define B1S3_O2_V     0x16
#define B1S4_O2_V     0x17
#define B2S1_O2_V     0x18
#define B2S2_O2_V     0x19
#define B2S3_O2_V     0x1A
#define B2S4_O2_V     0x1B
#define OBD_STD       0x1C
#define OXY_SENSORS2  0x1D
#define AUX_INPUT     0x1E
#define RUNTIME_START 0x1F
#define PID_SUPPORT20 0x20
#define DIST_MIL_ON   0x21
#define FUEL_RAIL_P   0x22
#define FUEL_RAIL_DIESEL 0x23
#define O2S1_WR_V     0x24
#define O2S2_WR_V     0x25
#define O2S3_WR_V     0x26
#define O2S4_WR_V     0x27
#define O2S5_WR_V     0x28
#define O2S6_WR_V     0x29
#define O2S7_WR_V     0x2A
#define O2S8_WR_V     0x2B
#define EGR           0x2C
#define EGR_ERROR     0x2D
#define EVAP_PURGE    0x2E
#define FUEL_LEVEL    0x2F
#define WARM_UPS      0x30
#define DIST_MIL_CLR  0x31
#define EVAP_PRESSURE 0x32
#define BARO_PRESSURE 0x33
#define O2S1_WR_C     0x34
#define O2S2_WR_C     0x35
#define O2S3_WR_C     0x36
#define O2S4_WR_C     0x37
#define O2S5_WR_C     0x38
#define O2S6_WR_C     0x39
#define O2S7_WR_C     0x3A
#define O2S8_WR_C     0x3B
#define CAT_TEMP_B1S1 0x3C
#define CAT_TEMP_B2S1 0x3D
#define CAT_TEMP_B1S2 0x3E
#define CAT_TEMP_B2S2 0x3F
#define PID_SUPPORT40 0x40
#define MONITOR_STAT  0x41
#define CTRL_MOD_V    0x42
#define ABS_LOAD_VAL  0x43
#define CMD_EQUIV_R   0x44
#define REL_THR_POS   0x45
#define AMBIENT_TEMP  0x46
#define ABS_THR_POS_B 0x47
#define ABS_THR_POS_C 0x48
#define ACCEL_PEDAL_D 0x49
#define ACCEL_PEDAL_E 0x4A
#define ACCEL_PEDAL_F 0x4B
#define CMD_THR_ACTU  0x4C
#define TIME_MIL_ON   0x4D
#define TIME_MIL_CLR  0x4E

#define LAST_PID      0x4E  // same as the last one defined above

// returned length of the PID response.
// constants so put in flash
const unsigned char pid_reslen[] PROGMEM=
{
  // pid 0x00 to 0x1F
  4,4,2,2,1,1,1,1,1,1,1,1,2,1,1,1,
  2,1,1,1,2,2,2,2,2,2,2,2,1,1,1,4,

  // pid 0x20 to 0x3F
  4,2,2,2,4,4,4,4,4,4,4,4,1,1,1,1,
  1,2,2,1,4,4,4,4,4,4,4,4,2,2,2,2,

  // pid 0x40 to 0x4E
  4,8,2,2,2,1,1,1,1,1,1,1,1,2,2
};


typedef struct
{
  byte use_metric;     // 0=US units, 1=SI
  byte per_hour_speed; // speed from which we toggle to fuel/hour (km/h or mph)
  byte fuel_adjust;    // because of variation from car to car, temperature, etc
  byte speed_adjust;   // because of variation from car to car, tire size, etc
}
params_t;

// parameters default values
params_t params=
{
  0,
  20,
  100,
  100  
};

#define STRLEN  40


void set_params(void)
{

  params.use_metric=0; //0 means false, 1 means true
  params_save();
}
/*
 * for ISO9141-2 Protocol
 */
#define K_IN    0
#define K_OUT   1

// some globals, for trip calculation and others
unsigned long old_time;
byte has_rpm=0;
long vss=0;  // speed
long maf=0;  // MAF
long mpg=0; //miles per gallon
long avgMpg=0; //average miles per gallon
unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;
const long interval = 1000; 
const long interval2 = 500;

unsigned long getpid_time;
byte nbpid_per_second=0;

// flag used to save distance/average consumption in eeprom only if required
byte engine_started=0;
byte param_saved=0;

// the buttons interrupt
// this is the interrupt handler for button presses
ISR(PCINT1_vect)
{
#if 0
  static unsigned long last_millis = 0;
  unsigned long m = millis();

  if (m - last_millis > 20)
  { // do pushbutton stuff
    buttonState &= PINC;
  }
  //  else ignore interrupt: probably a bounce problem
  last_millis = m;
#else
//  buttonState &= PINC;
#endif
}



void serial_rx_on() {
//  UCSR0B |= _BV(RXEN0);  //enable UART RX
  Serial.begin(10400);    //setting enable bit didn't work, so do beginSerial
}

void serial_rx_off() {
  UCSR0B &= ~(_BV(RXEN0));  //disable UART RX
}

void serial_tx_off() {

   UCSR0B &= ~(_BV(TXEN0));  //disable UART TX
   delay(20);                 //allow time for buffers to flush
}


int iso_read_byte()
{
  int b;
  byte t=0;
  while(t!=125  && (b=Serial.read())==-1) {
    delay(1);
    t++;
  }
  if (t>=125) {
    b = 0;
  }
  return b;
}

void iso_write_byte(byte b)
{
  serial_rx_off();
  Serial.write(b);
  delay(10);    // ISO requires 5-20 ms delay between bytes.
  serial_rx_on();
}

// inspired by SternOBDII\code\checksum.c
byte iso_checksum(byte *data, byte len)
{
  byte i;
  byte crc;

  crc=0;
  for(i=0; i<len; i++)
    crc=crc+data[i];

  return crc;
}

// inspired by SternOBDII\code\iso.c
byte iso_write_data(byte *data, byte len)
{
  byte i, n;
  byte buf[20];

  // ISO header
  buf[0]=0x68;
  buf[1]=0x6A;    // 0x68 0x6A is an OBD-II request
  buf[2]=0xF1;    // our requesters address (off-board tool)
  // append message
  for(i=0; i<len; i++)
    buf[i+3]=data[i];

  // calculate checksum
  i+=3;
  buf[i]=iso_checksum(buf, i);

  // send char one by one
  n=i+1;
  for(i=0; i<n; i++)
  {
    iso_write_byte(buf[i]);
  }

  return 0;
}

// read n byte of data (+ header + cmd and crc)
// return the result only in data
byte iso_read_data(byte *data, byte len)
{
  byte i;
  byte buf[20];

  // header 3 bytes: [80+datalen] [destination=f1] [source=01]
  // data 1+1+len bytes: [40+cmd0] [cmd1] [result0]
  // checksum 1 bytes: [sum(header)+sum(data)]

    for(i=0; i<3+1+1+1+len; i++)
    buf[i]=iso_read_byte();

  // test, skip header comparison
  // ignore failure for the moment (0x7f)
  // ignore crc for the moment

  // we send only one command, so result start at buf[4] Actually, result starts at buf[5], buf[4] is pid requested...
  memcpy(data, buf+5, len);

  delay(55);    //guarantee 55 ms pause between requests

  return len;
}

/* ISO 9141 init */
byte iso_init()
{
  first_attempt = false;
  byte b;
  byte kw1, kw2;
  serial_tx_off(); //disable UART so we can "bit-Bang" the slow init.
  serial_rx_off();
  delay(3000); //k line should be free of traffic for at least two secconds.
  // drive K line high for 300ms
  digitalWrite(K_OUT, HIGH);
  delay(300);

  // send 0x33 at 5 bauds
  // start bit
  digitalWrite(K_OUT, LOW);
  delay(200);
  // data
  b=0x33;
  for (byte mask = 0x01; mask; mask <<= 1)
  {
    if (b & mask) // choose bit
      digitalWrite(K_OUT, HIGH); // send 1
    else
      digitalWrite(K_OUT, LOW); // send 0
    delay(200);
  }
  // stop bit + 60 ms delay
  digitalWrite(K_OUT, HIGH);
  delay(260);

  // switch now to 10400 bauds
  Serial.begin(10400);

  // wait for 0x55 from the ECU (up to 300ms)
  //since our time out for reading is 125ms, we will try it three times
  for(int i=0; i<3; i++) {
    b=iso_read_byte(); 
    if(b!=0)
      break;
  }
  
  if(b!=0x55)
    return -1;

  // wait for kw1 and kw2
  kw1=iso_read_byte();

  kw2=iso_read_byte();
//  delay(25);

  // sent ~kw2 (invert of last keyword)
  iso_write_byte(~kw2);

  // ECU answer by 0xCC (~0x33)
  b=iso_read_byte();
  if(b!=0xCC)
    return -1;

  // init OK!
  return 0;
}



// return 0 if pid is not supported, 1 if it is.
// mode is 0 for get_pid() and 1 for menu config to allow pid > 0xF0
byte is_pid_supported(byte pid, byte mode)
{
  // note that pid PID_SUPPORT00 (0x00) is always supported
  if(  (pid>0x00 && pid<=0x20 && ( 1L<<(0x20-pid) & pid01to20_support ) == 0 )
    || (pid>0x20 && pid<=0x40 && ( 1L<<(0x40-pid) & pid21to40_support ) == 0 )
    || (pid>0x40 && pid<=0x60 && ( 1L<<(0x60-pid) & pid41to60_support ) == 0 )
    || (pid>LAST_PID && (pid<0xF0 || mode==0) )
    )
    {
      return 0;
    }

  return 1;
}

// get value of a PID, return as a long value
// and also formatted for string output in the return buffer
long get_pid(byte pid, char *retbuf)
{

  byte cmd[2];    // to send the command

  byte i;
  byte buf[10];   // to receive the result
  long ret;       // will be the return value
  byte reslen;
  char decs[16];
  unsigned long time_now, delta_time;
  static byte nbpid=0;

  nbpid++;
  // time elapsed
  time_now = millis();
  delta_time = time_now - getpid_time;
  if(delta_time>1000)
  {
    nbpid_per_second=nbpid;
    nbpid=0;
    getpid_time=time_now;
  }

  // check if PID is supported (should not happen except for some 0xFn)
  if(!is_pid_supported(pid, 0))
  {
    // nope
    sprintf_P(retbuf, PSTR("%02X N/A"), pid);
    return -1;
  }

  // receive length depends on pid
  reslen=pgm_read_byte_near(pid_reslen+pid);

  cmd[0]=0x01;    // ISO cmd 1, get PID
  cmd[1]=pid;
  // send command, length 2
  iso_write_data(cmd, 2);
  // read requested length, n bytes received in buf
  iso_read_data(buf, reslen);
//#endif

  // a lot of formulas are the same so calculate a default return value here
  // even if it's scrapped after, we still saved 40 bytes!
  ret=buf[0]*256U+buf[1];

  // formula and unit for each PID
  switch(pid)
  {
  case ENGINE_RPM:
#ifdef DEBUG
    ret=1726;
#else
    ret=ret/4U;
#endif
    //sprintf_P(retbuf, PSTR("%ld RPM"), ret);
    break;
  case MAF_AIR_FLOW:
#ifdef DEBUG
    ret=2048;
#endif
    // ret is not divided by 100 for return value!!
    long_to_dec_str(ret, decs, 2);
    //sprintf_P(retbuf, PSTR("%s g/s"), decs);
    break;
  case VEHICLE_SPEED:
#ifdef DEBUG
    ret=100;
#else
    ret=(buf[0] * params.speed_adjust) / 100U;
#endif
    if(!params.use_metric)
          ret=((ret*1000U)/1609U);      
//    sprintf_P(retbuf, pctldpcts, ret, params.use_metric?"\003\004":"\006\004");
    break;
    // do not touch vss, it is used by fuel calculation after, so reset it
    //wrong, if you don t break here, it will always be in SI units
//#ifdef DEBUG
//    ret=100;
//#else
    ret=(buf[0] * params.speed_adjust) / 100U;
    
//#endif
    break;
  case FUEL_STATUS:
//#ifdef DEBUG
 //   ret=0x0200;
//#endif
    if(buf[0]==0x01)
      sprintf_P(retbuf, PSTR("OPENLOWT"));  // open due to insufficient engine temperature
    else if(buf[0]==0x02)
      sprintf_P(retbuf, PSTR("CLSEOXYS"));  // Closed loop, using oxygen sensor feedback to determine fuel mix. should be almost always this
    else if(buf[0]==0x04)
      sprintf_P(retbuf, PSTR("OPENLOAD"));  // Open loop due to engine load, can trigger DFCO
    else if(buf[0]==0x08)
      sprintf_P(retbuf, PSTR("OPENFAIL"));  // Open loop due to system failure
    else if(buf[0]==0x10)
      sprintf_P(retbuf, PSTR("CLSEBADF"));  // Closed loop, using at least one oxygen sensor but there is a fault in the feedback system
    else
      sprintf_P(retbuf, PSTR("%04lX"), ret);
    break;
  case LOAD_VALUE:
  case THROTTLE_POS:
  case REL_THR_POS:
  case EGR:
  case EGR_ERROR:
  case FUEL_LEVEL:  
  case ABS_THR_POS_B:
  case ABS_THR_POS_C:
  case ACCEL_PEDAL_D:
  case ACCEL_PEDAL_E:
  case ACCEL_PEDAL_F:
  case CMD_THR_ACTU:
//#ifdef DEBUG
//    ret=17;
//#else
    ret=(buf[0]*100U)/255U;
//#endif
    sprintf_P(retbuf, PSTR("%ld %%"), ret);
    break;
  case B1S1_O2_V:
  case B1S2_O2_V:
  case B1S3_O2_V:
  case B1S4_O2_V:
  case B2S1_O2_V:
  case B2S2_O2_V:
  case B2S3_O2_V:
  case B2S4_O2_V:
    ret=buf[0]*5U;  // not divided by 1000 for return!!
    if(buf[1]==0xFF)  // not used in trim calculation
      sprintf_P(retbuf, PSTR("%ld mV"), ret);
    else
      sprintf_P(retbuf, PSTR("%ldmV/%d%%"), ret, ((buf[1]-128)*100)/128);
    break;
  case O2S1_WR_V:
  case O2S2_WR_V:
  case O2S3_WR_V:
  case O2S4_WR_V:
  case O2S5_WR_V:
  case O2S6_WR_V:
  case O2S7_WR_V:
  case O2S8_WR_V:
  case O2S1_WR_C:
  case O2S2_WR_C:
  case O2S3_WR_C:
  case O2S4_WR_C:
  case O2S5_WR_C:
  case O2S6_WR_C:
  case O2S7_WR_C:
  case O2S8_WR_C:
  case CMD_EQUIV_R:
    ret=(ret*100)/32768; // not divided by 1000 for return!!
    long_to_dec_str(ret, decs, 2);
    sprintf_P(retbuf, PSTR("l:%s"), decs);
    break;
  case DIST_MIL_ON:
  case DIST_MIL_CLR:
    if(!params.use_metric)
      ret=(ret*1000U)/1609U;
//    sprintf_P(retbuf, pctldpcts, ret, params.use_metric?"\003":"\006");
    break;
  case TIME_MIL_ON:
  case TIME_MIL_CLR:
    sprintf_P(retbuf, PSTR("%ld min"), ret);
    break;
  case COOLANT_TEMP:
  case INT_AIR_TEMP:
  case AMBIENT_TEMP:
  case CAT_TEMP_B1S1:
  case CAT_TEMP_B2S1:
  case CAT_TEMP_B1S2:
  case CAT_TEMP_B2S2:
    if(pid>=CAT_TEMP_B1S1 && pid<=CAT_TEMP_B2S2)
#ifdef DEBUG
      ret=600;
#else
      ret=ret/10U - 40;
#endif
    else
#ifdef DEBUG
      ret=40;
#else
      ret=buf[0]-40;
#endif
    if(!params.use_metric)
      ret=(ret*9)/5+32;
    sprintf_P(retbuf, PSTR("%ld\005%c"), ret, params.use_metric?'C':'F');
    break;
  case STFT_BANK1:
  case LTFT_BANK1:
  case STFT_BANK2:
  case LTFT_BANK2:
    ret=(buf[0]-128)*7812;  // not divided by 10000 for return value
    long_to_dec_str(ret/100, decs, 2);
    sprintf_P(retbuf, PSTR("%s %%"), decs);
    break;
  case FUEL_PRESSURE:
  case MAN_PRESSURE:
  case BARO_PRESSURE:
    ret=buf[0];
    if(pid==FUEL_PRESSURE)
      ret*=3U;
    sprintf_P(retbuf, PSTR("%ld kPa"), ret);
    break;
  case TIMING_ADV:
    ret=(buf[0]/2)-64;
    sprintf_P(retbuf, PSTR("%ld\005"), ret);
    break;
  case CTRL_MOD_V:
    long_to_dec_str(ret/10, decs, 2);
    sprintf_P(retbuf, PSTR("%s V"), decs);
    break;
#ifndef DEBUG  // takes 254 bytes, may be removed if necessary
  case OBD_STD:
    ret=buf[0];
    if(buf[0]==0x01)
      sprintf_P(retbuf, PSTR("OBD2CARB"));
    else if(buf[0]==0x02)
      sprintf_P(retbuf, PSTR("OBD2EPA"));
    else if(buf[0]==0x03)
      sprintf_P(retbuf, PSTR("OBD1&2"));
    else if(buf[0]==0x04)
      sprintf_P(retbuf, PSTR("OBD1"));
    else if(buf[0]==0x05)
      sprintf_P(retbuf, PSTR("NOT OBD"));
    else if(buf[0]==0x06)
      sprintf_P(retbuf, PSTR("EOBD"));
    else if(buf[0]==0x07)
      sprintf_P(retbuf, PSTR("EOBD&2"));
    else if(buf[0]==0x08)
      sprintf_P(retbuf, PSTR("EOBD&1"));
    else if(buf[0]==0x09)
      sprintf_P(retbuf, PSTR("EOBD&1&2"));
    else if(buf[0]==0x0a)
      sprintf_P(retbuf, PSTR("JOBD"));
    else if(buf[0]==0x0b)
      sprintf_P(retbuf, PSTR("JOBD&2"));
    else if(buf[0]==0x0c)
      sprintf_P(retbuf, PSTR("JOBD&1"));
    else if(buf[0]==0x0d)
      sprintf_P(retbuf, PSTR("JOBD&1&2"));
    else
      sprintf_P(retbuf, PSTR("OBD:%02X"), buf[0]);
    break;
#endif
    // for the moment, everything else, display the raw answer
  default:
    // transform buffer to an hex value
    ret=0;
    for(i=0; i<reslen; i++)
    {
      ret*=256L;
      ret+=buf[i];
    }
    sprintf_P(retbuf, PSTR("%08lX"), ret);
    break;
  }

  return ret;
}

// ex: get a long as 687 with prec 2 and output the string "6.87"
// precision is 1 or 2
void long_to_dec_str(long value, char *decs, byte prec)
{
  byte pos;

  // sprintf_P does not allow * for the width ?!?
  if(prec==1)
    sprintf_P(decs, PSTR("%02ld"), value);
  else if(prec==2)
    sprintf_P(decs, PSTR("%03ld"), value);

  pos=strlen(decs)+1;  // move the \0 too
  // a simple loop takes less space than memmove()
  for(byte i=0; i<=prec; i++)
  {
    decs[pos]=decs[pos-1];  // move digit
    pos--;
  }

  // then insert decimal separator
  decs[pos]=params.use_metric?',':'.';
}

void check_supported_pids(void)
{
  char str[STRLEN];

#ifdef DEBUG
  pid01to20_support=0xBE1FA812;
#else
  pid01to20_support=get_pid(PID_SUPPORT00, str);
#endif

  if(is_pid_supported(PID_SUPPORT20, 0))
    pid21to40_support=get_pid(PID_SUPPORT20, str);

  if(is_pid_supported(PID_SUPPORT40, 0))
    pid41to60_support=get_pid(PID_SUPPORT40, str);
}


/*
 * Initialization
 */

void setup()                    // run once, when the sketch starts
{
    ///////////for OLED display
// by default, we'll generate the high voltage from the 3.3v 
  lcd.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C 
  // init done 
  lcd.display();
   // Clear the buffer.
  lcd.clearDisplay(); 
  // text display tests
  lcd.setTextSize(1);
  lcd.setTextColor(WHITE);
  lcd.setCursor(0,0);  
  lcd.display();  
  lcd.clearDisplay();
  

/////////////////end OLED display
    set_params(); //sets US or SI units
    
#ifndef ELM
  byte r;

  // init pinouts
  pinMode(K_OUT, OUTPUT);
  pinMode(K_IN, INPUT);
#endif


  // low level interrupt enable stuff
  // interrupt 1 for the 3 buttons
  PCMSK1 |= (1 << PCINT11) | (1 << PCINT12) | (1 << PCINT13);
  PCICR  |= (1 << PCIE1);

  // load parameters
  params_load();  // if something is wrong, default parms are used

    

  do // init loop
  {
  lcd.clearDisplay();
  lcd.setCursor(0,0);
  lcd.print("ISO9141 Init");
    
    lcd.setCursor(0,20); //3rd line;
    if(params.use_metric) {
  lcd.print("SI units");
  } else { lcd.print("US units"); }
  
  lcd.display();
  if(first_attempt) {
    delay(5000);
  }
    r=iso_init();
      if(r==0)  {
      lcd.setCursor(0,10); //Move to second line
    lcd.println("Successful!");
    lcd.display();
    lcd.setTextSize(2);
      } else    { 
      lcd.setCursor(0,10); //Move to second line
      lcd.println("Failed"); 
      lcd.display(); 
      } 
    delay(1000);
  }
  while(r!=0); // end init loop


  // check supported PIDs
  check_supported_pids();

  old_time=millis();  // epoch
  getpid_time=old_time;
}

/*
 * Main loop
 */

void loop()                     
{
  //constantly get vss
  get_vss();

  unsigned long currentMillis = millis();
  //only get mpg once per second and average it
  if (currentMillis - previousMillis >= interval) {
    // save the last time you did it
    previousMillis = currentMillis;
    get_mpg();
  }
//only update display 2 times per second
  if (currentMillis - previousMillis2 >= interval2) {
    // save the last time you did it
    previousMillis2 = currentMillis;
    print_data();
  }

}

/*
 * Memory related functions
 */

void get_vss() {
  char str[STRLEN];

vss = get_pid(VEHICLE_SPEED, str);

}

void get_mpg() {
  char str[STRLEN];
  // MPG
//14.7 grams of air to 1 gram of gasoline - ideal air/fuel ratio
//6.17 pounds per gallon - density of gasoline
//4.54 grams per pound - conversion
//VSS - vehicle speed in kilometers per hour
//0.621371 miles per hour/kilometers per hour - conversion
//3600 seconds per hour - conversion
//MAF - mass air flow rate in 100 grams per second
//100 - to correct MAF to give grams per second

   maf = get_pid(MAF_AIR_FLOW, str);
  //long mpg =  14.7 * 6.17 * 454 * (vss * 0.621371) / (3600 * maf / 100);
  //vss is already in US units
  mpg =  (14.7 * 6.17 * 454 * vss) / (3600 * maf / 100);
  if(avgMpg == 0) { 
    avgMpg = mpg; 
  } else {
   avgMpg = (avgMpg + mpg) / 2;   
    }   
 }

void print_data() 
{
  lcd.clearDisplay();
  lcd.setCursor(0,1);
  lcd.print(vss);
  lcd.print(" MPH");  
  lcd.setCursor(0,18); //Move to 2nd line
  lcd.print("MPG:");
  lcd.print(avgMpg);
  lcd.display();
}

// we have 512 bytes of EEPROM on the 168P, more than enough
void params_save(void)
{
  uint16_t crc;
  byte *p;

  // CRC will go at the end
  crc=0;
  p=(byte*)&params;
  for(byte i=0; i<sizeof(params_t); i++)
    crc+=p[i];

  // start at address 0
  eeprom_write_block((const void*)&params, (void*)0, sizeof(params_t));
  // write CRC after params struct
  eeprom_write_word((uint16_t*)sizeof(params_t), crc);
}

void params_load(void)
{
  params_t params_tmp;
  uint16_t crc, crc_calc;
  byte *p;

  // read params
  eeprom_read_block((void*)&params_tmp, (void*)0, sizeof(params_t));
  // read crc
  crc=eeprom_read_word((const uint16_t*)sizeof(params_t));

  // calculate crc from read stuff
  crc_calc=0;
  p=(byte*)&params_tmp;
  for(byte i=0; i<sizeof(params_t); i++)
    crc_calc+=p[i];

  // compare CRC
  if(crc==crc_calc)     // good, copy read params to params
    params=params_tmp;
}
