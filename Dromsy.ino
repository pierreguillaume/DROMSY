//
// MAX7219 Control. Assumes two 8x8 LED Matrices
//
// http://www.amazon.co.uk/MAX7219-Matrix-Display-Module-Board/dp/B00NQB4ICG/ref=lh_ni_t?ie=UTF8&psc=1&smid=A50DZI580G3JX
//
// http://playground.arduino.cc/Main/LedControl 


#include <LedControl.h>

#include <TimeLib.h> 
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include <TimeAlarms.h>

#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
ESP8266WebServer server ( 80 );

#include "SoftwareSerial.h"
#include <DFPlayer_Mini_Mp3.h>
SoftwareSerial mySoftwareSerial(0, 4); // RX, TX


const char ssid[] = <your_SSID>;  //  your network SSID (name)
const char pass[] = <your_password>;       // your network password

// NTP Server for obtaining UTC time:
IPAddress timeServer(132, 163, 4, 101); // time-a.timefreq.bldrdoc.gov
const int timeZone = 1;     // Central European Time - This will adjust the NTP UTC time to France local time and manage Daylight saving adjustements automatically
WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets


const int numDevices = 5;      // number of MAX7219 matrix used
const int dataPin = D7;
const int clkPin = D5;
const int csPin = D8;
LedControl lc=LedControl(dataPin,clkPin,csPin,numDevices);

const int interruptSoundPin = D1; // digital pin for microphone sound detector

// Define MAX7219 icons
#define C 0
#define L1 1
#define L2 2
#define L3 3
#define L4 4
#define L5 5
#define L6 6
#define L7 7
#define L8 8
#define L9 9
#define L0 10
#define CR1 11
#define CR2 12		
#define CL1 13
#define CL2 14    
#define U1 15
#define U1R1 16
#define U1R2 17		
#define U1L1 18
#define U1L2 19
#define D1 20
#define D1R1 21
#define D1R2 22    
#define D1L1 23
#define D1L2 24   
#define U2 25
#define U2R1 26
#define U2L1 27
#define D2 28
#define D2R1 29
#define D2L1 30
#define B1 31
#define B2 32
#define B3 33
#define B4 34
#define B5 35
#define Sleep 36
#define AngryR 37

#define Happy 38
#define Upset 39
#define Neutral 40

#define AngryL 41
#define UpperNose 42

// Define all possible states for Drmosy
enum State {
  SLEEP,
  AWAKE,
  UPSET,
  TIME_ALWAYS,
  TIME
  };


// track current state and previous state
volatile State current_state;
State previous_state;


// Variables storing user settings in the EEPROM
String weekday_sleep_time;
String weekday_time_alarm;
String weekday_sound_alarm;
String weekend_sleep_time;
String weekend_time_alarm;
String weekend_sound_alarm;
		
typedef struct {		
	 const unsigned char array1[8];	
} binaryArrayType;		
		

binaryArrayType binaryArray[43] =    
{ 
  { // C, 0 
B00111100,
B01111110,
B11111111,
B11100111,
B11100111,
B11111111,
B11111111,
B11111111
  }, 
 { //  L1 -1 
  B00010000,
  B00110000,
  B00010000,
  B00010000,
  B00010000,
  B00010000,
  B00010000,
  B00111000
},{ // L2 - 2
  B00111000,
  B01000100,
  B00000100,
  B00000100,
  B00001000,
  B00010000,
  B00100000,
  B01111100
},{ // L3 - 3
  B00111000,
  B01000100,
  B00000100,
  B00011000,
  B00000100,
  B00000100,
  B01000100,
  B00111000
},{ // L4 - 4
  B00000100,
  B00001100,
  B00010100,
  B00100100,
  B01000100,
  B01111100,
  B00000100,
  B00000100
},{ // L5 - 5
  B01111100,
  B01000000,
  B01000000,
  B01111000,
  B00000100,
  B00000100,
  B01000100,
  B00111000
},{  // L6 -6
  B00111000,
  B01000100,
  B01000000,
  B01111000,
  B01000100,
  B01000100,
  B01000100,
  B00111000
},{  // L7 - 7
  B01111100,
  B00000100,
  B00000100,
  B00001000,
  B00010000,
  B00100000,
  B00100000,
  B00100000
},{ // L8 - 8 
  B00111000,
  B01000100,
  B01000100,
  B00111000,
  B01000100,
  B01000100,
  B01000100,
  B00111000
},{ // L9 - 9 
  B00111000,
  B01000100,
  B01000100,
  B01000100,
  B00111100,
  B00000100,
  B01000100,
  B00111000
}, 
{  // L0 - 0
  B00111000,
  B01000100,
  B01000100,
  B01000100,
  B01000100,
  B01000100,
  B01000100,
  B00111000
},
  
  { // CR1, 0  
B00111100,
B01111110,
B11111111,
B11110011,
B11110011,
B11111111,
B11111111,
B11111111
  }, { // CR2, 0 
B00111100,
B01111110,
B11111111,
B11111001,
B11111001,
B11111111,
B11111111,
B11111111
  },
{ // CL1, 0  
B00111100,
B01111110,
B11111111,
B11001111,
B11001111,
B11111111,
B11111111,
B11111111
  }, { // CL2, 0 
B00111100,
B01111110,
B11111111,
B10011111,
B10011111,
B11111111,
B11111111,
B11111111
  },    
  { // U1, 1  
B00111100,
B01111110,
B11100111,
B11100111,
B11111111,
B11111111,
B11111111,
B11111111
  },
  { // U1R1, 1  
B00111100,
B01111110,
B11110011,
B11110011,
B11111111,
B11111111,
B11111111,
B11111111
  }, { // U1R2, 1 
B00111100,
B01111110,
B11111001,
B11111001,
B11111111,
B11111111,
B11111111,
B11111111
  }, { // U1L1, 1  
B00111100,
B01111110,
B11001111,
B11001111,
B11111111,
B11111111,
B11111111,
B11111111
  }, { // U1L2, 1 
B00111100,
B01111110,
B10011111,
B10011111,
B11111111,
B11111111,
B11111111,
B11111111
  },
  { // D1, 1  
B00111100,
B01111110,
B11111111,
B11111111,
B11100111,
B11100111,
B11111111,
B11111111
  },
  { // D1R1, 1  
B00111100,
B01111110,
B11111111,
B11111111,
B11110011,
B11110011,
B11111111,
B11111111
  }, { // D1R2, 1 
B00111100,
B01111110,
B11111111,
B11111111,
B11111001,
B11111001,
B11111111,
B11111111
  }, { // D1L1, 1  
B00111100,
B01111110,
B11111111,
B11111111,
B11001111,
B11001111,
B11111111,
B11111111
  }, { // D1L2, 1 
B00111100,
B01111110,
B11111111,
B11111111,
B10011111,
B10011111,
B11111111,
B11111111
  }, 
 { // U2, 1 
B00111100,
B01100110,
B11100111,
B11111111,
B11111111,
B11111111,
B11111111,
B11111111
  },
 { // U2R1, 1 
B00111100,
B01110010,
B11110011,
B11111111,
B11111111,
B11111111,
B11111111,
B11111111
  }, 
  { // U2L1, 1 
B00111100,
B01001110,
B11001111,
B11111111,
B11111111,
B11111111,
B11111111,
B11111111
  },
  { // D2, 1 
B00111100,
B01111110,
B11111111,
B11111111,
B11111111,
B11100111,
B11100111,
B11111111
  },
 { // D2R1, 1 
B00111100,
B01111110,
B11111111,
B11111111,
B11111111,
B11110011,
B11110011,
B11111111
  }, 
  { // D2L1, 1 
B00111100,
B01111110,
B11111111,
B11111111,
B11111111,
B11001111,
B11001111,
B11111111
  }, 
  { // B1, 1 
B00000000,
B00111100,
B01111110,
B01100110,
B01100110,
B11111111,
B11111111,
B00000000
  }, 
  { // B2, 1 
B00000000,
B00000000,
B00111100,
B01100110,
B01100110,
B00111100,
B00000000,
B00000000
  }, 
  { // B3, 1 
B00000000,
B00000000,
B00000000,
B01100110,
B01100110,
B01111110,
B00000000,
B00000000
  }, 
  { // B4, 1 
B00000000,
B00000000,
B00000000,
B01111110,
B01111110,
B00000000,
B00000000,
B00000000
  }, 
  { // B5, 1 
B00000000,
B00000000,
B00000000,
B10000001,
B01111110,
B00000000,
B00000000,
B00000000
  }, 
  { // Sleep, 1 
B00000000,
B00000000,
B00000000,
B10000001,
B01111110,
B00000000,
B00000000,
B00000000
  }, 
  { // Angry Right eye, 1 
B00001110,
B00011111,
B00111111,
B01110111,
B11100111,
B11111111,
B01111110,
B00111100
  }, 
  { // happy
  B00011000,
  B00000000,
  B00000000,
  B10000001,
  B10000001,
  B10000001,
  B01000010,
  B00111100
  
  }, { // upset
  B00000000,
  B00000000,
  B00000000,
  B00111100,
  B01000010,
  B10000001,
  B10000001,
  B10000001  
  }, { // neutral
  B00011000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B10000001,
  B01111110,
  B00000000
  
  }, {//AngryL 
  B01110000,
  B11111000,
  B11111100,
  B11101110,
  B11100111,
  B11111111,
  B01111110,
  B00111100 
  }, {// UpperNose
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
  B00011000,
  B00011000,    
  }
 
};    
	

typedef struct {			
	int frameCount;		// index pointer into binaryArray signifying animation frame
	int frameDelay;	        // Approx delay in MilliSeconds to hold display this animated frame
} frameType;			
    


frameType movie[][30] = 
{
  
  // Eye specific
  {{B1,20},{B2,20},{B3,20},{B4,20},{B5,100},{B4,20},{B3,20},{B2,20},{B1,20}}, // BLINK 
  {{C,1000}, {D1L1, 20}, {D2L1,500} , {D1L2, 20} , {CL2, 500}, {CL1,20}},  // center-down-left-center    
  {{C,1000}, {U1,100}, {U2,500}, {U2R1,200},{U2,200}, {U2L1,200},{U2,300},  {U1,100}}, // center-up-left/right-center
  {{C,1000}, {CL1,100}, {CL2,1000},{CL1,100},{C,100}, {CR1,100},{CR2,1000},{CR1,100}, {C,100}, {CL1,100}, {CL2,1000},{CL1,100},{C,100}, {CR1,100},{CR2,1000},{CR1,100}},  // left-right-left-right   
  {{C,1000}, {CR1,100},{CR2,100}, {U1R2,100},{U2R1,100}, {U2,100}, {U2L1,100},{U1L2,100},{CL2,100}, {CL1,100}}, //round right-up-left
  {{B1,20},{B2,20},{B3,20},{B4,20},{B5,100},{B4,20},{B3,20},{B2,20},{B1,20}}, // BLINK 
  {{C,1000}, {CR1,100},{CR2,100}, {U1R2,100},{U2R1,100}, {U2,100}, {U2L1,100},{U1L2,100},{CL2,100}, {D1L2,100},{D2L1,100}, {D2,100},{D2R1,100},{D1R2,100},{CR2,100},{CR1,100}} //round trip 
  
};


void setup(){

   Serial.begin(9600);

   // Initialize LED Displays
   Serial.println("Initializing LED displays");
   for (int x=0; x<numDevices; x++){
        lc.shutdown(x,false);       //The MAX72XX is in power-saving mode on startup
        lc.setIntensity(x,2);       // Set the brightness to default value
        lc.clearDisplay(x);         // and clear the display
    }

   randomSeed(42);

   Serial.print("Initializing Wifi - Connecting to: ");
   Serial.println(ssid);
   WiFi.begin(ssid, pass);
  
   while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
   }

  Serial.println("connected with IP:");
  Serial.println(WiFi.localIP());
  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());
  Serial.println("waiting for NTP Time sync");
  
  // get time from NTP and get synced
  setSyncProvider(getNtpTime);
  
  // Se time Manually for debuggind and testing alarms
  //setTime(hr,min,sec,day,month,yr); 
  //setTime(21,30,0    ,4,2,2017); 

  Serial.println("Obtained NTP time:");
  printTime();

  // Initialize Webserver and EEPROM
  server.on ( "/", handleRoot );
  server.on ("/update_settings", HTTP_POST, handleUpdateSettings);
  server.onNotFound ( handleNotFound );
  server.begin();
  Serial.println ( "HTTP server started" );

  EEPROM.begin(128);
  readSettings();
  createAlarms();
  Serial.println("EEPROM initialized, loaded settings and alarms created");

  //create a timer that every 40 seconds will show the time
  Alarm.timerRepeat(40, stateToTime); 

  // Initialize MP3
  mySoftwareSerial.begin(9600);
  mp3_set_serial (mySoftwareSerial); //set Serial for DFPlayer-mini mp3 module 
  delay(1);  //wait 1ms for mp3 module to set volume
  mp3_set_volume (15);
  Serial.println("mp3 Initialized");
    
  // Initialize Microphone and attach to Interrupt routine
  attachInterrupt(digitalPinToInterrupt(interruptSoundPin), hearNoise, CHANGE);

  //Initialize state
  current_state = getCurrentState();
  //at startup previous state is initialized with the same state
  previous_state =  current_state;
}



// will be called every 20 seconds to cycle between AWAKE/SLEEP and TIME
void stateToTime() {

  if (current_state == AWAKE || current_state == SLEEP) {
    previous_state = current_state;
    current_state = TIME;
  }
  
}


void loop(){ 

  while (true){
  
   clearDisplays();   
   switch (current_state ) {

    case AWAKE:
    {
      // select randomly a movie to play
      int x = random(0,6);
      //int x = 2;
     
      for (int y = 0; y < (sizeof(movie[x])/sizeof(frameType)); y++)
       {
         for (int i = 0; i < 8; i++)
         {
           // Eyes
           lc.setRow(1,i, binaryArray[movie[x][y].frameCount].array1[i]); 
           lc.setRow(3,i, binaryArray[movie[x][y].frameCount].array1[i]); 
  
           // Mouth and nose - only one frame 
           lc.setRow(0,i, binaryArray[Neutral].array1[i]);
         }          
         delay(movie[x][y].frameDelay);
       
       
       }
       break;
      }
       case SLEEP:
        {
          for (int i = 0; i < 8; i++)
         {
           // Eyes
           lc.setRow(1,i, binaryArray[Sleep].array1[i]); 
           lc.setRow(3,i, binaryArray[Sleep].array1[i]); 
            // Mouth and nose 
           lc.setRow(0,i, binaryArray[Happy].array1[i]);
         }  
         delay(1000);

         break;
        }
       

        case UPSET:
        {
          for (int i = 0; i < 8; i++)
         {
           // Eyes
           lc.setRow(1,i, binaryArray[AngryR].array1[i]); 
           lc.setRow(3,i, binaryArray[AngryL].array1[i]); 
            // Mouth and nose 
           lc.setRow(0,i, binaryArray[Upset].array1[i]);
           lc.setRow(2,i, binaryArray[UpperNose].array1[i]);
         }  
          delay(3000);
          current_state= SLEEP;

        break;
        }
        

        case TIME:
        {
          displayTime();
          delay(5000);
          // come back to the previous state AWAKE or SLEEP
          current_state = previous_state;

       
        }
        case TIME_ALWAYS:
        {
          displayTime();
          delay(5000);

       
        }
        break;

        
   
       
    } // end switch

    // Handle webserver connections
    server.handleClient(); 
    Alarm.delay(1000); // check timers and alarms from TimeAlarms
    
  }  // end While
} // end Loop







AlarmID_t alarmIDs[4]; // 

// Updates all alarms with current user settings saved in the EEPROM
void createAlarms() {

  // All alarms will run every day, discard them in callback function if they are not relevant for the current weekday or weekend 

  // weekday sleep alarm  
  alarmIDs[0] = Alarm.alarmRepeat(getHour(weekday_sleep_time), getMinute(weekday_sleep_time), 0, goSleepWeekday);
  // weekday wake up alarm
  alarmIDs[1] = Alarm.alarmRepeat(getHour(weekday_time_alarm), getMinute(weekday_time_alarm), 0, goWakeupWeekday); //ok
  // weekend sleep alarm
  alarmIDs[2] = Alarm.alarmRepeat(getHour(weekend_sleep_time), getMinute(weekend_sleep_time), 0, goSleepWeekend); // ok
  //weekend wake up alarm
  alarmIDs[3] = Alarm.alarmRepeat(getHour(weekend_time_alarm), getMinute(weekend_time_alarm), 0, goWakeupWeekend); //ok

  Serial.print("Alarm weekday wake up:"); Serial.println(alarmIDs[1]);


}

// Returns the State DROMSY should be (SLEEP or AWAKE) based on the current time and user settings
State getCurrentState() {

  // function Logic assumes wake up time is always before sleep time
  // checks if current time is within AWAKE window  (timer -- --- sleep) otherwise assmumes its SLEEP
   int weekDay = weekday();
   int hr = hour();
   int mins = minute();
   int hr_weekday_alarm =  getHour(weekday_time_alarm);
   int mins_weekday_alarm = getMinute(weekday_time_alarm); 
   int hr_weekday_sleep = getHour(weekday_sleep_time);
   int mins_weekday_sleep = getMinute(weekday_sleep_time);
   int hr_weekend_alarm =  getHour(weekend_time_alarm);
   int mins_weekend_alarm = getMinute(weekend_time_alarm); 
   int hr_weekend_sleep = getHour(weekend_sleep_time);
   int mins_weekend_sleep = getMinute(weekend_sleep_time);
   State res;

  //Monday to Thursday (early wake up time, early sleep time)
   if (weekDay == 2 || weekDay == 3 || weekDay == 4 || weekDay == 5) { 

      // check if should be awake   
      if ((hr == hr_weekday_alarm  && mins >= mins_weekday_alarm) || (hr > hr_weekday_alarm  && hr < hr_weekday_sleep) || (hr == hr_weekday_sleep && mins < mins_weekday_sleep)) {
          Serial.println("Monday to Thursday - state is AWAKE now");
          res = AWAKE;
      } else {
          Serial.println("Monday to Thursday - state is SLEEP now");
          res = SLEEP;  
      }
    }
    
    //Friday (early wake up time, late sleep time)
    else if (weekDay == 6) {
      // check if should be awake   
      if ((hr == hr_weekday_alarm  && mins >= mins_weekday_alarm) || (hr > hr_weekday_alarm  && hr < hr_weekend_sleep) || (hr == hr_weekend_sleep && mins < mins_weekend_sleep)) {
          Serial.println("Friday - state is AWAKE now");
          res = AWAKE;
      } else {
          Serial.println("Friday - state is SLEEP now");
          res = SLEEP;  
      }
    
    
    }
  // Saturday (late wake up time, late sleep time)
   else if (weekDay == 7) {
      // check if should be awake   
      if ((hr == hr_weekend_alarm  && mins >= mins_weekend_alarm) || (hr > hr_weekend_alarm  && hr < hr_weekend_sleep) || (hr == hr_weekend_sleep && mins < mins_weekend_sleep)) {
          Serial.println("Saturday - state is AWAKE now");
          res = AWAKE;
      } else {
          Serial.println("Saturday - state is SLEEP now");
          res = SLEEP;  
      }
    
    
    } 
  //Sunday (late wake up time, early sleep time)
  else if (weekDay == 1) {
     
     // check if should be awake   
      if ((hr == hr_weekend_alarm  && mins >= mins_weekend_alarm) || (hr > hr_weekend_alarm  && hr < hr_weekday_sleep) || (hr == hr_weekday_sleep && mins < mins_weekday_sleep)) {
          Serial.println("Sunday - state is AWAKE now");
          res = AWAKE;
      } else {
          Serial.println("Sunday - state is SLEEP now");
          res = SLEEP;  
      }
    
  }

    return res;
}


void goSleepWeekday() {

  int weekDay = weekday();  // sunday = 1, Monday = 2 etc .. Saturday = 7
  if (weekDay == 6 || weekDay == 7) {
    //its Friday or Saturday weekend sleep time DO NOTHING
  } else {
      // handle logic here
      Serial.println("Alarm Sleep Weekday");
      printTime();
      current_state = SLEEP;
  } 
};

void goWakeupWeekday() {

  Serial.println("Wake up week day alarm triggered");

  int weekDay = weekday();  // sunday = 1, Monday = 2 etc .. Saturday = 7
  if (weekDay == 1 || weekDay == 7) {
    //its Sunday or Saturday weekend wake up time DO NOTHING
  } else {
      // handle logic here for week day wake up
      Serial.println("Alarm wake-up Weekday");
      printTime();
      current_state = AWAKE;
      // Launch sound alarm of active under settings
      if (weekday_sound_alarm == "on") {
          Serial.println("Alarm sound is on");
          mp3_next ();
      }
      
  } 
  
};

void goSleepWeekend() {

  int weekDay = weekday();  // sunday = 1, Monday = 2 etc .. Saturday = 7
  if (weekDay == 6 || weekDay == 7) {
    //its Friday or Saturday weekend sleep time 
    Serial.println("Alarm Sleep Weekend");
    current_state = SLEEP;
    printTime();
  } else {
      // its Sunday to Thursday evening, DO NOTHING      
  }     
};

void goWakeupWeekend() {

  int weekDay = weekday();  // sunday = 1, Monday = 2 etc .. Saturday = 7
  if (weekDay == 1 || weekDay == 7) {
    //its Sunday or Saturday morning wake up time 
    Serial.println("Alarm wake-up Weekend");
    current_state = AWAKE;
    printTime();
    // Launch sound alarm of active under settings
      if (weekend_sound_alarm == "on") {
          mp3_next ();
      }
  } else {
      // its Monday to Friday wake up time, DO NOTHING    
  }  
};


// disables all Alarms
void disableAlarms() {

  for (int i=0; i < 4; i++){
      Alarm.disable(alarmIDs[i]);
      Alarm.free(alarmIDs[i]);
   } 


}


// Helper to convert State string to State enum
State stringStatetoEnum(String state) {

    if (state == "AWAKE") return AWAKE;
    if (state == "SLEEP") return SLEEP;
    if (state == "TIME_ALWAYS") return TIME_ALWAYS;
    if (state == "TIME") return TIME;
    if (state == "UPSET") return UPSET;
       
}

// Helper to convert State State to State String
String enumStatetoString(State state) {

    if (state == AWAKE) return "AWAKE";
    if (state == SLEEP) return "SLEEP";
    if (state == TIME_ALWAYS) return "TIME_ALWAYS";
    if (state == TIME) return "TIME";
    if (state == UPSET) return "UPSET";
       
}


// If noise is heard on the microphone during sleep time, DROMSY gets upset !
void hearNoise() {

  if (current_state == SLEEP) {  
    Serial.print("Sound detected:"); Serial.println(millis());
    current_state = UPSET;
  }
}


// Clears all LED matrix display contents 
void clearDisplays() {
  for (int x=0; x<numDevices; x++){
     lc.clearDisplay(x); 
  }
}




/*-------------------------------------- NTP code -----------------------------------------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  sendNTPpacket(timeServer);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:                 
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

//----------------------------------------  END NTP CODE ------------------------------------------------------------




//---------------------------------------- WEBSERVER EEPROM CODE -----------------------------------------------------

// Read settings from EEPROM and updates the global settings variables. Each setting is 5 character in length and starts every ten adresses 0, 10, 20 etc
void readSettings() {
  
  weekday_sleep_time = read_StringEE(0,5);
  
  weekday_time_alarm = read_StringEE(10,5);
  Serial.print("Read EEPROM weekday wake up time:"); Serial.println(weekday_time_alarm);
  weekday_sound_alarm = read_StringEE(20,5);
  weekend_sleep_time = read_StringEE(30,5);
  weekend_time_alarm = read_StringEE(40,5);;
  weekend_sound_alarm = read_StringEE(50,5);

}


// Root webpage for user to configure settings
void handleRoot() {

  String temp;
  readSettings();
   
temp = 
" <!DOCTYPE html> \
<html> \
    <head> \
    <script src='https://ajax.googleapis.com/ajax/libs/jquery/1.12.4/jquery.min.js'></script> \
    <script src='https://cdnjs.cloudflare.com/ajax/libs/moment.js/2.17.1/moment.min.js'></script> \
    <script src='https://cdnjs.cloudflare.com/ajax/libs/moment.js/2.17.1/locale/fr.js'></script> \
    <!-- Latest compiled & minified CSS --> \
    <link rel='stylesheet' href='https://maxcdn.bootstrapcdn.com/bootstrap/3.3.7/css/bootstrap.min.css' integrity='sha384-BVYiiSIFeK1dGmJRAkycuHAHRg32OmUcww7on3RYdg4Va+PmSTsz/K68vbdEjh4u' crossorigin='anonymous'> \
    <!-- Optional theme --> \
    <link rel='stylesheet' href='https://maxcdn.bootstrapcdn.com/bootstrap/3.3.7/css/bootstrap-theme.min.css' integrity='sha384-rHyoN1iRsVXV4nD0JutlnGaslCJuC7uwjduW9SVrLvRYooPp2bWYgmgJQIXwl/Sp' crossorigin='anonymous'> \
    <!-- Latest compiled & minified JavaScript --> \
    <script src='https://maxcdn.bootstrapcdn.com/bootstrap/3.3.7/js/bootstrap.min.js' integrity='sha384-Tc5IQib027qvyjSMfHjOMaLkfuWVxZxUPnCJA7l2mCWNIpG9mGCD8wGNIcPD7Txa' crossorigin='anonymous'></script> \
    <!--Datetime picker --> \
    <link rel='stylesheet' href='https://cdnjs.cloudflare.com/ajax/libs/bootstrap-datetimepicker/4.17.45/css/bootstrap-datetimepicker.min.css'> \
    <script src='https://cdnjs.cloudflare.com/ajax/libs/bootstrap-datetimepicker/4.17.45/js/bootstrap-datetimepicker.min.js'></script> \
    <meta name='viewport' content='width=device-width, initial-scale=1'> \
    </head> \
<body> \
<div class='container-fluid'> \
  <h2> DROMSY Settings</h2> \
  <br> \
  <form action='/update_settings' method='post'> \
  <div class='form-group' > \
    <div class='panel panel-info' style='width:300px'> \
        <div class='panel-heading'>DROMSY status</div> \
        <div class='panel-body'> \
            <p>Current state is "+ enumStatetoString(current_state) +"</p> \
            <label for='new_state'>Select new status:</label> \
            <select class='form-control' style='width:200px' id='new_state' name='new_state'> \
                <option>AWAKE</option> \
                <option>SLEEP</option> \
                <option>TIME_ALWAYS</option> \
            </select> \
        </div> \
    </div> \
  </div> \            
  <div class='panel panel-info' style='width:300px'> \
    <div class='panel-heading'>Weekdays</div> \
    <div class='panel-body'> \
        <div class='form-group'> \
            <div class='row'> \
                <div class='col-sm-6'> \
                <label> \
                Sleep time \
                  <input  type='text' style='width:200px' class='form-control' name='weekday_sleep_time' id='weekday_sleep_time' value='" + weekday_sleep_time + "'/> \
                </label> \ 
             </div>  \
            </div> \
            <div class='row'> \
                <div class='col-sm-6'> \
                <label> \
                Wake-up time \
                  <input  type='text' style='width:200px' class='form-control' name='weekday_time_alarm' id='weekday_time_alarm' value='" + weekday_time_alarm + "'/> \
                </label> \ 
             </div>  \
            </div> \
        </div> \
        <div class='checkbox'> \
            <label> \
                <input type='checkbox' name='weekday_sound_alarm' id='weekday_sound_alarm' " + ((weekday_sound_alarm == "on") ? ("checked") : (""))  + "> Morning Sound Alarm \
            </label> \
        </div> \
    </div> \
  </div> \
  <div class='panel panel-info' style='width:300px'> \
    <div class='panel-heading'>Weekends</div> \
    <div class='panel-body'> \
        <div class='form-group'> \
            <div class='row'> \
                <div class='col-sm-6'> \
                <label> \
                Sleep time \
                  <input  type='text' style='width:200px' class='form-control' name='weekend_sleep_time' id='weekend_sleep_time' value='" + weekend_sleep_time + "'/> \
                </label> \ 
             </div>  \
            </div> \
            <div class='row'> \
                <div class='col-sm-6'> \
                <label> \
                Wake-up time \
                <input  type='text' style='width:200px' class='form-control' name='weekend_time_alarm' id='weekend_time_alarm' value='" + weekend_time_alarm + "'/> \
                </label> \
             </div> \ 
            </div> \
        </div> \
        <div class='checkbox'> \
            <label> \
                <input type='checkbox' name='weekend_sound_alarm' id='weekend_sound_alarm' " + ((weekend_sound_alarm == "on") ? ("checked") : (""))  + "> Morning Sound Alarm \
            </label> \
        </div> \
    </div> \
  </div> \
  <button type='submit' class='btn btn-primary'>Save Changes</button> \
  <br> \
  <br> \
</form> \
</div> \
 <script type='text/javascript'> \
            $(function () { \
                $('#weekday_sleep_time').datetimepicker({ \
                    format: 'LT' \
                }); \
                 $('#weekday_time_alarm').datetimepicker({ \
                    format: 'LT' \
                }); \
                $('#weekend_sleep_time').datetimepicker({ \
                    format: 'LT' \
                }); \
                $('#weekend_time_alarm').datetimepicker({ \
                    format: 'LT' \
                }); \
            }); \
        </script> \
</body> \
</html>";


server.send ( 200, "text/html", temp );

}



// If an EPPROM write fails it commits changes and sends user to error page inviting to try again
void check_error_updating(bool EEPROM_write_res) {

  if (EEPROM_write_res == false) {
    EEPROM.commit();
    server.send ( 200, "text/html", "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'></head><body><h2>Something went wrong</h2><a href='/'>Start Again</a></body></html>" );
    
  }
  
}


// Write new user settings to EEPROM
// every variable starts every ten address positions (0, 10, 20, etc)
void handleUpdateSettings() {

  //update current state with new state user has set manually
  current_state = stringStatetoEnum(server.arg("new_state"));

  bool res; 
  res = write_StringEE(0,server.arg("weekday_sleep_time"));
  check_error_updating(res);
  res = write_StringEE(10,server.arg("weekday_time_alarm"));
  check_error_updating(res);
  res = write_StringEE(20,server.arg("weekday_sound_alarm"));
  check_error_updating(res);
  res = write_StringEE(30,server.arg("weekend_sleep_time"));
  check_error_updating(res);
  res = write_StringEE(40,server.arg("weekend_time_alarm"));
  check_error_updating(res);
  res = write_StringEE(50,server.arg("weekend_sound_alarm"));
  check_error_updating(res);

  EEPROM.commit();

  //Once user succesfully updates settings, delete all previous alarms and recreate them
  readSettings();
  disableAlarms();
  createAlarms();

  //Send user to succesful update page confirmation
  server.send ( 200, "text/html", "<html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'></head><body><h2>Settings succesfully updated</h2><a href='/'>Go Back</a></body></html>" );

}

// 404 HTML ERROR PAGE
void handleNotFound() {
  
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += ( server.method() == HTTP_GET ) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for ( uint8_t i = 0; i < server.args(); i++ ) {
    message += " " + server.argName ( i ) + ": " + server.arg ( i ) + "\n";
  }

  server.send ( 404, "text/plain", message );
}


//---------------------------------------- END WEBSERVER EEPROM CODE -----------------------------------------------------




//---------------------------------------- EEPROM UTILS ------------------------------------------------------------------



// EEPROM UTILS
//
// http://playground.arduino.cc/Code/EepromUtil  
//
// Absolute min and max eeprom addresses.
// Actual values are hardware-dependent.
//
// These values can be changed e.g. to protect
// eeprom cells outside this range.
//
const int EEPROM_MIN_ADDR = 0;
const int EEPROM_MAX_ADDR = 511;


//
// Writes a string starting at the specified address.
// Returns true if the whole string is successfully written.
// Returns false if the address of one or more bytes
// fall outside the allowed range.
// If false is returned, nothing gets written to the eeprom.
//
boolean eeprom_write_string(int addr, const char* string) {
  // actual number of bytes to be written
  int numBytes;

  // we'll need to write the string contents
  // plus the string terminator byte (0x00)
  numBytes = strlen(string) + 1;

  return eeprom_write_bytes(addr, (const byte*)string, numBytes);
}


//
// Reads a string starting from the specified address.
// Returns true if at least one byte (even only the
// string terminator one) is read.
// Returns false if the start address falls outside
// or declare buffer size os zero.
// the allowed range.
// The reading might stop for several reasons:
// - no more space in the provided buffer
// - last eeprom address reached
// - string terminator byte (0x00) encountered.
// The last condition is what should normally occur.
//
boolean eeprom_read_string(int addr, char* buffer, int bufSize) {
  // byte read from eeprom
  byte ch;

  // number of bytes read so far
  int bytesRead;

  // check start address
  if (!eeprom_is_addr_ok(addr)) {
    return false;
  }

  // how can we store bytes in an empty buffer ?
  if (bufSize == 0) {
    return false;
  }

  // is there is room for the string terminator only,
  // no reason to go further
  if (bufSize == 1) {
    buffer[0] = 0;
    return true;
  }

  // initialize byte counter
  bytesRead = 0;

  // read next byte from eeprom
  ch = EEPROM.read(addr + bytesRead);

  // store it into the user buffer
  buffer[bytesRead] = ch;

  // increment byte counter
  bytesRead++;

  // stop conditions:
  // - the character just read is the string terminator one (0x00)
  // - we have filled the user buffer
  // - we have reached the last eeprom address
  while ( (ch != 0x00) && (bytesRead < bufSize) && ((addr + bytesRead) <= EEPROM_MAX_ADDR) ) {
    // if no stop condition is met, read the next byte from eeprom
    ch = EEPROM.read(addr + bytesRead);

    // store it into the user buffer
    buffer[bytesRead] = ch;

    // increment byte counter
    bytesRead++;
  }

  // make sure the user buffer has a string terminator
  // (0x00) as its last byte
  if ((ch != 0x00) && (bytesRead >= 1)) {
    buffer[bytesRead - 1] = 0;
  }

  return true;
}

//
// Writes a sequence of bytes to eeprom starting at the specified address.
// Returns true if the whole array is successfully written.
// Returns false if the start or end addresses aren't between
// the minimum and maximum allowed values.
// When returning false, nothing gets written to eeprom.
//
boolean eeprom_write_bytes(int startAddr, const byte* array, int numBytes) {
  // counter
  int i;

  // both first byte and last byte addresses must fall within
  // the allowed range  
  if (!eeprom_is_addr_ok(startAddr) || !eeprom_is_addr_ok(startAddr + numBytes)) {
    return false;
  }

  for (i = 0; i < numBytes; i++) {
    EEPROM.write(startAddr + i, array[i]);
  }

  return true;
}


//
// Returns true if the address is between the
// minimum and maximum allowed values,
// false otherwise.
//
// This function is used by the other, higher-level functions
// to prevent bugs and runtime errors due to invalid addresses.
//
boolean eeprom_is_addr_ok(int addr) {
  return ((addr >= EEPROM_MIN_ADDR) && (addr <= EEPROM_MAX_ADDR));
}

//
// Dump eeprom memory contents over serial port.
// For each byte, address and value are written.
//
void eeprom_serial_dump_column() {
  // counter
  int i;

  // byte read from eeprom
  byte b;

  // buffer used by sprintf
  char buf[10];

  for (i = EEPROM_MIN_ADDR; i <= EEPROM_MAX_ADDR; i++) {
    b = EEPROM.read(i);
    sprintf(buf, "%03X: %02X", i, b);
    Serial.println(buf);
  }
}

// Credits
// http://mario.mtechcreations.com/programing/write-string-to-arduino-eeprom/
//


//Takes in a String and converts it to be used with the EEPROM Functions
//
bool write_StringEE(int Addr, String input)
{
    char cbuff[input.length()+1];//Finds length of string to make a buffer
    input.toCharArray(cbuff,input.length()+1);//Converts String into character array
    return eeprom_write_string(Addr,cbuff);//Saves String
}

//Given the starting address, and the length, this function will return
//a String and not a Char array
String read_StringEE(int Addr, int length)
{
  
  char cbuff[length+1];
  eeprom_read_string(Addr, cbuff, length+1);

  String stemp(cbuff);
  return stemp;
  
}




//---------------------------------------- END EEPROM UTILS ------------------------------------------------------------------

//-----------------------------------------  TIME UTILS ----------------------------------------------------------------------
void printTime(){
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(".");
  Serial.print(month());
  Serial.print(".");
  Serial.print(year()); 
  Serial.println(); 
}

void printDigits(int digits){
  // utility for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}


// Displays the current time in the LED Matrix in this format H1H2:m1m2 
void displayTime() {

 int H1,H2, m1, m2;
 char buf[2];
 int Hour = hour();  // the hour now (0-23)
 if (Hour < 10 ) {
  H1 = 0;
  H2 = Hour;
 } else {
  sprintf (buf, "%i", Hour);
  H1 = buf[0]-'0';  // -0 is to convert char to int
  H2 = buf[1]-'0';   // -0 is to convert char to int    
 };

 int Minute = minute();  // the minute now (0-59)
 if (Minute < 10 ) {
  m1 = 0;
  m2 = Minute;
 } else {
  sprintf (buf, "%i", Minute);
  m1 = buf[0]-'0';  // -0 is to convert char to int
  m2 = buf[1]-'0';   // -0 is to convert char to int    
 };

 // check if one of the integers is zero and allocate to the right array
 if (H1 == 0) H1= 10;
 if (H2 == 0) H2= 10;
 if (m1 == 0) m1= 10;
 if (m2 == 0) m2= 10;

  for (int i = 0; i < 8; i++)
       {
     lc.setRow(1,i, binaryArray[m2].array1[i]);
     lc.setRow(2,i, binaryArray[m1].array1[i]);

     //Adds automatically separation between hours and minutes
     if (i==3 || i==4) {
      lc.setRow(3,i, binaryArray[H2].array1[i] | (1<<0));
     }else {
      lc.setRow(3,i, binaryArray[H2].array1[i] );
     }
     lc.setRow(4,i, binaryArray[H1].array1[i]);
     
  }
}


// returns the hour integer for a given string time.
//  22:34 will return 22
// 09:34  will return 9
int getHour(String stringTime) {

  int H1 = stringTime.charAt(0)-'0';
  if (H1 == 0) {
    // return H2
    return stringTime.charAt(1)-'0';  
  } else {
    // retrun H1H2
    char res[2] = { stringTime.charAt(0), stringTime.charAt(1)}; 
    return atoi(res);
  }
}


// returns the minute integer for a given string time.
//  22:34 will return 34
//  22:04  will return 4
int getMinute(String stringTime) {

  int m1 = stringTime.charAt(3)-'0';
  if (m1 == 0) {
    // return m2
    return stringTime.charAt(4)-'0';  
  } else {
    // retrun m1m2
    char res[2] = { stringTime.charAt(3), stringTime.charAt(4)}; 
    return atoi(res);
  }
}




// --------------------------------------- END TIME UTILS --------------------------------------------------------------------

