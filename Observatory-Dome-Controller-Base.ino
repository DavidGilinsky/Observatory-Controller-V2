// #include <SoftPWM.h>
#include <SPI.h> // Serial communication
#include <NRFLite.h> //RF Transceiver
#include <Wire.h> //General purpose IO
#include <Adafruit_GFX.h> // Graphics 
#include <Adafruit_SSD1306.h> / Display

#include <EEPROM.h> 
#include <avr/wdt.h>
#include <SoftwareSerial.h>

#include "serial_command.h"



#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32


// Configuration
//#define HAS_SHUTTER     // Uncomment if the shutter controller is available
//#define MOTOR_SHIELD  // Uncomment if the motor driver is a Monster Motor Shield
//#define USE_BUTTONS   // Uncomment if you want to move the dome with push buttons

#define AZ_TIMEOUT      30000   // Azimuth movement timeout (in ms)
#define AZ_TOLERANCE    5        // Azimuth target tolerance in encoder ticks
#define AZ_SLOW_RANGE   10       // The motor will run at slower speed when the
                                // dome is at this number of ticks from the target

// pin definitions
#define ENCODER1 2      // Encoder
#define ENCODER2 4      // Encoder
#define HOME_SENSOR 3  // Home sensor (active low)
#define BUTTON_CW   11  // CW movement button (active low)
#define BUTTON_CCW  12  // CCW movement button (Active low)
#define MOTOR_ENABLE 5    // Going to use the enable pin for PWM to control dome rotation speed
#define MOTOR_CW 7      // Move motor clockwise
#define MOTOR_CCW 6     // Move motor counterclockwise

#ifdef HAS_SHUTTER
//#ifdef MOTOR_SHIELD
//#error "HAS_SHUTTER and MOTOR_SHIELD cannot be defined at the same time"
//#endif

// pins of HC12 module (serial radio transceiver)
//#define HC12_RX 4       // Receive Pin on HC12
//#define HC12_TX 5       // Transmit Pin on HC12
#endif





// Message Destination
#define TO_MAXDOME  0x00
#define TO_COMPUTER 0x80

#define VBAT_FACTOR (5.0/1024.0)
#define VBAT_OFFSET 10.55

#define BAUDRATE 19200

// Commands
#define ABORT_CMD   0x03 // Abort azimuth movement
#define HOME_CMD    0x04 // Move until 'home' position is detected
#define GOTO_CMD    0x05 // Go to azimuth position
#define SHUTTER_CMD 0x06 // Send a command to shutter
#define STATUS_CMD  0x07 // Retrieve status
#define TICKS_CMD   0x09 // Set the number of tick per revolution of the dome
#define ACK_CMD     0x0A // ACK (?)
#define SETPARK_CMD 0x0B // Set park coordinates and shutter closing policy
#define VBAT_CMD    0x0C // Read shutter's battery voltage

// Shutter commands
#define OPEN_SHUTTER            0x01
#define OPEN_UPPER_ONLY_SHUTTER 0x02
#define CLOSE_SHUTTER           0x03
#define EXIT_SHUTTER            0x04 // Command sent to shutter on program exit
#define ABORT_SHUTTER           0x07

#define DIR_CW  0x01
#define DIR_CCW 0x02

// EEPROM addresses
#define ADDR_PARK_POS           0
#define ADDR_TICKS_PER_TURN     2
#define ADDR_PARK_ON_SHUTTER    4

// Dome rotation speeds in percentage duty cycle
#define DOME_STOP               0
#define DOME_SLOW               75
#define DOME_FAST               100



const static uint8_t RADIO_ID = 0;       // Our radio's id.  The transmitter will send to this id.
const static uint8_t PIN_RADIO_CE = 9;
const static uint8_t PIN_RADIO_CSN = 10;

struct RadioPacket // Any packet up to 32 bytes can be sent.
{
    uint8_t FromRadioId;
    uint8_t systemCal;
    uint8_t magCal;
    uint16_t battery;
    uint32_t domeAzimuthA; //primary mag sensor
};


enum MotorSpeed { MOTOR_STOP, MOTOR_SLOW, MOTOR_FAST };

enum AzimuthEvent {
    EVT_NONE,
    EVT_GOTO,
    EVT_HOME,
    EVT_ABORT
};

enum AzimuthStatus {
    ST_IDLE,
    ST_MOVING,
    ST_APPROACHING,
    ST_HOMING,
    ST_ERROR,
};

// MaxDome II azimuth status
enum MDAzimuthStatus {
    AS_IDLE = 1,
    AS_MOVING_CW,
    AS_MOVING_CCW,
    AS_IDLE2,
    AS_ERROR
};

// MaxDome II shutter status
enum ShutterStatus {
    SS_CLOSED = 0,
    SS_OPENING,
    SS_OPEN,
    SS_CLOSING,
    SS_ABORTED,
    SS_ERROR
};


#ifdef HAS_SHUTTER
// Create a Software Serial Port to communicate with the shutter controller
SoftwareSerial HC12(HC12_TX, HC12_RX);
#endif

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

NRFLite _radio;
RadioPacket _radioData;

SerialCommand sCmd;



bool park_on_shutter = false;
bool home_reached = false;
bool parking = false;
bool magDataValid = false;      // Status of the mag sensor
bool azSensorAlive = false;     // Receiving data from the dome azimith sensor
bool encoderOverMagnet;

uint8_t current_dir = DIR_CW;   // Current azimuth movement direction
uint16_t park_pos = 0;          // Parking position
uint16_t current_pos = 0;       // Current dome position in degrees (used to be ticks)
uint16_t encoder_pos = 0;       // Current dome position in actual encoder ticks. This gets updated by the Hal sensor interrupt service routine
uint16_t encoder_az = 0;         // Current dome encoder position in degrees
uint16_t mag_heading = 0;        // Heading data from magnetic sensor + compass_offset
float degrees_per_tick;          // Dome rotation per encoder tick
uint16_t target_pos = 0;        // Target dome position
uint16_t home_pos = 0;          // Home position
uint16_t ticks_per_turn = 360;  // No longer useful - Used to be encoder ticks via MaxDomeII protocl. Now we take input in degrees and always set this to 360.
uint16_t encoder_resolution = 73; // Actual magnetic rotary encoder ticks per turn. This will be the backup for mag azimuth sensor failure.
uint32_t timeout_count_ms = 0;
uint32_t ms = 0;
uint16_t compass_offset = 17;   // offset between dome azimuth sensor magnetic N and dome home sensor position

AzimuthStatus state = ST_IDLE;
AzimuthEvent az_event = EVT_NONE;


// Convert two bytes to a uint16_t value (big endian)
uint16_t bytesToInt(uint8_t *data)
{
    uint16_t value1 = data[1];
    uint16_t value2 = data[0];
    return (value1 & 0xff) | ((value2 << 8) & 0xff00);
}

// Convert a uint16_t value to bytes (big endian)
void intToBytes(uint16_t value, uint8_t *data)
{
    data[1] = value & 0xff;
    data[0] = (value >> 8) & 0xff;
}

// Read a uint16_t value from EEPROM (little endian)
uint16_t eepromReadUint16(int address)
{
    uint16_t value1 = EEPROM.read(address);
    uint16_t value2 = EEPROM.read(address + 1);
    return (value1 & 0xff) | ((value2 << 8) & 0xff00);
}

// Read a uint16_t value from EEPROM (little endian)
void eepromWriteUint16(int address, uint16_t value)
{
    uint8_t value1 = value & 0xff;
    uint8_t value2 = (value >> 8) & 0xff;
    EEPROM.write(address, value1);
    EEPROM.write(address + 1, value2);
}


// Obtain the direction of the shortest path between two positons
uint8_t getDirection(uint16_t current, uint16_t target)
{
    if (target > current)
        return ((target - current) > ticks_per_turn / 2) ? DIR_CCW : DIR_CW;

    return ((current - target) > ticks_per_turn / 2) ? DIR_CW : DIR_CCW;
}

// Obtain the distance between two positons
uint16_t getDistance(uint16_t current, uint16_t target)
{
    // obtain the absolute value of the difference
    uint16_t diff = (target > current) ?  target - current : current - target;

    if (diff > ticks_per_turn / 2)
        return ticks_per_turn - diff;

    return diff;
}

uint16_t addDegrees(uint16_t a, uint16_t b)
{
  uint16_t result = a+b;
  if(result>=360) result=result-360;
  return result;
}

inline void moveAzimuth(uint8_t dir, int dome_speed)
{
    digitalWrite(LED_BUILTIN, HIGH);

//    digitalWrite(MOTOR_ENABLE, HIGH);
    digitalWrite(MOTOR_CW, dir == DIR_CW);
    digitalWrite(MOTOR_CCW, dir != DIR_CW);
    analogWrite(MOTOR_ENABLE, 2.55*dome_speed);
//    SoftPWMSetPercent(MOTOR_ENABLE, dome_speed);

}

inline void stopAzimuth()
{
    digitalWrite(LED_BUILTIN, LOW);

//    SoftPWMSet(MOTOR_ENABLE, DOME_STOP);
    digitalWrite(MOTOR_ENABLE, LOW);
    digitalWrite(MOTOR_CW, LOW);
    digitalWrite(MOTOR_CCW, LOW);
    

}

float getShutterVBat()
{
    int adc = 0;

#ifdef HAS_SHUTTER
    char buffer[5];

    HC12.flush();
    for (int i = 0; i < 4; i++) {
        HC12.println("vbat");
        delay(100);

        if (HC12.read() == 'v') {
            for (int j = 0; j < 4; j++) {
                buffer[j] = HC12.read();
            }

            buffer[4] = 0;
            adc = atoi(buffer);
            break;
        }
    }
#endif

    // Convert ADC reading to voltage
    return (float)adc * VBAT_FACTOR + VBAT_OFFSET;
}

ShutterStatus getShutterStatus()
{
    ShutterStatus st = SS_OPEN;

#ifdef HAS_SHUTTER
    st = SS_ERROR;
    HC12.flush();
    for (int i = 0; i < 4; i++) {
        HC12.println("stat");
        delay(100);

        char c = 0;
        while (HC12.available() > 0) {
            c = HC12.read();
        }

        if (c >= '0' && c <= ('0' + SS_ERROR)) {
            st = (ShutterStatus)(c - '0');
            break;
        }
    }
#endif
    return st;
}


void cmdAbort(uint8_t *cmd)
{
#ifdef HAS_SHUTTER
    HC12.println("abort");  // abort shutter movement
#endif

    az_event = EVT_ABORT;

    uint8_t resp[] = {START, 2, TO_COMPUTER | ABORT_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdHomeAzimuth(uint8_t *cmd)
{
    if (!parking) {
        current_dir = getDirection(current_pos, 0);
        az_event = EVT_HOME;
    }

    uint8_t resp[] = {START, 3, TO_COMPUTER | HOME_CMD, 0x01, 0x00};
    sCmd.sendResponse(resp, 5);
}

void cmdGotoAzimuth(uint8_t *cmd)
{
    if (!parking) {
        current_dir = cmd[3];
        target_pos = bytesToInt(cmd + 4);
        az_event = EVT_GOTO;
    }

    uint8_t resp[] = {START, 3, TO_COMPUTER | GOTO_CMD, 0x01, 0x00};
    sCmd.sendResponse(resp, 5);
}

void parkDome()
{
    parking = true;
    target_pos = park_pos;
    az_event = EVT_GOTO;
}

void cmdShutterCommand(uint8_t *cmd)
{
#ifdef HAS_SHUTTER
    switch(cmd[3]) {
    case OPEN_SHUTTER:
        HC12.println("open");
        break;
    case OPEN_UPPER_ONLY_SHUTTER:
        HC12.println("open1");
        break;
    case CLOSE_SHUTTER:
        if (park_on_shutter)
            parkDome();
        else
            HC12.println("close");
        break;
    case EXIT_SHUTTER:
        HC12.println("exit");
        break;
    case ABORT_SHUTTER:
        HC12.println("abort");
        break;
    }
#endif

    uint8_t resp[] = {START, 2, TO_COMPUTER | SHUTTER_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdStatus(uint8_t *cmd)
{
    MDAzimuthStatus az_status;
    if (state == ST_IDLE) {
        az_status = AS_IDLE;
    }
    else if (state == ST_ERROR) {
        az_status = AS_ERROR;
    }
    else {
        if (current_dir == DIR_CW)
            az_status = AS_MOVING_CW;
        else
            az_status = AS_MOVING_CCW;
    }

    uint8_t sh_status = (uint8_t)getShutterStatus();
    uint8_t resp[] = {START, 9, TO_COMPUTER | STATUS_CMD, sh_status, az_status,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                     };

    intToBytes(current_pos, resp + 5);
    intToBytes(home_pos, resp + 7);

    sCmd.sendResponse(resp, 11);
}

void cmdSetPark(uint8_t *cmd)
{
    park_on_shutter = cmd[3];
    park_pos = bytesToInt(cmd + 4);

    EEPROM.write(ADDR_PARK_ON_SHUTTER, park_on_shutter);
    eepromWriteUint16(ADDR_PARK_POS, park_pos);

    uint8_t resp[] = {START, 2, TO_COMPUTER | SETPARK_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdSetTicks(uint8_t *cmd)
{
    ticks_per_turn = bytesToInt(cmd + 3);
    eepromWriteUint16(ADDR_TICKS_PER_TURN, ticks_per_turn);

    uint8_t resp[] = {START, 2, TO_COMPUTER | TICKS_CMD, 0x00};
    sCmd.sendResponse(resp, 4);
}

void cmdVBat(uint8_t *cmd)
{
    uint8_t resp[] = {START, 4, TO_COMPUTER | VBAT_CMD, 0x00, 0x00, 0x00};

    int vbat = getShutterVBat() * 100;
    intToBytes(vbat, resp + 3);

    sCmd.sendResponse(resp, 6);
}

void cmdAck(uint8_t *cmd)
{
    uint8_t resp[] = {START, 3, TO_COMPUTER | ACK_CMD, 0x03, 0x00};
    sCmd.sendResponse(resp, 5);
}

void updateAzimuthFSM()
{
    static unsigned long t0;

    switch(state) {
    case ST_HOMING:
        if (az_event == EVT_ABORT) {
            parking = false;
            stopAzimuth();
            state = ST_IDLE;
        }
        else if (home_reached) {
            stopAzimuth();
            state = ST_IDLE;
            home_reached = false;
        }
        else if (millis() - t0 > AZ_TIMEOUT) {
            stopAzimuth();
            state = ST_ERROR;
        }
        break;

    case ST_MOVING:
        if (az_event == EVT_ABORT) {
            parking = false;
            stopAzimuth();
            state = ST_IDLE;
        }
        else if (getDistance(current_pos, target_pos) <= AZ_SLOW_RANGE) {
            moveAzimuth(current_dir, DOME_SLOW);
            state = ST_APPROACHING;
        }
        else if (millis() - t0 > AZ_TIMEOUT) {
            stopAzimuth();
            state = ST_ERROR;
        }
        break;

    case ST_APPROACHING:
        if (az_event == EVT_ABORT) {
            parking = false;
            stopAzimuth();
            state = ST_IDLE;
        }
        else if (getDistance(current_pos, target_pos) <= AZ_TOLERANCE) {
            stopAzimuth();

            // close shutter after parking
            if (parking) {
                parking = false;
#ifdef HAS_SHUTTER
                HC12.println("close");
#endif
            }

            state = ST_IDLE;
        }
        else if (millis() - t0 > AZ_TIMEOUT) {
            stopAzimuth();
            state = ST_ERROR;
        }
        break;

    case ST_ERROR:
    case ST_IDLE:
        if (az_event == EVT_HOME) {
            t0 = millis();
            state = ST_HOMING;
            moveAzimuth(current_dir, DOME_FAST);
        }
        else if (az_event == EVT_GOTO && getDistance(current_pos, target_pos)>AZ_TOLERANCE) {
            t0 = millis();
            state = ST_MOVING;
            moveAzimuth(current_dir, DOME_FAST);
        }
        break;
    }
    az_event = EVT_NONE;
}

// Encoder interrupt service routine
void encoderISR()
{

// CW Rotation
// Leading endge of magnet
//    Encoder1 = 1, Encoder2=1
// Trailing edge of magnet
//    Encoder1=0, Encoder2=0
//
// CCW Rotation
// Leading edge of magnet
//    Encoder1=1, Encoder2=0
// Trailing edge of magnet
//    Encoder1=0, Encoder2=1
//
// Angular diameter of magnet = 50 degrees on gear, 4.1 degrees on dome

  if(digitalRead(ENCODER1) && digitalRead(ENCODER2)) { // CW - Trailing edge
    encoderOverMagnet=0;
//  Serial.println("CW - TE");
  } 
  if(!digitalRead(ENCODER1) && !digitalRead(ENCODER2)) { // CW - Leading Edge
    encoderOverMagnet=1;
//  Serial.println("CW - LE");
    if(encoder_pos==0)
//    current_pos=360-(360/ticks_per_turn);
    encoder_pos=encoder_resolution -1;
  else
//    current_pos=current_pos-(360/ticks_per_turn);
    encoder_pos--;
  }
  if(digitalRead(ENCODER1) && !digitalRead(ENCODER2)) { // CCW - Trailing edge
    encoderOverMagnet=0;
//  Serial.println("CCW - TE");
//        if (current_pos >= 360-(360/ticks_per_turn)) {
        if(encoder_pos >= encoder_resolution -1) {
            encoder_pos = 0;
        }
        else
//            current_pos=current_pos+(360/ticks_per_turn);
        encoder_pos++;
  } 
  if(!digitalRead(ENCODER1) && digitalRead(ENCODER2)) {   // CCW - Leading Edge
    encoderOverMagnet=1;
//  Serial.println("CCW - LE");
  }

  current_pos = encoder_pos*degrees_per_tick;

}

void messageDisplay(char *message)
{
  int messageWidth = strlen(message);
  int fontSize=1;

  
  if(messageWidth<=4) fontSize=5;
  if(messageWidth>4 && messageWidth <=6) fontSize=4;
  if(messageWidth>6 && messageWidth <=8) fontSize = 3;
  if(messageWidth>8 && messageWidth <=12) fontSize = 2;
  
  display.setTextSize(fontSize);
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();
  display.setCursor(0,0);
  display.print(message);
  display.display();
}

float getMagHeading()
{
  int ms = millis(); //start timeout counter
  while(millis()-ms < 5000) //wait 5 seconds for Azimuth data by radio
  {
    while(_radio.hasData())
    {
//      Serial.println("Radio has data");
      _radio.readData(&_radioData);
      if(_radioData.FromRadioId==1)
      {
        timeout_count_ms = ms = millis();
        azSensorAlive = true;
        mag_heading = addDegrees(_radioData.domeAzimuthA, compass_offset);
        break;
//        Serial.println("Radio data from AZ sensor");
        if(_radioData.systemCal >0)
        {
          magDataValid = 1;
          
          current_pos = mag_heading;
        } else
        {
          magDataValid = 0;
          current_pos=encoder_pos*degrees_per_tick;
        }
      }
    }
  }
  if(azSensorAlive)
  {
    return(mag_heading);
  } else
  {
    return(-1);
  }
}

int goHome()
{
  float heading=getMagHeading();
  int direction;
  if(heading != -1)
  {
    direction=getDirection((int)heading, 0);
  } else
  {
    direction = DIR_CCW;
  }

  moveAzimuth(direction, DOME_FAST);

  while(digitalRead(HOME_SENSOR))
  {

  }

  stopAzimuth();

  return(1);
}

// Use the high precision magnetic compass in the Dome Azimuth Sensor to callibrate the drive wheel rotary encoder
int callibrateEncoder()
{

  digitalWrite(MOTOR_ENABLE, 0); // Make sure the dome is not rotating
  if(digitalRead(HOME_SENSOR)) // Make sure we start at the home position. Home sensor is active low
  {
    messageDisplay("Homing");
    if(!goHome())
    {
      messageDisplay("CAL ERROR");
      delay(5000);
      return(0);
    }
  }

  

}

void setup()
{
    wdt_disable();
    wdt_enable(WDTO_2S);

    sCmd.addCommand(ABORT_CMD, 2, cmdAbort);
    sCmd.addCommand(HOME_CMD, 2, cmdHomeAzimuth);
    sCmd.addCommand(GOTO_CMD, 5, cmdGotoAzimuth);
    sCmd.addCommand(SHUTTER_CMD, 3, cmdShutterCommand);
    sCmd.addCommand(STATUS_CMD, 2, cmdStatus);
    sCmd.addCommand(SETPARK_CMD, 5, cmdSetPark);
    sCmd.addCommand(TICKS_CMD, 4, cmdSetTicks);
    sCmd.addCommand(ACK_CMD, 2, cmdAck);
    sCmd.addCommand(VBAT_CMD, 2, cmdVBat);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(HOME_SENSOR, INPUT_PULLUP);
    pinMode(BUTTON_CW, INPUT_PULLUP);
    pinMode(BUTTON_CCW, INPUT_PULLUP);
    pinMode(ENCODER1, INPUT_PULLUP);
    pinMode(ENCODER2, INPUT_PULLUP);
//    pinMode(MOTOR_ENABLE, OUTPUT);
    pinMode(MOTOR_CW, OUTPUT);
    pinMode(MOTOR_CCW, OUTPUT);
    digitalWrite(MOTOR_ENABLE,0); // Diable azimuth motor
    pinMode(8, INPUT); // This is a workaround for a mistake in schematic v22 and PCB v39 in which digital 8 was used for motor enable PWM. The nano every doesn't use pin 8 for PWM
                      // so pin 5 was jumpered to pin 8 and pin 8 set to high impedance (input)

    attachInterrupt(digitalPinToInterrupt(ENCODER1), encoderISR, CHANGE); // Trigger interrupt on Hal effect sensor number one to increment or decrement encoder_pos

    park_pos = eepromReadUint16(ADDR_PARK_POS);
    park_on_shutter = EEPROM.read(ADDR_PARK_ON_SHUTTER);
//    ticks_per_turn = eepromReadUint16(ADDR_TICKS_PER_TURN);

    degrees_per_tick = 360.0/encoder_resolution;
    
    Serial.begin(19200);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
      } 
        Serial.println("Display object initialized");
        display.display();
        display.clearDisplay();
        messageDisplay("Hello");
        delay(5000);

   if (!_radio.init(RADIO_ID, PIN_RADIO_CE, PIN_RADIO_CSN))
    {
        Serial.println("Cannot communicate with radio");
        messageDisplay("AZ RF FAIL");
        while (1); // Wait here forever.
    }


#ifdef HAS_SHUTTER
    HC12.begin(9600);   // Open serial port to HC12
#endif
}

// move the motor when the buttons are pressed
void read_buttons()
{
    static int prev_cw_button = 0, prev_ccw_button = 0;
    int cw_button = !digitalRead(BUTTON_CW);
    int ccw_button = !digitalRead(BUTTON_CCW);

    if (cw_button != prev_cw_button) {
        if (cw_button) {
            digitalWrite(LED_BUILTIN, HIGH);
      moveAzimuth(DIR_CW, DOME_FAST);
        }
        else {
            digitalWrite(LED_BUILTIN, LOW);
      stopAzimuth();
        }
    }
    else if (ccw_button != prev_ccw_button) {
        if (ccw_button) {
            digitalWrite(LED_BUILTIN, HIGH);
      moveAzimuth(DIR_CCW, DOME_FAST);
        }
        else {
            digitalWrite(LED_BUILTIN, LOW);
      stopAzimuth();
        }
    }
    prev_cw_button = cw_button;
    prev_ccw_button = ccw_button;
}


void loop()
{


  if(!azSensorAlive)
  {
    current_pos=encoder_pos*degrees_per_tick;
  }

    encoder_az = encoder_pos * degrees_per_tick;
    
    sCmd.readSerial();
    while(_radio.hasData())
    {
//      Serial.println("Radio has data");
      _radio.readData(&_radioData);
      if(_radioData.FromRadioId==1)
      {
        timeout_count_ms = ms = millis();
        azSensorAlive = true;
        mag_heading = addDegrees(_radioData.domeAzimuthA, compass_offset);
//        Serial.println("Radio data from AZ sensor");
        if(_radioData.systemCal >0)
        {
          magDataValid = 1;
          
          current_pos = mag_heading;
        } else
        {
          magDataValid = 0;
          current_pos=encoder_pos*degrees_per_tick;
        }
      }
    }

    if(millis()-timeout_count_ms > 750) // If we haven't heardf from the azimuth sensor in 750ms mark it dead and invalidate the magheading data
    {
      azSensorAlive = false;
      magDataValid = false;
    }
        

        display.setTextSize(3);             // Draw 2X-scale text
        display.setTextColor(SSD1306_WHITE);
        display.clearDisplay();
        display.setCursor(0,0);
        display.print(current_pos);
        display.setCursor(60,0);
        if(azSensorAlive)
        {
          display.print(mag_heading);
//        display.print(getDistance(current_pos, target_pos));
        } else
        {
          display.print("NO");
        }
        display.setTextSize(1);
        display.setCursor(0,25);
        display.print(encoder_pos);
        display.setCursor(60,25);
        if(_radioData.systemCal>0) {
          display.fillRect(58,25,20,10,SSD1306_WHITE);
          display.setTextColor(SSD1306_BLACK);
          display.print(_radioData.systemCal);
        } else {
          display.fillRect(60,25,5,8,SSD1306_BLACK);
          display.setTextColor(SSD1306_WHITE);
          display.print(_radioData.systemCal);
        }
        display.display();


    
    updateAzimuthFSM();
#ifdef USE_BUTTONS
    read_buttons();
#endif
    wdt_reset();

    // store detected home position
    if (!digitalRead(HOME_SENSOR)) {
        if (state == ST_HOMING) {
            home_pos = 0;
            current_pos = 0;
            if(azSensorAlive && magDataValid)
            {
              compass_offset=getDistance(_radioData.domeAzimuthA, 0);
              if(getDirection(0,_radioData.domeAzimuthA)==DIR_CCW)
              {
                compass_offset=compass_offset*-1;
              }
            }
            home_reached = true;
        }
        else
//            Serial.println("home");
            current_pos=0;
            encoder_pos=0;
            home_pos = current_pos;
    }
}
