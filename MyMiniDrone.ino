#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <EEPROM.h>

#define PPM_MIN 1000
#define PPM_SAFE_THROTTLE 1050 
#define PPM_MID 1500
#define PPM_MAX 2000
#define PPM_MIN_COMMAND 1300
#define PPM_MAX_COMMAND 1700
#define GET_FLAG(ch, mask) (ppm[ch] > PPM_MAX_COMMAND ? mask : 0)
#define GET_FLAG_INV(ch, mask) (ppm[ch] < PPM_MIN_COMMAND ? mask : 0)

RF24 radio(8,10);

static bool reset = true;
u8        ledPin = 2;

uint8_t packet[32];
static uint16_t ppm[12] = {PPM_MIN,PPM_MIN,PPM_MIN,PPM_MIN,PPM_MID,PPM_MID,
                           PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,};
enum chan_order{
    THROTTLE,
    AILERON,
    ELEVATOR,
    RUDDER,
    AUX1,  // (CH5)  led light, or 3 pos. rate on CX-10, H7, or inverted flight on H101
    AUX2,  // (CH6)  flip control
    AUX3,  // (CH7)  still camera (snapshot)
    AUX4,  // (CH8)  video camera
    AUX5,  // (CH9)  headless
    AUX6,  // (CH10) calibrate Y (V2x2), pitch trim (H7), RTH (Bayang, H20), 360deg flip mode (H8-3D, H22)
    AUX7,  // (CH11) calibrate X (V2x2), roll trim (H7)
    AUX8,  // (CH12) Reset / Rebind
};

void setup() {
  Serial.begin(115200);

  //#### RADIO SETUP
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  printf_begin(); // Opens standard output stream using SPI

  //#### Arduino Power
  pinMode(9, OUTPUT); //enable power in seeedrino board
  digitalWrite(9, HIGH); //enable power in seeedrino board
  pinMode(ledPin, OUTPUT);
  randomSeed((analogRead(A4) & 0x1F) | (analogRead(A5) << 5));
}

void loop() {
  if(reset && radio.isChipConnected()){
    mjx_init();
    mjx_bind();
  }
}    
