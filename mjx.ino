#define MJX_RF_NUM_CHANNELS 4
#define MJX_BIND_COUNT      150
#define MJX_PACKET_PERIOD   4000 // Timeout for callback in uSec
#define MJX_PACKET_SIZE     16

#define MJX_CHANNEL_LED         AUX1
#define MJX_CHANNEL_FLIP        AUX2
#define MJX_CHANNEL_PICTURE     AUX3
#define MJX_CHANNEL_VIDEO       AUX4
#define MJX_CHANNEL_HEADLESS    AUX5
#define MJX_CHANNEL_RTH         AUX6
#define MJX_CHANNEL_AUTOFLIP    AUX7  // X800, X600
#define MJX_CHANNEL_PAN         AUX7  // H26D
#define MJX_CHANNEL_TILT        AUX8

#define CHAN2TRIM(X) (((X) & 0x80 ? (X) : 0x7f - (X)) >> 1)

static u8 mjx_rf_channels[MJX_RF_NUM_CHANNELS];
static u8 mjx_rf_channel;
static u16 mjx_counter;
static u8 mjx_txid[3];

u8 mjx_checksum()
{
    u8 sum = packet[0];
    for (int i=1; i < MJX_PACKET_SIZE-1; i++) sum += packet[i];
    return sum;
}

// Channel values are sign + magnitude 8bit values
u8 mjx_convert_channel(u8 num)
{
    u8 val = map(ppm[num], PPM_MIN, PPM_MAX, 0, 255);
    return (val < 128 ? 127-val : val);  
}

void mjx_send_packet(u8 bind){
  packet[0] = map(ppm[THROTTLE], PPM_MIN, PPM_MAX, 0, 255);
  packet[1] = mjx_convert_channel(RUDDER);          // rudder
  packet[4] = 0x40;         // rudder does not work well with dyntrim
  packet[2] = mjx_convert_channel(ELEVATOR);   // elevator
  // driven trims cause issues when headless is enabled
  packet[5] = GET_FLAG(MJX_CHANNEL_HEADLESS, 1) ? 0x40 : CHAN2TRIM(packet[2]); // trim elevator
  packet[3] = mjx_convert_channel(AILERON);          // aileron
  packet[6] = GET_FLAG(MJX_CHANNEL_HEADLESS, 1) ? 0x40 : CHAN2TRIM(packet[3]); // trim aileron
  packet[7] = mjx_txid[0];
  packet[8] = mjx_txid[1];
  packet[9] = mjx_txid[2];
  
  packet[10] = 0;   // overwritten below for feature bits
  packet[11] = 0;   // overwritten below for X600
  packet[12] = 0;
  packet[13] = 0;
  
  packet[14] = 0xc0;  // bind value
  packet[10] += GET_FLAG(MJX_CHANNEL_RTH, 0x02)
                        | GET_FLAG(MJX_CHANNEL_HEADLESS, 0x01);
  packet[15] = mjx_checksum();

  radio.setChannel( mjx_rf_channels[mjx_rf_channel++ / 2] );
  mjx_rf_channel %= 2 * sizeof(mjx_rf_channels);
  
  radio.flush_tx();
  Serial.println("########SENDING STATUS#######");
  radio.printDetails();
  delay(100);
  if(radio.write(packet, 16)){
    Serial.println("%%%%%%%SENT%%%%%%%");
  }
}

void mjx_bind(){
  mjx_counter = MJX_BIND_COUNT;
  while(mjx_counter--) {
    mjx_send_packet(1);
    delayMicroseconds(MJX_PACKET_PERIOD);
    digitalWrite(ledPin, mjx_counter & 0x10);
  }
  mjx_init2();
  digitalWrite(ledPin, HIGH);    
}

void mjx_init2(){
}

static const struct {
    u8 e010_txid[2];
    u8 rfchan[MJX_RF_NUM_CHANNELS];
}
e010_tx_rf_map[] = {{{0x4F, 0x1C}, {0x3A, 0x35, 0x4A, 0x45}},
                   {{0x90, 0x1C}, {0x2E, 0x36, 0x3E, 0x46}}, 
                   {{0x24, 0x36}, {0x32, 0x3E, 0x42, 0x4E}},
                   {{0x7A, 0x40}, {0x2E, 0x3C, 0x3E, 0x4C}},
                   {{0x61, 0x31}, {0x2F, 0x3B, 0x3F, 0x4B}},
                   {{0x5D, 0x37}, {0x33, 0x3B, 0x43, 0x4B}},
                   {{0xFD, 0x4F}, {0x33, 0x3B, 0x43, 0x4B}}, 
                   {{0x86, 0x3C}, {0x34, 0x3E, 0x44, 0x4E}}};
                   
uint8_t transmitterID[4];
enum{
    ee_PROTOCOL_ID = 0,
    ee_TXID0,
    ee_TXID1,
    ee_TXID2,
    ee_TXID3
};
                   
void set_txid(bool renew)
{
    uint8_t i;
    for(i=0; i<4; i++)
        transmitterID[i] = EEPROM.read(ee_TXID0+i);
    if(renew || (transmitterID[0]==0xFF && transmitterID[1]==0x0FF)) {
        for(i=0; i<4; i++) {
            transmitterID[i] = random() & 0xFF;
            EEPROM.update(ee_TXID0+i, transmitterID[i]); 
        }            
    }
}
                  
void initialize_mjx_txid(){
   memcpy(mjx_txid, e010_tx_rf_map[transmitterID[0] % (sizeof(e010_tx_rf_map)/sizeof(e010_tx_rf_map[0]))].e010_txid, 2);
        mjx_txid[2] = 0x00;
  
}
void mjx_init()
{
  u8 rx_tx_addr[5];
  memcpy(mjx_rf_channels,"\x36\x3e\x46\x2e", MJX_RF_NUM_CHANNELS);
  set_txid(true);
  initialize_mjx_txid();
  for(u8 i=0; i<3 ; i++) Serial.println(mjx_txid[i], HEX);
  reset=false;
  radio.txStandBy(); 
  delay(5);
  radio.setCRCLength(RF24_CRC_16);
  uint8_t buf[] = { 0x55, 0x0F, 0x71, 0x0C, 0x00 }; // bytes for XN297 preamble 0xC710F55 (28 bit)
  radio.powerUp();
  //radio.setAddressWidth(5);
  radio.setPayloadSize(16);
  radio.openWritingPipe(buf);
  delay(5);
  radio.flush_rx();
  radio.flush_tx();
  radio.setAutoAck(false);
  radio.setRetries(0,0); // 250us delay + disable retries 
  radio.setDataRate(RF24_1MBPS);
  radio.startListening();
  radio.stopListening();
  radio.printDetails();
}
