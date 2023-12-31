#include "LoRaWan_APP.h"
#include "Arduino.h"


#define RF_FREQUENCY                                915000000 // Hz

#define TX_OUTPUT_POWER                             5        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                             //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 30 // Define the payload size here

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

double txNumber;
int16_t rssi,rxSize;
bool lora_idle=true;

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );

void setup() {
    Serial.begin(9600);
    Mcu.begin();
  
    txNumber=0;
    rssi=0;
  
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 ); 
   }


String droneCoords = "";//example coords (long, lat)
String phoneCoords = "";
bool started = false;
void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}
void loop()
{
  //Serial.println(Serial.available());
  if(started){
        while(Serial.available()) { 
    char data_rcvd = Serial.read();   // read one byte from serial buffer and save to data_rcvd
    if(data_rcvd == ' '){
      break;
    }
droneCoords += data_rcvd;
//Serial.println(data_rcvd);
  }
  if(droneCoords.length() > 30){
    Serial.println(droneCoords);
    droneCoords = "";
    serialFlush();
    return;
  }
  if(lora_idle == true && droneCoords != "")
  {
    Serial.println(droneCoords);
    txNumber += 0.01;
    sprintf(txpacket,droneCoords.c_str());  //start a package
    droneCoords = "";
    Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",txpacket, strlen(txpacket));

    Radio.Send( (uint8_t *)txpacket, strlen(txpacket) ); //send the package out 
    lora_idle = false;
  }
  Radio.IrqProcess( );
  }
  else{
      if(lora_idle)
  {
    lora_idle = false;
    Radio.Rx(0);
  }
  Radio.IrqProcess( );
  }
}

void OnTxDone( void )
{
  //Serial.println("TX done......");
  lora_idle = true;
}
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    rssi=rssi;
    rxSize=size;
    memcpy(rxpacket, payload, size );
    phoneCoords = String(rxpacket);
    Serial.println(phoneCoords);
      if(phoneCoords != ""){
    started = true;
  }
    rxpacket[size]='\0';
    Radio.Sleep( );
    //Serial.printf("\r\nreceived packet \"%s\" with rssi %d , length %d\r\n",rxpacket,rssi,rxSize);
    lora_idle = true;
}
void OnTxTimeout( void )
{
    Radio.Sleep( );
    //Serial.println("TX Timeout......");
    lora_idle = true;
}