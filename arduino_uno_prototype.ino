#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <OneWire.h>
#include <avr/pgmspace.h>

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#define SCOUNT 30
#define DS18S20_Pin 4 //DS18S20 Signal pin on A0
#define turbidity_Pin A0 //turbidity sensor pin on A1
#define TDS_Pin A1 //turbidity sensor pin on A1
#define Ph_pin A2 // ph sensor pin
#define Do_pin A3 // dissolved oxigen sensor pin
#define total_delay 450 //for one hour //8 seconds per total_delay

/*
 * These following can be changed to write and read from eeprom
 */
#define Offset 0.00            //deviation compensate
#define first_ph_reference_voltage 1.52
#define second_ph_reference_voltage 1.15
#define SaturationDoVoltage 1767.58
#define SaturationDoTemperature 24.25

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
static const PROGMEM u1_t NWKSKEY[16] = { 0x73, 0x60, 0x92, 0xB0, 0x49, 0x77, 0xDC, 0x7C, 0x3E, 0x22, 0x2E, 0x59, 0xD6, 0xDF, 0x8A, 0xE6 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
static const u1_t PROGMEM APPSKEY[16] = { 0xAD, 0xAE, 0x0C, 0x51, 0xF0, 0x69, 0xC0, 0x5A, 0x72, 0x00, 0x6C, 0x0F, 0x29, 0x7E, 0x7E, 0x43 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x26011402 ; 

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[32] = "";
static osjob_t sendjob;

/*
 * Temperature values
 */
OneWire ds(DS18S20_Pin);  //Temperature chip i/o on A0
float temperature_value;
int counter = 0;

/*
 * Table to compare the results of the Dissolved oxigen sensor
 */
const float SaturationValueTab[41] PROGMEM = {      //saturation dissolved oxygen concentrations at various temperatures
14.46, 14.22, 13.82, 13.44, 13.09,
12.74, 12.42, 12.11, 11.81, 11.53,
11.26, 11.01, 10.77, 10.53, 10.30,
10.08, 9.86,  9.66,  9.46,  9.27,
9.08,  8.90,  8.73,  8.57,  8.41,
8.25,  8.11,  7.96,  7.82,  7.69,
7.56,  7.43,  7.30,  7.18,  7.07,
6.95,  6.84,  6.73,  6.63,  6.53,
6.41,
};

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 1;
unsigned int count_watchdog = 0;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

/*
 * Sensors function
 */

float getDo(){
   int analogReadBuffer[SCOUNT];
  int analogBufferIndex = 0;
  float voltage = 0, doValue = 0;

  while (analogBufferIndex <= SCOUNT) {
        analogReadBuffer[analogBufferIndex] = analogRead(Do_pin);
        analogBufferIndex++;
        delayMicroseconds(30);
  }
  analogBufferIndex = 0;
  voltage = averageArray(analogReadBuffer, SCOUNT) * (float)5000.0 / 1024.0;
  doValue = pgm_read_float_near( &SaturationValueTab[0] + (int)(SaturationDoTemperature+0.5) ) * voltage / SaturationDoVoltage;  //calculate the do value, doValue = Voltage / SaturationDoVoltage * SaturationDoValue(with temperature compensation)

  return doValue;  
}


float getPh(){
  int analogReadBuffer[SCOUNT];
  int analogBufferIndex = 0;
  float voltage = 0, phValue = 0;

  while (analogBufferIndex <= SCOUNT) {
        analogReadBuffer[analogBufferIndex] = analogRead(Ph_pin);
        analogBufferIndex++;
        delayMicroseconds(40);
  }

  analogBufferIndex = 0;
  voltage = averageArray(analogReadBuffer, SCOUNT) * 5.0 / 1024; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
  Serial.print("voltage for pH: ");
  Serial.print(voltage);
  Serial.print(" ");
  float slope = (6.8-4.0)/((first_ph_reference_voltage-1.50)/3-(second_ph_reference_voltage-1.50)/3);
  float intercept = 6.8 - slope * (first_ph_reference_voltage-1.50) / 3;
  phValue = slope * (voltage - 1.50)/3 + intercept + Offset;
  return phValue;
}

float getTds(){
  int analogReadBuffer[SCOUNT];
  int analogBufferIndex = 0;
  float averageVoltage = 0, tdsValue = 0;

  while (analogBufferIndex <= SCOUNT) {
        analogReadBuffer[analogBufferIndex] = analogRead(TDS_Pin);
        analogBufferIndex++;
        delayMicroseconds(40);
  }

  analogBufferIndex = 0;
  averageVoltage = averageArray(analogReadBuffer, SCOUNT) * 5.0 / 1024; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
  float compensationCoefficient=1.0+0.02*(temperature_value-25.0);    //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
  float compensationVoltage=averageVoltage/compensationCoefficient;  //temperature compensation
  tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5; //convert voltage value to tds value
  return tdsValue;
}

float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad

  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  
  ds.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  
  return TemperatureSum;
  
}

void watchdogSetup(void)
{
  cli(); // disable all interrupts
  wdt_reset(); // reset the WDT timer
  //WDP3 - WDP2 - WPD1 - WDP0 - time
  // 0      0      0      0      16 ms
  // 0      0      0      1      32 ms
  // 0      0      1      0      64 ms
  // 0      0      1      1      0.125 s
  // 0      1      0      0      0.25 s
  // 0      1      0      1      0.5 s
  // 0      1      1      0      1.0 s
  // 0      1      1      1      2.0 s
  // 1      0      0      0      4.0 s
  // 1      0      0      1      8.0 s
  // Enter Watchdog Configuration mode:
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // Set Watchdog settings:
  WDTCSR = (1<<WDIE) | (0<<WDE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);
  sei();
}

void goToSleep() {
  /*** Setup the Watch Dog Timer ***/
  /* Clear the reset flag. */
  Serial.println("Going to sleep...");
  delay(1000);
  watchdogSetup(); //enable watchDog
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  power_adc_disable();
  
  while (count_watchdog < total_delay){
    sleep_mode();
    count_watchdog += 1;
    wdt_reset();
  }
  //disable interrupt buttons and watchdog after sleep
  count_watchdog = 0;
  wdt_disable();

  power_adc_enable();
  
}

//-------------------------------------------------
//This is the interrupt service routine for the WDT. It is called when the WDT times out. 
//This ISR must be in your Arduino sketch or else the WDT will not work correctly
ISR (WDT_vect) 
{
  
}  // end of WDT_vect
//-------------------------------------------------

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            delay(1000);
            goToSleep();
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                Serial.print(F("Data Received: "));
                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                Serial.println();
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        digitalWrite(5, HIGH);
        delay(5000);
        getTemp();
        delay(25000);
        // Prepare upstream data transmission at the next possible time.
        String data = "";
        temperature_value = getTemp();
        Serial.print("Temperature: ");
        Serial.println(temperature_value);
        data += String(temperature_value);
        data += "-";
        
        //get turbidity
        int turbidity = analogRead(turbidity_Pin);
        Serial.print("Turbidity: ");
        Serial.println(turbidity);
        data += String(turbidity);
        data += "-";
        
        //get tds
        float tdsSensor = getTds();
        Serial.print("TDS: ");
        Serial.println(tdsSensor,1);
        data += String(tdsSensor);
        data += "-";

        //get PH
        float ph_value = getPh();
        Serial.print("Ph: ");
        Serial.println(ph_value, 1);
        data += String(ph_value);
        data += "-";

        //get DO
        float do_value = getDo();
        Serial.print("Do: ");
        Serial.println(do_value, 2);
        data += String(do_value);
        data += "-";
        
        data.toCharArray((char*)mydata, 32);
        Serial.println((char*)mydata);
        //In ttn every are shown the ANSI-C code behind every digit
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        Serial.println(F("Packet queued"));

        digitalWrite(5, LOW);
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));

    Serial.println(F("Switch ON"));

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    //setup pins for the ultrasonic sensors
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    wdt_disable(); //Datasheet recommends disabling WDT right away in case of low probabibliy event
    
    pinMode(5, OUTPUT); //To switch on and off the sensors

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly 
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. 
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g2-band

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14); //14
    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}

/*
 * Util Function
 */

double averageArray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}
