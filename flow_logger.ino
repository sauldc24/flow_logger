//#define DEBUG
#include <Adafruit_SleepyDog.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <SD.h>
#include <CayenneLPP.h>
#include <Sodaq_PcInt.h>
#include <Sodaq_DS3231.h>
#include <DHT.h>
#include <DHT_U.h>
#include <RAK811V2.h>
#include <LowPower.h>

// switched power lines pin
#define SWITCHED_POWER 22 // Enable pin for switched 3.3

// Error flag and error LED pin
#define ERROR_LED 9
bool error;

// Operation Success pin
#define SUCCESS_LED 8
bool success;

// DateTime variable for the time
DateTime now;

// create a string buffer for SD card and printing info to the debugging serial port
String string_buffer;

// File object variable
File file;
// COnfiguration of battery voltage variables
int batteryPin = A6;       // on the Mayfly board, pin A6 is connected to a resistor divider on the battery input; R1 = 10 Mohm, R2 = 2.7 Mohm
int batterysenseValue = 0; // variable to store the value coming from the analogRead function
float batteryvoltage;      // the battery voltage as calculated by the formula below
// formula for battery level calculation is batteryvoltage = (3.3/1023.) * 4.7 * batterysenseValue;

// create and configure CayenneLPP buffer wit a buffer size of 51
CayenneLPP lpp(51);

// Create and configure DHT21 object and variables for temperature and humidity
#define DHTPIN 6
#define DHTTYPE DHT21
DHT dht(DHTPIN, DHTTYPE);
float temperature2;
float humidity;

// Configure DS18B20 sensor and variable for temperature
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20(&oneWire);
float temperature = 0;

// declare LoRa Radio
RAK811 lora(Serial1, Serial);
bool joined_network;

// pin for the RTC alarm interrupt
int interruptPin = A7;
// a flag to indicate that device was awaken by RTC, set to true in setup to make sure measurments are executed once when loop is first executed
volatile int rtcWakeFlag = 1;

// pin to handle the tipping bucket interrupt and variables for antidebouncing
#define tippingBucketPin 10
#define TIP_TO_MM_FACTOR 0.3
int tippingcounts;
volatile int tipFlag = 0;
float rainLevel = 0;

// Interrupt service routine for RTC alarm
void INT0_ISR()
{
  // set flag to indicate device was awaken by RTC
  rtcWakeFlag = 1;
}

// interrupt service routine for Tipping bucket when low
void TB_ISR()
{
  // rise flag to signal tipping of the rain gauge
  tipFlag = 1;
  rtcWakeFlag = 0;
}

void setup()
{
  // configure switched power, error and success pins
  pinMode(SWITCHED_POWER, OUTPUT);
  digitalWrite(SWITCHED_POWER, LOW);
  pinMode(ERROR_LED, OUTPUT);
  digitalWrite(ERROR_LED, LOW);
  pinMode(SUCCESS_LED, OUTPUT);
  digitalWrite(SUCCESS_LED, LOW);

  // blink success led to notify powered on state
  blink(SUCCESS_LED, 3, 500);

  // initialize debugging serial
  #if defined DEBUG
  Serial.begin(9600);
  #endif

  //Initializa ds18b20 sensor and check presence
  ds18b20.begin();
  if (!ds18b20.getDS18Count())
  {
    blink(ERROR_LED, 3, 500);
    #if defined DEBUG
    Serial.println(F("Primary temperature sensor error"));
    Serial.flush();
    #endif
  } 

  //initialize dht sensor and check presence
  dht.begin();
  if (dht.readTemperature() == NAN)
  {
    blink(ERROR_LED, 3, 500);
    #if defined DEBUG
    Serial.println(F("Secondary Temp/Humidity sensor error"));
    Serial.flush();
    #endif
  }

  // initialize the RAK811 serial port
  Serial1.begin(9600);
  if (lora.rk_begin())
  {
    #if defined DEBUG
    Serial.println(F("LoRa initialization OK"));
    #endif
    joined_network = true;
    blink(SUCCESS_LED, 3, 1000);
  }
  else
  {
    #if defined DEBUG
    Serial.println(F("LoRa initialization Error"));
    #endif
    joined_network = false;
    blink(ERROR_LED, 3, 1000);
  }
  //Test to see if payload limit reached for current data rate when sending data
  //lora.sendRawCommand(F("at+set_config=lora:adr:1",2000));
  //initilize SD
  while (!SD.begin(12))
  { // si no se logra inicializar la SD
    #if defined DEBUG
    Serial.println(F("No SD"));
    #endif
    digitalWrite(ERROR_LED, HIGH);
    delay(500); // espera antes de volver a intentarlo
  }
  digitalWrite(ERROR_LED, LOW);
  // adjust RTC if neccesary
  //DateTime dt (__DATE__, __TIME__);
  //rtc.setDateTime(dt); //Adjust date-time as defined 'dt' above
  // Configure the interrupt to awake the device
  pinMode(interruptPin, INPUT_PULLUP);
  PcInt::attachInterrupt(interruptPin, INT0_ISR);
  // initialize the rtc
  rtc.begin();
#if defined DEBUG
  rtc.enableInterrupts(EveryMinute);
#else
  rtc.enableInterrupts(EveryHour);
#endif
  // atach tipping bucket interrupt
  pinMode(tippingBucketPin, INPUT_PULLUP);
  attachInterrupt(2, TB_ISR, LOW);
}

void loop()
{
  //enable the watchdog timer in case the program halts
  Watchdog.enable(8000);
  // check if the device was awaken by RTC
  if (rtcWakeFlag == 1)
  {
    // This section clears the alarm flag of the RTC
    rtc.clearINTStatus();
    // turn the switched source on
    digitalWrite(SWITCHED_POWER, HIGH);
    // initialize DHT library
    dht.begin();
    // read DS18B20
    ds18b20.requestTemperatures();
    temperature = ds18b20.getTempCByIndex(0);
    if (temperature == DEVICE_DISCONNECTED_C)//try to read only one more time in case of an error
    {
      delay(500);
      ds18b20.requestTemperatures();
      temperature = ds18b20.getTempCByIndex(0);
    }
    //reset the watchdog timer at this point to avoid device reset
    Watchdog.reset();
    // calculate rain level
    rainLevel = tippingcounts * TIP_TO_MM_FACTOR;
    // reset tipping counts to zero
    tippingcounts = 0;
    // delay for DHT21 startup
    delay(500);
    // read DHT21
    temperature2 = dht.readTemperature();
    humidity = dht.readHumidity();
    // power sensor off as soon as it finishes reading
    digitalWrite(SWITCHED_POWER, LOW);
    string_buffer = "";
    batterysenseValue = analogRead(batteryPin);
    batteryvoltage = (3.3 / 1023.) * 4.7037 * batterysenseValue;
    now = rtc.now();
    // set wake flag to false again
    rtcWakeFlag = 0;
    // Might be a good idea to store the files in a y/m/d format to make it easier to navigate files in SD
    string_buffer = String(now.year()-2000) + "_" + String(now.month()) + "_" + String(now.date()) + ".csv";
    // string_buffer = String(now.date()) + "_" + String(now.month()) + ".csv";
    //  section to verify if the file exists, if it doesn't then create it and generate the text file header
    //reset the watchdog timer at this point to avoid device reset
    Watchdog.reset();
    if (!SD.exists(string_buffer))
    {
      file = SD.open(string_buffer, FILE_WRITE);
      if (file)
      {
        file.println(F("Fecha/hora,Temperatura 1,Temperatura 2,Humedad,Bateria,Pluvial"));
        file.close();
      }
      else{
        #if defined DEBUG
          Serial.println("Error al crear archivo");
        #endif
        error = true;}
    }
    file = SD.open(string_buffer, FILE_WRITE);
    string_buffer = "";
    now.addToString(string_buffer);
    string_buffer.concat("," + String(temperature) + "," + String(temperature2) + "," + String(humidity) + "," + String(batteryvoltage) + "," + String(rainLevel));
    #if defined DEBUG
      Serial.println(string_buffer);
    #endif
    if (file)
    {
      file.println(string_buffer);
      file.close();
      #if defined DEBUG
        Serial.println(F("Succesfully written to the SD"));
      #endif
      error = false;
    }
    else{
      #if defined DEBUG
        Serial.println("Error de escritura");
      #endif
      error = true;}
    //reset the watchdog timer at this point to avoid device reset
    Watchdog.reset();
    if (joined_network) SendCayenne();
    // update error status pin
    if (error)
    {
      digitalWrite(ERROR_LED, HIGH);
    }
    // This section clears the alarm flag of the RTC
    //rtc.clearINTStatus();
  }
  // if not awaken by RTC, check if was awaken by tipping bucket
  if (tipFlag == 1)
  {
    // wait for reed switch to bounce, change again (momentary pulse device) and bounce again
    delay(200);
    tippingcounts++;
    tipFlag = 0;
    #if defined DEBUG
      Serial.println(F("Tipping Bucket counter increment"));
      Serial.flush();
    #endif
  }
  #if defined DEBUG
    Serial.println(F("Going to sleep"));
    Serial.flush();
  #endif
  //reset the watchdog timer and disable it before going to sleep mode
  Watchdog.reset();
  Watchdog.disable();
  // This section puts the device in deep sleep
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}


void SendCayenne()
{
  //reset the watchdog timer at this point to avoid device reset
  Watchdog.reset();
  lora.rk_wake();
  lpp.reset();
  lpp.addTemperature(1, temperature);
  lpp.addRelativeHumidity(3, humidity);
  lpp.addAnalogInput(10, batteryvoltage);
  lpp.addAnalogInput(11, rainLevel);
  lpp.addTemperature(2, temperature2);
  Watchdog.reset();
  if (!lora.rk_sendBytes(2, lpp.getBuffer(), lpp.getSize()))//check if te payload size is not supported by the actual Data Rate (Error code 101)
  {
    //reset the watchdog timer at this point to avoid device reset
    Watchdog.reset();
    //Send data again without the last 4 bytes correspondig to the last temperature channel data (channel byte plus type byte plus 2 data bytes for a temperature sensor)
    lora.rk_sendBytes(2,lpp.getBuffer(), lpp.getSize() - 8);
    //reset the watchdog timer at this point to avoid device reset
    Watchdog.reset();
  }
  lora.rk_sleep();
  //reset the watchdog timer at this point to avoid device reset
  Watchdog.reset();
}

void blink(int pin, int cycles, int delay_time_ms)
{
  for (int i = 0; i < cycles; i++)
  {
    digitalWrite(pin, HIGH);
    delay(delay_time_ms);
    digitalWrite(pin, LOW);
    delay(delay_time_ms);
  }
}