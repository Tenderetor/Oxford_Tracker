#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_GPS.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
const int chipSelect = 10;

// what's the name of the hardware serial port?
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  false

int fix_led = 2;
int switch_pin = 4;
int record_led = 3;

float x=0;
float y=0;
float z=0;
float counterAccell=0;
float temp;
float spoed;
float conversion=1.852;
char c;
String NMEA1;  
String NMEA2;

File mySensorData;

uint32_t timer1 = millis();
uint32_t timer2 = millis();

int testCounter;
bool newTest;

float myMinutes;
int myDegrees;
float myLocation;

void readGPS() {

  while (!GPS.newNMEAreceived()) {
    c = GPS.read();
  }
  GPS.parse(GPS.lastNMEA());
  //   NMEA1=GPS.lastNMEA();

  while (!GPS.newNMEAreceived()) {
    c = GPS.read();
  }
  GPS.parse(GPS.lastNMEA());
  //   NMEA2=GPS.lastNMEA();


}

void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayDataRate(void)
{
  Serial.print  ("Data Rate:    ");

  switch (accel.getDataRate())
  {
    case ADXL345_DATARATE_3200_HZ:
      Serial.print  ("3200 ");
      break;
    case ADXL345_DATARATE_1600_HZ:
      Serial.print  ("1600 ");
      break;
    case ADXL345_DATARATE_800_HZ:
      Serial.print  ("800 ");
      break;
    case ADXL345_DATARATE_400_HZ:
      Serial.print  ("400 ");
      break;
    case ADXL345_DATARATE_200_HZ:
      Serial.print  ("200 ");
      break;
    case ADXL345_DATARATE_100_HZ:
      Serial.print  ("100 ");
      break;
    case ADXL345_DATARATE_50_HZ:
      Serial.print  ("50 ");
      break;
    case ADXL345_DATARATE_25_HZ:
      Serial.print  ("25 ");
      break;
    case ADXL345_DATARATE_12_5_HZ:
      Serial.print  ("12.5 ");
      break;
    case ADXL345_DATARATE_6_25HZ:
      Serial.print  ("6.25 ");
      break;
    case ADXL345_DATARATE_3_13_HZ:
      Serial.print  ("3.13 ");
      break;
    case ADXL345_DATARATE_1_56_HZ:
      Serial.print  ("1.56 ");
      break;
    case ADXL345_DATARATE_0_78_HZ:
      Serial.print  ("0.78 ");
      break;
    case ADXL345_DATARATE_0_39_HZ:
      Serial.print  ("0.39 ");
      break;
    case ADXL345_DATARATE_0_20_HZ:
      Serial.print  ("0.20 ");
      break;
    case ADXL345_DATARATE_0_10_HZ:
      Serial.print  ("0.10 ");
      break;
    default:
      Serial.print  ("???? ");
      break;
  }
  Serial.println(" Hz");
}

void displayRange(void)
{
  Serial.print  ("Range:         +/- ");

  switch (accel.getRange())
  {
    case ADXL345_RANGE_16_G:
      Serial.print  ("16 ");
      break;
    case ADXL345_RANGE_8_G:
      Serial.print  ("8 ");
      break;
    case ADXL345_RANGE_4_G:
      Serial.print  ("4 ");
      break;
    case ADXL345_RANGE_2_G:
      Serial.print  ("2 ");
      break;
    default:
      Serial.print  ("?? ");
      break;
  }
  Serial.println(" g");
}

void setup(void)
{

while (!Serial); // for Leonardo/Micro/Zero

  Serial.begin(9600);
  Serial.println("Accelerometer Test"); Serial.println("");
  pinMode(fix_led, OUTPUT);
  pinMode(record_led, OUTPUT);
  pinMode(switch_pin, INPUT_PULLUP);

  testCounter=1;
  newTest=true;

  //digitalWrite(7, HIGH); //turns on LED to show power is received
  timer1 = millis();
  timer2 = millis();

  /* Initialise the sensor */
  if (!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while (1);
  }

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset or reopen this Serial Monitor after fixing your issue!");
    while (true);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_16_G);


  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Display additional settings (outside the scope of sensor_t) */
  displayDataRate();
  displayRange();
  Serial.println("");

  Serial.println("Adafruit GPS logging data dump!");

  // 9600 NMEA is the default baud rate for MTK - some use 4800
  GPS.begin(9600);
  //GPS.sendCommand("$PGCMD,33,0*6D");
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
}

void loop(void)
{
  if (GPS.fix){
    digitalWrite(fix_led, HIGH);
  }
  else{
    digitalWrite(fix_led, LOW);
  }

  readGPS(); //ensures 2 newly parsed sentences

  if (digitalRead(switch_pin) == LOW) {

    
    
  
    while(newTest){//calculates the test number
      if(SD.exists((String)"Test"+testCounter+".CSV")){
      testCounter++;
        }
        else{ 
          newTest = false;
          Serial.println((String) "New Test Detected: TEST"+testCounter);
          mySensorData = SD.open((String)"Test"+testCounter+".CSV", FILE_WRITE );
          
          mySensorData.print("Date");
          mySensorData.print(",");
          mySensorData.print("Time");
          mySensorData.print(",");
          mySensorData.print("Latitude");
          mySensorData.print(",");
          mySensorData.print("Longitude");
          mySensorData.print(",");
          mySensorData.print("Altitude");
          mySensorData.print(",");
          mySensorData.print("Speed");
          mySensorData.print(",");
          mySensorData.print("Heading");
          mySensorData.print(",");
          mySensorData.print("X-Axis Acceleration");
          mySensorData.print(",");
          mySensorData.print("Y-Axis Acceleration");
          mySensorData.print(",");
          mySensorData.print("Z-Axis Acceleration");
          mySensorData.print(",");
          mySensorData.println("Number of Satellites Connected");
          mySensorData.close();
     
        }
    }

    if(millis() - timer2 > 100){

      timer2=millis();

      sensors_event_t event;
      accel.getEvent(&event);

      //add measurement to x
      x = x + event.acceleration.x;
      //add measurement to y
      y = y + event.acceleration.y;
      //add measurement to z
      z = z + event.acceleration.z;

      counterAccell = counterAccell + 1;  

         
    }

  if (millis() - timer1 > 1000) {
    timer1=millis();
    
      mySensorData = SD.open((String) "Test"+testCounter+".CSV", FILE_WRITE );
      
          if((mySensorData)){ //if file succesfully openened
            Serial.println((String)"File "+testCounter+" opened");
            digitalWrite(8, HIGH);

                //DATE
                mySensorData.print(GPS.day, DEC); mySensorData.print('/');
                mySensorData.print(GPS.month, DEC); mySensorData.print("/20");
                mySensorData.print(GPS.year, DEC);

                mySensorData.print(",");

                //TIME
                if (GPS.hour < 8) { mySensorData.print('0'); }
                mySensorData.print(GPS.hour+2, DEC); mySensorData.print(':');
                if (GPS.minute < 10) { mySensorData.print('0'); }
                mySensorData.print(GPS.minute, DEC); mySensorData.print(':');
                if (GPS.seconds < 10) { mySensorData.print('0'); }
                mySensorData.print(GPS.seconds, DEC);

                mySensorData.print(",");
                
                
                
                if(GPS.fix){

                  //latitude
                  myDegrees = GPS.latitude/100;
                  myMinutes = (float) GPS.latitude - myDegrees*100.0;
                  myLocation = (float) myDegrees + (myMinutes/60.0) ;
                  if (GPS.lat == 'S'){
                    myLocation = (float) myLocation*-1.0;
                  }
                  mySensorData.print(myLocation, 5);


                  mySensorData.print(",");
                  
                  //longitude                 
                  myDegrees = GPS.longitude/100;
                  myMinutes = (float) GPS.longitude - myDegrees*100.0;
                  myLocation = (float) myDegrees + (myMinutes/60.0) ;
                  if (GPS.lon == 'W'){
                    myLocation = (float) myLocation*-1.0;
                  }
                  mySensorData.print(myLocation, 5);

                  mySensorData.print(",");
                  
                  //altitude
                  mySensorData.print(GPS.altitude);

                  Serial.println((String)"Altitude: "+GPS.altitude);

                  mySensorData.print(",");

                  //speed
                  if(GPS.speed == NULL){
                    mySensorData.print(" 0 ");
                  }
                  else{
                  spoed = (float)  GPS.speed*conversion;
                  mySensorData.print(spoed);
                  }

                  mySensorData.print(",");

                  //heading
                  if(GPS.angle == NULL){
                    mySensorData.print(" 0 ");
                  }
                  else{
                  mySensorData.print(GPS.angle, 4);
                  }

                  mySensorData.print(",");
                
               
                }
                else{\
                  //no fix
                  //latitude 
                  mySensorData.print(" 0 ");

                  mySensorData.print(",");
                  
                  //longitude
                  mySensorData.print(" 0 ");

                  mySensorData.print(",");

                  
                  //altitude
                  mySensorData.print(" 0 ");

                  mySensorData.print(",");

                  //speed
                  mySensorData.print(" 0 ");

                  mySensorData.print(",");

                  //heading
                  mySensorData.print(" 0 ");

                  mySensorData.print(",");
                }

                //x axis
                temp = (float) (x/counterAccell)/9.80665;
                mySensorData.print(temp, 4);
                mySensorData.print(",");
                //y axis
                temp = (float) (y/counterAccell)/9.80665;
                mySensorData.print(temp, 4);
                mySensorData.print(",");
                //z axis
                temp = (float) (z/counterAccell)/9.80665;
                mySensorData.print(temp, 4);
               

                x=0;
                y=0;
                z=0;
                counterAccell=0;


                //# satellites connected
                mySensorData.print((int)GPS.satellites);
                mySensorData.print(",");


                mySensorData.println("");
                mySensorData.close();
        
                }
                else{
            //File could not be opened
            newTest = true;
            digitalWrite(8, LOW);
            Serial.println("file could not be opened not recording");
           }
  }       
      }
  else{
          newTest = true;
          digitalWrite(record_led, LOW);
          Serial.println("Not Saving... ");
  }

}
