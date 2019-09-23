#include <Adafruit_Sensor.h>
#include <SimpleDHT.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MMA8451.h>
#include <Servo.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>

//create sensor objects and declare pins
/* SD - SPI connection
 *  5V: 5V
 *  GND: GND
 *  CLK: pin 52
 *  DO: pin 50 (MISO)
 *  DI: pin 51 (MOSI)
 *  CS: pin 53
 */
void sd_Setup();

/* DHT11
 *  VCC: 5V 
 *  GND: GND
 *  DATA: Digitial Pin 2
 */
SimpleDHT11 dht11;
void getDHTData();

/* BMP180 - I2C connection
 *  Vin: 5V
 *  GND: GND
 *  SCL: SCL
 *  SDA: SDA
 */
Adafruit_BMP085 bmp;
void bmpSetup();
void getBMPData();

/* MMA8451 - I2C connection
 *  Vin: 5V
 *  GND: GND
 *  SCL: SCL
 *  SDA: SDA
 */
 Adafruit_MMA8451 mma = Adafruit_MMA8451();
 void MMA_Setup();
 void getMMAData();

/* Servo
 *  Signal: Digital Pin 9 (or any pwm)
 */
Servo servo1;
void servoPosition();

/* LEDs
 *  green - Digital Pin 5 (things are working)
 *  red - Digital Pin 6 (something failed)
 */
int ledG = 5;
int ledR = 6;
void flash(int);

unsigned long prev_Millis = 0;

void setup() {
  // put your setup code here, to run once:
  //call servoposition in setup (only open servo once)

  Serial.begin(9600);
  servo1.attach(9);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  sd_Setup();
  bmpSetup();
  MMA_Setup();

  servoPosition();
}

void loop() {
  // put your main code here, to run repeatedly:
 
  unsigned long currentMillis = millis(); // variable holds amount of time program has been running
  int Time = currentMillis * 0.001; // gives time in seconds
  if((currentMillis-prev_Millis >= 1000)) // checks to see if one second has passed since the last loop
  {
    prev_Millis = currentMillis;
    Serial.print("Time = "); //record time on serial monitor
    Serial.print(Time);
    Serial.println("");
    
    File dataFile = SD.open("data.txt", FILE_WRITE);
    if(dataFile)
    {
      dataFile.print(Time);
      dataFile.print(",");
      dataFile.close();
    }
  }
 
  getBMPData();
  getDHTData();
  getMMAData();
  delay(1000);
}

//functions:
//Set up the sd card
void sd_Setup()
{
  Serial.println("Initializing SD card...");
  int CS = 53;

  if(!SD.begin(CS))
  {
    Serial.println("Card failed, or not present.");
    while(1)
    {
      flash(ledR);
    }
  }

  Serial.println("Card is Ready");
  flash(ledG);

  //Add headers to data file
  File dataFile = SD.open("data.txt", FILE_WRITE);
  if(dataFile)
  {
    dataFile.print("Time(s), BMP_Temperature(C), BMP_Pressure(Pa), BMP_Altitude(m), ");
    dataFile.print("DHT_Temperature(C), DHT_Humidity(%), ");
    dataFile.println("x(m/s^2), y(m/s^2), z(m/s^2), Orientation");
    dataFile.println("");
    dataFile.close();
  }
  else
  {
    Serial.println("Couldn't open data file");
  }

}

//setup bmp
void bmpSetup()
{
  if (!bmp.begin())
  {
    Serial.println("BMP initialization failed.");
    while(1)
    {
      flash(ledR);
    }
  }

  Serial.println("BMP initialized.");
  flash(ledG);
}


//set up accelerometer
void MMA_Setup()
{
  if (! mma.begin())
  {
    Serial.println("MMA8451 initialization failed.");
    while(1)
    {
      flash(ledR);
    }
  }

  mma.setRange(MMA8451_RANGE_2_G);
  Serial.println("MMA8451 initialized.");
  flash(ledG);
}

//get pressure/altitude/temp
void getBMPData()
{
  float altitude, pressure, temp;

  temp = bmp.readTemperature();
  pressure = bmp.readPressure();
  altitude = bmp.readAltitude(101500);

  // create data string to store to sd card
  String dataString = String(temp) + "," + String(pressure) + "," + String(altitude);


  //writing to SD card//
  File dataFile = SD.open("data.txt", FILE_WRITE); // open file
  if(dataFile)
  {
    dataFile.print(dataString); //write temperature to SD card
    dataFile.close(); //close the file

    //Displaying data to serial port//
    Serial.println("BMP280 Readings");
    Serial.print("Temperature: ");
    Serial.print(temp);
    Serial.println(" C");
    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.println(" Pa");
    Serial.print("Altitude = ");
    Serial.print(altitude);
    Serial.println(" meters");
    Serial.println("");
  }
  else
  {
    Serial.println("Couldn't open data file");
  }

}

//get temp/humidity
void getDHTData()
{
  int DHTpin = 2;
  byte temperature = 0;
  byte humidity = 0;
  int Null = 00;
  //DHT sampling rate is 1 Hz
  
  File dataFile = SD.open("data.txt", FILE_WRITE);
  if(dataFile)
  {
    //display results
    if(dht11.read(DHTpin, &temperature, &humidity, NULL))
    {
      String dataString = "," + String(Null) + "," + String(Null) + ",";
      dataFile.print(dataString);
      dataFile.close();

      Serial.println("DHT Readings: ");
      Serial.print("Temperature: "); Serial.print("NULL"); Serial.println(" C");
      Serial.print("Humidity: "); Serial.print("NULL"); Serial.println(" %");
  
      Serial.println("");
    }
    else
    {
      String dataString = "," + String(temperature) + "," + String(humidity) + ",";
      dataFile.print(dataString);
      dataFile.close();
    
      Serial.println("DHT Readings: ");
      Serial.print("Temperature: "); Serial.print((int)temperature); Serial.println(" C");
      Serial.print("Humidity: "); Serial.print((int)humidity); Serial.println(" %");
  
      Serial.println("");
    }
  }
}
//detect motion, tilt and basic orientation
void getMMAData()
{

  //Get a new sensor event
  sensors_event_t event;
  mma.getEvent(&event);

  float x = 0, y = 0, z = 0;
  x = event.acceleration.x;
  y = event.acceleration.y;
  z = event.acceleration.z;

  //display results
  Serial.println("MMA sensor Readings: ");
  Serial.print("X: \t"); Serial.print(x); Serial.print("\t");
  Serial.print("Y: \t"); Serial.print(y); Serial.print("\t");
  Serial.print("Z: \t"); Serial.print(z); Serial.print("\t");
  Serial.println("m/s^2 ");

  /* Get the orientation of the sensor */
  uint8_t o = mma.getOrientation();
  String Orient;

  switch (o)
  {
    case MMA8451_PL_PUF:
      Serial.println("Portrait Up Front");
      Orient = "Portrait Up Front";
      break;
    case MMA8451_PL_PUB:
      Serial.println("Portrait Up Back");
      Orient = "Portrait Up Back";
      break;
    case MMA8451_PL_PDF:
      Serial.println("Portrait Down Front");
      Orient = "Portrait Down Front";
      break;
    case MMA8451_PL_PDB:
      Serial.println("Portrait Down Back");
      Orient = "Portrait Down Back";
      break;
    case MMA8451_PL_LRF:
      Serial.println("Landscape Right Front");
      Orient = "Landscape Right Front";
      break;
    case MMA8451_PL_LRB:
      Serial.println("Landscape Right Back");
      Orient = "Landscape Right Back";
      break;
    case MMA8451_PL_LLF:
      Serial.println("Landscape Left Front");
      Orient = "Landscape Left Front";
      break;
    case MMA8451_PL_LLB:
      Serial.println("Landscape Left Back");
      Orient = "Landscape Left Back";
      break;
    }

  String dataString = String(x) + "," + String(y) + "," + String(z) + "," + Orient;

  //print to sd card
  File dataFile = SD.open("data.txt", FILE_WRITE);
  if(dataFile)
  {
    dataFile.println(dataString);
    dataFile.close();
  }

}

//write servo/servos to position to expand arms
void servoPosition()
{

  //wait until info from pixhawk is received
  //while loop checking for info

  servo1.write(180); //turn servo to pull arms down
}

//flash an led 3x
void flash(int led)
{
  for(int i = 0; i < 3; i++)
  {
    digitalWrite(led, HIGH);
    delay(500);
    digitalWrite(led, LOW);
    delay(500);
  }
}
