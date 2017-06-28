#include <SPI.h>
#include <Encoder.h>
#include <TaskScheduler.h>

const int dacChipSelectPin = 45; 
const int lickSensor = 3; //lick sensor out port
const int solenoidL = 4; //left lick solenoid
const int resetPin = A4; //RFID reset pin
const int formPin = A5; //RFID form pin (keep low)
const int rfidGnd = A3; //ground pin for RFID reader (max current draw = 5 mA)
const int rfid5V = A2; 
const int outputRFID = 28; //RFID soft output to prarie

const int triggerBNC = 22; //BNC connector that sends trigger to Prarie
const int outputSTIM = 24;
const int outputLICK = 26;
const int outputROTARY = 28; //also ooutput for RFID sensor

//var for storing random position generated
long randPos = 0;
const int numRewardsPerLap = 5;
long randPosListGlobal[numRewardsPerLap];
long randPosListGlobalNext[numRewardsPerLap];
int currentRandPosIdx = 0;
const int alwaysDeliverWater = 1;
int stopWaterFromList = 0;

//how wide is the reward pick area (cm) from start
int rangeReward = 10;

//flag for indicating if the mouse is in the reward zone to receive water reward
//and if it has collected a water reward within that zone already
int rewardZone = 1;

//randPosGenerator(randPosListGlobal,numRewardsPerLap)
//Serial.println(randPostListGlobal[0]);
//Serial.println(randPostListGlobal[1]);
//Serial.println(randPostListGlobal[2]);

//deliver 6500 toward end ofw run; 10000 in the first day; 8000 in the second day; 6500 in the third day 
const int waterAmount = 12000; //6500 //microseconds - 6500 us corresponds to roughly 1.0 - 1.1 uL of water delivered per droplet
const float volPerDrop = 1.1; //in uL
const float cmConvFactor = 0.0155986;

volatile long previousPosition = 0;
volatile long currentPosition = 0;
volatile long currentPositionCm = 0.0;


int lapOdorStart = 1;
int lapOdorEnd = 5;

int lapOdorStart2 = 1; 
int lapOdorEnd2 = 5;

volatile float currentTime = 0;
volatile float previousTime = 0;

volatile float speedOut = 0.00;

bool enableWater = false; //allow first droplet of water to dispense

bool unlimited = false; //delivers water when mouse licks regardless of speed

int currentLickState = LOW;
int previousLickState = LOW;

//array holding previous run speeds - yes/no states - binary coding; NOT USED - easier and faster to do with counter
//int previousSpeeds[20]; //index address 0 - 19
volatile float timeStart = 0; 
//unsigned long currentTime = 0;

volatile int chargeState = 0;
float currentDistance = 0;

float speedThreshold = 5; //speed that the mouse has to exceed in order to register as ON event in above array
int speedThresCount = 0; //counter that increments every time speed is above speedThres; once counter crosses sum of previous ON events (sum to 1s), water delivered
int countThres = 30; //if 20 previous checks (t4 timer) have returned speeds > speedThres, deliver water
//20 cm threshold distance before RFID reset
volatile float thresRFIDdist = 30;
volatile long thresRFIDpos = 3000;
//measured in centimeters
const float lapLength = 200;

int waterDispensed = 0; //counter for how many water droplets were dispensed

unsigned long clockStart = 0;

//lap counter - increase every time RFID tag is scanned
int lapCounter = 0;

//RFID-related variables
  byte i = 0;
  byte val = 0;
  byte code[6]; //index from 0 to 5
  byte checksum = 0;
  byte bytesread = 0;
  byte tempbyte = 0;

//Tag ID bytes
//lap tag
byte tag_Lap = 0xCB;
int firstCrossLap = 1;

//50 cm ahead tag
byte tag_OdorStart1 = 0x0D; //(2) 

//75 cm ahead tag
byte tag_OdorEnd1 = 0x5A; //(3)

byte tag_Reward1 = 0x99;

byte tag_OdorStart2 = 0xD8;
byte tag_OdorEnd2 = 0x1C;

byte tag_Reward2 = 0x3A;

byte tag_Cue1 = 0x0F;
//byte tag_Cue2 = tag_OdorEnd2 = 0xC1;
byte tag_Cue3 = 0x73;
byte tag_Cue4 = 0x70;

int waterDeliveryDetailsFromGui;
//water delivery tag
//+byte tag4 = 0x0D;
//variable to indicate if the mouse just crossed the reward tag
int firstCrossW = 1;
Scheduler runner; 

//Callback prototypes
void readPosAndSpeed();
void printSpeed();
void printDistance();
void checkSpeed();
void dispTimeSinceStart();
void checkLickState();
void checkRFID();
void checkKeyboardIn();
void randWaterDeliverCheck();

Task t1(10, TASK_FOREVER, &readPosAndSpeed, &runner, true); //read position and calculate speed
Task t2(1000, TASK_FOREVER, &printSpeed, &runner, true); //print speed to serial
Task t3(1000, TASK_FOREVER, &printDistance, &runner, true);  //calculate distance traveled since the beginning
//Task t4(50,TASK_FOREVER, &checkSpeed, &runner, true); //poll at 50 - record 20 positions
Task t5(1000, TASK_FOREVER, &dispTimeSinceStart, &runner, true); //display total runtime of program
Task t6(1, TASK_FOREVER, &checkLickState, &runner, true); //check if the mouse is licking every 1 ms 
Task t7(20, TASK_FOREVER, &checkRFID, &runner, true); //check RFID and do something...reset position, deliver reward, reset # of licks 
Task t8(100, TASK_FOREVER, &checkKeyboardIn, &runner, true);
Task t9(10, TASK_FOREVER, &randWaterDeliverCheck, &runner, true);

//rotary encoder setup using 2 interrupts - place this on a serial port coming in from Atilla encoder chip
Encoder wheel(18,19);

void readPosAndSpeed()
{
  previousPosition = currentPosition;
  previousTime = currentTime;
   currentPosition = -wheel.read(); //current position always stored
   currentPositionCm = currentPosition*cmConvFactor;
   
   currentTime = millis();
   speedOut = ((currentPosition - previousPosition)*cmConvFactor)/((currentTime - previousTime)/1000);
}

void printSpeed()
{
  Serial.println("Speed in cm/s is-- " + String(speedOut));
  //Serial.println(speedThresCount, DEC);
}

void printDistance()
{//this needs to be changed accordingly in everything!!!
  Serial.print("Distance traveled forward in cm--"); Serial.println(currentPosition * cmConvFactor,3);
  Serial.print("Amount of water dispensed in drops-- "); Serial.println(waterDispensed, DEC);
  Serial.print("Volume dispensed in uL-- "); Serial.println(waterDispensed*volPerDrop,2); 
  Serial.print("Full laps traveled-- "); Serial.println(lapCounter - 1);
  Serial.println("");
  Serial.print("Charge state-- "); Serial.println(chargeState);
}


void indexAlert()
{
  Serial.println("index!");
}

void deliverWater()
{
  if (enableWater)
  {
    digitalWrite(solenoidL, HIGH);
    Serial.println("Water delivered"); Serial.println("");
    waterDispensed++;
    delayMicroseconds(waterAmount);//time. call it n times for n drops
    digitalWrite(solenoidL, LOW);

    enableWater = false; //must clear this state to true before next drop of water is delivered
  }

//give second water reward if only one drop per lap
  if (numRewardsPerLap == 1)
  {
    digitalWrite(solenoidL, HIGH);
    Serial.println("Water delivered"); Serial.println("");
    waterDispensed++;
    delayMicroseconds(waterAmount);
    digitalWrite(solenoidL, LOW);
    
  }
} 

void deliverWaterByGui(int waterAmountss)
{

    digitalWrite(solenoidL, HIGH);
    Serial.println("Water delivered"); Serial.println("");
    waterDispensed+=1;
    delayMicroseconds(waterAmountss);
    digitalWrite(solenoidL, LOW);

 
} 



void randPosGenerator(long randPosList[], int numRewards)
{
  //randomSeed(millis());
  for (int idx = 0; idx <= (numRewards-1); idx++)
  {
    randPosList[idx] =  random(0,196);
    //Serial.println(randPosList[idx]);
  }  

}

//used this to control the current index of water reward range
void randWaterDeliverCheck()
{
  //Serial.println(currentRandPosIdx);
  //Serial.println(((int) currentPositionCm));
  if (currentRandPosIdx <= (numRewardsPerLap-1))
  {
  if ((((int) currentPositionCm) > (randPosListGlobal[currentRandPosIdx]+rangeReward))) 
  {
    //advance to next index on the list
    currentRandPosIdx++;
    //reset the water delivery flag from previous zone
    rewardZone = 1;

  }
  }
  
}

void checkSpeed()
{

//    if (speedOut > speedThreshold)
//      {
//        speedThresCount++;
//
//        if (speedThresCount == countThres)
//          {
//            //enableWater = true;
//            deliverWater();
//            speedThresCount = 0;
//          }
//      
//      }
//    else
//      {
//        speedThresCount = 0;
//      }
  }

void checkLickState()
{
  int currentLickState = digitalRead(lickSensor);
  
  if ((currentLickState == HIGH) && (previousLickState == LOW)) 
  {

    Serial.println("Mouse licking");
    enableWater = true;
    //check if mouse in the area range of where random water zone corresponding to current index is
    //and that it has not collected a reward already in that zone
    //need for the index to advance simply based on position
      if (currentRandPosIdx <= (numRewardsPerLap-1))
  {
  if ((((int) currentPositionCm) > randPosListGlobal[currentRandPosIdx]) && (((int) currentPositionCm) < (randPosListGlobal[currentRandPosIdx] + rangeReward)) )
  {
//    if (alwaysDeliverWater == 1)
//    {
//      enableWater = true;
//    }

    if (rewardZone == 1)
    {
    Serial.println("XXXXXXXXXXXXXX");
    Serial.println(((int) currentPositionCm));
    Serial.println(randPosListGlobal[currentRandPosIdx]);
    deliverWater();
    signalStim();
    rewardZone = 0;
    }


  }
  }
        
    if (unlimited == true)
    {
      deliverWater();
    }
    previousLickState = currentLickState;
  }
  else if (currentLickState != previousLickState)
   {
    previousLickState = currentLickState;
   }
  
}
  
void dispTimeSinceStart()
{
  unsigned long currentClock = millis() - clockStart;
  
  unsigned long currentSec = currentClock/1000; //works
  unsigned long currentMin = currentSec/60;
  unsigned long currentSecLeft = currentSec%60;
  
  Serial.print("Program running for- "); Serial.print(currentMin,DEC); Serial.print(":"); 
  if (currentSecLeft < 10)
  {
    Serial.print("0");
  }
  Serial.println(currentSecLeft,DEC);
}

void checkRFID()
{
  i = 0;
  val = 0;
  code[6];
  checksum = 0;
  bytesread = 0;
  tempbyte = 0;
  
  if(Serial2.available() > 0) {
    unsigned long timeStart = millis();
    
    if((val = Serial2.read()) == 2) {                  // check for header 
      bytesread = 0;
      
      while (bytesread < 12) {                        // read 10 digit code + 2 digit checksum
        if( Serial2.available() > 0) { 
          val = Serial2.read();
          
          if((val == 0x0D)||(val == 0x0A)||(val == 0x03)||(val == 0x02)) { // DEC: 13 = \r ,10 = \n,3 - stop,2 - start if header or stop bytes before the 10 digit reading 
            break;                                    // stop reading
          }

          // Do Ascii --> Hex conversion:
          if ((val >= '0') && (val <= '9')) {
            val = val - '0';
          } else if ((val >= 'A') && (val <= 'F')) {
            val = 10 + val - 'A';
          }
          // val is now hex value
          
          // Every two hex-digits, add byte to code: //look at how the LSB changes 
          if (bytesread & 1 == 1) {
            // make some space for this hex-digit by
            // shifting the previous hex-digit with 4 bits to the left:
            code[bytesread >> 1] = (val | (tempbyte << 4)); //divide by 2 with every run and use as index for ea

            if (bytesread >> 1 != 5) {                // If we're at the checksum byte,
              checksum ^= code[bytesread >> 1];       // Calculate the checksum... (XOR) - XOR each value into ongoing checksum
            };
          } else {
            tempbyte = val;                           // Store the first hex digit first...
          };

          bytesread++;                                // ready to read next digit
          //Serial.println(bytesread, BIN);
        } 
      } 

      // Output to Serial:

      if (bytesread == 12) {                          // if 12 digit read is complete
        //unsigned long timeDiff = millis() - timeStart;
        
        //Serial.println(timeDiff);
        if (code[4] ==  tag_Lap)
        {
          Serial.println("Lap!");
          signalRFID();
          lapCounter++;
          //deliverWater(); enable water on every lap
          wheel.write(0); //zeros out the position on the rotary encoder read
          //need to update before check on next index
          readPosAndSpeed();
//          currentPosition = -wheel.read(); //current position always stored
//          currentPositionCm = currentPosition*cmConvFactor;

          //Assign next positions to current
          //memcpy(randPosListGlobal,randPosListGlobalNext,numRewardsPerLap);

          //Generate next positions
          randPosGenerator(randPosListGlobal,numRewardsPerLap);
          sort(randPosListGlobal, numRewardsPerLap);

          for(int idx=0; idx <= (numRewardsPerLap-1); idx++)
          {
            Serial.println(randPosListGlobal[idx]);
          }  
          currentRandPosIdx =0;
          //stopWaterFromList = 0;

        }
        if (code[4] == tag_OdorStart1)  //odor A start tag
        {
          if ((lapCounter >=lapOdorStart) && (lapCounter <=lapOdorEnd))
          {
            Serial.println(" ");
            Serial.println("OdorA Start");
            //deliverOdor();
          //signalRFID();
            setDAC(3000,0);
            delayMicroseconds(200); 
            setDAC(0,0);  
          }
           else
          {
            setDAC(3000,0);
            delayMicroseconds(200); 
            setDAC(0,0);            
          }         
          
        }
        if (code[4] == tag_OdorEnd1)
        {
        
          if ((lapCounter >=lapOdorStart) && (lapCounter <=lapOdorEnd))
          {
            Serial.println(" ");
            Serial.println("OdorA Stop");
            //stopOdor();

            setDAC(4000,0);
            delayMicroseconds(200); 
            setDAC(0,0); 
          }
          
          else
          {
            setDAC(4000,0);
            delayMicroseconds(200); 
            setDAC(0,0);            
          }

          }

          if (code[4] == tag_Cue1)
          {        
            firstCrossW = 1;
            Serial.println("XXXXXXXXXXXXXXXXXXXXXXXXX");
            Serial.println("Reward cleared");
            setDAC(500,0);
            delayMicroseconds(200); 
            setDAC(0,0);  
          }

          if (code[4] == tag_OdorEnd2) //cue2
          {        
            setDAC(1000,0);
            delayMicroseconds(200); 
            setDAC(0,0);  
          }

          
          if (code[4] == tag_Cue3)
          {        

            setDAC(1500,0);
            delayMicroseconds(200); 
            setDAC(0,0);  
          }

          if (code[4] == tag_Cue4)
          {        

            setDAC(2000,0);
            delayMicroseconds(200); 
            setDAC(0,0);  
          }
          
        if (code[4] == tag_Reward1) //water delivery
        {
//          if (firstCrossW == 1)
//          {
//            Serial.println("YYYYYYYYYYYYYYYYYYYYYYY");
//          //deliverWater();
//          //signalStim();
//          //setDAC(4000,0);
//          //delayMicroseconds(200); 
//          //setDAC(0,0);
//          //this variable must be reset by the absolute distance being greater than 25 cm from it
//          /firstCrossW = 0;
//          //distanceAtWaterCross = currentDistance;
//          }

          }          


        
      }
      bytesread = 0;
    }
  }  
}

void checkKeyboardIn()
{
  char serialInput = Serial.read();
  
  if (serialInput == 'w')
  {
    enableWater = true;
    deliverWater();
  }
  
}

void checkLapRFIDCross()
{
  //clear the stop on the RFID lap tag
  if ((currentDistance > 0) & (currentDistance <50))
    {
      if (currentDistance > thresRFIDdist)
      {
        firstCrossLap = 1;
      }
    }
}

void signalRFID()
{
    digitalWrite(outputRFID, HIGH); //PORTA = PORTA | B01000000; //set 28 high
    delayMicroseconds(200);
    digitalWrite(outputRFID, LOW); //PORTA = PORTA | B00000000; //set 28 low
    //delayMicroseconds(200);
}

void signalStim()
{
    digitalWrite(outputSTIM, HIGH); //PORTA = PORTA | B00000100; //set 24 high
    delayMicroseconds(200);
    digitalWrite(outputSTIM, LOW); //PORTA = PORTA | B00000000; //set 24 low
    //delayMicroseconds(200);
}

void signalTrigger()
{
    digitalWrite(triggerBNC, HIGH); //PORTA = PORTA | B00000100; //set 24 high
    delayMicroseconds(200);
    digitalWrite(triggerBNC, LOW); //PORTA = PORTA | B00000000; //set 24 low
    //delayMicroseconds(200);
}


void sort(long a[], int size) 
{ 
  for(int i=0; i<(size-1); i++) { 
    for(int o=0; o<(size-(i+1)); o++) { 
      if(a[o] > a[o+1]) {
        long t = a[o];
        a[o] = a[o+1]; 
        a[o+1] = t; 
      } 
    } 
  } 
}

void setup() {
  // put your setup code here, to run once:

  //DAC chip select pin setup
  pinMode (dacChipSelectPin, OUTPUT); 
  // set the ChipSelectPins high ini tially: 
  digitalWrite(dacChipSelectPin, HIGH);

    // initialise SPI:
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);         // Not strictly needed but just to be sure.
  SPI.setDataMode(SPI_MODE0);        // Not strictly needed but just to be sure.
  // Set SPI clock divider to 16, therfore a 1 MhZ signal due to the maximum frequency of the ADC.
  SPI.setClockDivider(SPI_CLOCK_DIV16);

    //set the DAC to Prarie to 0V output (digital value from 0 - 4095)
    setDAC(0,0); ///set out to zero volts

  randomSeed(analogRead(0));

Serial.begin(115200);
Serial2.begin(9600);
//Serial.println("Encoder start");

pinMode(solenoidL, OUTPUT);
digitalWrite(solenoidL, LOW);


pinMode(lickSensor, INPUT); //A9
//pinMode(A8, OUTPUT);
//pinMode(A10, OUTPUT);

//Signal to prarie that RFID tag detected
pinMode(outputRFID, OUTPUT);
digitalWrite(outputRFID, LOW);
//
//digitalWrite(A8, HIGH); //+5V for lick sensor
//digitalWrite(A10, LOW); //GND for lick sensor

 pinMode(formPin, OUTPUT); //RFID form pin (keep low)
 pinMode(rfidGnd, OUTPUT); //RFID ground pin
 digitalWrite(formPin,LOW);
 digitalWrite(rfidGnd, LOW);


 //Signal to prarie that RFID tag detected
pinMode(outputRFID, OUTPUT);
digitalWrite(outputRFID, LOW);
//
//digitalWrite(A8, HIGH); //+5V for lick sensor
//digitalWrite(A10, LOW); //GND for lick sensor

 pinMode(formPin, OUTPUT); //RFID form pin (keep low)
 pinMode(rfidGnd, OUTPUT); //RFID ground pin
 digitalWrite(formPin,LOW);
 digitalWrite(rfidGnd, LOW);

   pinMode(triggerBNC, OUTPUT); //set the output mode of the trigger BNC port
  digitalWrite(triggerBNC, LOW); //set to 0 volt state as default

  pinMode(outputSTIM, OUTPUT); //set the output mode of the trigger BNC port
  digitalWrite(outputSTIM, LOW); //set to 0 volt state as default

  pinMode(outputLICK, OUTPUT); //set the output mode of the trigger BNC port
  digitalWrite(outputLICK, LOW); //set to 0 volt state as default

  pinMode(outputROTARY, OUTPUT); //set the output mode of the trigger BNC port
  digitalWrite(outputROTARY, LOW); //set to 0 volt state as default

//initialize list of randow positions for next lap

//Generate random positions for current lap
randPosGenerator(randPosListGlobal,numRewardsPerLap);
sort(randPosListGlobal, numRewardsPerLap);

  for(int idx=0; idx <= (numRewardsPerLap-1); idx++)
  {
    Serial.println(randPosListGlobal[idx]);
  }

////Generate random positions for next lap
//randPosGenerator(randPosListGlobalNext,numRewardsPerLap);
//sort(randPosListGlobalNext, numRewardsPerLap);
//
//  for(int idx=0; idx <= (numRewardsPerLap-1); idx++)
//  {
//    Serial.println(randPosListGlobalNext[idx]);
//  }  

//attachInterrupt(digitalPinToInterrupt(21), indexAlert, RISING);

//attachInterrupt(digitalPinToInterrupt(lickSensor), deliverWater, RISING);

//Serial.println("Enabled t1");

unsigned long clockStart = millis();
runner.startNow();

}

void loop() {
  // put your main code here, to run repeatedly:

 runner.execute();

 
 if(Serial.available()>0){
   /*waterDeliveryDetailsFromGui = Serial.read();
   waterCommaIndex = waterDeliveryDetailsFromGui.indexOf(",");
   Serial.println(waterCommaIndex);
   Serial.println(waterDeliveryDetailsFromGui);
    
  // for(int i = 0; i<int(waterDeliveryDetailsFromGui.substring(0,waterCommaIndex-1)); i++){
  //  deliverWaterByGui(int(waterDeliveryDetailsFromGui.substring(watterCommaIndex,-1));
  // }
  **/
  waterDeliveryDetailsFromGui = Serial.readStringUntil("\n").toInt();
   Serial.println(waterDeliveryDetailsFromGui);
   deliverWaterByGui(waterDeliveryDetailsFromGui);

  
 }
 

}


//////////////DAC OUTPUT FUNCTION MCP4922///////////////
void setDAC(int value, int channel) {
  byte dacRegister = 0b00110000;                        // Sets default DAC registers B00110000, 1st bit choses DAC, A=0 B=1, 2nd Bit bypasses input Buffer, 3rd bit sets output gain to 1x, 4th bit controls active low shutdown. LSB are insignifigant here.
  int dacSecondaryByteMask = 0b0000000011111111;        // Isolates the last 8 bits of the 12 bit value, B0000000011111111.
  byte dacPrimaryByte = (value >> 8) | dacRegister;     //Value is a maximum 12 Bit value, it is shifted to the right by 8 bytes to get the first 4 MSB out of the value for entry into th Primary Byte, then ORed with the dacRegister  
  byte dacSecondaryByte = value & dacSecondaryByteMask; // compares the 12 bit value to isolate the 8 LSB and reduce it to a single byte. 
  // Sets the MSB in the primaryByte to determine the DAC to be set, DAC A=0, DAC B=1
  switch (channel) {
   case 0:
     dacPrimaryByte &= ~(1 << 7);     
   break;
   case 1:
     dacPrimaryByte |= (1 << 7);  
  }
  noInterrupts(); // disable interupts to prepare to send data to the DAC
  digitalWrite(dacChipSelectPin,LOW); // take the Chip Select pin low to select the DAC:
  SPI.transfer(dacPrimaryByte); //  send in the Primary Byte:
  SPI.transfer(dacSecondaryByte);// send in the Secondary Byte
  digitalWrite(dacChipSelectPin,HIGH);// take the Chip Select pin high to de-select the DAC:
  interrupts(); // Enable interupts
}
