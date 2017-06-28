import controlP5.*;
import grafica.*;
import processing.serial.*;
Serial myPort;
ControlP5 cp5;
float nPointsPos = 0;
float nPointsVel = 0;
String tempVal;
String[] splitTempVal;
GPlot posPlot;
GPlot velPlot;
GPlot lickPlot;
String totalTimeRunning = "NA";
String numberOfLaps = "NA";
String dropsDispensed = "NA";
String volumeDispensed = "NA";
String chargeState = "NA";
float startSendingTime;
float dropSendingTimeInc;
int numDrops;
int dropsSent;


void setup() {
 myPort = new Serial(this,"COM3",115200);
  myPort.bufferUntil('\n');
  size(1000, 650);
  PFont font = createFont("arial", 20);
  cp5 = new ControlP5(this);
  
 cp5.addTextfield("microsecond delay")
    .setPosition(250, 50)
      .setSize(100, 40)
        .setFont(font)
          .setAutoClear(false);
 cp5.addBang("Deliver Water")
    .setPosition(400, 50)
      .setSize(100, 40)
        .getCaptionLabel().align(ControlP5.CENTER, ControlP5.CENTER);    
  posPlot = new GPlot(this);
  posPlot.getXAxis().setAxisLabelText("time");
  posPlot.getYAxis().setAxisLabelText("position");
  posPlot.setPos(100,300);
  
  velPlot = new GPlot(this);
  velPlot.getXAxis().setAxisLabelText("time");
  velPlot.getYAxis().setAxisLabelText("velocity");
  velPlot.setPos(500,300);
  
  lickPlot = new GPlot(this);
  lickPlot.getXAxis().setAxisLabelText("time");
  lickPlot.getYAxis().setAxisLabelText("lick and deliver water");
  lickPlot.setPos(100,130);
  lickPlot.setDim(350,100);
  
}
 
 
 
 
void draw () {
  // delay(500);
   background(180);
   if(myPort.available()>0){
     tempVal = myPort.readStringUntil('\n');
    
     if(tempVal != null){
          println(tempVal);
         // println(tempVal.contains("--"));
          if(split(tempVal," ")[0].equals("Mouse")){//ignore the hackiness. it works
           lickPlot.addPoint(millis()/1000,0,"pt");
           
           
          }
          
          else if(split(tempVal," ")[0].equals("Water")){//ignore the hackiness. it works
           lickPlot.addPoint(millis()/1000,1,"pt");
           
         
          
          }
          
          else if(tempVal.contains("--")){
            splitTempVal = split(tempVal, "--");
            
            if(splitTempVal[0].equals("Distance traveled forward in cm")){
              posPlot.addPoint(millis()/1000.,float(splitTempVal[1]),"pt");
              nPointsPos++;
            }
            
             else if(splitTempVal[0].equals("Speed in cm/s is")){
              velPlot.addPoint(millis()/1000.,float(splitTempVal[1]),"pt");
              nPointsVel++;
            }
            else if(splitTempVal[0].equals("Amount of water dispensed in drops")){
              dropsDispensed = splitTempVal[1];
            }
            else if(splitTempVal[0].equals("Volume dispensed in uL")){
              volumeDispensed = splitTempVal[1];
            }
            else if(splitTempVal[0].equals("Charge state")){
              chargeState = splitTempVal[1];
            }
             else if(splitTempVal[0].equals("Full laps traveled")){
              numberOfLaps = splitTempVal[1];
            }
            else if(splitTempVal[0].equals("Program running for")){
              totalTimeRunning = splitTempVal[1];
            }
        }
     }
   }  
   
   lickPlot.setXLim( 0, millis()/1000*5/4);
   lickPlot.setYLim(-.1,1.1);
   posPlot.setXLim( 0, millis()/1000*5/4);
   velPlot.setXLim( 0, millis()/1000*5/4);

     
   posPlot.beginDraw();
   posPlot.drawBox();
   posPlot.drawXAxis();
   posPlot.drawYAxis();
   posPlot.drawPoints();
   posPlot.drawLines();
   posPlot.endDraw();
   
   velPlot.beginDraw();
   velPlot.drawBox();
   velPlot.drawXAxis();
   velPlot.drawYAxis();
   velPlot.drawPoints();
   velPlot.drawLines();
   velPlot.endDraw();
   
   lickPlot.beginDraw();
   lickPlot.drawBox();
   lickPlot.drawXAxis();
   //lickPlot.drawYAxis();
   lickPlot.drawPoints();
   lickPlot.endDraw();
   text("licks",100,260);
   text("H2O deliveries",0,200);

   
   fill(0);
   textSize(26); 
   text("program running for (secs): " + totalTimeRunning ,520,30);
   text("full laps traveled: " + numberOfLaps, 520,60);
   text("amount of water dispensed  in drops: " + dropsDispensed, 520,90);
   text("volume of water dispensed in uL: " + volumeDispensed, 520,120);
   text("charge state : " + chargeState, 520,150);

   
   //nPoints++;
   //delay(500);
}




void controlEvent(ControlEvent theEvent) {
  println(theEvent.getName());
  myPort.write(cp5.get(Textfield.class, "microsecond delay").getText()+"\n");
    
}