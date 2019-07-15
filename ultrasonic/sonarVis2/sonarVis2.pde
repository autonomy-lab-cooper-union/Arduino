//DHVANIL SHAH
//SONAR PARKING SENSORS FOR VEHICLES VISUALIZER

import processing.serial.*;

Serial myPort;  // Create object from Serial class
String val;     // Data received from the serial port

float rectWidth = 90; //widht of 18 inches
float rectHeight = 200; //height(length) of 42 inches -- fixed quantitiy. Adjust width only to account for sizing changes

int sensorCount = 0;
int maxSensors = 5;

int[] xvals;
int[] yvals;
int[] orientation;
int[] distances; 


void setup(){
  size(480, 480);
  xvals = new int[maxSensors];
  yvals = new int[maxSensors];
  orientation = new int[maxSensors];
  distances = new int[maxSensors + 1];
  //arc(50, 55, 70, 70, 5*(PI/12), HALF_PI+PI/12); //bottom
  //arc(50, 55, 70, 70, HALF_PI+5*(PI/12), PI+PI/12); //left
  //arc(50, 55, 70, 70, PI+5*(PI/12), PI+HALF_PI+PI/12); //top
  //arc(50, 55, 70, 70, -PI/12, PI/12); //right
  String portName = Serial.list()[4]; //change the 0 to a 1 or 2 etc. to match your port
  myPort = new Serial(this, portName, 115200);
  printArray(Serial.list());
}
 
void draw(){
  background(0);
  rectMode(CENTER);  // Set rectMode to CENTER
  fill(255,0,0);  // Set fill red
  rect(240, 240, rectWidth, rectHeight);
  line(0, 330, 480, 330); //bottom line 
  line(0, 150, 480, 150); // top line
  line((240 - rectWidth/2 + 10), 330, (240 - rectWidth/2 + 10), 150); // left line
  line((240 + rectWidth/2 - 10), 330, (240 + rectWidth/2 - 10), 150); // right line'
   
   if ( myPort.available() > 0) 
  {  // If data is available,
  val = myPort.readStringUntil('\n');         // read it and store it in val
  if (val != null){
    distances = int(val.split(","));
    //printArray(int(val.split(","))); //print it out in the console
    printArray(distances);
  }
  }
  
  for(int i=0; i<sensorCount; i++) {
    fill(0,255,255);
    circle(xvals[i], yvals[i], 10);
    textAlign(CENTER);
    textSize(16);
    fill(255, 255, 0);
    text(i+1, xvals[i], yvals[i]);
    
    fill(0, 255, 255);
    int distancePX = distances[i] * 2;
    switch(orientation[i]) {
      case 1: 
        arc(xvals[i], yvals[i], distancePX, distancePX, PI+5*(PI/12), PI+HALF_PI+PI/12); //top
        break;
      case 2: 
        arc(xvals[i], yvals[i], distancePX, distancePX, 5*(PI/12), HALF_PI+PI/12); //bottom
        break;
      case 3: 
         arc(xvals[i], yvals[i], distancePX, distancePX, HALF_PI+5*(PI/12), PI+PI/12); //left
        break;
      case 4: 
        arc(xvals[i], yvals[i], distancePX, distancePX, -PI/12, PI/12); //right
        break;
    }
  }
}

void mouseReleased() {
  if(sensorCount < maxSensors){
   if(mouseY < 330 && mouseY > 150){
     if(mouseX >  (240 - rectWidth/2) && mouseX < (240 - rectWidth/2 + 10)){
       orientation[sensorCount] = 3;
       println("left");
       xvals[sensorCount] = mouseX;
       yvals[sensorCount] = mouseY;
       sensorCount++;
     }
     else if(mouseX > (240 + rectWidth/2 - 10) && mouseX < (240 + rectWidth/2)){
       orientation[sensorCount] = 4;
       println("right");
       xvals[sensorCount] = mouseX;
       yvals[sensorCount] = mouseY;
       sensorCount++;
     }
     else{
       return;
     }
   }
   if(mouseX > (240 - rectWidth/2) && mouseX < (240 + rectWidth/2)){
     if(330 < mouseY && mouseY < 345){
       orientation[sensorCount] = 2;
       println("bottom");
       xvals[sensorCount] = mouseX;
       yvals[sensorCount] = mouseY;
       sensorCount++;
     }
     else if(150 > mouseY && mouseY > 135){
       orientation[sensorCount] = 1;
       println("top");
       xvals[sensorCount] = mouseX;
       yvals[sensorCount] = mouseY;
       sensorCount++;
     }
     else{
       return;
     }
   }
  }
}
