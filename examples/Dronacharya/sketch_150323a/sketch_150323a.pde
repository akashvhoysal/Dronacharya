import processing.serial.*;
import java.util.StringTokenizer;
Serial myPort;        // The serial port
int xPos = 50;         // horizontal position of the graph 

//Variables to draw a continuous line.
int lastxPos=1;
int lastheight=0;
int lastheightError=0;
int lastheightOut=0;

void setup () {
  // set the window size:
  size(600, 400);        

  // List all the available serial ports
  println(Serial.list());
  // Check the listed serial ports in your machine
  // and use the correct index number in Serial.list()[].

  myPort = new Serial(this, Serial.list()[5], 115200);  //

  // A serialEvent() is generated when a newline character is received :
  myPort.bufferUntil('\n');
  background(0Xffffff);      // set inital background:
}
void draw () {
  // everything happens in the serialEvent()
}

void serialEvent (Serial myPort) {
  // get the ASCII string:
  
  String inString = myPort.readStringUntil('\n');
  if (inString != null) {
    inString = trim(inString);   
    StringTokenizer st = new StringTokenizer(inString,",");    // trim off whitespaces.
    float inByte = Float.parseFloat(st.nextToken()); 
    float error =  Float.parseFloat(st.nextToken());
    float out = Float.parseFloat(st.nextToken());   // convert to a number.
    inByte = map(inByte, 0, 512, 0, height); //map to the screen height.
    error = map(error, 0, 512, 0, height);
    out = map(out, 0, 512, 0, height);  
    //Drawing a line from Last inByte to the new one.
    stroke(127,34,255);     //stroke color
    strokeWeight(1);        //stroke wider
    line(lastxPos, lastheight-200, xPos, (height - inByte)-200);
    stroke(34,34,255);
    line(0,height-200,700,height-200); 

    stroke(34,124,255);     //stroke color
    strokeWeight(1);        //stroke wider
    line(lastxPos, lastheightError-200, xPos, (height - error)-200);
    
    stroke(0,34,255);     //stroke color
    strokeWeight(1);        //stroke wider
    line(lastxPos, lastheightOut-200, xPos, (height - out)-200);
    
    lastxPos= xPos;
    lastheight= int(height-inByte);
    lastheightError= int(height-error);
    lastheightOut= int(height-out);
     
    // at the edge of the window, go back to the beginning:
    if (xPos >= width) {
      xPos = 0;
      lastxPos= 0;
      background(0Xffffff);  //Clear the screen.
    } 
    else {
      // increment the horizontal position:
      xPos++;
    }
  }
}

