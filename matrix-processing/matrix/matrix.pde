import processing.serial.*; //<>// //<>//

Serial myPort;  // Create object from Serial class
String dist;     // Data received from the serial port
String angle;     // Data received from the serial port

/* Initial position between 0- 700 */
int initialX = 350;
int initialY = 350;

// 2D Array of objects
Cell[][] grid;

// Number of columns and rows in the grid
int cols = 10;
int rows = 10;

void setup() {
  String portName = Serial.list()[0]; //change the 0 to a 1 or 2 etc. to match your port
  myPort = new Serial(this, portName, 9600);
  myPort.clear();
  //fullScreen();
  size(700, 700);
  grid = new Cell[cols][rows];
  // i = columns and j = rows
  stroke(0); //Sets the color used to draw lines and borders around shapes
  // Color calculated using sine wave
  noFill();
  for (int i = 0; i < cols; i++) {
    for (int j = 0; j < rows; j++) {
      // Initialize each object
      grid[i][j] = new Cell(i*70, j*70, 70, 70, i+j);
      grid[i][j].display();
    }
  }
  textSize(32);
  fill(50);
  text("180°", 320, 680);
  text("270°", 20, 350);
  text("0°", 335, 30);
  text("90°", 620, 350);
}


float distance = 0;
float x = 0;
float y = 0;
// https://it.wikipedia.org/wiki/Sistema_di_coordinate_polari#Conversione_da_coordinate_polari_a_cartesiane
void draw() {
  if ( myPort.available() > 0) 
  {  // If data is available,
    pushStyle();
    fill(255, 10);
    rect(0, 0, width, height);
    noFill();
    for (int i = 0; i < cols; i++) {
      for (int j = 0; j < rows; j++) {
        grid[i][j].display();
      }
    }
    fill(50);
    text("180°", 320, 680);
    text("270°", 20, 350);
    text("0°", 330, 30);
    text("90°", 620, 350);
    fill(255, 0, 0);
    ellipse(width/2, height/2, 20, 20);
    popStyle();


    angle = myPort.readStringUntil(10);         // read it and store it in val
    dist = myPort.readStringUntil(10);         // read it and store it in val
    if (dist != null && angle != null) {
      print("Distance: "); 
      println(dist); //print it out in the console
      print("Angle: "); 
      println(angle); //print it out in the console
      distance = ((float(dist)*100) * 70) / 60;
      x = distance * cos((float(angle)* PI/180.0)- HALF_PI);
      y = distance * sin((float(angle)* PI/180.0)- HALF_PI);
      fill(0, 180, 153);
      ellipse(initialX + x, initialY + y, 9, 9);
    }
  } 
  //println(val); //print it out in the console
  //line(mouseX, mouseY, pmouseX, pmouseY);
}

// A Cell object
class Cell {
  // A cell object knows about its location in the grid 
  // as well as its size with the variables x,y,w,h
  float x, y;   // x,y location
  float w, h;   // width and height
  float angle; // angle for oscillating brightness

  // Cell Constructor
  Cell(float tempX, float tempY, float tempW, float tempH, float tempAngle) {
    x = tempX;
    y = tempY;
    w = tempW;
    h = tempH;
    angle = tempAngle;
  } 

  void display() {
    rect(x, y, w, h);
  }
}         