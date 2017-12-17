import processing.serial.*; //<>// //<>// //<>// //<>// //<>// //<>//

Serial myPort;  // Create object from Serial class
String x;     // Data received from the serial port
String y;     // Data received from the serial port

ArrayList<PVector> path;

final float ALPHA= 0.5;
final int screenX = 1200;
final int screenY = 1200;
/* Number of columns and rows in the grid */
final int cols = 20;
final int rows = 20;
final int dimCellX = screenX/cols;
final int dimCellY = screenY/rows;

/* Initial position between 0- 700 */
int initialX = screenX/2;
int initialY = 30;

float oldX=0,oldY=0;

// 2D Array of objects
Cell[][] grid;


void setup() {
  String portName = Serial.list()[0]; //change the 0 to a 1 or 2 etc. to match your port
  myPort = new Serial(this, portName, 9600);
  myPort.bufferUntil('\n'); 
  myPort.clear();
  //fullScreen();
  size(1200, 1200);
  grid = new Cell[cols][rows];
  // i = columns and j = rows
  stroke(0); //Sets the color used to draw lines and borders around shapes
  // Color calculated using sine wave
  noFill();
  for (int i = 0; i < cols; i++) {
    for (int j = 0; j < rows; j++) {
      // Initialize each object
      grid[i][j] = new Cell(i*dimCellX, j*dimCellY, dimCellX, dimCellY, i+j);
      grid[i][j].display();
    }
  }
  //textSize(32);
  //fill(50);
  //text("180°", 320, 680);
  //text("270°", 20, 350);
  //text("0°", 335, 30);
  //text("90°", 620, 350);
  path=new ArrayList<PVector>();
}


float distance = 0;
float x_float = 0;
float y_float = 0;
// https://it.wikipedia.org/wiki/Sistema_di_coordinate_polari#Conversione_da_coordinate_polari_a_cartesiane
void draw() {
  pushStyle();
  fill(255, 2);
  rect(0, 0, width, height);
  noFill();
  for (int i = 0; i < cols; i++) {
    for (int j = 0; j < rows; j++) {
      grid[i][j].display();
    }
  }
  fill(50);
  text("Distance: ", 30, 50);
  try {
    if (Float.parseFloat(y) >0)
      text(y, 280, 50);
  }
  catch(Exception e) {
  }
  textSize(50);
  fill(255, 0, 0);
  ellipse(width/2, 30, 50, 50);
  popStyle();

  printCoordinate();
  pushStyle();
  noFill();
  strokeWeight(3);
  stroke(0xff0000ff);
  beginShape();
    for (PVector v:path) {
      vertex(v.x,v.y);
    }
  endShape();
  popStyle();

  //println(val); //print it out in the console
  //line(mouseX, mouseY, pmouseX, pmouseY);
}

void printCoordinate() {
  if (x != null && y != null) {
    print("X: "); 
    println(x); //print it out in the console
    print("Y: "); 
    println(y); //print it out in the console
    //distance = ((float(dist)*100) * 70) / 60;
    //x = distance * cos((float(angle)* PI/180.0)- HALF_PI);
    //y = distance * sin((float(angle)* PI/180.0)- HALF_PI);
    try {
      /* 1° convert in centimeter - 2° */
      x_float = ((Float.parseFloat(x)*100) * dimCellX) / 60;
      x_float = -x_float;
      y_float = ((Float.parseFloat(y)*100) * dimCellY) / 60;
      
      x_float=x_float*ALPHA+oldX*(1-ALPHA);
      y_float=y_float*ALPHA+oldY*(1-ALPHA);
      oldX=x_float;
      oldY=y_float;
      println(x_float,y_float);
    }
    catch(Exception e) {
    }
    fill(0, 180, 153);
    ellipse(initialX + x_float, initialY + y_float, 20, 20);
    path.add(new PVector(initialX + x_float,initialY + y_float));
    if (path.size()>32) path.remove(0);
    //ellipse(width/2, height/2, 20, 20);
  }
}

int num = 0;
void serialEvent(Serial myPort) {
  if (num == 0) {
    num++;
    x = myPort.readStringUntil('\n');         // read it and store it in val
    println(x); //print it out in the console
  } else {
    num = 0;
    y = myPort.readStringUntil('\n');         // read it and store it in val
    println(y); //print it out in the console
  }
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