import processing.serial.*; //<>// //<>// //<>// //<>// //<>//

Serial myPort;  // Create object from Serial class
String angle;     // Data received from the serial port
String dist;     // Data received from the serial port

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


float dist_float = 0;
float angle_float = 0;
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
    //if (Float.parseFloat(y) >0)
    //  text(y, 280, 50);
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

float new_angle = 0;
float old_angle = 180;
float alfa = 0.2;
void printCoordinate() {
  if (angle != null && dist != null) {
    try {
      dist_float = (float(dist)*100) * dimCellX / 60;
      //println(dist_float);
      if(float(angle) >= 90 && float(angle) <= 270){
        new_angle = float(angle) * alfa + (1- alfa) *old_angle;
        //println(new_angle);
        x_float = (dist_float * cos((new_angle * PI/180.0)- HALF_PI));
        y_float = (dist_float * sin((new_angle * PI/180.0)- HALF_PI));
        old_angle = new_angle;
      }
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
    angle = myPort.readStringUntil('\n');         // read it and store it in val
    print("angle: "); 
    println(angle); //print it out in the console
  } else {
    num = 0;
    dist = myPort.readStringUntil('\n');         // read it and store it in val
     print("distance: "); 
    println(dist); //print it out in the console
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