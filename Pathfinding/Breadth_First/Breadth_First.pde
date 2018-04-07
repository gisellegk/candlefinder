import java.util.*;
final int STEP = 4;

// define colors in a crappy way. These are the indices for the COLORS array bellow.
final int BLACK = 0;
final int WHITE = 1;
final int BLUE = 2;
final int GRAY1 = 3;
final int PINK = 4;
final int GREEN = 5;
final int GRAY2 = 6;
final int RED = 7;
final color[] COLORS= {color(0), color(255), color(50,50,255), color(100), color(200,95,170), color(50,255,50), color(200), color(255,50,50)};

int MAP_WIDTH = 0;
int MAP_HEIGHT = 0;
int firstPoint = 0;

int centerDistance = 0;

int[] map; // map array for holding data. data should be a color index

ArrayList<Integer> pathPoints = new ArrayList<Integer>(); // List of points on the path.
ArrayList<PVector> vectors = new ArrayList<PVector>(); // vectors from each pivot on the path. Pivots are not stored though... but 
//vectors.get(0) is the direction the robot should travel
PVector currentpose = new PVector(200,200); // actual position of the robot
PVector targetpose = new PVector(32,32); // target postion set by mouse click

void setup() {
  size(404, 404); // windows size in pixels
  MAP_WIDTH = width/STEP; // set map size = window size / step size
  MAP_HEIGHT = height/STEP;
  map = new int[MAP_WIDTH*MAP_HEIGHT]; // set map size and fill with zeros
  strokeWeight(0);
  rectMode(CENTER);
  frameRate(30);
}
void mousePressed() { // called on mouse press
  if(mouseX > 0 && mouseX < width && mouseY > 0 && mouseY < height) // don't use the mouse positon if it's outside the window
   targetpose = new PVector(mouseX,+mouseY); // set the target pose in window pixels
}
void draw() { // periodic function. rate is set by frameRate(xx)

  map = new int[MAP_WIDTH*MAP_HEIGHT]; // clear map and fill with zeros
  vectors = new ArrayList<PVector>(); // clear vector list
  background(0);
  genMap1(); // draw gray walls
  inflate();  // draw the inflated walls in white
  genMap1(); // redraw the gray walls on top of the white so we can see them.
  
  // now we pathfind
  // pathfind(current X, current Y, target X, target Y). These values should be in MAP units, not WINDOW units, so currentpose and targetpose are devided by STEP.
  //pathFind(1*MAP_WIDTH+1,10*MAP_WIDTH+100); // example path find between two points
  pathFind(floor(currentpose.x/(float)STEP)*MAP_WIDTH+floor(currentpose.y/STEP),floor(targetpose.x/(float)STEP)*MAP_WIDTH+floor(targetpose.y/STEP));
  
  // now we optimize the path using some basic line of sight ray trace thing
  // this overwites the pathPoints with the new optimized path
  // lineOfSight(COLOR Index)
  lineOfSight(PINK);
  divergeFromWall(RED);
  
  
  // draw all the map data in the window
  for (int y = 0; y < MAP_HEIGHT; y ++) {
    for (int x = 0; x < MAP_WIDTH; x ++) {  
      if(map[x*MAP_WIDTH+y] > 0){
        fill(COLORS[map[x*MAP_WIDTH+y]]); // set the fill color based on the map data
        rect(x*STEP,y*STEP,STEP,STEP); // draw a map pixel
      }
    }
  }
  
  // draw currentpose as a green dot
  stroke(COLORS[GREEN]);  
  strokeWeight(8);
  point(currentpose.x-STEP/2,currentpose.y-STEP/2);
  
  // if there is a path and a first vector, draw it and move
  if(pathPoints.size()>0 && vectors.size()>0) {
    // draw arrow
    strokeWeight(2);
    arrow(round((pathPoints.get(0)/MAP_WIDTH)*STEP),round((pathPoints.get(0)%MAP_WIDTH)*STEP),round((pathPoints.get(0)/MAP_WIDTH+vectors.get(0).x*10)*STEP),round((pathPoints.get(0)%MAP_WIDTH+vectors.get(0).y*10)*STEP));
    
    int distance = pathPoints.size(); // distance is the legth of the path in pixels
    
    // set a speed
    int speed = 1;
    if(distance < 40) // slow down if closer than 40px
       speed = floor(distance*speed/40.0)+1;
    // now move in the direction that vector.get(0) is pointing
    currentpose = new PVector(currentpose.x+vectors.get(0).x*speed,currentpose.y+vectors.get(0).y*speed);
  }
  noStroke();
}

void arrow(int x1, int y1, int x2, int y2) {
  line(x1, y1, x2, y2);
  pushMatrix();
  translate(x2, y2);
  float a = atan2(x1-x2, y2-y1);
  rotate(a);
  line(0, 0, -5, -5);
  line(0, 0, 5, -5);
  line(-5, -5, 5, -5);
  popMatrix();
} 

void drawMapLine(int angle, int x, int y, int length) {
  float rise = sin(angle/57.6);
  float run = cos(angle/57.6);
  for (int m = 0; m < length; m ++) {
    int xdraw = round(m*run+x);
    int ydraw = round(m*rise+y);
    if(!(xdraw < 0 || xdraw > MAP_WIDTH-1 || ydraw < 0 || ydraw> MAP_HEIGHT-1))
      map[xdraw*MAP_WIDTH + ydraw]=GRAY2;
 }
}

void genMap1() {
  drawMapLine(20, 10, 10, 30);
  drawMapLine(30, 25, 25, 25);
  drawMapLine(0, 25, 30, 80);
  drawMapLine(0, 0, 60, 50);
  drawMapLine(50, 80, 50, 25);
  drawMapLine(70, 30, 50, 60);
  drawMapLine(120, 130, 50, 50);
  drawMapLine(50, 80, 100, 25);
  drawMapLine(20, 80, 10, 40);
}

//angle, x, y, length (all in px)
//width and height = 0-99 pixels
void genMap2() {
  drawMapLine(0, 3,3 , 96);
  drawMapLine(90, 3,3 , 96);
  drawMapLine(0, 3, 99 , 96);
  drawMapLine(90, 99,3 , 96);
  drawMapLine(0, 3, 27 , 16);
  drawMapLine(90, 19, 27 , 8);
  drawMapLine(180, 19, 35 , 8);
  drawMapLine(0, 3, 43 , 28);
  drawMapLine(270, 31, 43, 8);
  drawMapLine(0, 3, 63, 16);
  drawMapLine(90, 19 , 59, 16);
  drawMapLine(0, 11,11 , 24);
  drawMapLine(90, 11,11 , 16);
  drawMapLine(90, 35,11 , 8);
  drawMapLine(0, 43,7 , 56);
  drawMapLine(90, 63, 7 , 4);
  drawMapLine(90, 79,7 , 4);
  drawMapLine(0, 35,15 , 20);
  drawMapLine(90, 55,15 , 20);
  drawMapLine(0, 95,11 , 4);
  drawMapLine(90, 71, 15, 4);
  drawMapLine(0, 71, 19, 4);
  drawMapLine(90,75,19,12);
  drawMapLine(0,87,15,1);
  drawMapLine(90,83,19,4);
  drawMapLine(0,83,23,8);
  drawMapLine(90,87,23,4);
  drawMapLine(0,19,19,8);
  drawMapLine(90,27,19,8);
  drawMapLine(0,27,27,20);
  drawMapLine(0,55,19,8);
  drawMapLine(90,63,19,8);
  drawMapLine(0,63,27,12);
  drawMapLine(0,55,35,8);
  drawMapLine(0,43,23,4);
  drawMapLine(90,39,27,8);
  drawMapLine(90,47,23,20);
  drawMapLine(0,39,43,36);
  drawMapLine(0,71,39,12);
  drawMapLine(0,83,35,9);
  drawMapLine(0,95,31,4);
  drawMapLine(0,83,43,8);
  drawMapLine(90,11,51,4);
  drawMapLine(0,11,51,16);
  drawMapLine(90,27,51,12);
  drawMapLine(0,27,63,8);
  drawMapLine(90,35,63,4);
  drawMapLine(0,35,51,8);
  drawMapLine(90,35,51,4);
  drawMapLine(90,11,71,12);
  drawMapLine(0,11,83,36);
  drawMapLine(0,3,91,16);
  drawMapLine(90,75,39,12);
  drawMapLine(0,51,51,28);
  drawMapLine(90,51,51,8);
  drawMapLine(0,43,59,28);
  drawMapLine(90,43,59,16);
  drawMapLine(0,43,75,4);
  drawMapLine(90,27,71,28);
  drawMapLine(0,27,75,8);
  drawMapLine(0,87,51,12);
  drawMapLine(0,87,55,4);
  drawMapLine(90,79,51,16);
  drawMapLine(0,51,67,28);
  drawMapLine(90,63,67,8);
  drawMapLine(0,55,75,8);
  drawMapLine(0,95,63,4);
  drawMapLine(0,95,79,4);
  drawMapLine(90,87,63,36);
  drawMapLine(0,87,71,4);
  drawMapLine(0,71,75,16);
  drawMapLine(90,91,87,4);
  drawMapLine(0,55,83,24);
  drawMapLine(90,55,83,8);
  drawMapLine(0,35,91,44);
  drawMapLine(90,79,91,8);
}

void inflate() {
  int [] newmap = new int[MAP_WIDTH*MAP_HEIGHT];
  for(int i = 0; i < MAP_WIDTH*MAP_HEIGHT; i+=1) {
    if(map[i] == GRAY2) {
      newmap[i] = WHITE;
      int currentPixel_X = i/MAP_WIDTH;
      int currentPixel_Y = i%MAP_WIDTH;
      int NUMRAYS = 60;
      int inflateSize = 2;
      for(int ray = 0; ray < NUMRAYS; ray++) {
        int a = (360/NUMRAYS)*ray;
        int rise = round(sin(a/57.6)*inflateSize);
        int run = round(cos(a/57.6)*inflateSize);
        for(int m = 1; m < inflateSize+1; m++){
          int next_X = currentPixel_X + rise*m/inflateSize;
          int next_Y = currentPixel_Y + run*m/inflateSize;
          if(!(next_X < 0 || next_X > MAP_WIDTH-1 || next_Y < 0 || next_Y> MAP_HEIGHT-1)) {
            newmap[next_X*MAP_WIDTH+next_Y] = WHITE;
          }
        }
      }
    }
  }
  map=newmap;
}

void pathFind(int start, int goal) {
  pathPoints = new ArrayList<Integer>();
  if(start == 0 || goal ==0 || start == goal) return;
  pathPoints = new ArrayList<Integer>();
  ArrayList<Integer> frontier = new ArrayList<Integer>();
  int[] cameFrom = new int[MAP_WIDTH*MAP_HEIGHT];
  frontier.add(start);
  cameFrom[frontier.get(0)] = frontier.get(0);
  int finalTarget = -1;
  boolean escape = false;// (map[start] == WHITE || map[start] == GRAY2);    

  
  while(frontier.size() > 0 && finalTarget == -1){
    int currentPixel = frontier.get(0);
    frontier.remove(0);
    if(currentPixel == goal || (escape == true && map[currentPixel] == BLACK)) {
      finalTarget = currentPixel;
    } //else if(map[currentPixel]==GRAY2 || map[currentPixel]==WHITE){
      // ur stuck
      
    //}
    else {
      //map[currentPixel] = GRAY1;
      int currentPixel_X = currentPixel/MAP_WIDTH;
      int currentPixel_Y = currentPixel%MAP_WIDTH;
      for(int i = 0; i < 4; i++) {
        int a = 90*i;
        int rise = round(sin(a/57.6));
        int run = round(cos(a/57.6));
        int next_X = currentPixel_X + run;
        int next_Y = currentPixel_Y + rise;
        if(next_X < 0 || next_X > MAP_WIDTH-1 || next_Y < 0 || next_Y> MAP_HEIGHT-1) continue;
        int next =next_X*MAP_WIDTH+next_Y;
        if(cameFrom[next] == 0) {
          if(map[next] != WHITE || escape) {
            frontier.add(next);
            cameFrom[next] = currentPixel;
          }
        }
      }
    }
    if(frontier.size() == 0 && finalTarget == -1) escape = true;
  }
  if(finalTarget!=-1 ) {
    int linePos = finalTarget;
    do{
      pathPoints.add(0,linePos);
      //map[linePos] = BLUE;
      linePos = cameFrom[linePos];
    } while(linePos != start);
    pathPoints.add(0,start);
    //map[linePos] = BLUE;
  }
  
}
void divergeFromWall(int drawColor) {
  if(vectors.size() > 0) {
    float angle = atan(vectors.get(0).y/vectors.get(0).x);
    if(vectors.get(0).x > 0 && vectors.get(0).y > 0) {
      //first quadrant
    } else if(vectors.get(0).x < 0 && vectors.get(0).y > 0) {
      //second quadrant
      angle+=PI;
    } else if(vectors.get(0).x < 0 && vectors.get(0).y <= 0) {
      //third quadrant
      angle+=PI;
    } else if(vectors.get(0).x > 0 && vectors.get(0).y <= 0) {
      //fourth quadrant
      angle+=2*PI;
    }

    int OFFEST = 2;
    int degOFFSET = 10;

    int length_L = 0;
    int START_X_L = round(pathPoints.get(0)/MAP_WIDTH + OFFEST * cos(angle-PI/2));
    int START_Y_L = round(pathPoints.get(0)%MAP_WIDTH + OFFEST * sin(angle-PI/2));
    if(START_X_L > 0 && START_Y_L > 0 && START_X_L < MAP_WIDTH && START_Y_L < MAP_HEIGHT) {
      for(length_L = 0; length_L < 100; length_L++){
        int x = round(START_X_L + length_L * cos(angle-(degOFFSET/57.2958)));
        int y = round(START_Y_L + length_L * sin(angle-(degOFFSET/57.2958)));
        if(x > 0 && y > 0 && x < MAP_WIDTH && y < MAP_HEIGHT) {
          if(map[x*MAP_WIDTH+y] == WHITE || map[x*MAP_WIDTH+y] == GRAY2) break;
          //map[x*MAP_WIDTH+y] = drawColor;
        } else break;
      }
    }
    
    int length_R = 0;
    int START_X_R = round(pathPoints.get(0)/MAP_WIDTH + OFFEST * cos(angle+PI/2));
    int START_Y_R = round(pathPoints.get(0)%MAP_WIDTH + OFFEST * sin(angle+PI/2));
    if(START_X_R > 0 && START_Y_R > 0 && START_X_R < MAP_WIDTH && START_Y_R < MAP_HEIGHT) {
      for(length_R = 0; length_R < 100; length_R++){
        int x = round(START_X_R + length_R * cos(angle+(degOFFSET/57.2958)));
        int y = round(START_Y_R + length_R * sin(angle+(degOFFSET/57.2958)));
        if(x > 0 && y > 0 && x < MAP_WIDTH && y < MAP_HEIGHT) {
          if(map[x*MAP_WIDTH+y] == WHITE || map[x*MAP_WIDTH+y] == GRAY2) break;
          //map[x*MAP_WIDTH+y] = drawColor;
        } else break;
      }
    }
    
    if(length_L > length_R + 2 && length_L > 2) {
      //go a little to the left
      vectors.set(0,new PVector(cos(angle-(degOFFSET/57.2958)), sin(angle-(degOFFSET/57.2958))));
    } else if(length_R > length_L + 2 && length_R > 2) {
      //go a little to the right
      vectors.set(0,new PVector(cos(angle+(degOFFSET/57.2958)), sin(angle+(degOFFSET/57.2958))));
    } else {
    }
    
  }
}
  
void lineOfSight(int drawColor) {
  if(pathPoints.size()>0) {
    ArrayList<Integer> newPathPoints = new ArrayList<Integer>();
    int s = 0;
    while(s<pathPoints.size()-1) {
      for(int e = pathPoints.size()-1; e > s; e--) {
        boolean lineofsight = false;
        float start_X = pathPoints.get(s)/MAP_WIDTH;
        float start_Y = pathPoints.get(s)%MAP_WIDTH;
        float end_X = pathPoints.get(e)/MAP_WIDTH;
        float end_Y = pathPoints.get(e)%MAP_WIDTH;
        //map[round(end_X)*MAP_WIDTH+round(end_Y)] = PINK;
        float diff_X = end_X-start_X;
        float diff_Y = end_Y-start_Y;
        float angle = atan(diff_Y/diff_X);
        if(diff_X > 0 && diff_Y > 0) {
          //first quadrant
        } else if(diff_X < 0 && diff_Y > 0) {
          //second quadrant
          angle+=PI;
        } else if(diff_X < 0 && diff_Y <= 0) {
          //third quadrant
          angle+=PI;
        } else if(diff_X > 0 && diff_Y <= 0) {
          //fourth quadrant
          angle+=2*PI;
        }
        int distance = ceil(sqrt(pow(diff_Y,2)+pow(diff_X,2)));
        for(int m = 0; m < distance+1; m++){
          int x = round(start_X + m * cos(angle));
          int y = round(start_Y + m * sin(angle));
          if(x < 0 ||  x> MAP_WIDTH-1 || y < 0 || y> MAP_HEIGHT-1) continue;
          if(map[x*MAP_WIDTH+y]  == 1) {
            break;
          } else {
            if(x == end_X && y == end_Y) 
              lineofsight = true;
          }
        }
        if(lineofsight) {
          if(vectors.size() == 0) centerDistance = distance;
          vectors.add(new PVector(cos(angle), sin(angle)));
          for(int m = 0; m < distance; m++){
            int x = round(start_X + m* cos(angle));
            int y = round(start_Y + m* sin(angle));
            map[x*MAP_WIDTH+y] = drawColor;
            newPathPoints.add(x*MAP_WIDTH+y);
          }
          s=e;
          break;
        }
      }
    }
    newPathPoints.add(pathPoints.get(pathPoints.size()-1));
    map[pathPoints.get(pathPoints.size()-1)] = drawColor;
    pathPoints=(newPathPoints);
  }
}
