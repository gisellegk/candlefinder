import java.util.*;
final int STEP = 4;


final int BLACK = 0;
final int WHITE = 1;
final int BLUE = 2;
final int GRAY1 = 3;
final int PINK = 4;
final int GREEN = 5;
final int GRAY2 = 6;
final color[] COLORS= {color(0), color(255), color(50,50,255), color(100), color(255,105,180), color(50,255,50), color(200)};

int MAP_WIDTH = 0;
int MAP_HIEGHT = 0;
int firstPoint = 0;

int[] map;

ArrayList<Integer> pathPoints = new ArrayList<Integer>();
ArrayList<PVector> vectors = new ArrayList<PVector>();
PVector robopose = new PVector(10,10);

void setup() {
  size(500, 500);
  MAP_WIDTH = width/STEP;
  MAP_HIEGHT = height/STEP;
  map = new int[MAP_WIDTH*MAP_HIEGHT];
  strokeWeight(0);
  rectMode(CENTER);
  frameRate(30);
}
void mousePressed() {
  if(mouseX > 0 && mouseX < width && mouseY > 0 && mouseY < height)
   robopose = new PVector(mouseX,+mouseY);
}
void draw() {
  map = new int[MAP_WIDTH*MAP_HIEGHT];
  vectors = new ArrayList<PVector>();
  background(0);
  genMap();
  inflate();
  genMap();
  //pathFind(1*MAP_WIDTH+1,10*MAP_WIDTH+100);
  //pathFind(mouseX/STEP*MAP_WIDTH+mouseY/STEP,10*MAP_WIDTH+100);
  pathFind(floor(robopose.x/(float)STEP)*MAP_WIDTH+floor(robopose.y/STEP),MAP_WIDTH/2*MAP_WIDTH+MAP_HIEGHT/2);
  lineOfSight(PINK);
  for (int y = 0; y < MAP_HIEGHT; y ++) {
    for (int x = 0; x < MAP_WIDTH; x ++) {  
      if(map[x*MAP_WIDTH+y] > 0){
        fill(COLORS[map[x*MAP_WIDTH+y]]);
        rect(x*STEP,y*STEP,STEP,STEP);
      }
    }
  }
  stroke(COLORS[GREEN]);  
  strokeWeight(8);
  point(robopose.x-STEP/2,robopose.y-STEP/2);
  strokeWeight(2);
  if(pathPoints.size()>0 && vectors.size()>0) {
    arrow(round((pathPoints.get(0)/MAP_WIDTH)*STEP),round((pathPoints.get(0)%MAP_WIDTH)*STEP),round((pathPoints.get(0)/MAP_WIDTH+vectors.get(0).x*10)*STEP),round((pathPoints.get(0)%MAP_WIDTH+vectors.get(0).y*10)*STEP));
    strokeWeight(6);
    int distance = pathPoints.size();
    int speed = 1;
    if(distance < 40)
       speed = floor(distance*speed/40.0)+1;      
    robopose = new PVector(robopose.x+vectors.get(0).x*speed,robopose.y+vectors.get(0).y*speed);
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
    if(!(xdraw < 0 || xdraw > MAP_WIDTH-1 || ydraw < 0 || ydraw> MAP_HIEGHT-1))
      map[xdraw*MAP_WIDTH + ydraw]=GRAY2;
 }
}

void genMap() {
  drawMapLine(20, 10, 10, 30);
  drawMapLine(30, 25, 25, 25);
  drawMapLine(0, 25, 30, 80);
  drawMapLine(0, 0, 60, 50);
  drawMapLine(50, 80, 50, 25);
  drawMapLine(70, 30, 50, 60);
  drawMapLine(120, 130, 50, 50);
  drawMapLine(50, 80, 100, 25);
}

void inflate() {
  int [] newmap = new int[MAP_WIDTH*MAP_HIEGHT];
  for(int i = 0; i < MAP_WIDTH*MAP_HIEGHT; i+=1) {
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
          if(!(next_X < 0 || next_X > MAP_WIDTH-1 || next_Y < 0 || next_Y> MAP_HIEGHT-1)) {
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
  int[] cameFrom = new int[MAP_WIDTH*MAP_HIEGHT];
  frontier.add(start);
  cameFrom[frontier.get(0)] = frontier.get(0);
  int finalTarget = -1;
  boolean escape = false;// (map[start] == WHITE || map[start] == GRAY2);    
  println(escape);
  
  while(frontier.size() > 0 && finalTarget == -1){
    int currentPixel = frontier.get(0);
    frontier.remove(0);
    if(currentPixel == goal || (escape == true && map[currentPixel] == BLACK)) {
      finalTarget = currentPixel;
    } else {
      //map[currentPixel] = GRAY1;
      int currentPixel_X = currentPixel/MAP_WIDTH;
      int currentPixel_Y = currentPixel%MAP_WIDTH;
      for(int i = 0; i < 4; i++) {
        int a = 90*i;
        int rise = round(sin(a/57.6));
        int run = round(cos(a/57.6));
        int next_X = currentPixel_X + run;
        int next_Y = currentPixel_Y + rise;
        if(next_X < 0 || next_X > MAP_WIDTH-1 || next_Y < 0 || next_Y> MAP_HIEGHT-1) continue;
        int next =next_X*MAP_WIDTH+next_Y;
        if(cameFrom[next] == 0) {
          if(map[next] != WHITE || escape) {
            frontier.add(next);
            cameFrom[next] = currentPixel;
          }
        }
      }
    }
    println(frontier.size());
    if(frontier.size() == 0 && finalTarget == -1) escape = true;
  }
  if(finalTarget!=-1 ) {
    int linePos = finalTarget;
    do{
      pathPoints.add(0,linePos);
      map[linePos] = BLUE;
      linePos = cameFrom[linePos];
    } while(linePos != start);
    pathPoints.add(0,start);
    map[linePos] = BLUE;
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
          if(x < 0 ||  x> MAP_WIDTH-1 || y < 0 || y> MAP_HIEGHT-1) continue;
          if(map[x*MAP_WIDTH+y]  == 1) {
            break;
          } else {
            if(x == end_X && y == end_Y) 
              lineofsight = true;
          }
        }
        if(lineofsight) {
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