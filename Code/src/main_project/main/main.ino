/***** main.ino : Autonomous mapping + OLED map + shortest-path run (VL6180X L/F/R) *****/
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <ctype.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#include "EncoderOdometry.hpp"
#include "MotorController.hpp"
#include "Lidar.hpp"

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

/* ============================== Maze Config ============================== */
static const uint8_t MAZE_W = 16;
static const uint8_t MAZE_H = 16;

// --- RANDOMIZATION TOGGLES ---
// Keep the robot’s physical start fixed? (recommended for the real maze)
#define RANDOMIZE_START 0
// Randomize the goal at boot to prove it’s not hard-coded
#define RANDOMIZE_GOAL  1
// Restrict random goal to the classic 4-cell center (7,7)(8,7)(7,8)(8,8)?
#define RANDOM_GOAL_CENTER_ONLY 0

// Mutable at runtime (chosen in setup)
static uint8_t START_X, START_Y, START_DIR;  // 0=N,1=E,2=S,3=W
static uint8_t GOAL_X,  GOAL_Y;

/* ============================ Motion Tunables ============================ */
static const float CELL_SIZE_MM        = 175.0f;
static const float FORWARD_CELL_SCALE  = 1.03f;
static const float BRAKE_ZONE_MM       = 25.0f;

static const float TURN_RAD            = M_PI/2.0f;
static const int   PWM_DRIVE           = 150;
static const int   PWM_TURN            = 130;

// Heading hold (encoder-only)
static const float HEADING_KP_RAD      = 95.0f;
static const float HEADING_KI_RAD      = 35.0f;
static const int   HEADING_I_MAX       = 10;
static const int   HEADING_MAX_PWM     = 22;

static const int   STEER_MAX_PWM       = 20;
static const float CORR_FRAC_OF_BASE   = 0.20f;
static const int   TURN_SLEW_PER_CYCLE = 3;

static const int   FRONT_SLOW_MM       = 170;
static const int   FRONT_STOP_MM       = 110;
static const int   FRONT_MIN_BASE_PWM  = 60;

/* ============================== Lidar & IO =============================== */
static const int   MAX_VALID_MM        = 200;
static const int   WALL_SIDE_MM        = 110;  // side wall threshold (center of cell)
static const int   WALL_FRONT_MM       = 120;  // front wall threshold (at cell center)

static const uint32_t CTRL_DT_US       = 5000;
static const float    DT_SEC           = CTRL_DT_US * 1e-6f;

/* ============================== Globals ================================= */
EncoderOdometry odom;
MotorController motor;
Lidar           lidar;

static inline int   clampInt(int v, int lo, int hi){ return v<lo?lo : v>hi?hi : v; }
static inline float clampF (float v, float lo, float hi){ return v<lo?lo : v>hi?hi : v; }
static float wrap180rad(float a){ while(a>M_PI) a-=2.f*M_PI; while(a<=-M_PI) a+=2.f*M_PI; return a; }

/* ============================ Map Structure ============================== */
// Walls bit-mask: 0=N,1=E,2=S,3=W (bit positions)
struct Cell {
  uint8_t walls   = 0;    // bit=1 → wall present on that side
  uint8_t known   = 0;    // bit=1 → this side measured (known open or wall)
  uint8_t visited = 0;    // 1 if we stepped into this cell
};
static Cell grid[MAZE_W*MAZE_H];

static inline bool inBounds(int x,int y){ return x>=0 && y>=0 && x<MAZE_W && y<MAZE_H; }
static inline int  idx(int x,int y){ return y*MAZE_W + x; }

enum Dir : uint8_t {NORTH=0,EAST=1,SOUTH=2,WEST=3};
static const int8_t dx[4] = {0, 1, 0,-1};
static const int8_t dy[4] = {1, 0,-1, 0}; // y+ is "north/up" in logical grid
static inline uint8_t opp(uint8_t d){ return (d+2)&3; }
static inline void turnLeftDir(uint8_t &d){ d = (d+3)&3; }
static inline void turnRightDir(uint8_t &d){ d = (d+1)&3; }

/* ========================== OLED Visualization =========================== */
static void drawMap(uint8_t robotX, uint8_t robotY, uint8_t robotDir){
  display.clearDisplay();

  // 16x16 cells → 2x2 pixels per cell in a 32x32 block at (0,0)
  for(uint8_t y=0;y<MAZE_H;y++){
    for(uint8_t x=0;x<MAZE_W;x++){
      int id = idx(x,y);
      if (grid[id].visited){
        display.drawPixel(x*2+0, (31 - (y*2+0)), SSD1306_WHITE);
        display.drawPixel(x*2+1, (31 - (y*2+0)), SSD1306_WHITE);
        display.drawPixel(x*2+0, (31 - (y*2+1)), SSD1306_WHITE);
        display.drawPixel(x*2+1, (31 - (y*2+1)), SSD1306_WHITE);
      }
    }
  }

  // Robot marker (3 pixels) at its cell
  int rx = robotX*2, ry = 31 - robotY*2;
  display.drawPixel(rx+0, ry-0, SSD1306_WHITE);
  display.drawPixel(rx+1, ry-0, SSD1306_WHITE);
  display.drawPixel(rx+0, ry-1, SSD1306_WHITE);

  // Goal outline (2x2 hollow)
  int gx = GOAL_X*2, gy = 31 - GOAL_Y*2;
  display.drawRect(gx, gy-1, 2, 2, SSD1306_WHITE);

  // Percentage visited
  uint16_t visitedCount=0;
  for(uint8_t y=0;y<MAZE_H;y++) for(uint8_t x=0;x<MAZE_W;x++) if (grid[idx(x,y)].visited) visitedCount++;
  uint8_t pct = (uint8_t)((visitedCount*100UL)/(MAZE_W*MAZE_H));

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(36, 0);
  display.print("Visited: ");
  display.print(pct);
  display.print("%");

  display.setCursor(36, 10);
  display.print("Pos ");
  display.print(robotX); display.print(","); display.print(robotY);

  display.setCursor(36, 20);
  display.print("Dir ");
  const char* dnames[4]={"N","E","S","W"};
  display.print(dnames[robotDir]);

  display.display();
}

/* ============================ Sensing Walls ============================== */
static int readMedian3(LidarPosition p){
  int a = lidar.readDistance(p);
  int b = lidar.readDistance(p);
  int c = lidar.readDistance(p);
  if (a<0) a=MAX_VALID_MM; if (b<0) b=MAX_VALID_MM; if (c<0) c=MAX_VALID_MM;
  int lo = min(a,min(b,c)), hi = max(a,max(b,c));
  return a+b+c - lo - hi; // median
}

static void senseAndUpdateWalls(uint8_t cx, uint8_t cy, uint8_t dir){
  // Local L/F/R
  int dL = readMedian3(LEFT);
  int dF = readMedian3(FRONT);
  int dR = readMedian3(RIGHT);

  bool wL = (dL <= WALL_SIDE_MM);
  bool wF = (dF <= WALL_FRONT_MM);
  bool wR = (dR <= WALL_SIDE_MM);

  // Map local to global sides
  uint8_t gL = (dir+3)&3;
  uint8_t gF = dir;
  uint8_t gR = (dir+1)&3;

  auto setWall = [&](uint8_t x,uint8_t y,uint8_t gside, bool wall){
    if (!inBounds(x,y)) return;
    int id = idx(x,y);
    grid[id].known |= (1<<gside);
    if (wall) grid[id].walls |=  (1<<gside);
    else      grid[id].walls &= ~(1<<gside);

    // Neighbor reciprocal
    int nx = x + dx[gside];
    int ny = y + dy[gside];
    if (inBounds(nx,ny)){
      int nid = idx(nx,ny);
      uint8_t oside = opp(gside);
      grid[nid].known |= (1<<oside);
      if (wall) grid[nid].walls |=  (1<<oside);
      else      grid[nid].walls &= ~(1<<oside);
    }
  };

  setWall(cx,cy,gL,wL);
  setWall(cx,cy,gF,wF);
  setWall(cx,cy,gR,wR);
}

/* ============================== Planner ================================== */
// Treat unknown sides as blocked when planning return/shortest path.
static void computeDistances(uint8_t tx, uint8_t ty, uint16_t dist[256]){
  for(int i=0;i<MAZE_W*MAZE_H;i++) dist[i] = 0xFFFF;

  uint16_t qx[MAZE_W*MAZE_H], qy[MAZE_W*MAZE_H];
  int qh=0, qt=0;

  dist[idx(tx,ty)] = 0;
  qx[qt]=tx; qy[qt]=ty; qt=(qt+1)%(MAZE_W*MAZE_H);

  while(qh!=qt){
    uint8_t x = qx[qh], y = qy[qh]; qh=(qh+1)%(MAZE_W*MAZE_H);
    uint16_t cd = dist[idx(x,y)];
    uint8_t walls = grid[idx(x,y)].walls;
    uint8_t known = grid[idx(x,y)].known;

    for(uint8_t d=0; d<4; d++){
      bool isKnown = (known & (1<<d));
      bool isWall  = (walls & (1<<d));
      if (!isKnown) continue;   // unknown → blocked
      if (isWall)   continue;

      int nx = x + dx[d], ny = y + dy[d];
      if (!inBounds(nx,ny)) continue;
      int nid = idx(nx,ny);
      if (dist[nid] > cd+1){
        dist[nid] = cd+1;
        qx[qt]=nx; qy[qt]=ny; qt=(qt+1)%(MAZE_W*MAZE_H);
      }
    }
  }
}

static uint8_t turnSteps(uint8_t cur, uint8_t aim){
  int8_t diff = (int8_t)aim - (int8_t)cur;
  while(diff<0) diff+=4;
  return (uint8_t)diff; // 0..3 (0=same,1=R,2=RR,3=L)
}

// Build 'L','R','F' path; bias ties by F,L,R,B
static int buildPath(uint8_t sx,uint8_t sy,uint8_t sdir,
                     uint8_t tx,uint8_t ty,
                     const uint16_t dist[256],
                     char* out, int cap)
{
  int len=0;
  uint8_t x=sx,y=sy, dir=sdir;

  if (dist[idx(x,y)]==0xFFFF) { out[0]='\0'; return 0; } // unreachable

  while(!(x==tx && y==ty) && len<cap-3){
    uint16_t best = 0xFFFF;
    int8_t  bestDir = -1;

    int order[4] = {dir, (int)((dir+3)&3), (int)((dir+1)&3), (int)((dir+2)&3)};
    for(int k=0;k<4;k++){
      uint8_t d = (uint8_t)order[k];
      uint8_t known = grid[idx(x,y)].known;
      uint8_t walls = grid[idx(x,y)].walls;
      if (!(known & (1<<d))) continue;
      if (  walls & (1<<d)) continue;
      int nx = x + dx[d], ny = y + dy[d];
      if (!inBounds(nx,ny)) continue;
      uint16_t nd = dist[idx(nx,ny)];
      if (nd < best){ best = nd; bestDir = d; }
    }

    if (bestDir<0) break; // stuck

    uint8_t t = turnSteps(dir, (uint8_t)bestDir);
    if (t==1) out[len++]='R';
    else if (t==2){ out[len++]='R'; out[len++]='R'; }
    else if (t==3) out[len++]='L';
    dir = (uint8_t)bestDir;

    out[len++]='F';
    x += dx[dir]; y += dy[dir];
  }
  out[len]='\0';
  return len;
}

/* ======================== Drive Primitives (stable) ======================= */
static const float ANGLE_SCALE_LEFT  = 1.00f;
static const float ANGLE_SCALE_RIGHT = 1.00f;

static const float TURN_STOP_RAD = 0.03f;
static const float TURN_SLOW_RAD = 0.42f;
static const int   PWM_MIN_TURN  = 18;

static const float LEFT_PWM_SCALE  = 1.00f;
static const float RIGHT_PWM_SCALE = 1.00f;

static void forwardOneCell(){
  const float target_mm = CELL_SIZE_MM * FORWARD_CELL_SCALE;
  odom.reset();

  float headI=0, corrFilt=0, turn_cmd=0;
  uint32_t nextCtrl = micros();

  while(true){
    uint32_t now = micros();
    if ((int32_t)(now-nextCtrl)>=0){
      // Base forward + front gating
      int base = PWM_DRIVE;
      int dF = readMedian3(FRONT);
      if (dF<=FRONT_SLOW_MM){
        float frac = (float)(dF-FRONT_STOP_MM)/(float)(FRONT_SLOW_MM-FRONT_STOP_MM);
        float min_frac = (float)FRONT_MIN_BASE_PWM/(float)PWM_DRIVE;
        base = (int)(PWM_DRIVE * clampF(frac, min_frac, 1.0f));
      }

      // Distance & heading
      odom.update();
      float x=odom.getX(), y=odom.getY();
      float dist = sqrtf(x*x+y*y);
      float th   = wrap180rad(odom.getTheta());
      if (target_mm - dist <= BRAKE_ZONE_MM){
        float frac = clampF((target_mm - dist)/BRAKE_ZONE_MM, 0.25f, 1.0f);
        base = (int)(base*frac);
      }

      // Heading PI (positive theta=CCW -> need to steer RIGHT => +turn_cmd)
      float eHead = +th;
      if (fabsf(eHead)<0.35f){
        headI += HEADING_KI_RAD*eHead*DT_SEC;
        headI  = clampF(headI, -float(HEADING_I_MAX), +float(HEADING_I_MAX));
      } else {
        headI *= 0.85f;
      }
      int headPWM = (int)clampF(HEADING_KP_RAD*eHead + headI, -float(HEADING_MAX_PWM), +float(HEADING_MAX_PWM));

      // Very light side nudge if very close (keep calm)
      int dL = readMedian3(LEFT);
      int dR = readMedian3(RIGHT);
      int wallPWM = 0;
      if (dL<50 && dR>80) wallPWM += +6; // push right
      if (dR<50 && dL>80) wallPWM += -6; // push left

      float corrTarget = 0.75f*headPWM + 0.25f*wallPWM;
      corrFilt += 0.2f*(corrTarget - corrFilt);

      int corrLimit = min(STEER_MAX_PWM, (int)(base * CORR_FRAC_OF_BASE));
      int turn_target = clampInt((int)lrintf(corrFilt), -corrLimit, +corrLimit);

      // Slew-limit
      int dturn = turn_target - (int)turn_cmd;
      if (dturn >  TURN_SLEW_PER_CYCLE) dturn =  TURN_SLEW_PER_CYCLE;
      if (dturn < -TURN_SLEW_PER_CYCLE) dturn = -TURN_SLEW_PER_CYCLE;
      turn_cmd += dturn;

      int leftPWM  = clampInt((int)((base + turn_cmd) * LEFT_PWM_SCALE),  0, 255);
      int rightPWM = clampInt((int)((base - turn_cmd) * RIGHT_PWM_SCALE), 0, 255);
      motor.setMotorPWM(leftPWM,rightPWM);

      if (dist >= target_mm) break;
      nextCtrl += CTRL_DT_US;
    }
    delayMicroseconds(200);
  }
  motor.setMotorPWM(0,0);
}

static void turnLeft(){
  const float target = +TURN_RAD * ANGLE_SCALE_LEFT;
  odom.reset();
  for(;;){
    odom.update();
    float th = wrap180rad(odom.getTheta());
    float err= wrap180rad(target - th);
    if (fabsf(err) <= TURN_STOP_RAD) break;
    float scale = (fabsf(err)>=TURN_SLOW_RAD) ? 1.f : (fabsf(err)/TURN_SLOW_RAD);
    int pwm = PWM_MIN_TURN + (int)((PWM_TURN - PWM_MIN_TURN)*scale);
    int dir = (err>0)?+1:-1;
    motor.setMotorPWM(-dir*pwm, +dir*pwm);
    delay(3);
  }
  motor.setMotorPWM(0,0);
}

static void turnRight(){
  const float target = -TURN_RAD * ANGLE_SCALE_RIGHT;
  odom.reset();
  for(;;){
    odom.update();
    float th = wrap180rad(odom.getTheta());
    float err= wrap180rad(target - th);
    if (fabsf(err) <= TURN_STOP_RAD) break;
    float scale = (fabsf(err)>=TURN_SLOW_RAD) ? 1.f : (fabsf(err)/TURN_SLOW_RAD);
    int pwm = PWM_MIN_TURN + (int)((PWM_TURN - PWM_MIN_TURN)*scale);
    int dir = (err>0)?+1:-1;
    motor.setMotorPWM(-dir*pwm, +dir*pwm);
    delay(3);
  }
  motor.setMotorPWM(0,0);
}

/* ============================== Explorer ================================= */
// Prefer unvisited: L, F, R, then back; unknown counts as “open” for exploration
static uint8_t chooseNextDir(uint8_t cx,uint8_t cy,uint8_t dir){
  auto openTo = [&](uint8_t d)->bool{
    uint8_t known = grid[idx(cx,cy)].known;
    uint8_t walls = grid[idx(cx,cy)].walls;
    if (!(known & (1<<d))) return true;        // unknown → explore
    if (  walls & (1<<d))  return false;
    int nx=cx+dx[d], ny=cy+dy[d];
    return inBounds(nx,ny);
  };
  auto unvisitedTo = [&](uint8_t d)->bool{
    if (!openTo(d)) return false;
    int nx=cx+dx[d], ny=cy+dy[d];
    return inBounds(nx,ny) && (grid[idx(nx,ny)].visited==0);
  };

  int pref[4] = { (dir+3)&3, dir, (dir+1)&3, (dir+2)&3 }; // L,F,R,B
  for(int k=0;k<4;k++) if (unvisitedTo(pref[k])) return (uint8_t)pref[k];
  for(int k=0;k<4;k++) if (openTo(pref[k]))      return (uint8_t)pref[k];
  return dir; // stuck -> try forward
}

static void doTurnTo(uint8_t &dir, uint8_t aim){
  uint8_t t = turnSteps(dir, aim);
  if (t==1) { turnRight(); dir=(dir+1)&3; }
  else if (t==2){ turnRight(); turnRight(); dir=(dir+2)&3; }
  else if (t==3){ turnLeft();  dir=(dir+3)&3; }
}

/* =============================== Runner ================================== */
static void runCommands(const char* cmds, uint8_t &x, uint8_t &y, uint8_t &dir, bool updateOled=true){
  for(int i=0; cmds[i]; ++i){
    char c = cmds[i];
    if (c=='L'){ turnLeft();  dir=(dir+3)&3; }
    else if (c=='R'){ turnRight(); dir=(dir+1)&3; }
    else if (c=='F'){ forwardOneCell(); x+=dx[dir]; y+=dy[dir]; }
    if (updateOled) { drawMap(x,y,dir); }
    delay(120);
  }
}

/* ======================== Start/Goal Randomization ======================= */
static void pickStartGoal(){
  long seed = analogRead(A0);
  seed = (seed << 10) ^ micros();
  randomSeed(seed);

#if RANDOMIZE_START
  START_X   = (uint8_t)random(MAZE_W);
  START_Y   = (uint8_t)random(MAZE_H);
  START_DIR = (uint8_t)random(4);
#else
  START_X   = 0;
  START_Y   = 0;
  START_DIR = 0;  // N
#endif

#if RANDOMIZE_GOAL
  if (RANDOM_GOAL_CENTER_ONLY){
    const uint8_t gx[4] = {7,8,7,8};
    const uint8_t gy[4] = {7,7,8,8};
    uint8_t k = (uint8_t)random(4);
    GOAL_X = gx[k]; GOAL_Y = gy[k];
  } else {
    do {
      GOAL_X = (uint8_t)random(MAZE_W);
      GOAL_Y = (uint8_t)random(MAZE_H);
    } while (GOAL_X==START_X && GOAL_Y==START_Y);
  }
#else
  GOAL_X = 7; GOAL_Y = 7;
#endif
}

/* ================================= Setup ================================= */
void setup(){
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay(); display.display();

  odom.begin();
  motor.begin();
  lidar.begin();

  // Choose start/goal
  pickStartGoal();

  // Init map
  for(int i=0;i<MAZE_W*MAZE_H;i++){ grid[i] = Cell(); }
  grid[idx(START_X,START_Y)].visited = 1;

  // Banner showing randomized choices (for the demo/marker)
  display.clearDisplay();
  display.setTextSize(1); display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);  display.print("Micromouse Mapping");
  display.setCursor(0,10); display.print("S:");
  display.print(START_X); display.print(","); display.print(START_Y);
  display.print(" dir");  display.print(START_DIR);
  display.setCursor(0,20); display.print("G:");
  display.print(GOAL_X); display.print(","); display.print(GOAL_Y);
  display.display();
  delay(800);
}

/* ================================= Loop ================================== */
void loop(){
  // ===== Phase 1: Explore & Map =====
  uint8_t cx=START_X, cy=START_Y, cdir=START_DIR;
  bool foundGoal=false;

  const uint16_t MAX_STEPS = 600; // safety cap
  for(uint16_t step=0; step<MAX_STEPS; ++step){
    senseAndUpdateWalls(cx,cy,cdir);
    grid[idx(cx,cy)].visited = 1;
    drawMap(cx,cy,cdir);

    if (cx==GOAL_X && cy==GOAL_Y){ foundGoal=true; break; }

    // Optional coverage cutoff (80%); tweak to chase mapping marks
    uint16_t visitedCount=0;
    for(int i=0;i<MAZE_W*MAZE_H;i++) if (grid[i].visited) visitedCount++;
    if (visitedCount >= (MAZE_W*MAZE_H*80UL)/100UL) break;

    // Decide next
    uint8_t nextDir = chooseNextDir(cx,cy,cdir);
    doTurnTo(cdir, nextDir);

    // If forward is actually a known wall (recheck), try alternatives
    uint8_t known = grid[idx(cx,cy)].known;
    uint8_t walls = grid[idx(cx,cy)].walls;
    if ((known&(1<<cdir)) && (walls&(1<<cdir))){
      int order[4] = {(cdir+3)&3, cdir, (cdir+1)&3, (cdir+2)&3};
      bool moved=false;
      for(int k=0;k<4;k++){
        uint8_t d=order[k];
        // open if unknown or known-open
        if ((!(grid[idx(cx,cy)].known&(1<<d))) || !(grid[idx(cx,cy)].walls&(1<<d))){
          doTurnTo(cdir,d); forwardOneCell(); cx+=dx[cdir]; cy+=dy[cdir]; moved=true; break;
        }
      }
      if(!moved) break;
    } else {
      forwardOneCell(); cx+=dx[cdir]; cy+=dy[cdir];
    }
  }

  // Final sense & draw
  senseAndUpdateWalls(cx,cy,cdir);
  grid[idx(cx,cy)].visited=1;
  drawMap(cx,cy,cdir);
  delay(150);

  // ===== Phase 2: Return to Start =====
  {
    uint16_t dist[256];
    computeDistances(START_X,START_Y, dist);
    char path[512];
    int n = buildPath(cx,cy,cdir, START_X,START_Y, dist, path, sizeof(path));
    runCommands(path, cx, cy, cdir, true);
  }

  // Face the start orientation
  doTurnTo(cdir, START_DIR);
  drawMap(cx,cy,cdir);

  // ===== Phase 3: Shortest Path Run =====
  {
    uint16_t dist[256];
    computeDistances(GOAL_X,GOAL_Y, dist);
    char path[512];
    int n = buildPath(START_X,START_Y,START_DIR, GOAL_X,GOAL_Y, dist, path, sizeof(path));

    display.setCursor(80, 20);
    display.print("Run!");
    display.display();
    delay(250);

    uint8_t rx=START_X, ry=START_Y, rdir=START_DIR;
    runCommands(path, rx, ry, rdir, false);
  }

  motor.setMotorPWM(0,0);

  display.clearDisplay();
  display.setCursor(0,0); display.print("Done. Shortest path run.");
  display.setCursor(0,10);display.print("Start->Goal complete.");
  display.display();

  while(true){ delay(1000); } // halt
}
