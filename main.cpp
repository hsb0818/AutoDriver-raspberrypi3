#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>
#include <sys/time.h>
#include <list>
#include "L298N.h"
#include "Ultrasonic.h"
#include "Dijikstra.h"

#define DIR_DISTANCE_ALERT 20
#define USONICK_DELAY 500

#define L298N_ENA 25
#define L298N_IN1 27
#define L298N_IN2 23
#define L298N_IN3 5
#define L298N_IN4 4
#define L298N_ENB 1
#define ULTRASONIC_TRIG 21
#define ULTRASONIC_ECHO 22
#define ILINE_R 7
#define ILINE_C 0
#define ILINE_L 2

#define CAR_DIR_FW 0
#define CAR_DIR_BK 1
#define CAR_DIR_LF 2
#define CAR_DIR_RF 3
#define CAR_DIR_ST 4

#define CAR_DEF_FORWARD_TICK 500

#define OBS_NONE 0
#define OBS_STATIC 1
#define OBS_DYNAMIC 2

int g_car_dir = CAR_DIR_ST;

static unsigned char SoftPWM[] = {L298N_ENA, L298N_ENB};
static unsigned char OUT[] = {
  L298N_IN1, L298N_IN2, L298N_IN3, L298N_IN4,
  ULTRASONIC_TRIG
 };
static unsigned char IN[] = {ULTRASONIC_ECHO, ILINE_R, ILINE_C, ILINE_L};

void Load();
void SetUp();
void Loop();
bool checkUltraSonick();
void CheckCarDir();
int CalcDir(const int h, const int v);
bool LineTracing();
bool ReverseLineTracing();
int MSP(std::list<Dijikstra::Vertex>& path, const int cur, const int next);
void PathFindLoop();

L298N g_driver(L298N_ENA, L298N_IN1, L298N_IN2, L298N_ENB, L298N_IN3, L298N_IN4);
Ultrasonic g_ultra(ULTRASONIC_TRIG, ULTRASONIC_ECHO);

const int VC = 64;
const int EC = 64;
int start = 0;
int dest = 0;

Dijikstra spf(VC, EC);
vector<Dijikstra::Vertex> g_path;
Dijikstra::VertexPos g_curdir(0, 1);

struct Timer
{
  Timer()
    : timer(0)
  {}
  
  unsigned int timer;
  void Reset() { timer = Timer::GetTickCount(); }
  unsigned int Get() { return Timer::GetTickCount() - timer; }

  static unsigned int GetTickCount()
  {
    struct timeval gettick;
    unsigned int tick;
    int ret;
    gettimeofday(&gettick, NULL);

    tick = gettick.tv_sec*1000 + gettick.tv_usec/1000;

    return tick;
  }  
};

Timer g_timer;

int main()
{
  Load();
  SetUp();
  Loop();
    
  return 0;
}

void Load()
{
  for (int y=0; y<5; y++)
    {
      for (int x=0; x<4; x++)
	{
	  spf.RegistVertexPosToMap(y*4 + x, x, y);
	}
    }

  for (int y=0; y<4; y++)
    {
      for (int x=0; x<3; x++)
	{
	  spf.RegistVertexToMap(y*4 + x, (y+1)*4 + x, true);
	  spf.RegistVertexToMap(y*4 + x, y*4 + (x+1), true);
	}
    }
  
  printf("Edges Loaded..\n");

  printf("Start : ");
  scanf("%d", &start);

  spf.Run(start, -1);

  printf("Dest : ");
  scanf("%d", &dest);
  g_path = spf.GetShortestPath(dest);

  printf("Load Completed.\n");
}

void SetUp()
{
    int err = wiringPiSetup();
    if (err == -1)
      printf("wiringPiSetup Error!\n");

    for (int i=0; i<sizeof(OUT); i++)
    {
	pinMode(OUT[i], OUTPUT);
	digitalWrite(OUT[i], LOW);
    }
    
    for (int i=0; i<sizeof(IN); i++)
    {
        pinMode(IN[i], INPUT);
    }

    g_driver.begin();
    
    printf("setup completed!\n");
}

void Cornering(int rot, int type = CAR_DIR_RF)
{
  printf("cornering %d\n", type);
  g_timer.Reset();
  while(g_timer.Get() < rot)
    {
      if (type == CAR_DIR_RF)
	g_driver.goRight();
      else
	g_driver.goLeft();
    }
  	
  g_timer.Reset();
  while(g_timer.Get() < 10000)
    {
      CheckCarDir();
      if (g_car_dir == CAR_DIR_ST)
	g_driver.goForward();
      else {
	g_driver.stop();
	return;
      }
    }
}


int MSP(std::list<Dijikstra::Vertex>& path, const int cur, const int next)
{
  if (checkUltraSonick())
    {
      printf("Obstacle founded. wait 5 sec\n");

      g_timer.Reset();
      while(g_timer.Get() < 5000)
	{
	  g_driver.stop();
	  delay(500);

	  if (checkUltraSonick() == false)
	    {
	      while(LineTracing()) { delay(10); }
	      return OBS_DYNAMIC;
	    }
	}

      printf("this is static obstacle.\n");
      printf("reverse linetracing...\n");

      while (ReverseLineTracing()) { delay(10); }

      g_driver.goBack();
      delay(100);
	     
      printf("reverse enn\n");

      spf.RemoveEdge(cur, next);
      spf.Run(cur, dest);
      g_path = spf.GetShortestPath(dest);
      for (int i=0; i<g_path.size(); i++)
	printf("%d -> ", g_path[i].dest_id);
      printf("\n");
      
      path = std::list<Dijikstra::Vertex>(g_path.begin(), g_path.end());
      path.pop_front();

      return OBS_STATIC;
    }

  return OBS_NONE;
}

void Loop()
{
  printf("start!\n");

  int rot = 0;
  printf("rot time : ");
  scanf("%d", &rot);
 
  std::list<Dijikstra::Vertex> path(g_path.begin(), g_path.end());
  if (path.size() == 0)
    return;

  int cur = path.front().dest_id;
  Dijikstra::VertexPos cur_pos = spf.GetVertexPos(start);
  path.pop_front();

  while (path.size() > 0)
  {
    const int id = path.front().dest_id;
    const Dijikstra::VertexPos pos = spf.GetVertexPos(id);
    path.pop_front();

    printf("[%d]%d -> %d\n", path.size(), cur, id);
    switch(CalcDir(pos.x - cur_pos.x, pos.y - cur_pos.y))
      {
      case CAR_DIR_FW:
	printf("fw\n");
	g_timer.Reset();
	while(g_timer.Get() < CAR_DEF_FORWARD_TICK)
	  {
	    g_driver.goForward();
	  }

	break;
      case CAR_DIR_LF:
	printf("lf\n");
	Cornering(rot, CAR_DIR_LF);
	break;
      case CAR_DIR_RF:
	printf("rf\n");
	Cornering(rot, CAR_DIR_RF);
	break;
      case CAR_DIR_ST:
	printf("st\n");
	g_driver.stop();
	delay(1000);
	break;
      }
    
    printf("tracing...\n");
    //    while(LineTracing()) {delay(10);}

    int obs = false;
    while(LineTracing())
      {
	obs = MSP(path, cur, id);
	if (obs != OBS_NONE)
	  break;
	
	delay(10);
      }

    if (obs == OBS_STATIC)
      continue;
    
    printf("trc end\n");
    
    cur = id;
    cur_pos = pos;
  }
  
  g_driver.stop();
}

bool checkUltraSonick()
{
  float dis_val = g_ultra.ReadDistByCentimeters();
  if (dis_val <= DIR_DISTANCE_ALERT)
    {
      printf("dir_dist_alert\n");
      g_driver.stop();
      return true;
    }

  return false;
}

int CalcDir(const int h, const int v)
{
  if (h == 0 && v == 0)
    return CAR_DIR_ST;
  else if (g_curdir.x == h &&
	   g_curdir.y == v)
    return CAR_DIR_FW;

  int ret = CAR_DIR_ST;
  if (g_curdir.x > 0)
    {
      if (v > 0)
	ret = CAR_DIR_LF;
      else
	ret = CAR_DIR_RF;
    }
  else if (g_curdir.x < 0)
    {
      if (v > 0)
	ret = CAR_DIR_RF;
      else
	ret = CAR_DIR_LF;
    }
  else if (g_curdir.y > 0)
    {
      if (h > 0)
	ret = CAR_DIR_RF;
      else
	ret = CAR_DIR_LF;
    }
  else if (g_curdir.y < 0)
    {
      if (h > 0)
	ret = CAR_DIR_LF;
      else
	ret = CAR_DIR_RF;
    }
  
  g_curdir.x = h;
  g_curdir.y = v;
    
  return ret;
}

void CheckCarDir()
{
  const bool iline_l = (digitalRead(ILINE_L) == 1) ? true : false;
  const bool iline_c = (digitalRead(ILINE_C) == 1) ? true : false;
  const bool iline_r = (digitalRead(ILINE_R) == 1) ? true : false;

  //  printf("iline - %d %d %d\n", iline_l, iline_c, iline_r);

  if (iline_l && iline_c && iline_r)
    g_car_dir = CAR_DIR_FW;
  else if (!iline_l && !iline_c && !iline_r)
    g_car_dir = CAR_DIR_ST;
  else if (iline_l && !iline_r)
    g_car_dir = CAR_DIR_LF;
  else if (iline_r && !iline_l)
    g_car_dir = CAR_DIR_RF;
  else if (iline_c)
    g_car_dir = CAR_DIR_FW;
}

bool LineTracing()
{
  CheckCarDir();  
  switch (g_car_dir)
  {
  case CAR_DIR_FW:
    g_driver.goForward();
    break;
  case CAR_DIR_BK:
    g_driver.goBack();
    break;
  case CAR_DIR_LF:
    g_driver.goLeft();
    break;
  case CAR_DIR_RF:
    g_driver.goRight();    
    break;
  case CAR_DIR_ST:
    g_driver.stop();
    return false;
  }

  return true;
}

bool ReverseLineTracing()
{
  CheckCarDir();
  switch(g_car_dir)
    {
    case CAR_DIR_FW:
      g_driver.goBack();
      break;
    case CAR_DIR_RF:
      g_driver.goRight();
      break;
    case CAR_DIR_LF:
      g_driver.goLeft();
      break;
    case CAR_DIR_ST:
      g_driver.stop();
      return false;
    }

  return true;
}

void PathFindLoop()
{
    for (int i=0; i<g_path.size(); i++)
  {
    const Dijikstra::Vertex& cur = g_path[i];
    printf("to %d\n", cur.dest_id);

    int r = rand()%5;
    if (r == 0)
      g_driver.goForward();
    else if (r == 1)
      g_driver.goForward();
    else if (r == 2)
      g_driver.goLeft();
    else if (r == 3)
      g_driver.goRight();
    else
      g_driver.stop();

    printf("dist : %d\n", cur.dist);
    delay(cur.dist * 1000);
  }

    g_driver.stop();
}
