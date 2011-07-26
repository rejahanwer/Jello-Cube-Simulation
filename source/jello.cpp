/*
Jello cube simulation
-Rejah Anuvar 
*/ 

#include "performanceCounter.h"
#include "jello.h"
#include "showCube.h"
#include "input.h"
#include "physics.h"
#include <iostream>

using namespace std;
PerformanceCounter counter;

// camera parameters tweaked for a good first look
double Theta = 0.3136 ; 
double Phi   = 0.7136 ;
double R     = 4.6    ;

// mouse control
int g_iMenuId;
int g_vMousePos[2];
int g_iLeftMouseButton  ,
    g_iMiddleMouseButton,
    g_iRightMouseButton ;

// number of images saved to disk so far
int sprite=0;

// these variables control what is displayed on screen
int shear            = 0,
    bend             = 0,
    structural       = 1,
    doPause          = 0,
    viewingMode      = 1,
    saveScreenToFile = 0;

//window params
int    windowWidth ,
       windowHeight;

struct world jello ;

void myinit()
{
    glMatrixMode   (GL_PROJECTION);
    glLoadIdentity ();
    gluPerspective (90.0,1.0,0.01,1000.0);

    // set background color to grey
    glClearColor   (0.7, 0.7, 0.7, 0.0);
	
    glCullFace     (GL_BACK);
    glEnable       (GL_CULL_FACE);

    glShadeModel   (GL_SMOOTH);
    glEnable       (GL_POLYGON_SMOOTH);
    glEnable       (GL_LINE_SMOOTH);

    return; 
}

void display()
{
    glClear       (GL_COLOR_BUFFER_BIT |
                   GL_DEPTH_BUFFER_BIT );
    glMatrixMode  (GL_MODELVIEW        );
  
    glLoadIdentity();

    // camera parameters - Phi, Theta, R
    gluLookAt(R * cos(Phi) * cos (Theta),
              R * sin(Phi) * cos (Theta),
              R * sin (Theta),
              0.0,0.0,0.0, 0.0,0.0,1.0 );
    
    // global ambient light
    GLfloat aGa[] = { 0.0, 0.0, 0.0, 0.0 };
  
    // light 's ambient, diffuse, specular components
    GLfloat lKa0[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd0[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat lKs0[] = { 1.0, 1.0, 1.0, 1.0 };

    GLfloat lKa1[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd1[] = { 1.0, 0.0, 0.0, 1.0 };
    GLfloat lKs1[] = { 1.0, 0.0, 0.0, 1.0 };

    GLfloat lKa2[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd2[] = { 1.0, 1.0, 0.0, 1.0 };
    GLfloat lKs2[] = { 1.0, 1.0, 0.0, 1.0 };

    GLfloat lKa3[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd3[] = { 0.0, 1.0, 1.0, 1.0 };
    GLfloat lKs3[] = { 0.0, 1.0, 1.0, 1.0 };

    GLfloat lKa4[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd4[] = { 0.0, 0.0, 1.0, 1.0 };
    GLfloat lKs4[] = { 0.0, 0.0, 1.0, 1.0 };

    GLfloat lKa5[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd5[] = { 1.0, 0.0, 1.0, 1.0 };
    GLfloat lKs5[] = { 1.0, 0.0, 1.0, 1.0 };

    GLfloat lKa6[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd6[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat lKs6[] = { 1.0, 1.0, 1.0, 1.0 };

    GLfloat lKa7[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat lKd7[] = { 0.0, 1.0, 1.0, 1.0 };
    GLfloat lKs7[] = { 0.0, 1.0, 1.0, 1.0 };

    // light positions and directions
    GLfloat lP0[] = { -1.999, -1.999, -1.999, 1.0 };
    GLfloat lP1[] = {  1.999, -1.999, -1.999, 1.0 };
    GLfloat lP2[] = {  1.999,  1.999, -1.999, 1.0 };
    GLfloat lP3[] = { -1.999,  1.999, -1.999, 1.0 };
    GLfloat lP4[] = { -1.999, -1.999,  1.999, 1.0 };
    GLfloat lP5[] = {  1.999, -1.999,  1.999, 1.0 };
    GLfloat lP6[] = {  1.999,  1.999,  1.999, 1.0 };
    GLfloat lP7[] = { -1.999,  1.999,  1.999, 1.0 };
  
    // jelly material color
    GLfloat mKa[] = { 0.4, 0.0, 0.4, 1.0 };
    GLfloat mKd[] = { 0.4, 0.0, 0.4, 1.0 };
    GLfloat mKs[] = { 0.4, 0.0, 0.4, 1.0 };
    GLfloat mKe[] = { 0.4, 0.0, 0.4, 1.0 };

    /* set up lighting */
    glLightModelfv (GL_LIGHT_MODEL_AMBIENT     , aGa     );
    glLightModelf  (GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE );
    glLightModelf  (GL_LIGHT_MODEL_TWO_SIDE    , GL_FALSE);

    // set up cube color
    glMaterialfv (GL_FRONT, GL_AMBIENT  , mKa);
    glMaterialfv (GL_FRONT, GL_DIFFUSE  , mKd);
    glMaterialfv (GL_FRONT, GL_SPECULAR , mKs);
    glMaterialfv (GL_FRONT, GL_EMISSION , mKe);
    glMaterialf  (GL_FRONT, GL_SHININESS, 10 );
    
    // macro to set up light i
    #define LIGHTSETUP(i)\
    glLightfv(GL_LIGHT##i, GL_POSITION, lP##i);\
    glLightfv(GL_LIGHT##i, GL_AMBIENT, lKa##i);\
    glLightfv(GL_LIGHT##i, GL_DIFFUSE, lKd##i);\
    glLightfv(GL_LIGHT##i, GL_SPECULAR, lKs##i);\
    glEnable(GL_LIGHT##i)
  
    LIGHTSETUP (0);
    LIGHTSETUP (1);
    LIGHTSETUP (2);
    LIGHTSETUP (3);
    LIGHTSETUP (4);
    LIGHTSETUP (5);
    LIGHTSETUP (6);
    LIGHTSETUP (7);

    // enable lighting
    glEnable(GL_LIGHTING   );  
    glEnable(GL_DEPTH_TEST);

    // draw cube
    showCube(&jello);

    glDisable(GL_LIGHTING);

    // draw bounding box
    showBoundingBox();
 
    glutSwapBuffers();
}

void doIdle()
{
    counter.StopCounter();
    //To check frame rate
    //std::cout<<"FPS:" << 1/counter.GetElapsedTime() <<endl;
	counter.StartCounter();
    
    char s[20]="picxxxx.ppm";
    int  i;
  
    // save screen to file
    s[3] = 48 + (sprite / 1000)      ;
    s[4] = 48 + (sprite % 1000) / 100;
    s[5] = 48 + (sprite % 100 ) / 10 ;
    s[6] = 48 +  sprite % 10;

  if (saveScreenToFile==1)
     {
     saveScreenshot(windowWidth, windowHeight, s);
     saveScreenToFile=0; // save only once, can change this we want continuos image generation.
     sprite++;
     }

  if (sprite >= 300)    // allow only 300 snapshots
     {
     exit(0);	
     }

  if (doPause == 0)
  {   
      //one step of cube simulation
	  for (i=1; i<=jello.n; i++)
	  {
		  if (jello.integrator[0]=='E') // Euler Intergration
              Euler(&jello);
		  if (jello.integrator[0]=='R') // RK4 Integration
              RK4(&jello);
	  }
   }
   glutPostRedisplay();
}

int main (int argc, char ** argv)
{
  if (argc<2)
  {  
    printf ("Oops! Say the jello world file!\n");
    printf ("Usage: %s [worldfile]\n",  argv[0]);
    exit(0);
  }

  readWorld(argv[1], &jello);
  glutInit (&argc  ,  argv );
  
  /* double buffered window, use depth testing, 640x480 */
  glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  
  windowWidth  = 640;
  windowHeight = 480;
  
  glutInitWindowSize     (windowWidth, windowHeight);
  glutInitWindowPosition (400,0);
  glutCreateWindow       ("Jello cube");

  // tells glut use 'display' function to redraw 
  glutDisplayFunc(display);

  // replace with any animate code
  glutIdleFunc   (doIdle);

  // callback for mouse drags
  glutMotionFunc (mouseMotionDrag);

  // callback for mouse movement
  glutPassiveMotionFunc(mouseMotion);

  // callback for mouse button changes
  glutMouseFunc(mouseButton);

  // register for keyboard events
  glutKeyboardFunc(keyboardFunc);

  // do initialization
  myinit();

  //loop
  glutMainLoop();

  return(0);
}

