#ifndef _JELLO_H_
#define _JELLO_H_

#ifdef WIN32
 #include <windows.h>
#endif

#include "openGL-headers.h"
#include "pic.h"

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define  pi 3.141592653589793238462643383279 

// camera angles
extern double Theta;
extern double Phi;
extern double R;

// number of images saved to disk so far
extern int sprite;

// mouse control
extern int g_vMousePos[2];
extern int g_iLeftMouseButton,g_iMiddleMouseButton,g_iRightMouseButton;

struct index
{
	int i;
	int j;
	int k;
};

struct point 
{
   double x;
   double y;
   double z;
};

// these variables control what is displayed on the screen
extern int shear, bend, structural, doPause, viewingMode, saveScreenToFile;

struct world
{
  double dt;               
  char   integrator[10];  // "RK4" or "Euler"
  int    n;               // display nth timepoint
  
  double kElastic;        // Hook's elasticity coefficient for internal springs
  double dElastic;        // Damping coefficient
  double kCollision;      // Hook's elasticity coefficient for collision springs
  double dCollision;      // Damping coefficient collision springs
  double mass;            // mass of each of the 512 control points, mass assumed to be equal for every control point
  
  int    incPlanePresent; // Is the inclined plane present? 1 = YES, 0 = NO (always NO in this assignment)
  double a,b,c,d;         // inclined plane has equation a * x + b * y + c * z + d = 0; if no inclined plane, these four fields are not used
  
  int    resolution;         // resolution for the 3d grid specifying the external force field; value of 0 means that there is no force field
  struct point * forceField; // pointer to the array of values of the force field
  
  struct point p[8][8][8];   // positions  of the 512 control points
  struct point v[8][8][8];   // velocities of the 512 control points
  
  struct point pCollission[8][8][8];
  bool   inBox[8][8][8];	 	
};

extern struct world jello;

// computes crossproduct of two vectors,
// struct point vector1, vector2, dest
// result goes into dest
#define CROSSPRODUCTp(vector1   ,  vector2   ,dest)                \
        CROSSPRODUCT((vector1).x, (vector1).y, (vector1).z,\
                     (vector2).x, (vector2).y, (vector2).z,\
                     (dest).x   , (dest).y   , (dest).z    )

// computes crossproduct of two vectors
// double coordinates x1,y1,z1,x2,y2,z2,x,y,z
// result goes into x,y,z
#define  CROSSPRODUCT(x1,y1,z1,x2,y2,z2,x,y,z)\
\
  x = (y1) * (z2) - (y2) * (z1);\
  y = (x2) * (z1) - (x1) * (z2);\
  z = (x1) * (y2) - (x2) * (y1)

// computes dotProduct of two vectors, 
// struct point vector1, vector2     , 
// result goes into double result

#define DOTPRODUCT(vector1,vector2,result)\
\
 result = (vector1.x * vector2.x) + (vector1.y * vector2.y) + (vector1.z * vector2.z);


// normalizes vector  dest, result returned in dest
// must declare a double variable called 'length' inside the scope of the NORMALIZE macro
// macro will change 'length'
#define pNORMALIZE(dest)\
\
  length = sqrt((dest).x * (dest).x + (dest).y * (dest).y + (dest).z * (dest).z);\
  (dest).x /= length;\
  (dest).y /= length;\
  (dest).z /= length;

// copies vector source to vector dest
// struct point source,dest
#define pCPY(source,dest)\
\
  (dest).x = (source).x;\
  (dest).y = (source).y;\
  (dest).z = (source).z;
  
// assigns values x,y,z to point vector dest
// struct point dest
// double x,y,z
#define pMAKE(x0,y0,z0,dest)\
\
  dest.x = x0;\
  dest.y = y0;\
  dest.z = z0;

// sums points src1 and src2 to dest
// struct point src1,src2,dest
#define pSUM(src1,src2,dest)\
\
  (dest).x = (src1).x + (src2).x;\
  (dest).y = (src1).y + (src2).y;\
  (dest).z = (src1).z + (src2).z;

// dest = src1 - src2
// struct point src1,src2,dest
#define pDIFFERENCE(src1,src2,dest)\
\
  (dest).x = (src1).x - (src2).x;\
  (dest).y = (src1).y - (src2).y;\
  (dest).z = (src1).z - (src2).z;

// mulitplies components of point src by scalar and returns the result in dest
// struct point src,dest
// double scalar
#define pMULTIPLY(src,scalar,dest)\
\
  (dest).x = (src).x * (scalar);\
  (dest).y = (src).y * (scalar);\
  (dest).z = (src).z * (scalar);

#endif

