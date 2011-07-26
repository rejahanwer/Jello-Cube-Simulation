/*
Jello cube simulation
Rejah Anuvar
CreateWorld utility to create our own world files
*/

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <cstdlib>

struct point 
{
   double x;
   double y;
   double z;
};

struct world
{
  char   integrator[10];  // "RK4" or "Euler"
  double dt;              // timestep
  int    n;               // display every nth timestep
  
  double kElastic;        // Hook's  coefficient for all internal springs
  double dElastic;        // Damping coefficient for all internal springs 
  double kCollision;      // Hook's  coefficient for collision springs
  double dCollision;      // Damping coefficient collision springs
  
  double mass;               // mass 512 control points, assumed to be equal
  int    incPlanePresent;    // 1=yes, 0 = no
  double a,b,c,d;            // inclined plane has equation a * x + b * y + c * z + d = 0; if no inclined plane, these four fields are not used
  int    resolution;         // resolution for the 3d grid specifying the external force field; value of 0 means that there is no force field
  struct point * forceField; // pointer to the array of values of the force field
  struct point p[8][8][8];   // position of the 512 control points
  struct point v[8][8][8];   // velocities of the 512 control points
};


/* 
Writes the world parameters to a world file on disk.
FileName = name of the output world file, ex: jello1.w.
*/

void writeWorld (char * fileName, struct world * jello)
{
  int i,j,k;
  FILE * file;
  
  file = fopen(fileName, "w");
  if (file == NULL) 
    {
    printf ("can't open file\n");
    exit(1);
    }

  /* write integrator algorithm */ 
  fprintf(file,"%s\n",jello->integrator);

  /* write timestep */
  fprintf(file,"%lf %d\n",jello->dt,jello->n);

  /* write physical parameters */
  fprintf(file, "%lf %lf %lf %lf\n", 
    jello->kElastic, jello->dElastic, jello->kCollision, jello->dCollision);

  /* write mass */
  fprintf(file, "%lf\n", jello->mass);

  /* write info about the plane */
  fprintf(file, "%d\n", jello->incPlanePresent);
  if (jello->incPlanePresent == 1)
    fprintf(file, "%lf %lf %lf %lf\n", jello->a, jello->b, jello->c, jello->d);

  /* write info about the force field */
  fprintf(file, "%d\n", jello->resolution);
  if (jello->resolution != 0)
    for (i=0; i<= jello->resolution-1; i++)
      for (j=0; j<= jello->resolution-1; j++)
        for (k=0; k<= jello->resolution-1; k++)
          {
          fprintf(file, "%lf %lf %lf\n", 
          jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].x, 
          jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].y, 
          jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].z);
          }

  /* write initial point positions */
  for (i = 0; i <= 7 ; i++)
    for (j = 0; j <= 7; j++)
      for (k = 0; k <= 7; k++)
        {
        fprintf(file, "%lf %lf %lf\n", 
        jello->p[i][j][k].x, jello->p[i][j][k].y, jello->p[i][j][k].z);
        }
    
  /* write initial point velocities */
  for (i = 0; i <= 7 ; i++)
    for (j = 0; j <= 7; j++)
      for (k = 0; k <= 7; k++)
        {
        fprintf(file, "%lf %lf %lf\n", 
        jello->v[i][j][k].x, jello->v[i][j][k].y, jello->v[i][j][k].z);
        }
    
  fclose(file);
  return;
}

/* modify main to create our own world file */
int main()
{
  struct world jello;
  int    i,j,k;
  double x,y,z;

  // set the integrator and the physical parameters
  strcpy     (jello.integrator,"RK4");
  jello.dt         = 0.0001000;
  jello.n          = 5;
  jello.kElastic   = 300;
  jello.dElastic   = 0.50;
  jello.kCollision = 300;
  jello.dCollision = 0.25;
  jello.mass       = 1.0 / 512;

  // set the inclined plane
  jello.incPlanePresent=0;
  jello.a =-1;
  jello.b = 1;
  jello.c = 1;
  jello.d = 2;

  //set the external force field
  jello.resolution=30;
  jello.forceField = (struct point *)
                     malloc (jello.resolution*jello.resolution*jello.resolution*sizeof(struct point));
    
  int forceIndex;  
  for (i=0; i<= jello.resolution-1; i++)
    for (j=0; j<= jello.resolution-1; j++)
      for (k=0; k<= jello.resolution-1; k++)
      {
        // set the force at node i,j,k
        // actual space location = x,y,z
        x = -2 + 4*(1.0 * i / (jello.resolution-1));
        y = -2 + 4*(1.0 * j / (jello.resolution-1));
        z = -2 + 4*(1.0 * k / (jello.resolution-1));

        forceIndex =   i * jello.resolution * jello.resolution + j * jello.resolution + k;
        jello.forceField[forceIndex].x = 0; 
        jello.forceField[forceIndex].y = 0;
        jello.forceField[forceIndex].z = 0;
	  }

  //set the positions of control points
  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
		  jello.p[i][j][k].x=(1.0 * i / 7) ;
	  	  jello.p[i][j][k].y=(1.0 * j / 7) ;
		  jello.p[i][j][k].z=(1.0 * k / 7) ;
	  }

  //set the velocities of control points
  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      { 
		  jello.v[i][j][k].x= 10;// * rand() % 100;
	  	  jello.v[i][j][k].y= 30;// * rand() % 100;
		  jello.v[i][j][k].z= 0; // * rand() % 100;
      }

  //write the jello variable out to file on disk
    writeWorld ("jello.w", &jello);

  return 0;
}
