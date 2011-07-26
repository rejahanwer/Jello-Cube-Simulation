#include "jello.h"
#include "input.h"

// Write a screenshot in the PPM format
void saveScreenshot(int windowWidth, int windowHeight, char *filename)
{
  if (filename == NULL)
      return;

  // Allocate a picture buffer 
  Pic * in = pic_alloc(windowWidth, windowHeight, 3, NULL);
  printf("File to save to: %s\n", filename);

  for (int i=windowHeight-1; i>=0; i--) 
      {
       glReadPixels(0, windowHeight-i-1, windowWidth, 1, GL_RGB, GL_UNSIGNED_BYTE,
       &in->pix[i*in->nx*in->bpp]);
      }
  if  (ppm_write(filename, in))
       printf("File saved Successfully\n");
  else
      printf("Error in Saving\n");
    
  pic_free(in);
}

/* converts mouse drags into information about rotation/translation/scaling */
void mouseMotionDrag(int x, int y)
{
  int vMouseDelta[2] = {x-g_vMousePos[0], y-g_vMousePos[1]};
  if (g_iRightMouseButton) // handle camera rotations
     {
      Phi   += vMouseDelta[0] * 0.01;
      Theta += vMouseDelta[1] * 0.01;
    
      if (Phi  > 2*pi)
         Phi -= 2*pi;
    
      if (Phi < 0)
         Phi += 2*pi;
    
      if (Theta > pi / 2 - 0.01) 
         Theta = pi / 2 - 0.01;
    
      if (Theta <-pi / 2 + 0.01)
         Theta =-pi / 2 + 0.01;
    
      g_vMousePos[0] = x;
      g_vMousePos[1] = y;
     }
}

void mouseMotion (int x, int y)
{
     g_vMousePos[0] = x;
     g_vMousePos[1] = y;
}

void mouseButton(int button, int state, int x, int y)
{
  switch (button)
    {
     case GLUT_LEFT_BUTTON:
         g_iLeftMouseButton = (state==GLUT_DOWN);
         break;
    
     case GLUT_MIDDLE_BUTTON:
         g_iMiddleMouseButton = (state==GLUT_DOWN);
         break;
    
     case GLUT_RIGHT_BUTTON:
         g_iRightMouseButton = (state==GLUT_DOWN);
         break;
    }
    
  g_vMousePos[0] = x;
  g_vMousePos[1] = y;
}

// gets called whenever a key is pressed
void keyboardFunc (unsigned char key, int x, int y)
{
  switch (key)
    {
     case 27:
          exit(0);
          break;

     case 'e':
           Theta = pi / 6;
           Phi = pi / 6;
           viewingMode = 0;
           break;

     case 'v':
           viewingMode = 1 - viewingMode;
           break;

     case 'h':
           shear = 1 - shear;
           break;

     case 's':
           structural = 1 - structural;
           break;

     case 'b':
           bend = 1 - bend;
           break;

     case 'p':
           doPause = 1 - doPause;
           break;

     case 'z':
           R -= 0.2;
           if (R < 0.2)
               R = 0.2;
           break;
            
     case 'x':
           R += 0.2;
           break;

     case ' ':
           saveScreenToFile = 1 - saveScreenToFile;
           break;
    }
}

// reads the world parameters from a world file 
void readWorld (char * fileName, struct world * jello)
{
  int i,j,k;
  FILE * file;
  
  file = fopen(fileName, "r");
  if (file == NULL) 
     {
      printf ("can't open file\n");
      exit(1);
     }
 
/*File format: 
  integrator (EULER or RK4)
  dt n - timestep, every nth timestep is drawn     
  kElastic dElastic kCollision dCollision
  mass
  inclinedPlanePresent? (0/1)
  a b c d
  
  Next line: forceField block, first with the resolution and then the data, one point per row.
  Example:
    30
    <here 30 * 30 * 30 = 27 000 lines follow, each containing 3 real numbers>
  
  After this, there should be 1024 lines, each containing three floating-point numbers.
  The first 512 lines correspond to initial point locations.
  The last  512 lines correspond to initial point velocities.
  
  no blank lines in the file 
*/
       
  //read integrator algorithm 
  fscanf(file,"%s\n",&jello->integrator);

  //read timestep size and render
  fscanf(file,"%lf %d\n",&jello->dt,&jello->n);

  //read physical parameters
  fscanf(file, "%lf %lf %lf %lf\n", 
    &jello->kElastic, &jello->dElastic, &jello->kCollision, &jello->dCollision);

  // read mass of each of the 512 points
  fscanf(file, "%lf\n", &jello->mass);

  // read info about the plane
  fscanf(file, "%d\n", &jello->incPlanePresent);
  if (jello->incPlanePresent == 1)
    fscanf(file, "%lf %lf %lf %lf\n", &jello->a, &jello->b, &jello->c, &jello->d);

  // read info about the force field
  fscanf(file, "%d\n", &jello->resolution);
  jello->forceField = (struct point *)
                       malloc(jello->resolution*jello->resolution*jello->resolution*sizeof(struct point));
  if (jello->resolution != 0)
    for (i=0; i<= jello->resolution-1; i++)
      for (j=0; j<= jello->resolution-1; j++)
        for (k=0; k<= jello->resolution-1; k++)
            {
             fscanf(file, "%lf %lf %lf\n", 
             &jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].x, 
             &jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].y, 
             &jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].z);
            }    
             
  /* read initial point positions */
  for (i= 0; i <= 7 ; i++)
    for (j = 0; j <= 7; j++)
      for (k = 0; k <= 7; k++)
          {
           fscanf(file, "%lf %lf %lf\n", 
           &jello->p[i][j][k].x, &jello->p[i][j][k].y, &jello->p[i][j][k].z);
          }    
      
  /* read initial point velocities */
  for (i = 0; i <= 7 ; i++)
    for (j = 0; j <= 7; j++)
      for (k = 0; k <= 7; k++)
          {
           fscanf(file, "%lf %lf %lf\n", 
          &jello->v[i][j][k].x, &jello->v[i][j][k].y, &jello->v[i][j][k].z);
          }

  //All points are in	
  for (i = 0; i < 8; i++)
    for (j = 0; j < 8; j++)
      for (k = 0; k < 8; k++)
          {
           if((jello->p[i][j][k].x>-2 && jello->p[i][j][k].x<2) &&
             (jello->p[i][j][k].y>-2 && jello->p[i][j][k].y<2) &&
             (jello->p[i][j][k].z>-2 && jello->p[i][j][k].z<2)  )
              jello->inBox[i][j][k] = true;
           else 
              jello->inBox[i][j][k] = false;
          }
	
  fclose(file);
  return;
}

/* writes the world parameters to a world file on disk*/
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

  // write integrator algorithm 
  fprintf(file,"%s\n",jello->integrator);

  // write timestep
  fprintf(file,"%lf %d\n",jello->dt,jello->n);

  // write physical parameters
  fprintf(file, "%lf %lf %lf %lf\n", 
    jello->kElastic, jello->dElastic, jello->kCollision, jello->dCollision);

  // write mass
  fprintf(file, "%lf\n", 
    jello->mass);

  // write info about the plane
  fprintf(file, "%d\n", jello->incPlanePresent);
  if (jello->incPlanePresent == 1)
    fprintf(file, "%lf %lf %lf %lf\n", jello->a, jello->b, jello->c, jello->d);

  // write info about the force field 
  fprintf(file, "%d\n", jello->resolution);
  if (jello->resolution != 0)
    for (i=0; i<= jello->resolution-1; i++)
      for (j=0; j<= jello->resolution-1; j++)
        for (k=0; k<= jello->resolution-1; k++)
          fprintf(file, "%lf %lf %lf\n", 
             jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].x, 
             jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].y, 
             jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k].z);
  


  // write initial point positions
  for (i = 0; i <= 7 ; i++)
    for (j = 0; j <= 7; j++)
      for (k = 0; k <= 7; k++)
        {
        fprintf(file, "%lf %lf %lf\n", 
        jello->p[i][j][k].x, jello->p[i][j][k].y, jello->p[i][j][k].z);
        }    
      
  // write initial point velocities
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

