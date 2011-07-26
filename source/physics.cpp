// Physics code.
//-Rejah anuvar

#include "jello.h"
#include "physics.h"
#include <vector>
#include <iostream>

using  namespace std;

static int collided=0;

// computes struct force at any point
// takes point i,j,k; updates Fstruct.
void computeStructForce(struct world * jello, int pti, int ptj, int ptk, struct point * Fstruct)
{
	//get the neighbour point indexes in neighbours list vector.
	vector<struct index> neighbours;
	struct index curIndex;
	
	if(pti!=0)
      {	
        curIndex.i = pti-1;
		curIndex.j = ptj;
		curIndex.k = ptk;
		neighbours.push_back(curIndex);	
      }
	if(pti!=7)
      {   
        curIndex.i = pti+1;
		curIndex.j = ptj;
		curIndex.k = ptk;
		neighbours.push_back(curIndex);	
      }
	if(ptj!=0)
      {
        curIndex.i = pti;
		curIndex.j = ptj-1;
		curIndex.k = ptk;
		neighbours.push_back(curIndex);	
      }
	if(ptj!=7)
      {
        curIndex.i = pti;
		curIndex.j = ptj+1;
		curIndex.k = ptk;
		neighbours.push_back(curIndex);	
      }
	if(ptk!=0)
      {
        curIndex.i = pti;
		curIndex.j = ptj;
		curIndex.k = ptk-1;
		neighbours.push_back(curIndex);	
      }
	if(ptk!=7)
      {
        curIndex.i = pti;
		curIndex.j = ptj;
		curIndex.k = ptk+1;
		neighbours.push_back(curIndex);	
      }
	
	vector<struct index>::iterator iter; 
	struct point  FtempStruct;      //force of 1 struct spring
	
	Fstruct->x = 0;
	Fstruct->y = 0;
	Fstruct->z = 0;
	
	FtempStruct.x = 0;
	FtempStruct.y = 0;
	FtempStruct.z = 0;
	
	//  iterate through neighbours 
    //  find force exerted by neighbour(i,j,k) on jello->p(i,j,k)
    for (iter = neighbours.begin(); iter!=neighbours.end(); iter++)
        {
		
		int ni,nj,nk; //neighbour i,j,k
		ni = iter->i;
		nj = iter->j;
		nk = iter->k;
		
        //vector L from neighbour to this point    
		point L;
		pDIFFERENCE(jello->p[pti][ptj][ptk],jello->p[ni][nj][nk],L );
		
		//velocity difference for damping
		point diffVel;
		pDIFFERENCE(jello->v[pti][ptj][ptk],jello->v[ni][nj][nk],diffVel );
		
		//magnitude of L, abs(L)
		double absL;
		absL = sqrt((L).x * (L).x + (L).y * (L).y + (L).z * (L).z);
		
		//dot product for damping
		double vDotL;
		DOTPRODUCT(diffVel, L, vDotL);
		
		//normalize L vector
		double length;
		pNORMALIZE(L);
		
		//Rest length of struct springs
		double Rlength;
		Rlength = 0.142857; //1/8
		
		//dx for Hooks law
		double dX;
		dX = absL - Rlength;
		
		//force magnitude
		double FstructMag;
		FstructMag = -1 * jello->kElastic * dX ;
		
		//force vector
		point FstructV;
		pMULTIPLY(L, FstructMag, FstructV);
		
		//add up single forces to Fstruct
        //Fstruct is final total strurct force on the point
		pSUM(FtempStruct,FstructV,FtempStruct); 
		
		//damping force
		double Fdamp;
		
		//damp force magnitude
		Fdamp = (-1 * jello->dElastic * vDotL) / absL;
		
		//direction of Fdamp = Fdamp * normalized L
		point FdampV;
		pMULTIPLY(L, Fdamp, FdampV);
		
		//add Fdamp to Fstruct
		pSUM(FtempStruct, FdampV, FtempStruct);
	}
	
	Fstruct->x = FtempStruct.x;
	Fstruct->y = FtempStruct.y;
	Fstruct->z = FtempStruct.z;
}

void computeShearForce(struct world * jello, int pti, int ptj, int ptk, struct point * Fshear)
{
	vector<struct index> Shneighbours;
	struct index curIndex;
	
	//i,j-1,k-1; i,j-1,k+1
	if(ptj-1>=0)
      {
       if(ptk-1>=0)
		 {
          curIndex.i = pti;
          curIndex.j = ptj-1;
          curIndex.k = ptk-1;
          Shneighbours.push_back(curIndex);	
		 }
       if(ptk+1<=7)
		 {
		  curIndex.i = pti;
          curIndex.j = ptj-1;
          curIndex.k = ptk+1;
          Shneighbours.push_back(curIndex);	
		 }
      }
	//i,j+1,k-1;i,j+1,k+1
	if(ptj+1<=7)
      {
       if(ptk-1>=0)
         {
          curIndex.i = pti;
          curIndex.j = ptj+1;
          curIndex.k = ptk-1;
          Shneighbours.push_back(curIndex);	
		 }
       if(ptk+1<=7)
		 {
          curIndex.i = pti;
          curIndex.j = ptj+1;
          curIndex.k = ptk+1;
          Shneighbours.push_back(curIndex);	
		 }
      }
	//i-1
	if(pti-1>=0)
      {   
       if(ptk-1>=0)
         {   
          curIndex.i = pti-1;
          curIndex.j = ptj;
          curIndex.k = ptk-1;
          Shneighbours.push_back(curIndex);	
         }
     if(ptk+1<=7)
       {   
        curIndex.i = pti-1;
        curIndex.j = ptj;
        curIndex.k = ptk+1;
        Shneighbours.push_back(curIndex);	
       }
     if(ptj-1>=0)
       {
          {   //i-1,j-1,k
				curIndex.i = pti-1;
				curIndex.j = ptj-1;
				curIndex.k = ptk;
				Shneighbours.push_back(curIndex);	
          }
        if(ptk-1>=0)
          {   
				curIndex.i = pti-1;
				curIndex.j = ptj-1;
				curIndex.k = ptk-1;
				Shneighbours.push_back(curIndex);	
          }
		if(ptk+1<=7)
          {   
				curIndex.i = pti-1;
				curIndex.j = ptj-1;
				curIndex.k = ptk+1;
				Shneighbours.push_back(curIndex);	
          }
       }
     if(ptj+1<=7)
       {	
		  {   //i-1,j+1,k
           curIndex.i = pti-1;
           curIndex.j = ptj+1;
           curIndex.k = ptk;
           Shneighbours.push_back(curIndex);	
          }
        if(ptk-1>=0)
          {   
           curIndex.i = pti-1;
           curIndex.j = ptj+1;
           curIndex.k = ptk-1;
           Shneighbours.push_back(curIndex);	
          }
        if(ptk+1<=7)
          {   
           curIndex.i = pti-1;
           curIndex.j = ptj+1;
           curIndex.k = ptk+1;
           Shneighbours.push_back(curIndex);	
          }
       }
	}
	if(pti+1<=7)
      {
		if(ptk-1>=0)
          {   
           curIndex.i = pti+1;
           curIndex.j = ptj;
           curIndex.k = ptk-1;
           Shneighbours.push_back(curIndex);	
          }
		if(ptk+1<=7)
          {   
           curIndex.i = pti+1;
           curIndex.j = ptj;
           curIndex.k = ptk+1;
           Shneighbours.push_back(curIndex);	
          }
		if(ptj-1>=0)
          {	
			{   //i+1,j-1,k
			curIndex.i = pti+1;
			curIndex.j = ptj-1;
			curIndex.k = ptk;
			Shneighbours.push_back(curIndex);	
			}
          if(ptk-1>=0)
			{   
				curIndex.i = pti+1;
				curIndex.j = ptj-1;
				curIndex.k = ptk-1;
				Shneighbours.push_back(curIndex);	
			}
          if(ptk+1<=7)
			{   
				curIndex.i = pti+1;
				curIndex.j = ptj-1;
				curIndex.k = ptk+1;
				Shneighbours.push_back(curIndex);	
			}
		}
		if(ptj+1<=7)
		  {	
			{   //i+1,j+1,k
				curIndex.i = pti+1;
				curIndex.j = ptj+1;
				curIndex.k = ptk;
				Shneighbours.push_back(curIndex);	
			}
          if(ptk-1>=0)
			{   
				curIndex.i = pti+1;
				curIndex.j = ptj+1;
				curIndex.k = ptk-1;
				Shneighbours.push_back(curIndex);	
			}
          if(ptk+1<=7)
			{   
				curIndex.i = pti+1;
				curIndex.j = ptj+1;
				curIndex.k = ptk+1;
				Shneighbours.push_back(curIndex);	
			}
		}
	}
	vector<struct index>::iterator iter; 
	point FtempShear;
	
	Fshear->x = 0;
	Fshear->y = 0;
	Fshear->z = 0;
	
	FtempShear.x = 0;
	FtempShear.y = 0;
	FtempShear.z = 0;
	
	//iterate through shear neighbours
	for (iter = Shneighbours.begin(); iter!=Shneighbours.end(); iter++)
	{
		int ni,nj,nk; 
		ni = iter->i;
		nj = iter->j;
		nk = iter->k;
		
		point L;
		pDIFFERENCE(jello->p[pti][ptj][ptk],jello->p[ni][nj][nk],L );
		
		//velocity difference for damping
		point diffVel;
		pDIFFERENCE(jello->v[pti][ptj][ptk],jello->v[ni][nj][nk],diffVel );
		
		//magnitude of L, abs(L)
		double absL;
		absL = sqrt((L).x * (L).x + (L).y * (L).y + (L).z * (L).z);
		
		//dot product for damping
		double vDotL;
		DOTPRODUCT(diffVel, L, vDotL);
		
		//normalize L1 vector
		double length;
		pNORMALIZE(L);		
		
		//Rest length of shear springs
		double Rshlength1,Rshlength2;
               Rshlength1 = sqrt(2) * 0.142857; //square diagonal
               Rshlength2 = sqrt(3) * 0.142857; //cube diagonal
		
		double dX;
		if((abs(pti-ni)==1)&&(abs(ptj-nj)==1)&&(abs(ptk-nk)==1)) //cube diagonal
            dX = absL - Rshlength2;
		else
            dX = absL - Rshlength1;
		
		//force magnitude
		double FshearMag;
               FshearMag = -1 * jello->kElastic * dX ;
		
		//Force vector
		point FshearV;
		pMULTIPLY(L, FshearMag, FshearV);
		
		//***add up single forces to Fstruct
		pSUM(FtempShear,FshearV,FtempShear); //Fstruct is final total strurct force on the point
	
		//damping force
		double Fdamp;
			
		//damp force magnitude
		Fdamp = (-1 * jello->dElastic * vDotL) / absL;
		
		//Direction of Fdamp = Fdamp1 * normalized L1
		point FdampV;
		pMULTIPLY(L, Fdamp, FdampV);
		
		//add Fdamp to Fstruct
		pSUM(FtempShear, FdampV, FtempShear);
	}
	Fshear->x = FtempShear.x;
	Fshear->y = FtempShear.y;
	Fshear->z = FtempShear.z;
}

void computeBendForce(struct world * jello, int pti, int ptj, int ptk, struct point * Fbend) //current point i, j, k
{
	vector<struct index> Bneighbours;
	struct index curIndex;
	
	if(pti>=2)
      {
       curIndex.i = pti-2;
       curIndex.j = ptj;
       curIndex.k = ptk;
       Bneighbours.push_back(curIndex);	
      }
	if(pti<=5)
      {
       curIndex.i = pti+2;
       curIndex.j = ptj;
       curIndex.k = ptk;
       Bneighbours.push_back(curIndex);	
      }
	if(ptj>=2)
      {
       curIndex.i = pti;
       curIndex.j = ptj-2;
       curIndex.k = ptk;
       Bneighbours.push_back(curIndex);	
      }
	if(ptj<=5)
      {
       curIndex.i = pti;
       curIndex.j = ptj+2;
       curIndex.k = ptk;
       Bneighbours.push_back(curIndex);	
      }
	if(ptk>=2)
      {
       curIndex.i = pti;
       curIndex.j = ptj;
       curIndex.k = ptk-2;
       Bneighbours.push_back(curIndex);	
      }
	if(ptk<=5)
      {
       curIndex.i = pti;
       curIndex.j = ptj;
       curIndex.k = ptk+2;
       Bneighbours.push_back(curIndex);	
      }
	
	vector<struct index>::iterator iter; 
	Fbend->x = 0;
	Fbend->y = 0;
	Fbend->z = 0;
	
	point FtempBend;	
	FtempBend.x = 0;
	FtempBend.y = 0;
	FtempBend.z = 0;
	
	//iterate through bend neighbours
	for (iter = Bneighbours.begin(); iter!=Bneighbours.end(); iter++)
	{
        //neighbour i,j,k
		int ni,nj,nk; 
		ni = iter->i;
		nj = iter->j;
		nk = iter->k;
		
        //L vector
		point L;
		pDIFFERENCE(jello->p[pti][ptj][ptk],jello->p[ni][nj][nk],L );
		
		//velocity difference for damping
		point diffVel;
		pDIFFERENCE(jello->v[pti][ptj][ptk],jello->v[ni][nj][nk],diffVel );
		
		//magnitude of L, abs(L)
		double absL;
		absL = sqrt((L).x * (L).x + (L).y * (L).y + (L).z * (L).z);
		
		//dot product for damping
		double vDotL;
		DOTPRODUCT(diffVel, L, vDotL);
		
		//normalize L1 vector
		double length;
		pNORMALIZE(L);
		
		//Rest length of bend springs
		double Rblength;
		Rblength = 2 * 0.142857;
		
		//dx for Hooks law
		double dX;
		dX = absL - Rblength;
		
		//force magnitude
		double FbendMag;
		FbendMag = -1 * jello->kElastic * dX ;
		
		//Force vector
		point FbendV;
		pMULTIPLY(L, FbendMag, FbendV);
		
		//add up single forces to Fstruct
		pSUM(FtempBend,FbendV,FtempBend); //Fstruct is final total strurct force on the point
		
		//damping force
		double Fdamp;
		//jello->dElastic = 0.08;
		
		//damp force magnitude
		Fdamp = (-1 * jello->dElastic * vDotL) / absL;
		
		//Direction of Fdamp = Fdamp1 * normalized L1
		point FdampV;
		pMULTIPLY(L, Fdamp, FdampV);
		
		//add Fdamp to Fstruct
		pSUM(FtempBend, FdampV, FtempBend);
	}	
	Fbend->x = FtempBend.x;
	Fbend->y = FtempBend.y;
	Fbend->z = FtempBend.z;
}

void computeExternalForce(struct world * jello, int pti, int ptj, int ptk, struct point * Fext)
{
	double px  , py  , pz   ,
           FFx0, FFy0, FFz0 ,
           FFx1, FFy1, FFz1 ;
    
	point  FFxyz  [2][2][2],
           FVector[2][2][2];
    
	int    found = 0;
	
	//point x, y, z.
	px = jello->p[pti][ptj][ptk].x;
	py = jello->p[pti][ptj][ptk].y;
	pz = jello->p[pti][ptj][ptk].z;
	
	
	// for a single point,
	// find the force field cube that it is in 
	// TODO - optimize this by calculating the cube directly
    
	int i,j,k;
	for (i=0; i< jello->resolution; i++)
	{
		FFx0 = -2 + 4 * (1.0 * i     / ( jello->resolution-1 ));
		FFx1 = -2 + 4 * (1.0 * (i+1) / ( jello->resolution-1));
		
        if ((px>=FFx0) && (px<=FFx1))
             break;
	}
	for (j=0; j< jello->resolution; j++)
	{
	    FFy0 = -2 + 4 * (1.0 * j   / (jello->resolution-1));
	    FFy1 = -2 + 4 * (1.0 *(j+1)/ (jello->resolution-1));
		
        if ((py>=FFy0)&&(py<=FFy1))
             break;
	}
	for (k=0; k< jello->resolution; k++)
	{
		FFz0 = -2 + 4 * (1.0 * k / (jello->resolution-1));
		FFz1 = -2 + 4*(1.0 * (k+1) / (jello->resolution-1));
		if((pz>=FFz0)&&(pz<=FFz1))
			break;
	}
		
	// get forces at the corners of the ForceField cube,
    // interpolate to get the overall force inside the cube.
	if((i<jello->resolution-1)&&(j<jello->resolution-1)&&(k<jello->resolution-1)) // point inside force field
	{
		int a,b,c,d;
		a = i * jello->resolution * jello->resolution;
		b = j * jello->resolution;
		c = (j+1) * jello->resolution;	  
		d = (i+1) * jello->resolution * jello->resolution;
		
		//Get 
		FVector[0][0][0].x = jello->forceField[a + b + k].x;
		FVector[0][0][0].y = jello->forceField[a + b + k].y;
		FVector[0][0][0].z = jello->forceField[a + b + k].z;
		
		FVector[0][0][1].x = jello->forceField[a + b + (k+1)].x;
		FVector[0][0][1].y = jello->forceField[a + b + (k+1)].y;
		FVector[0][0][1].z = jello->forceField[a + b + (k+1)].z;
		
		FVector[0][1][0].x = jello->forceField[a + c + k].x;
		FVector[0][1][0].y = jello->forceField[a + c + k].y;
		FVector[0][1][0].z = jello->forceField[a + c + k].z;
		
		FVector[0][1][1].x = jello->forceField[a + c + (k+1)].x;
		FVector[0][1][1].y = jello->forceField[a + c + (k+1)].y;
		FVector[0][1][1].z = jello->forceField[a + c + (k+1)].z;
		
		FVector[1][0][0].x = jello->forceField[d + b + k].x;
		FVector[1][0][0].y = jello->forceField[d + b + k].y;
		FVector[1][0][0].z = jello->forceField[d + b + k].z;
		
		FVector[1][0][1].x = jello->forceField[d + b + (k+1)].x;
		FVector[1][0][1].y = jello->forceField[d + b + (k+1)].y;
		FVector[1][0][1].z = jello->forceField[d + b + (k+1)].z;
		
		FVector[1][1][0].x = jello->forceField[d + c + k].x;
		FVector[1][1][0].y = jello->forceField[d + c + k].y;
		FVector[1][1][0].z = jello->forceField[d + c + k].z;
		
		FVector[1][1][1].x = jello->forceField[d + c + (k+1)].x;
		FVector[1][1][1].y = jello->forceField[d + c + (k+1)].y;
		FVector[1][1][1].z = jello->forceField[d + c + (k+1)].z;
		
        //trilinear interpolation
        double alpha,beta,gama,GL;
		alpha = px   - FFx0;
		beta  = py   - FFy0;
		gama  = pz   - FFz0;
		GL    = FFx1 - FFx0;
		
		double GLaGL, GLbGL, GLgGL,aGL,bGL,gGL;
		GLaGL = (GL - alpha)/GL ;
		GLbGL = (GL - beta)/GL ;
		GLgGL = (GL - gama)/GL ;
		
        aGL = alpha/GL;
		bGL = beta/GL;
		gGL = gama/GL;
		
		//interpolate
		point Fpoint;
		Fpoint.x = FVector[0][0][0].x * GLaGL * GLbGL * GLgGL +
                   FVector[1][0][0].x * aGL   * GLbGL * GLgGL +
                   FVector[0][1][0].x * GLaGL * bGL   * GLgGL +
                   FVector[0][0][1].x * GLaGL * GLbGL * gGL   +
                   FVector[1][0][1].x * aGL   * GLbGL * gGL   +
                   FVector[0][1][1].x * GLaGL * bGL   * gGL   +
                   FVector[1][1][0].x * aGL   * bGL   * GLgGL +
                   FVector[1][1][1].x * aGL   * bGL   * gGL   ; 
		
		Fpoint.y = FVector[0][0][0].y * GLaGL * GLbGL * GLgGL +
                   FVector[1][0][0].y * aGL   * GLbGL * GLgGL +
                   FVector[0][1][0].y * GLaGL * bGL   * GLgGL +
                   FVector[0][0][1].y * GLaGL * GLbGL * gGL   +
                   FVector[1][0][1].y * aGL   * GLbGL * gGL   +
                   FVector[0][1][1].y * GLaGL * bGL   * gGL   +
                   FVector[1][1][0].y * aGL   * bGL   * GLgGL +
                   FVector[1][1][1].y * aGL   * bGL   * gGL   ; 
		
		Fpoint.z = FVector[0][0][0].z * GLaGL * GLbGL * GLgGL +
                   FVector[1][0][0].z * aGL   * GLbGL * GLgGL +
                   FVector[0][1][0].z * GLaGL * bGL   * GLgGL +
                   FVector[0][0][1].z * GLaGL * GLbGL * gGL   +
                   FVector[1][0][1].z * aGL   * GLbGL * gGL   +
                   FVector[0][1][1].z * GLaGL * bGL   * gGL   +
                   FVector[1][1][0].z * aGL   * bGL   * GLgGL +
                   FVector[1][1][1].z * aGL   * bGL   * gGL   ; 
		
		Fext->x  = Fpoint.x;
		Fext->y  = Fpoint.y;
		Fext->z  = Fpoint.z;
		}
}

// Computes acceleration of every control point of the jello cube, 
// Takes in state of the 'jello'
// Returns result in array 'a'
void computeAcceleration(struct world * jello, struct point a[8][8][8])
{	
	for(int i =0; i<8; i++)
      for(int j =0; j<8; j++)
        for(int k =0; k<8; k++)
           {
            //compute struct, shear, bend spring forces   
            point Fstruct;
            computeStructForce(jello, i, j, k, &Fstruct);
		
            point Fshear;
            computeShearForce(jello, i, j, k, &Fshear);
	
            point Fbend;
            computeBendForce(jello, i, j, k, &Fbend);
		
            //computer external forces   
            point Fext;	 
            computeExternalForce(jello, i, j, k, &Fext); 
	
            //Total force = struct+shear +bend+ external
            point Ftotal;
            pSUM(Fstruct, Fshear, Ftotal);
            pSUM(Ftotal, Fbend, Ftotal);
            pSUM(Ftotal, Fext, Ftotal);
	
            // check for collission with the box
            // if point was inside the box
            if(jello->inBox[i][j][k]) 
              { 
               collided = 0; 
               //check collision   
               if  ((jello->p[i][j][k].x<=-2)||(jello->p[i][j][k].x>=2) || 
                    (jello->p[i][j][k].y<=-2)||(jello->p[i][j][k].y>=2) || 
                    (jello->p[i][j][k].z<=-2)||(jello->p[i][j][k].z>=2)  ) 
                    {
                     //collided - point outside the box now   
                     jello->inBox[i][j][k] = false;             
                     collided++;
			
                     //first collision for collision point    
                     if (collided==1)							
                        {   
                        jello->pCollission[i][j][k].x =jello->p[i][j][k].x;   
                        jello->pCollission[i][j][k].y =jello->p[i][j][k].y;   
                        jello->pCollission[i][j][k].z =jello->p[i][j][k].z;
                        }
                    }
              }
            else // point was outside the box
              {
               // check if point inside the box now   
               if((jello->p[i][j][k].x>=-2) && (jello->p[i][j][k].x<=2)&&
                  (jello->p[i][j][k].y>=-2) && (jello->p[i][j][k].y<=2)&&
			      (jello->p[i][j][k].z>=-2) && (jello->p[i][j][k].z<=2) ) 
                  {   
				   jello->inBox[i][j][k] = true;
                  }
			else 
                //point still outside, find force due to collission
                //calculate elastic force between surface and current position
                //create spring between pCollission[0][0][0] and current p[0][0][0]
			{
			point L;
			pDIFFERENCE(jello->p[i][j][k],jello->pCollission[i][j][k],L );
			
		    //velocity difference for damping
		    point       diffVel, vpCollision;
            pMAKE      (0, 0, 0, vpCollision);
		    pDIFFERENCE(jello->v[i][j][k],vpCollision,diffVel );
		
			//length of L
			double absL;
			absL = sqrt((L).x * (L).x + (L).y * (L).y + (L).z * (L).z);
			
			//normal vector
			struct point Nfloor; //normal to floor
		
		    //update Nfloor according to the colliding wall
			if(jello->p[i][j][k].x<=-2)
			{
				pMAKE(1,0,0,Nfloor);
			}
		
			else if(jello->p[i][j][k].x>=2)
			{
				pMAKE(-1,0,0,Nfloor);
			}
		
			else if(jello->p[i][j][k].y<=-2)
			{
				pMAKE(0,1,0,Nfloor);
			}
			
			else if(jello->p[i][j][k].y>=2)
			{
				pMAKE(0,-1,0,Nfloor);
			}	
			
			else if(jello->p[i][j][k].z<=-2)
			{
				pMAKE(0,0,1,Nfloor);
			}
			else if(jello->p[i][j][k].z>=2)
			{
				pMAKE(0,0,-1,Nfloor);
			}
		
			//dot product for damping
			double vDotL;
			DOTPRODUCT(diffVel, L, vDotL);

            //collission force magnitude    
			double FcolMag; 
			FcolMag = jello->kCollision * absL;
			
			//collision force vector
			point FcolV;
			pMULTIPLY(Nfloor, FcolMag, FcolV);
		
			//damping force magnitude
			double FcolDamp;
			FcolDamp = ( jello->dCollision * vDotL) / absL;
		
			//direction of Fdamp = Fdamp1 * normal of floor
			point FcolDampV;
			pMULTIPLY(Nfloor, FcolDamp , FcolDampV);
			pSUM     (FcolV , FcolDampV, FcolV    );
                
			//add collision force to total force
            pSUM(Ftotal, FcolV, Ftotal);
			}	
		}
		

	point accel;
	pMULTIPLY( Ftotal, (1/jello->mass), accel);
		
	// Keep the system stable by cutting the forces in half when it goes beyond control	
		if((Ftotal.x>70)||(Ftotal.x<-70))
			Ftotal.x/=2;
		
		if((Ftotal.y>70)||(Ftotal.y<-70))
			Ftotal.y/=2;
		
		if((Ftotal.z>70)||(Ftotal.z<-70))
			Ftotal.z/=2;
		
	//Update a[][][] matrix
	a[i][j][k] = accel;
	}
}

// performs one step of Euler Integration
// updates the jello structure
void Euler(struct world * jello)
{
  int   i, j, k   ;
  point a[8][8][8];
    
  computeAcceleration(jello, a);
    
  for(i=0; i<=7; i++)
    for(j=0; j<=7; j++)
      for(k=0; k<=7; k++)
         { 
          jello->p[i][j][k].x += jello->dt * jello->v[i][j][k].x;
          jello->p[i][j][k].y += jello->dt * jello->v[i][j][k].y;
          jello->p[i][j][k].z += jello->dt * jello->v[i][j][k].z;
             
		  jello->v[i][j][k].x += jello->dt * a[i][j][k].x;
          jello->v[i][j][k].y += jello->dt * a[i][j][k].y;
          jello->v[i][j][k].z += jello->dt * a[i][j][k].z;
         }
}

// performs one step of RK4 Integration
// updates the jello structure
void RK4(struct world * jello)
{
  point F1p[8][8][8], F1v[8][8][8], 
        F2p[8][8][8], F2v[8][8][8],
        F3p[8][8][8], F3v[8][8][8],
        F4p[8][8][8], F4v[8][8][8];
    
  point a[8][8][8];
  
  struct world buffer;
  int    i, j, k     ;

  // make a copy of jello  
  buffer = *jello; 

  computeAcceleration(jello, a);
  for(i=0; i<=7; i++)
    for(j=0; j<=7; j++)
      for(k=0; k<=7; k++)
         {
          pMULTIPLY(jello->v[i][j][k], jello->dt, F1p[i][j][k]     );
          pMULTIPLY(a[i][j][k]       , jello->dt, F1v[i][j][k]     );
          pMULTIPLY(F1p[i][j][k]     , 0.5      , buffer.p[i][j][k]);
          pMULTIPLY(F1v[i][j][k]     , 0.5      , buffer.v[i][j][k]);
         
          pSUM (jello->p[i][j][k], buffer.p[i][j][k], buffer.p[i][j][k]);
          pSUM (jello->v[i][j][k], buffer.v[i][j][k], buffer.v[i][j][k]);
         }
  
	
  computeAcceleration(&buffer, a);
  for(i=0; i<=7; i++)
    for(j=0; j<=7; j++)
      for(k=0; k<=7; k++)
         {
          //    F2p = dt * buffer.v;
          pMULTIPLY ( buffer.v[i][j][k], jello->dt, F2p[i][j][k]     );
          //    F2v = dt * a(buffer.p,buffer.v);     
          pMULTIPLY ( a[i][j][k]       , jello->dt, F2v[i][j][k]     );
          pMULTIPLY ( F2p[i][j][k]     , 0.5      , buffer.p[i][j][k]);
          pMULTIPLY ( F2v[i][j][k]     , 0.5      , buffer.v[i][j][k]);
          
          pSUM(jello->p[i][j][k], buffer.p[i][j][k], buffer.p[i][j][k]);
          pSUM(jello->v[i][j][k], buffer.v[i][j][k], buffer.v[i][j][k]);
      }
  	
  computeAcceleration(&buffer, a);
  for(i=0; i<=7; i++)
    for(j=0; j<=7; j++)
      for(k=0; k<=7; k++)
         {
          //   F3p = dt * buffer.v;
          pMULTIPLY(buffer.v[i][j][k], jello->dt, F3p[i][j][k]     );
          //   F3v = dt * a(buffer.p,buffer.v);     
          pMULTIPLY(a[i][j][k]       , jello->dt, F3v[i][j][k]     );
          pMULTIPLY(F3p[i][j][k]     , 0.5      , buffer.p[i][j][k]);
          pMULTIPLY(F3v[i][j][k]     , 0.5      , buffer.v[i][j][k]);
          
          pSUM (jello->p[i][j][k], buffer.p[i][j][k], buffer.p[i][j][k]);
          pSUM (jello->v[i][j][k], buffer.v[i][j][k], buffer.v[i][j][k]);
      }
     
  computeAcceleration(&buffer, a);
  for(i=0; i<=7; i++)
    for(j=0; j<=7; j++)
      for(k=0; k<=7; k++)
         {
          // F3p = dt * buffer.v;
          pMULTIPLY (buffer.v[i][j][k], jello->dt, F4p[i][j][k]);
          // F3v = dt * a(buffer.p,buffer.v);     
          pMULTIPLY (a[i][j][k]       , jello->dt, F4v[i][j][k]);
             
          pMULTIPLY (F2p[i][j][k], 2, buffer.p[i][j][k]        );
          pMULTIPLY (F3p[i][j][k], 2, buffer.v[i][j][k]        );
          
          pSUM      (buffer.p[i][j][k], buffer.v[i][j][k], buffer.p[i][j][k]);
          pSUM      (buffer.p[i][j][k], F1p[i][j][k]     , buffer.p[i][j][k]);
          pSUM      (buffer.p[i][j][k], F4p[i][j][k]     , buffer.p[i][j][k]);
          pMULTIPLY (buffer.p[i][j][k], 1.0 / 6          , buffer.p[i][j][k]);
          pSUM      (buffer.p[i][j][k], jello->p[i][j][k], jello->p[i][j][k]);
             
          pMULTIPLY (F2v[i][j][k]     , 2                , buffer.p[i][j][k]);
          pMULTIPLY (F3v[i][j][k]     , 2                , buffer.v[i][j][k]);
          pSUM      (buffer.p[i][j][k], buffer.v[i][j][k], buffer.p[i][j][k]);
          pSUM      (buffer.p[i][j][k], F1v[i][j][k]     , buffer.p[i][j][k]);
          pSUM      (buffer.p[i][j][k], F4v[i][j][k]     , buffer.p[i][j][k]);
          pMULTIPLY (buffer.p[i][j][k], 1.0 / 6          , buffer.p[i][j][k]);
          pSUM      (buffer.p[i][j][k], jello->v[i][j][k], jello->v[i][j][k]);
         }
  return;  
}
