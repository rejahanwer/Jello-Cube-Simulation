#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pic.h"


// pic_alloc: allocate picture memory.
// If opic!=0, then memory from opic->pix is reused (after checking that
// size is sufficient), else a new pixel array is allocated.

Pic* pic_alloc(int nx, int ny, int bytes_per_pixel, Pic *opic) 
{
  Pic *p;
  int size = ny*nx*bytes_per_pixel;

  ALLOC(p, Pic, 1);
  p->nx  = nx;
  p->ny  = ny;
  p->bpp = bytes_per_pixel;
  
  if (opic && opic->nx*opic->ny*opic->bpp >= p->nx*p->ny*p->bpp) 
     {
         p->pix = opic->pix;
      // now opic and p have a common pix array */
     }
  else
     ALLOC(p->pix, Pixel1, size);
  
  return p;
}

void pic_free(Pic *p) 
{
  free(p->pix);
  free(p);
}

//pic_xxx routines will call ppm_xxx 
Pic_file_format pic_file_type(char *file)
{
  unsigned char byte[10];
            int i       ;

  FILE *pic = fopen(file, "r");
  if     (!pic)
  return PIC_UNKNOWN_FILE;

  return PIC_PPM_FILE;

}

Pic_file_format pic_filename_type(char *file)
{
  char *suff;
  suff = strrchr(file, '.');
  if (!strcmp(suff, ".jpg")) 
      return PIC_JPEG_FILE;
  if (!strcmp(suff, ".tiff") || !strcmp(suff, ".tif")) return PIC_TIFF_FILE;
  if (!strcmp(suff, ".ppm")) return PIC_PPM_FILE;
  return PIC_UNKNOWN_FILE;
}

int pic_get_size(char *file, int *nx, int *ny)
{
  switch( pic_file_type(file) )
  {
    case PIC_TIFF_FILE:
    //not implemented      
	//return tiff_get_size(file, nx, ny);
    break;
			
    case PIC_PPM_FILE:
	return ppm_get_size(file, nx, ny);
    break;

    case PIC_JPEG_FILE:
    //not implemented      
	//return jpeg_get_size(file, nx, ny);
    break;
			
    default:
    return FALSE;
  }
}


// pic_read: read a PPM file into memory.
// Normally, we should use opic==NULL.
// If opic!=NULL, then picture is read into opic->pix (after checking that
// size is sufficient), else a new Pic is allocated.
// Returns Pic pointer on success, NULL on failure.
Pic *pic_read(char *file, Pic *opic)
{
  switch( pic_file_type(file) )
  {
    case PIC_TIFF_FILE:
      //not implemented    
      //return tiff_read(file, opic);
    break;
			
    case PIC_PPM_FILE:
      return ppm_read(file, opic);
    break;

    case PIC_JPEG_FILE:
      //return jpeg_read(file, opic);
    break;

    default:
      return NULL;
    }
}


// pic_write: write pic to file in the specified format
// returns TRUE on success, FALSE on failure
int pic_write(char *file, Pic *pic, Pic_file_format format)
{
  switch( format )
  {
    case PIC_TIFF_FILE:
      //return tiff_write(file, pic);
    break;
			
    case PIC_PPM_FILE:
      return ppm_write(file, pic);
    break;

    case PIC_JPEG_FILE:
     //return jpeg_write(file, pic);
    break;
			
    default:
      fprintf(stderr, "pic_write: can't write %s, unknown format\n", file);
      return FALSE;
  }
}

