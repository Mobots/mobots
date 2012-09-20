/******************************************************************

   ROR source code
   ---------------

   These files are a C implementation of the ROR algorithm.
   
   This implementation was written by Moti Gornshtein, moti@lti.co.il

   The algorithm was developed by Amit Adam, amita@tx.technion.ac.il
  

   The algorithm is described in:

   "ROR: Rejection of Outliers by Rotations in Stereo Matching", A. Adam, E. Rivlin 
   and I. Shimshoni, Proc. IEEE Conference on Computer Vision and Pattern Recognition
   (CVPR) 2000, pp. 2-9.

   ------------------------

   Important notice:
 
   Feel free to use this algorithm for research purposes.
   Please notify the author (Amit Adam, amita@tx.technion.ac.il) of your
   experience with it, and include a reference to it in your work.

   Please do not distribute modified versions of these source files. 

   A patent on the algorithm is pending (filed in the US). For uses other than 
   academic research please contact the author (Amit Adam).

  
   --------------------------

   For additional details please see http://www.cs.technion.ac.il/~amita

*********************************************************************/    

/********

run_ror.c - a program which shows how to use ror

********/

#include "ror.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/timeb.h>

#define PNT_FILE "features.dat"

int main()
{
  struct timeb tstruct;
  int numf;
  int i;
  point pnts_frame1[600],pnts_frame2[600]; 
  FILE *pnt_file;

  int * mask;

  struct timeb tsruct;

  // random number generator seed

  ftime(&tstruct);
  srand(tstruct.millitm);

  // read list of matching points from file

  if ((pnt_file=fopen(PNT_FILE,"r"))==NULL)
  {
    printf("Can't read input file!\n");
    exit(1);
  }
  i=0;
  while (fscanf(pnt_file,"%lf %lf %lf %lf",
	            &pnts_frame1[i].x,&pnts_frame1[i].y,
	            &pnts_frame2[i].x,&pnts_frame2[i].y)>0)
  {
	  i++;
  }
  numf=i;
  fclose(pnt_file);

  /********************

     allocate space for the mask which will be the output:

  *********************/

   mask=calloc(numf,sizeof(int));

/*********************

  call the function

*********************/
	      
   ror(pnts_frame1,pnts_frame2,numf,10,mask);

// output the resulting mask

  for (i=0;i<numf;i++)
  {
	printf("%d\n",mask[i]);
  }
  

  return 0;

}
