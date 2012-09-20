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

#ifndef ROR_H

#define ROR_H

/* the ror function expects two arrays of this type */

typedef struct point_st
{
  double x,y;
} point;


/**************************************************************
  Function Name: ror
  Description: encapsulates the idinls function 
               runs idinls numruns times and calculates the 
			   average mask
  Parameters:  point pnts_frame1[] - the points of frame 1
               point pnts_frame2[] - the points of frame 2
	       int numf - the number of points, features
	       numruns  - the number of runs 
  Returns:     int avgmask[] - a mask defining the inliers and outliers
               1 means a correct match, 0 means a false match.

***************************************************************/
int ror(point pnts_frame1[],point pnts_frame2[],
		int numf,int numruns,int avgmask[]);

#endif
