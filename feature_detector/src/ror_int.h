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



#ifndef RORINT_H

#define RORINT_H

/* FOCUS - the focus of the camera, in pixels */

#define FOCUS 994.0

/* MSMSAMPLES - the number of samples whith which
   the msm algorithm tries to calculate the 
   mode of distribution.
   Since the angles are sorted before calculating
   the mean values in various windows this value has 
   only a medium influence on the performance speed of
   the entire algorithm.
   Beyond 20 samples no significant increase in precision 
   was recorded */

#define MSM_SAMPLES 20

/* NUMROT - the number of rotation to sample in the random
            search */

#define NUMROT 1000 

/* NUMBEST - the number of the good rotation to be extracted */

#define NUMBEST 50

/* THRESHHOLD - for finding the good rotations
   this should be slightly higher than
   the expected percentage of  correct pairs 
   in the input list
   
   (we had roughly 25% inliers --> 1/3) */

#define THRESHHOLD 1.0/3.0

/* FINAL_MSM_WINDOW - the size of half a window used in 
   calculating the MSM of the average of good rotations */

#define FINAL_MSM_WINDOW 0.0174532925 // 1 degree                      

/* REGULAR_MSM_WINDOW - the size of half a window used in
   calculating the MSM of each rotation */

#define REGULAR_MSM_WINDOW 0.1308996  // 7.5 degrees

/* ADDITION_FINAL_TRESHOLD - added to the msm calculated 
   from the average of good rotation.
   The resulting sum is used to determine the inliers */

#define ADDITION_FINAL_THRESHOLD 0.0174532925 // 1 degree

typedef struct best_rotations_st
{
  double pl;
  double *ang;
} best_rotation_struct;

#define MEMORY_ALLOC_ERROR 0

#ifdef WIN32
  #define M_PI 3.14159265358979 
#endif


#endif
