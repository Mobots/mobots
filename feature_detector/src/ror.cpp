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

#include "ror.h"
#include "ror_int.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

void quickSort(double a[], int N);
int binarySearch(double key, double a[],int n);

#define random() ((double)(rand())/(double)(RAND_MAX))

/**************************************************************
  Function Name: rorError
  Description: the ror implementation's error handler
  Parameters:  int error_code - the error code
***************************************************************/
int rorError(int error_code) 
{
  switch (error_code)
  {
	case MEMORY_ALLOC_ERROR: 
	  printf("Memory Allocation Error\n");
      exit(1);
	  break;
  }
  return 0;
}


/**************************************************************
  Function Name: zSelect
  Description: finds the k-th smallest element in 
               the array
  Parameters:  double array[] - the array of elements
               int size - the size of the array
			   int k - the index of the smallest element
  Returns:     double - the k-th smallest element in the array
***************************************************************/
double zSelect(double array[],int size,int k)
{
  int l=0,r=size-1,i,j;
  double x,temp;
  double *temp_array;
  if ((temp_array= (double *) malloc((size)*sizeof(double)))==NULL)
    rorError(MEMORY_ALLOC_ERROR);

  memcpy(temp_array,array,(size)*sizeof(double));
  while (l<r)
  {
	x=temp_array[k];
	i=l;
	j=r;
    while (j>=k&&k>=i)
	{
	  while (temp_array[i]<x) i++;
	  while (x<temp_array[j]) j--;
	  temp=temp_array[i];
	  temp_array[i]=temp_array[j];
      temp_array[j]=temp;
	  i++;
	  j--;
	}
    if (j<k) l=i;
	if (k<i) r=j;
  }
  temp=temp_array[k];
  free(temp_array);
  return temp;
}


/**************************************************************
  Function Name: findMean_sorted
  Description: calculates the mean and the number of radian 
               elements in the sorted array ang[] which are 
			   between a_start and a_end 
			   if a_end<a_start then the window is considered 
			   to be circular
  Parameters:  double ang[] - the array of radian elements
               int size - the size of the array
               double a_start - the start of the window
			   double a_end - the end of the window
  Returns:     double - the mean of the elements in the window
               int rcount - the count of elements in the window
***************************************************************/
double findMean_sorted(double ang[],double a_start,
		        double a_end,int size,int *rcount)
{
  double sum=0,mean;
  int count=0,i,start_index;

  if (a_start<-M_PI) a_start=a_start+2.0*M_PI;
  else if (a_start>M_PI) a_start=a_start-2.0*M_PI;
  
  if (a_end<-M_PI) a_end=a_end+2.0*M_PI;
  else if (a_end>M_PI) a_end=a_end-2.0*M_PI;
     
  start_index=binarySearch(a_start,ang,size);
  /*find the element with the smallest index which is larger than a_start*/
  for(;(start_index>0) && (ang[start_index-1]==ang[start_index]);start_index--);
  

  if (a_start<=a_end) 
  {  
	
	for(i=start_index;(ang[i]<=a_end)&&(i<size);i++)
	{
      sum=sum+ang[i];
      count++;
    }  
    if (count>0) 
	  mean=sum/(double)count;
	else
	  mean=0;
  }
  else   
  {
    for(i=start_index;i<size;i++)
	{
	  sum=sum+ang[i];
      count++;
	}
	for(i=0;(ang[i]<=a_end)&&(i<size);i++)
	{
      sum=sum+ang[i]+2*M_PI;
      count++;
    } 
	if (count>0) 
	  mean=sum/(double)count;
	else
	  mean=0;
    if (mean>M_PI) mean=mean-2*M_PI;
  }
  *rcount=count;
  return mean;
}
/**************************************************************
  Function Name: findMode
  Description: calculates the mode of distribution
               of radian elements in the array ang[] 
  Parameters:  double ang[] - the array of radian elements
               int size - the size of the array
               int steps - the number of sub segments the 
			   segment from -PI to PI is divided for search
			   of candidate for the msm algorithm
  Returns:     double - the mode of distribution of the elements
***************************************************************/
double findMode(double ang[],int size,int steps,double window)
{
  double delta=2.0*M_PI/(double)steps;
  double half_delta=0.5*delta;
  double a_start=-M_PI-delta;
  double a_end=a_start+delta;
  double maxstart=0,maxmode=0,mode,new_mode;
  double epsilon=0.001;
  int i,count,maxv=0;
  double *sorted_ang;

  if ((sorted_ang= (double *) malloc((size)*sizeof(double)))==NULL)
    rorError(MEMORY_ALLOC_ERROR);
  
  memcpy(sorted_ang,ang,(size)*sizeof(double));
  
  quickSort(sorted_ang,size);

  while (a_start<M_PI+delta)
  {
	new_mode=findMean_sorted(sorted_ang,a_start,a_start+delta,size,&count);
	mode=20; /*so the first comparison will succed*/
    i=0;
    while ((fabs(new_mode-mode)>epsilon)&&(++i<30))
	{
      mode=new_mode;
	  new_mode=findMean_sorted(sorted_ang,mode-window,mode+window,size,&count);
	}
    if (count>maxv)
	{
      maxmode=mode;
      maxv=count;
	}
    a_start+=delta;
  }  

  free(sorted_ang);
  return maxmode;
}


/**************************************************************
  Function Name: fillPxo
  Description: calculates the array of radial angles of 
               the outlier
  Parameters:  point rotated_points[] - the array of 
               rotated features of the second frame
               point static_points[] - the array of
			   the static features of the first frame
               int numf - the number of points, features
  Returns:     double ang[] - the array of radial angles 
               of the outlier
***************************************************************/
int fillPxo(double ang[], point rotated_points[],
		   point static_points[],int numf )
{
  int i;
  double msm_x,temp;
  
  
  
  for (i=0;i<numf;i++)
    ang[i]=atan2(rotated_points[i].y-static_points[i].y,
	             rotated_points[i].x-static_points[i].x);
 
  
  
  msm_x=findMode(ang,numf,MSM_SAMPLES,REGULAR_MSM_WINDOW);

  for (i=0;i<numf;i++)
  {
    /*calculates for the radial angle in [-PI,PI] 
      the absolute value in [0,PI]*/
	temp=fabs(ang[i]-msm_x);
	if (temp>M_PI)
      ang[i]=fabs(temp-2*M_PI);
    else
	  ang[i]=temp;
  }


  return 0;
}


/**************************************************************
  Function Name: rotatePic
  Description: calculates the points after rotatation
               using the rotation matrix R
  Parameters:  double R[3][3] - the rotation matrix
               point op[] - the original points
               int numf - the number of points, features
  Returns:     point rotated_points[] - the rotated points
***************************************************************/
int rotatePic(double R[3][3],point op[], 
			   point rotated_points[], int numf)
{
  int i;
  double z;
  double x,y;
  double r02f,r12f,r22f;
    
  r02f=R[0][2]*FOCUS;
  r12f=R[1][2]*FOCUS;
  r22f=R[2][2]*FOCUS;

  for (i=0;i<numf;i++)
  {
    x=op[i].x;
	y=op[i].y;
	z=R[2][0]*x+R[2][1]*y+r22f;
	rotated_points[i].x=FOCUS*(R[0][0]*x+R[0][1]*y+r02f)/z;
    rotated_points[i].y=FOCUS*(R[1][0]*x+R[1][1]*y+r12f)/z;
  }
  
  return 0;
}


/**************************************************************
  Function Name: angle2r
  Description: calculates the rotation matrix rot_mat
               from the rotation axis and angle of rotation
			   (By Rodrigues formula)
  Parameters:  double theta - the radial angle of rotation
               double axis[3] - the rotation axis
  Returns:     double rot_mat[3][3] - the calculated rotation
               matrix
***************************************************************/
int angle2r(double theta,double axis[3],double rot_mat[3][3])
{
  double cos_theta, sin_theta,one_m_cos;

  cos_theta = cos(theta);
  sin_theta = sin(theta);
  one_m_cos=1.0-cos_theta;

  rot_mat[0][0]=cos_theta+one_m_cos*axis[0]*axis[0]; 
  rot_mat[0][1]=one_m_cos*axis[0]*axis[1]-sin_theta*axis[2];
  rot_mat[0][2]=one_m_cos*axis[0]*axis[2]+sin_theta*axis[1];

  rot_mat[1][0]=one_m_cos*axis[0]*axis[1]+sin_theta*axis[2];
  rot_mat[1][1]=cos_theta+one_m_cos*axis[1]*axis[1];
  rot_mat[1][2]=one_m_cos*axis[1]*axis[2]-sin_theta*axis[0];

  rot_mat[2][0]=one_m_cos*axis[2]*axis[0]-sin_theta*axis[1];
  rot_mat[2][1]=one_m_cos*axis[2]*axis[1]+sin_theta*axis[0];
  rot_mat[2][2]=cos_theta+one_m_cos*axis[2]*axis[2];

  return 0;
}


/**************************************************************
  Function Name: insertBestAngs
  Description: inserts the outlier's radial angle array
               into the best_rotations array
			   if the pl is smaller than one of the outlier's
			   rotations already in the best_rotations array
			   if the ang array is not good enought its allocated
			   memory is freed
  Parameters:  best_rotation_struct best_rotations[] - the best 
               rotations array
               double ang[] - the outlier's rotation array 
			   considered
			   int ang_size - the size of the outlier's array
			   int thr - the threshold for calculating the pl
               int rotation_num - the number of the current 
			   rotation
***************************************************************/
int insertBestAngs(best_rotation_struct best_rotations[],double ang[],
			       int ang_size,int thr,int rotation_num)
{
  int i,max_pl_index;
  double current_pl;
  double max_pl=0; // minimum pl value is 0

  current_pl=zSelect(ang,ang_size,thr);

  if (rotation_num<NUMBEST)
  {
    best_rotations[rotation_num].ang=ang;
	best_rotations[rotation_num].pl=current_pl;
  }
  else 
  {
	for (i=0;i<NUMBEST;i++)
	{
	  if (max_pl<best_rotations[i].pl) 
	  {
	    max_pl=best_rotations[i].pl;
		max_pl_index=i;
	  }
	}
	
	if (max_pl>current_pl)
	{
	  if(best_rotations[max_pl_index].ang!=NULL)
	    free(best_rotations[max_pl_index].ang);
	  
	  best_rotations[max_pl_index].ang=ang;
	  best_rotations[max_pl_index].pl=current_pl;
	  ang=NULL;
	}
	else if (ang!=NULL) free(ang);

  }
  return 0;
}


/**************************************************************
  Function Name: idinls
  Description: the workhourse function of the ror algorithm
               cleans out the outliers 
  Parameters:  point pnts_frame1[] - the points of frame 1
               point pnts_frame2[] - the points of frame 2
			   int numf - the number of points, features
  Returns:     int mask[] - the resulting array
               1 in the place the pair is considered correct
			   0 in the place the pair is considered incorrect
***************************************************************/
int idinls(point pnts_frame1[],point pnts_frame2[],
		   int numf,int mask[])
{
  
  double theta,angw;
  int i,j;
  double *ang,*mean_best;
  best_rotation_struct best_rotations[NUMBEST];
  int thr;
  point *rotated_points; 
  double axis[3];
  double axis_norm;
  double rotation_matrix[3][3];

  if ((rotated_points= (point *) malloc((numf)*sizeof(point)))==NULL)
    rorError(MEMORY_ALLOC_ERROR);
  
  thr=(int)floor((double)numf*THRESHHOLD);
  
  for (i=0;i<NUMROT;i++)
  {
	theta=random()*M_PI/4;

	axis[0]=-1.0+2.0*random(); 
	axis[1]=-1.0+2.0*random(); 
	axis[2]=-1.0+2.0*random();
   
	axis_norm=sqrt(axis[0]*axis[0]+axis[1]*axis[1]+axis[2]*axis[2]);
	axis[0]/=axis_norm; 
	axis[1]/=axis_norm; 
	axis[2]/=axis_norm;

	angle2r(theta,axis,rotation_matrix);
    
	rotatePic(rotation_matrix,pnts_frame2,rotated_points,numf);

	if ((ang = (double *) malloc((numf)*sizeof(double)))==NULL)
      rorError(MEMORY_ALLOC_ERROR);

    fillPxo(ang,rotated_points,pnts_frame1,numf);
    
	insertBestAngs(best_rotations,ang,numf,thr,i);
  }


  if ((mean_best = (double *) malloc((numf)*sizeof(double)))==NULL)
    rorError(MEMORY_ALLOC_ERROR);
 

  for (j=0;j<numf;j++)
  {
    mean_best[j]=0; 
 	for (i=0;i<NUMBEST;i++)
	{
      mean_best[j]+=best_rotations[i].ang[j];	  
	}
	mean_best[j]=mean_best[j]/(double)NUMBEST;
  }


  /* 10*MSM_SAMPLES samples are taken to calculate the 
     mode of the mean of best angles more precisely.
	 Since this is only done once per run it 
     doesn't slow down the algorithm significantly */
  angw=findMode(mean_best,numf,10*MSM_SAMPLES,FINAL_MSM_WINDOW);

  for (i=0;i<numf;i++)
    if (mean_best[i]<angw+ADDITION_FINAL_THRESHOLD) 
	  mask[i]=1;
	else 
	  mask[i]=0;

  for (i=0;i<NUMBEST;i++)
	free(best_rotations[i].ang);

  free(rotated_points);

  return 0;
}

/**************************************************************
  Function Name: ror
  Description: encapsulates the idinls function 
               runs idinls numruns times and calculates the 
			   average mask
  Parameters:  point pnts_frame1[] - the points of frame 1
               point pnts_frame2[] - the points of frame 2
			   int numf - the number of points, features
			   numruns  - the number of runs 
  Returns:     int mask[] - the resulting array
               1 in the place the pair is considered correct
			   0 in the place the pair is not considered correct
***************************************************************/
int ror(point pnts_frame1[],point pnts_frame2[],
		int numf,int numruns,int avgmask[])
{
  int *mask;
  int i,j;

  if ((mask = (int *) malloc(numf*sizeof(int)))==NULL) 
    rorError(MEMORY_ALLOC_ERROR);
  
  for (i=0;i<numf;i++)
    avgmask[i]=0;

  for (j=0;j<numruns;j++)
  {
    idinls(pnts_frame1,pnts_frame2,numf,mask);
    for (i=0;i<numf;i++)
  	  avgmask[i]+=mask[i];
  }
   
  for (i=0;i<numf;i++)
	avgmask[i]=(((double)avgmask[i]/(double)numruns)>0.5)?1:0;
  
  free(mask);
  
  return 1;
}
