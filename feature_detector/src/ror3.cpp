#include <opencv2/core/core.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

#include "ror.h"
#include "FeaturesMatcher.h"
#include "profile.h"

using namespace std;
using namespace cv;

#define rotationStep 0.008d
#define N int(2*M_PI/rotationStep) + 3
#define MID ((N)/2)
#define MIN (-MID)
#define MAX ((N-3)/2 + 1)


const int UNCERTAINITY_THRESHOL0D = 5; //if <
const int LENGTHDIFF_THRESHOLD = 2;    //if >

static uint counts[N];
static double sumsTheta[N];
static double sumsX1[N];
static double sumsY1[N];
static double sumsX2[N];
static double sumsY2[N];

static unsigned int* const countsMid = &counts[MID];
static double* const sumsMid = &sumsTheta[MID];
static double* const sumsMidX1 = &sumsX1[MID];
static double* const sumsMidY1 = &sumsY1[MID];
static double* const sumsMidX2 = &sumsX2[MID];
static double* const sumsMidY2 = &sumsY2[MID];

/*static uint uncertain_counts[N];
static double uncertain_sumsTheta[N];
static double uncertain_sumsX[N];
static double uncertain_sumsY[N];

static unsigned int* const uncertain_countsMid = &uncertain_counts[MID];
static double* const uncertain_sumsMid = &uncertain_sumsTheta[MID];
static double* const uncertain_sumsMidX = &uncertain_sumsX[MID];
static double* const uncertain_sumsMidY = &uncertain_sumsY[MID];*/

unsigned long nCr( unsigned int n, unsigned int k ){
    if (k > n) return 0;
    if (k * 2 > n) k = n-k;
    if (k == 0) return 1;

    long result = n;
    for( int i = 2; i <= k; ++i ) {
        result *= (n-i+1);
        result /= i;
    }
    return result;
}


inline double toDegree(double rad){
  return rad * 180 / M_PI;
}

double euclideanDistance(const Point2f& p1, const Point2f& p2){
  double yDiff = double(p2.y)-double(p1.y);
  double xDiff = double(p2.x)-double(p1.x);
  return sqrt(xDiff*xDiff + yDiff*yDiff);
}

/**
 * [0, pi]
 */
inline double getHeading(const Point2f& p1, const Point2f& p2){
  double yDiff = double(p2.y)-double(p1.y);
  double xDiff = double(p2.x)-double(p1.x);
  /*if(abs(xDiff) == 0 && abs(yDiff) == 0){
    return 0;
  }*/ //impossibru
  double result = atan2(yDiff, xDiff);
  if(result < 0)
    result = M_PI + result;
  return result;
}
/**
 * [-pi, pi]
 */
inline double getRotation(const Point2f& p11, const Point2f& p12, const Point2f& p21, const Point2f& p22){
  double heading1 = getHeading(p11, p12);
  double heading2 = getHeading(p21, p22);
  double diff = heading2-heading1;
  /*if(diff < M_PI)
    diff += 2*M_PI;
  else if(diff > M_PI)
    diff -= 2* M_PI;*/
    //if(diff < 0)
    //diff = M_PI + diff;
  return diff;
}

inline void rotate(const Point2f& in, Point2f& out, double angle){
  double cosa = cos(angle);
  double sina = sin(angle);
  double xOrig = in.x;
  double yOrig = in.y;
  out.x = xOrig*cosa - yOrig*sina;
  out.y = xOrig*sina + yOrig*cosa;
}

//DEBUG
/*extern double rotationStep;
extern int N;
extern int MID;*/

/**
 * FTW
 * Inspired by ror
 * maybe use a simpler method than computing all possible combinations when n is very high,
 * complexity is at least nC2
 */
bool rorAlternative(const vector<Point2f>& points1, const vector<Point2f>& points2, Delta& delta){
  //additional ideas:
  //first simple matching: p1[i],p1[i+1] <--> p2[i],p2[i+1]
  // 		     and: p1[i],p1[i+2] <--> p2[i],p2[i+2]
  //if one rotation get's more than 60% it's ok.
  
  for(int i = 0; i < N; i++){
    counts[i] = 0;
    sumsTheta[i] = 0;
    sumsX1[i] = 0;
    sumsY1[i] = 0;
    sumsX2[i] = 0;
    sumsY2[i] = 0;
    /*uncertain_counts[i] = 0;
    uncertain_sumsTheta[i] = 0;
    uncertain_sumsX[i] = 0;
    uncertain_sumsY[i] = 0;*/
  }
  int out = 0;
  int out2 = 0;
  const int p1Size = points1.size();
  for(int i1 = 0; i1 < p1Size; i1++){
    for(int i2 = i1+1; i2 < p1Size; i2++){
      Point2f p11 = points1[i1];
      Point2f p12 = points1[i2];
      Point2f p21 = points2[i1];
      Point2f p22 = points2[i2];
      if(abs(
	euclideanDistance(p11, p12) 
	- euclideanDistance(p21, p22)
	    ) > LENGTHDIFF_THRESHOLD){
	if(out < 100){
	  //cout << "sorted out " << i1 << "-" << i2 << endl;
	}
	out++;
	continue;
      }
      double rot = getRotation(p11, p12, p21, p22);
      int index = round(rot/rotationStep);
      /*if(abs(euclideanDistance(p11, p12)) < UNCERTAINITY_THRESHOL0D){
	if(out2 < 100){
	  //cout << "small val!! : " << abs(euclideanDistance(p11, p12)) << "  " << i1 << "-" << i2 << endl;
	}
	uncertain_countsMid[index]++;
	uncertain_sumsMid[index] += rot;
	Point2f b;
	rotate(p21, b, -rot);
	uncertain_sumsMidX[index] += p11.x - b.x;
	uncertain_sumsMidY[index] += p11.y - b.y;
	out2++;
	continue;
      }*/
      countsMid[index]++;
      sumsMid[index] += rot;
      
      Point2f b;
      rotate(p21, b, -rot);
      sumsMidX1[index] += p11.x - b.x;
      sumsMidY1[index] += p11.y - b.y;
      
      rotate(p22, b, -rot);	
      sumsMidX2[index] += p12.x - b.x;
      sumsMidY2[index] += p12.y - b.y;
    }
  }
  int bestAvgCount = 0;
  int bestIndex;
  for(int i = MIN; i < MAX; i++){
    int count = countsMid[i];
    if(count > bestAvgCount){
      bestIndex = i;
      bestAvgCount = count;
    }
   // if(count > 0)
      //cout << i*rotationStep << " = " << toDegree(i*rotationStep) << " => " << count << " times" << endl;
  }
  //cout << "min: " << MIN << " max: " << MAX << " MID: " << MID << endl; 
  cout << "in: " << p1Size << " | " << "bestAvgCount: " << bestAvgCount << " | out: " << out << " | out2: " 
  << out2 << " | possible: " << nCr(p1Size, 2) << " | ratio best/(possible-out): " 
  << double(bestAvgCount)/double((nCr(p1Size, 2)-out)) << endl;
  if(bestAvgCount <= 2)
    return false;
  delta.theta = sumsMid[bestIndex]/bestAvgCount;
  cout << "most likely rotation is: " << delta.theta  << " = " << toDegree(delta.theta) << "°" << endl;
  /*int j = 0;
  double xAvg1 = 0;
  double xAvg2 = 0;
  double yAvg1 = 0;
  double yAvg2 = 0;
  for(int i1 = 0; i1 < p1Size; i1++){
    for(int i2 = i1+1; i2 < p1Size; i2++){
      Point2f p11 = points1[i1];
      Point2f p12 = points1[i2];
      Point2f p21 = points2[i1];
      Point2f p22 = points2[i2];
      if(abs(
	euclideanDistance(p11, p12) 
	- euclideanDistance(p21, p22)
	    ) > LENGTHDIFF_THRESHOLD){
	continue;
      }
      double rot = getRotation(p11, p12, p21, p22);
      if((rot < delta.theta + rotationStep/2) && (rot > delta.theta - rotationStep/2)){
	j++;
	Point2f b;
	rotate(p21, b, -delta.theta);
	xAvg1 += p11.x - b.x;
	yAvg1 += p11.y - b.y;
	
	rotate(p22, b, -delta.theta);	
	xAvg2 += p12.x - b.x;
	yAvg2 += p12.y - b.y;
      }
    }
  }
  
  cout << "j " << j << " dX " << (xAvg1+xAvg2)/2/j << " dy " << (yAvg1+yAvg2)/2/j << endl;*/
  delta.x =  (sumsMidX1[bestIndex] + sumsMidX2[bestIndex])/bestAvgCount/2;
  delta.y =  (sumsMidY1[bestIndex] + sumsMidY2[bestIndex])/bestAvgCount/2;
  /*cout << "xdiff " <<  xDiff << endl;
  cout << "ydiff " <<  yDiff  << endl;
  cout << "xdiff2 " <<  delta2.x << endl;
  cout << "ydiff2 " <<  delta2.y  << endl;*/
  return true;
}

