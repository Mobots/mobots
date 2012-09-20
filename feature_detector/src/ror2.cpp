#include <opencv2/core/core.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

#include "ror.h"
#include "FeaturesMatcher.h"
#include "profile.h"

using namespace std;
using namespace cv;

#define rotationStep 0.004d
#define N int(2*M_PI/rotationStep) + 3
#define MID ((N)/2)


const int UNCERTAINITY_THRESHOL0D = 3; //if <
const int LENGTHDIFF_THRESHOLD = 2;    //if >

static uint counts[N];
static double sumsTheta[N];
static double sumsX[N];
static double sumsY[N];

static unsigned int* const countsMid = &counts[MID];
static double* const sumsMid = &sumsTheta[MID];
static double* const sumsMidX = &sumsX[MID];
static double* const sumsMidY = &sumsY[MID];


static uint uncertain_counts[N];
static double uncertain_sumsTheta[N];
static double uncertain_sumsX[N];
static double uncertain_sumsY[N];

static unsigned int* const uncertain_countsMid = &uncertain_counts[MID];
static double* const uncertain_sumsMid = &uncertain_sumsTheta[MID];
static double* const uncertain_sumsMidX = &uncertain_sumsX[MID];
static double* const uncertain_sumsMidY = &uncertain_sumsY[MID];



static double euclideanDistance(const Point2f& p1, const Point2f& p2){
  double yDiff = double(p2.y)-double(p1.y);
  double xDiff = double(p2.x)-double(p1.x);
  return sqrt(xDiff*xDiff + yDiff*yDiff);
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
bool ror2(const vector<Point2f>& points1, const vector<Point2f>& points2, vector<Point2f>& out1, vector<Point2f>& out2){
  //additional ideas:
  //first simple matching: p1[i],p1[i+1] <--> p2[i],p2[i+1]
  // 		     and: p1[i],p1[i+2] <--> p2[i],p2[i+2]
  //if one rotation get's more than 60% it's ok.
  
  /*for(int i = 0; i < N; i++){
    counts[i] = 0;
    sumsTheta[i] = 0;
    sumsX[i] = 0;
    sumsY[i] = 0;
    uncertain_counts[i] = 0;
    uncertain_sumsTheta[i] = 0;
    uncertain_sumsX[i] = 0;
    uncertain_sumsY[i] = 0;
  }*/
  const int p1Size = points1.size();
  int min = 4;
  for(int i1 = 0; i1 < p1Size; i1++){
    int curr = 0;
    for(int i2 = 0; i2 < p1Size; i2++){
      Point2f p11 = points1[i1];
      Point2f p12 = points1[i2];
      Point2f p21 = points2[i1];
      Point2f p22 = points2[i2];
      if(abs(
	euclideanDistance(p11, p12) 
	- euclideanDistance(p21, p22)
	    ) > LENGTHDIFF_THRESHOLD){
	  //cout << "sorted out " << i1 << "-" << i2 << endl;
	continue;
      }
      curr++;
      if(curr >= min){
	out1.push_back(p11);
	out2.push_back(p21);
	break;
      }
      /*double rot = getRotation(p11, p12, p21, p22);
      int index = round(rot/rotationStep);
      if(index < -MID+2 || index > MID-2)
	cout << "index error: " << index << " mid is " << MID << ", N is " << N << endl;
      if(abs(euclideanDistance(p11, p12)) < UNCERTAINITY_THRESHOL0D){
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
      }
      countsMid[index]++;
      sumsMid[index] += rot;
      Point2f b;
      rotate(p21, b, -rot);
      sumsMidX[index] += p11.x - b.x;
      sumsMidY[index] += p11.y - b.y;*/
    }
  }
  /*int bestAvgCount = 0;
  int bestIndex;
  for(int i = 0; i < N; i++){
    int count = counts[i];
    if(count > bestAvgCount){
      bestIndex = i;
      bestAvgCount = count;
    }
    //cout << i*rotationStep << " = " << toDegree(i*rotationStep) << " => " << count << " times" << endl;
  }*/
  cout << "ror2 " << "in: " << p1Size << " | " << " | out: " << out1.size() << "|" << out2.size() << endl;
  /*if(bestAvgCount < 2)
    return false;
  delta.theta = sumsTheta[bestIndex]/bestAvgCount;
  cout << "most likely rotation is: " << delta.theta  << " = " << toDegree(delta.theta) << "°" << endl;
  Point2f b;
  rotate(points2[bestIndex], b, -delta.theta);
  delta.x =  sumsX[bestIndex]/bestAvgCount;
  delta.y =  sumsY[bestIndex]/bestAvgCount;
  /*cout << "xdiff " <<  xDiff << endl;
  cout << "ydiff " <<  yDiff  << endl;
  cout << "xdiff2 " <<  delta2.x << endl;
  cout << "ydiff2 " <<  delta2.y  << endl;*/
  return true;
}

