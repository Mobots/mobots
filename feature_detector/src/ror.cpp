#include <opencv2/core/core.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

#include "ror.h"
#include "FeaturesMatcher.h"
#include "profile.h"

using namespace std;
using namespace cv;

#define rotationStep 0.001f
#define N int(2*M_PI/rotationStep + 3)
#define MID ((N)/2) -1

static unsigned int counts[N];
static double sumsTheta[N];
static double sumsX[N];
static double sumsY[N];

static unsigned int* const countsMid = &counts[MID];
static double* const sumsMid = &sumsTheta[MID];
static double* const sumsMidX = &sumsX[MID];
static double* const sumsMidY = &sumsY[MID];

Delta delta2;

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

/**
* FTW
* Inspired by ror
* maybe use a simpler method than computing all possible combinations when n is very high,
* complexity is at least nC2
*/
bool rorAlternative(const vector<Point2f>& points1, const vector<Point2f>& points2, Delta& delta){
  //additional ideas:
  //first simple matching: p1[i],p1[i+1] <--> p2[i],p2[i+1]
  // and: p1[i],p1[i+2] <--> p2[i],p2[i+2]
  //if one rotation get's more than 60% it's ok.
  vector<vector<double> > xOffsets(N);
  vector<vector<double> > yOffsets(N);
 for(int i = 0; i < N; i++){
   counts[i] = 0;
   sumsTheta[i] = 0;
   sumsX[i] = 0;
   sumsY[i] = 0;
 }
  int out = 0;
  const int p1Size = points1.size();
  vector<int> bestIndex1(N, 0);
  vector<int> bestIndex2(N, 0);
  int* best1Mid = &bestIndex1[MID];
  int* best2Mid = &bestIndex2[MID];
  vector<double>* xOffsetsMid = &xOffsets[MID];
  vector<double>* yOffsetsMid = &yOffsets[MID];
  for(int i1 = 0; i1 < p1Size; i1++){
    for(int i2 = i1+1; i2 < p1Size; i2++){
      Point2f p11 = points1[i1];
      Point2f p12 = points1[i2];
      Point2f p21 = points2[i1];
      Point2f p22 = points2[i2];
      if(abs(
euclideanDistance(p11, p12)
- euclideanDistance(p21, p22)
) > 10){
if(out < 100){
cout << "sorted out " << i1 << "-" << i2 << endl;
out++;
}
continue;
      }
      /*if(abs(euclideanDistance(p11, p12)) < 10){
cout << "small val!! : " << abs(euclideanDistance(p11, p12)) << endl;
continue;
}*/
      double rot = getRotation(p11, p12, p21, p22);
      /*float origR = rot;
if(rot > M_PI)
rot = 2*M_PI - rot;
else if(rot < -M_PI)
rot = -2*M_PI - rot;*/
      int index = round(rot/rotationStep);
      best1Mid[index] = i1;
      best2Mid[index] = i2;
      countsMid[index]++;
      sumsMid[index] += rot;
      Point2f b;
      rotate(p21, b, -rot);
      sumsMidX[index] += p11.x - b.x;
      sumsMidY[index] += p11.y - b.y;
      xOffsetsMid[index].push_back(p11.x - b.x);
      yOffsetsMid[index].push_back(p11.y - b.y);
    }
  }
  int bestAvgCount = 0;
  int bestIndex;
  for(int i = 0; i < N; i++){
    int count = counts[i];
    if(count > bestAvgCount){
      bestIndex = i;
      bestAvgCount = count;
    }
    //cout << i*rotationStep << " = " << toDegree(i*rotationStep) << " => " << count << " times" << endl;
  }
  cout << "size " << xOffsets[bestIndex].size() << " should be " << bestAvgCount << endl;
  for(int i = 0; i < xOffsets[bestIndex].size(); i++){
    cout << "xOff: " << xOffsets[bestIndex][i] << " - yOff: " << yOffsets[bestIndex][i] << endl;
  }
  if(bestAvgCount == 0)
    return false;
  float bestAvg = sumsTheta[bestIndex]/bestAvgCount;
  cout << "most likely rotation is: " << bestAvg << " = " << toDegree(bestAvg) << "°" << endl;
  double rot = getRotation(points1[bestIndex1[bestIndex]], points1[bestIndex2[bestIndex]],
points2[bestIndex1[bestIndex]], points2[bestIndex2[bestIndex]]);
  cout << "most likely rotation2 is: " << rot << " = " << toDegree(rot) << "°" << endl;
  delta2.theta = rot;
  Point2f b;
  rotate(points2[bestIndex], b, -rot);
  delta2.x = points1[bestIndex].x - b.x;
  delta2.y = points1[bestIndex].y - b.y;
  delta.theta = bestAvg;
  float xDiff = sumsX[bestIndex]/bestAvgCount;
  float yDiff = sumsY[bestIndex]/bestAvgCount;
  delta.x = xDiff;
  delta.y = yDiff;
  cout << "xdiff " << xDiff << endl;
  cout << "ydiff " << yDiff << endl;
  cout << "xdiff2 " << delta2.x << endl;
  cout << "ydiff2 " << delta2.y << endl;
  return true;
}