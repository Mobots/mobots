#include <opencv2/core/core.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

#include "ror.h"
#include "combination.hpp"
#include "FeaturesMatcher.h"
#include "profile.h"

using namespace std;
using namespace cv;

#define rotationStep 0.001f
#define N 2*int(M_PI/rotationStep) +1

/*static unsigned int counts[N];
static float sumsTheta[N];
static float sumsX[N];
static float sumsY[N];

static unsigned int* const countsMid = &counts[N/2];
static float* const sumsMid = &sumsTheta[N/2];
static float* const sumsMidX = &sumsX[N/2];
static float* const sumsMidY = &sumsY[N/2];*/

inline double toDegree(double rad){
  return rad * 180 / M_PI;
}

float euclideanDistance(const Point2f& p1, const Point2f& p2){
  if((p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y) < 0)
    cout << "MEHEEHEHEHH" << endl;
  return sqrt((p2.x - p1.x)*(p2.x - p1.x) + (p2.y - p1.y)*(p2.y - p1.y));
}

//stackoverflow
unsigned int nCr(unsigned int n, unsigned int k){
    if (k > n) return 0;
    if (k * 2 > n) k = n-k;
    if (k == 0) return 1;
    unsigned int result = n;
    for(unsigned int i = 2; i <= k; ++i) {
        result *= (n-i+1);
        result /= i;
    }
    return result;
}

inline float getHeading(const Point2f& p1, const Point2f& p2){
  return atan2(p2.y-p1.y, p2.x-p1.x);
}

inline float getRotation(const Point2f* p1, const Point2f* p2){
  float heading1 = getHeading(p1[0], p1[1]);
  float heading2 = getHeading(p2[0], p2[1]);
  float diff = heading2-heading1;
  return diff;
}

inline float getRotation(const Point2f& p11, const Point2f& p12, const Point2f& p21, const Point2f& p22){
  float heading1 = getHeading(p11, p12);
  float heading2 = getHeading(p21, p22);
  float diff = heading2-heading1;
  return diff;
}

inline void rotate(const Point2f& in, Point2f& out, float angle){
    float cosa = cos(angle);
    float sina = sin(angle);
    float xOrig = in.x;
    float yOrig = in.y;
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
  //first simple matching: p1[i],p1[i+1] <--> p2[i],p2[i+1]
  // 		     and: p1[i],p1[i+2] <--> p2[i],p2[i+2]
  //if one rotation get's more than 60% it's ok.
  /*int counts[] = new int[2*M_PI/threshold +1]; //2° steps
  float sums[] = new float[2*M_PI/threshold +1];
  int count = 0;
  for(int i = 0; i < points1.size(); i+=2){
    float rot = calcRotation(&points1[i], &points2[i]);
    int index = rot/threshold;
    counts[index]++;
    sums[index] += rot;
    count++;
  }
  for(int i = 0; i < 2*M_PI/threshold +1; i++){
    sums[i] /= counts[i];
    cout << i*threshold << " = " << toDegree(i*threshold) << " => " << counts[i] << " times" << endl;
  }*/
  
   unsigned int counts[N];
 float sumsTheta[N] = {0};
 float sumsX[N] = {0};
 float sumsY[N] = {0};
   unsigned int* const countsMid = &counts[N/2];
 float* const sumsMid = &sumsTheta[N/2];
 float* const sumsMidX = &sumsX[N/2];
 float* const sumsMidY = &sumsY[N/2];
  memset(&counts, 0, N*sizeof(int));
  //memset(&sumsTheta, 0.0, N*sizeof(float));
  //memset(&sumsX, 0, N*sizeof(float));
  //memset(&sumsY, 0, N*sizeof(float));
  const int p1Size = points1.size();
  vector<int> indices(p1Size);
  for(int i = 0; i < p1Size; i++){
    indices[i] = i;
  }
  for(int i1 = 0; i1 < p1Size; i1++){
    for(int i2 = i1; i2 < p1Size; i2++){
      float rot = getRotation(points1[i1], points1[i2], points2[i1], points2[i2]);
      cout << rot << "=" << toDegree(rot) << "°" << endl;
      float origR = rot;
      if(rot > M_PI)
	rot = 2*M_PI - rot;
      else if(rot < -M_PI)
	rot = -2*M_PI - rot;
      if(abs(
	euclideanDistance(points1[i1], points1[i2]) 
	- euclideanDistance(points2[i1], points2[i2])
	    ) < 3){
	    /*cout << i << "+" << i2 << " " << points1[i].x << "x " << points1[i].y << "y <->" << points1[i2].x << "x " << points1[i2].y << "y" 
	    << " :: dist " << euclideanDistance(points1[i], points1[i2]) << endl 
	    
	    << i << "+" << i2 << " "  << points2[i].x << "x " << points2[i].y << "y <->" << points2[i2].x << "x " << points2[i2].y << "y" 
	    << " :: dist " << euclideanDistance(points1[i], points1[i2]) << endl
	    << " ||| " << abs(euclideanDistance(points1[i], points1[i2]) - euclideanDistance(points1[i], points1[i2])) << endl; 
	/*cout << euclideanDistance(points1[indices[0]], points2[indices[0]]) << "  "
      << euclideanDistance(points1[indices[0]], points1[indices[1]]) << "  "
      << euclideanDistance(points2[indices[0]], points2[indices[1]]) << "  "
      << indices[0] << "|" << indices[1] << "-> " << rot << " = " << toDegree(rot) << "°" << endl;*/
      }else{
	cout << "sorted out " << i1 << "-" << i2 << endl;
	continue;
      }
      int index = rot/rotationStep;
      countsMid[index]++;
      sumsMid[index] += rot;
      Point2f b;
      rotate(points2[i1], b, -origR);
      sumsMidX[index] += points1.at(i1).x - b.x;
      sumsMidY[index] += points1.at(i1).y - b.y;
    }
  }
  /*while(boost::next_combination(indices.begin(), indices.begin()+2, indices.end())){     
    float rot = getRotation(&points1[indices[0]], &points2[indices[0]]);
    float origR = rot;
    if(rot > M_PI)
      rot = 2*M_PI - rot;
    else if(rot < -M_PI)
      rot = -2*M_PI - rot;
    int i = indices[0];
    int i2 = indices[1];
    if(abs(
      euclideanDistance(points1.at(indices[0]), points1.at(indices[1])) 
      - euclideanDistance(points2.at(indices[0]), points2.at(indices[1]))
	  ) > 3){
      cout << "sorted out " << i << "-" << i2 << endl;
      continue;
    }
    int index = rot/rotationStep;
    countsMid[index]++;
    sumsMid[index] += rot;
    Point2f b;
    rotate(points2[indices[0]], b, -origR);
    sumsMidX[index] += points1.at(indices[0]).x - b.x;
    sumsMidY[index] += points1.at(indices[0]).y - b.y;
  }*/
  float bestAvg;
  int bestAvgCount = 0;
  int bestIndex;
  for(int i = -N/2; i < N/2; i++){
    int count = countsMid[i];
    if(count > bestAvgCount){
      bestIndex = i;
      bestAvgCount = count;
    }
    //cout << i*rotationStep << " = " << toDegree(i*rotationStep) << " => " << count << " times" << endl;
  }
  bestAvg = sumsMid[bestIndex]/bestAvgCount;
  cout << "x avg " << sumsMidX[bestIndex]/bestAvgCount << endl;
  cout << "y avg " << sumsMidY[bestIndex]/bestAvgCount << endl;
  cout << "most likely rotation is: " << bestAvg  << " = " << toDegree(bestAvg) << "°" << endl;
  delta.theta = bestAvg;
  float xDiff =  sumsMidX[bestIndex]/bestAvgCount;
  float yDiff =  sumsMidY[bestIndex]/bestAvgCount;
  delta.x = xDiff;
  delta.y = yDiff;
  cout << "xdiff " <<  xDiff << endl;
  cout << "ydiff " <<  yDiff  << endl;
  return true;
}

