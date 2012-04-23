#include <opencv2/core/core.hpp>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

#include "combination.hpp"

using namespace std;
using namespace cv;

extern Mat aff;

float rorAlternative(vector<Point2f> points1, vector<Point2f> points2);

static const float rotationStep = 0.01f; // ~0.6°

inline double toDegree(double rad){
  return rad * 180 / M_PI;
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

inline float getHeading(Point2f& p1, Point2f& p2){
  return atan2(p2.y-p1.y, p2.x-p1.x);
}

inline float calcRotation(Point2f* p1, Point2f* p2){
  float heading1 = getHeading(p1[0], p1[1]);
  float heading2 = getHeading(p2[0], p2[1]);
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

inline void rotate(const vector<Point2f>& in, vector<Point2f>& out, float angle){
  out.resize(in.size());
  const int N = in.size();
  for(int i = 0; i < N; i++){
    rotate(in[i], out[i], angle);
  }
}

void ror(vector<Point2f>& points1in, vector<Point2f>& points2in, 
	 vector<Point2f>& points1out, vector<Point2f>& points2out){
  int counts[int(2*M_PI/rotationStep) +1];
  memset(&counts, 0, sizeof(int)*(int(2*M_PI/rotationStep) +1));
  int mostMatches = 0;
  float mostMatchesRotation;
  float mostMatchesHeading;
  const int N = points1in.size();
  for(float rotation = 0; rotation < 2*M_PI; rotation+=rotationStep){
    vector<Point2f> rotated;
    rotate(points2in, rotated, rotation);
    for(int i = 0; i < N; i++){
      float heading = getHeading(points1in[i], rotated[i]);
      if(heading < 0)
	heading = 2*M_PI + heading;
      int index = heading/rotationStep;
      counts[index]++;
    }
    int best = 0;
    for(int i = 0; i < 2*M_PI/rotationStep; i++){
      if(counts[i] > best){
	best = counts[i];
	if(best > mostMatches){
	  mostMatches = counts[i];
	  mostMatchesRotation = rotation;
	  mostMatchesHeading = i*rotationStep;
	}
      }
    }
    memset(&counts, 0, sizeof(int)*(int(2*M_PI/rotationStep) +1));
    //cout << rotation << " = " << toDegree(rotation) << " : " << best << endl;
  }
  vector<Point2f> rotated;
  rotate(points2in, rotated, mostMatchesRotation);
  for(int i = 0; i < N; i++){
    float heading = getHeading(points1in[i], rotated[i]);
     if(heading < 0)
      heading = 2*M_PI + heading;
    if(abs(mostMatchesHeading-abs(heading)) <= 1*rotationStep){
      points1out.push_back(points1in[i]);
      points2out.push_back(points2in[i]);
    }
  }
  cout << "best rotation is" << mostMatchesRotation << " = " << toDegree(mostMatchesRotation) << "°" << endl;
}
/**
 * FTW
 * Inspired by ror
 * maybe use a simpler method than computing all possible combinations when n is very high,
 * complexity is at least nC2
 */
float rorAlternative(vector<Point2f>& points1, vector<Point2f>& points2){
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
  const int N = int(M_PI/rotationStep) +1;
  unsigned int counts[N];
  float sums[N];
  memset(&counts, 0, N*sizeof(int));
  memset(&sums, 0.0, N*sizeof(float));
  int ptrs[N];
  const int p1Size = points1.size();
  vector<int> indices(p1Size);
  for(int i = 0; i < p1Size; i++){
    indices[i] = i;
  }
  while(boost::next_combination(indices.begin(), indices.begin()+2, indices.end())){
    float rot = abs(calcRotation(&points1[indices[0]], &points2[indices[0]]));
    if(rot > M_PI)
      rot = 2*M_PI - rot;
    int index = rot/rotationStep;
    counts[index]++;
    sums[index] += rot;
    ptrs[index] = indices[0];
  }
  float bestAvg;
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
  bestAvg = sums[bestIndex]/bestAvgCount;
  cout << "most likely rotation is: " << bestAvg  << " = " << toDegree(bestAvg) << "°" << endl;
  Point2f p2Trans;
  rotate(points2[ptrs[bestIndex]], p2Trans, bestAvg);
  cout << "xdiff " << p2Trans.x - points1[ptrs[bestIndex]].x << endl;
  cout << "ydiff " << p2Trans.y - points1[ptrs[bestIndex]].y << endl;
  aff = getRotationMatrix2D(Point2f(0,0), -toDegree(bestAvg), 1.0);
  aff.at<double>(0,2) = -(p2Trans.x - points1[ptrs[bestIndex]].x);
  aff.at<double>(1,2) = -(p2Trans.y - points1[ptrs[bestIndex]].y);
  return bestAvg;
}

//old stuff
/*bool isSimilar(Mat& affine1, Mat& affine2){
  return 
  abs(affine1.at<double>(0,0) - affine1.at<double>(0,0)) < 1 &&
  abs(affine1.at<double>(0,1) - affine1.at<double>(0,1)) < 1 &&
  abs(affine1.at<double>(0,2) - affine1.at<double>(0,2)) < 1 &&
  abs(affine1.at<double>(1,0) - affine1.at<double>(1,0)) < 1 &&
  abs(affine1.at<double>(1,1) - affine1.at<double>(1,1)) < 1 &&
  abs(affine1.at<double>(1,2) - affine1.at<double>(1,2)) < 1;
}

Mat* findAffine(vector<Point2f>& points1, vector<Point2f>& points2){
  vector<Mat> mats;
  vector<int> indices(points1.size());
  int size = points1.size();
  for(int i = 0; i < size; i++)
    indices[i] = i;  
  while (boost::next_combination(indices.begin(), indices.begin() + 3, indices.end())){
    Mat affine = getAffineTransform(&points1[indices[0]], &points2[indices[0]]);
    printf("indices %i, %i, %i", indices[0], indices[1], indices[2]);
    for(int i = 0; i < mats.size(); i++){
      if(isSimilar(mats[i], affine)){
	cout << "found similar" << endl;
	cout << "affine1: " << endl << affine << endl;
	cout << "affine2: " << endl << mats[i] << endl;
	return new Mat(affine);	
      }
    }
    mats.push_back(affine);
    //cout << affine << endl;
  }
  return 0;
}*/

