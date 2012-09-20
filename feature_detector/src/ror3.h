#include <opencv2/core/core.hpp>

#include "FeaturesMatcher.h"

/**
 * ror - Rejection of outliers by rotation
 * schöne idee, scheint aber nicht wirklich besser zu sein als die Methode ausm Cookbook.
 */

#pragma once

using namespace cv;
using namespace std;

bool rorAlternative(const vector<Point2f>& points1, const vector<Point2f>& points2, Delta& delta);

/*void ror(vector<Point2f>& points1in, vector<Point2f>& points2in, 
	 vector<Point2f>& points1out, vector<Point2f>& points2out);*/
