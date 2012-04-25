/**
 * ror - Rejection of outliers by rotation
 * schöne idee, scheint aber nicht wirklich besser zu sein als die Methode ausm Cookbook.
 */

#pragma once

void rorAlternative(vector<Point2f>&, vector<Point2f>&);

void ror(vector<Point2f>& points1in, vector<Point2f>& points2in, 
	 vector<Point2f>& points1out, vector<Point2f>& points2out);
