#include<iostream>
#include<stdio.h>
#define _USE_MATH_DEFINES
#include<math.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/adapted/c_array.hpp>

typedef boost::geometry::model::d2::point_xy<double> point;
typedef boost::geometry::model::polygon<point> polygon;

using namespace boost::geometry;


class Geometry {
  
public: 
Geometry(double l, double b);
double checkPicture(double x, double y, double theta);

private:
polygon calcPol(double x, double y,double alpha); 
double l,b,t,r,a;
  
  
  
};