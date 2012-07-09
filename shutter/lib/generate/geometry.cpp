#include "geometry.h"
#include<math.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/adapted/c_array.hpp>

typedef boost::geometry::model::d2::point_xy<double> point;
typedef boost::geometry::model::polygon<point> polygon;

using namespace boost::geometry;


polygon calcPol(double x, double y,double alpha, double tb, double rb); 



Geometry::Geometry(double l, double b)
{
  this->l = l;
  this->b = b;
  t = atan2(l,b);
  r = sqrt((l*l/4) + (b*b/4));
  a = l*b;
  
}

double Geometry::checkPicture(double x, double y, double theta)
{
    polygon pol1 = calcPol(0,0,0,t,r);
    polygon pol2 = calcPol(x,y,theta,t,r);
    std::deque<polygon> pol_list;
    intersection(pol1 , pol2, pol_list);
    double a1 =0;
    if (!pol_list.empty()) {
        a1 = area(pol_list.at(0));
    }
    return a1 / a;
}



polygon calcPol(double x, double y, double alpha, double tb, double rb)
{
    double t=tb;
    double r=rb;
    double a = alpha;  //bogenmass !
   double d;
    int i;
    double c[5][2];
    for (i=0; i<4;i++)
    {
        /*1: oben links, 2:oR,3: uR, 4: uL */
        switch (i)
        {
        case 0:
            d=a + M_PI - t;
            break;
        case 1:
            d=a + t;
            break;
        case 2:
            d=a - t;
            break;
        case 3:
            d=a + M_PI + t;
            break;
        }
        c[i][0]=r * cos(d) + x;		//x-wert zuweisen
        c[i][1]=r * sin(d) + y;		//y-Wert
    }
    c[4][0] = c[0][0];
    c[4][1] = c[0][1];
    polygon pol;
    append(pol,c);
    return pol;
}
