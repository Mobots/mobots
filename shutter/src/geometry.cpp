#include "geometry.h"
#include <math.h>
#include "mobots_common/constants.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/adapted/c_array.hpp>

typedef boost::geometry::model::d2::point_xy<double> point;
typedef boost::geometry::model::polygon<point> polygon;

using namespace boost::geometry;

polygon referencePoly;

#if 0
polygon calcPol(double x, double y,double alpha, double tb, double rb); 
#endif
polygon calcPol(double, double, double); 

/*
int main()
{

   // timeval start, end;
     double dif;
     int i;
     Geometry g(4,3);
       for (i=0; i < 1000000; i++)
       {
           gettimeofday(&start, 0);
           double a = Schneid(0,0,(i % 3600) / 100);
           gettimeofday(&end, 0);
           dif += end.tv_usec - start.tv_usec;
           if (i % 100000 == 0)
           {
                 std::cout << dif / i << std::endl;
           }
       } 
      double a1;
      //gettimeofday(&start, 0);
      a1= g.checkPicture(0.7,1,120.00076);
      //gettimeofday(&end, 0);
    std::cout << "Überschneidung: " << a1 << std::endl;
    return 0;
} */

Geometry::Geometry(double l, double b)
{
  this->l = l;
  this->b = b;
  t = atan2(l,b);
  r = sqrt((l*l/4) + (b*b/4));
  //a = l*b;
	
	a = mobots_common::constants::image_width_in_meters*mobots_common::constants::image_height_in_meters;
	double c[5][2];
	const double halfWidth = mobots_common::constants::image_width_in_meters/2;
	const double halfHeight = mobots_common::constants::image_height_in_meters/2;
	c[0][0] = -halfWidth;
	c[0][1] = halfHeight;

	c[1][0] = halfWidth;
	c[1][1] = halfHeight;
	
	c[2][0] = halfWidth;
	c[2][1] = -halfHeight;
	
	c[3][0] = -halfWidth;
	c[3][1] = -halfHeight;
	
	c[4][0] = c[0][0];
	c[4][1] = c[0][1];
	
	append(referencePoly, c);
	correct(referencePoly);
}

double Geometry::checkPicture(double x, double y, double theta)
{
    //polygon pol1 = calcPol(0,0,0,t,r);
    //polygon pol2 = calcPol(x,y,theta,t,r);
    polygon poly = calcPol(x, y, theta);
    std::deque<polygon> pol_list;
    intersection(referencePoly , poly, pol_list);
    double a1 =0;
    if (!pol_list.empty()) {
	
        a1 = area(pol_list.at(0));
    }
    return a1 / a;
}

/**
 * input values in m and radian or gtfo
 */
polygon calcPol(double dx, double dy, double angle)
{
		double d;
    int i;
    double c[5][2];
		double cost = cos(angle);
		double sint = sin(angle);
		const double halfWidth = mobots_common::constants::image_width_in_meters/2;
		const double halfHeight = mobots_common::constants::image_height_in_meters/2;
		c[0][0] = -halfWidth;
		c[0][1] = halfHeight;
	
		c[1][0] = halfWidth;
		c[1][1] = halfHeight;
		
		c[2][0] = halfWidth;
		c[2][1] = -halfHeight;
		
		c[3][0] = -halfWidth;
		c[3][1] = -halfHeight;
		
		for(int i = 0; i < 4; i++){
			double x = c[i][0];
			double y = c[i][1];
			c[i][0] = x*cost - sint*y + dx;
			c[i][1] = x*sint + cost*y + dy;
		}
		
		c[4][0] = c[0][0];
		c[4][1] = c[0][1];
		
    polygon pol;
    append(pol,c);
		correct(pol);
    return pol;
}


#if 0
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
#endif
