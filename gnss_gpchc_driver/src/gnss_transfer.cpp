/*
高斯解算示例程序。
*/
/************.h*************/
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <fstream> //read write file
#include <stdio.h>

#define REQ 6378137.0
#define RPO 6356752.0


#define Rad(x) ( (x) * M_PI / 180.0 )
#define Deg(x) ( (x) * 180.0 / M_PI )
#define Sq(x) ( (x)*(x) )
#define Max(a,b) ((a)>(b)?(a):(b))
#define Min(a,b) ((a)<(b)?(a):(b))

using namespace Eigen;
using namespace std;

struct gps_linearize_t
{
    double lon0_deg, lat0_deg;
    double radius_ns, radius_ew;
};


/************.cpp*************/

/* Useful links:
   http://www.movable-type.co.uk/scripts/LatLongVincenty.html
   http://en.wikipedia.org/wiki/Earth_radius
*/
// 相当于横纵轴不相同给出的解
// gps_linearize_t base_gps;
void gps_linearize_init(gps_linearize_t *gl, const double ll_deg[2])
{
    gl->lat0_deg = ll_deg[0];
    gl->lon0_deg = ll_deg[1];

    double a = 6378137;  // R_equator//m
    double b = 6356752;  // R_polar//m

    double lat_rad = Rad(ll_deg[0]);

    // this is the best radius approximation, agnostic of direction
    // we don't use this anymore.
    //    gl->radius = a*a*b / (Sq(a*cos(lat_rad)) + Sq(b*sin(lat_rad)));

    // best radius approximation in ns and ew direction.
    gl->radius_ns = Sq(a*b) / pow((Sq(a*cos(lat_rad))) + Sq(b*sin(lat_rad)), 1.5);

    gl->radius_ew = a*a / sqrt(Sq(a*cos(lat_rad)) + Sq(b*sin(lat_rad)));

}
//xy[2] unit:m
int gps_linearize_to_xy(gps_linearize_t *gl, const double ll_deg[2], double xy[2])
{
    double dlat = Rad(ll_deg[0] - gl->lat0_deg);
    double dlon = Rad(ll_deg[1] - gl->lon0_deg);

    xy[0] = sin(dlon) * gl->radius_ew * cos(Rad(gl->lat0_deg));
    xy[1] = sin(dlat) * gl->radius_ns;
    
    return 0;
}

int gps_linearize_to_lat_lon(gps_linearize_t *gl, const double xy[2], double ll_deg[2])
{
    double dlat = asin(xy[1] / gl->radius_ns);
    ll_deg[0] = Deg(dlat) + gl->lat0_deg;

    double dlon = asin(xy[0] / gl->radius_ew / cos(Rad(gl->lat0_deg)));
    ll_deg[1] = Deg(dlon) + gl->lon0_deg;

    return 0;
}

// 测试与高斯正算相差多少. 采取点0 1 5
int main(){

    double ll_deg[2] = {40.00670625, 116.32815636};
    // {40.00688261, 166.32829583};
    double test_xy[2] = {40.00688458, 116.32820744};
    gps_linearize_t* gl= new gps_linearize_t();

    gps_linearize_init(gl, ll_deg);
    double res_xy[2] ;
    gps_linearize_to_xy(gl, test_xy, res_xy);
    cout<<"res "<<res_xy[0]<<" "<<res_xy[1];
    return 0;
}