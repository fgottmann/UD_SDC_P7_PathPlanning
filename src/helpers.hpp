#ifndef HELPERS_H
#define HELPERS_H

#include <fstream>
#include <iostream>
#include <vector>
#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

double fmod_usr(double in,double div)
{
  if (in > 0)
    return std::fmod(in,div);
  else
  {
    int multpl = -in/div + 1;
    return in + multpl*div;
  }
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double yaw,
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y,
                         const vector<double> &maps_yaw,
                         const vector<double> &maps_s, const double &s_max) {
  int next_wp = NextWaypoint(x,y, yaw, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double dist_total = sqrt(n_x*n_x + n_y*n_y);
  double inv_dist = 1.0/dist_total;
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)*inv_dist*inv_dist;
  proj_norm = std::max(0.0,std::min(1.0,proj_norm));
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = (x_y*n_x - x_x*n_y)*inv_dist;
  double frenet_yaw = maps_yaw[prev_wp] + proj_norm*(maps_yaw[next_wp]-maps_yaw[prev_wp]);
  double internal = maps_s[next_wp]-maps_s[prev_wp];
  double frenet_s = maps_s[prev_wp] + proj_norm*fmod_usr(maps_s[next_wp]-maps_s[prev_wp],s_max);

  return {frenet_s,frenet_d,frenet_yaw};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

vector<double> interpolate_linear(const vector<double> &x_b, const vector<double> &y_b,
                                  const vector<double> &x)
{
  vector<double> ret;

  for (int ii = 0; ii < x.size();ii++)
  {
    int ind0 = 0;
    while((ind0 < x_b.size() - 3) && x[ii] >= x_b[ind0])
      ind0++;

    double factor = (x[ii] - x_b[ind0])/std::max(1e-10,x_b[ind0 + 1] - x_b[ind0]);
    factor = std::min(1.0,std::max(0.0,factor));

    ret.push_back(y_b[ind0] + factor*(y_b[ind0+1]-y_b[ind0]));
  }

  return ret;
}

double interpolate_linear(const vector<double> &x_b, const vector<double> &y_b, double x)
{
  int ind0 = 1;
  while((ind0 < x_b.size() - 1) && x > x_b[ind0])
    ind0++;

  double factor = (x - x_b[ind0-1])/std::max(1e-1,x_b[ind0] - x_b[ind0-1]);
  factor = std::min(1.0,std::max(0.0,factor));

  return y_b[ind0-1] + factor*(y_b[ind0]-y_b[ind0-1]);
}

#endif  // HELPERS_H
