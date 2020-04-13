#ifndef SENSORFUSION_H
#define SENSORFUSION_H

#define __SF_T_Pred 4.0
#define __SF_dT_Pred 0.02

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <array>
#include <cmath>

struct sEgoData
{
 double x;
 double y;
 double yaw;
 double v;
 double s;
 double d;
 double e_yaw;
};

struct sObjData
{
 int id;
 double s;
 double d;
 double v;
};

struct sFusionEgoData
{
 double s;
 double d;
 double e_yaw;
 double v;
 double LaneID;
};

struct sFusionObjectData
{
  int id;
  std::vector<double> s;
  std::vector<double> d;
  std::vector<double> v;
  std::vector<double> LaneID;
};

//
// Relevant Car ID:
// 0     1      2
// 7    ego     3
// 6     5      4
struct sFusionObjectList
{
  std::vector<double> t;
  std::vector<sFusionObjectData> ObjectData;
  std::array<int, 8> RelevantCarID;
};

struct sFusionData
{
  sFusionEgoData egoData;
  sFusionObjectList ObjectList;
  int LaneID[3];
};

class SensorFusion
{
 public:
  SensorFusion();
  ~SensorFusion();

  void getOutput(sFusionData &DataOut);
  void calcOutput(const std::vector<sObjData> &ObjData, const sEgoData &EgoData, sFusionData& DataOut);

 private:
  double LaneID_ego; // is double to return it in percentage
  sFusionData FusionData;


};

#endif
