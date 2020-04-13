#include "sensorfusion.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>

using std::vector;


SensorFusion::SensorFusion()
{

}

SensorFusion::~SensorFusion()
{

}

// get lane id 0 -> most right; 2 = most_left
static double associateLane(double d)
{
  return 2.0 + (d + 2.0)/4;

}

// use old traj data to give a prediction
static void getRelevantCarIDMatrix(const sFusionEgoData &EgoData, sFusionObjectList& ObjList)
{
  double t_max_f = 3;
  double t_max_r = 3;
  double t_max_f_sec = 3;
  double t_max_r_sec = 3;
  double t_max_s  = 0.2;
  double t_max_s_r  = 1.0;
  double t_diff_f =  5;
  double t_diff_r  = 5;
  double t_diff_s  = 0.5;
  double d_f = 30;
  double d_r  = 30;
  double d_s  = 10;
  double d_s_r  = 4;
  double s_max = 200;

  for (int ii = 0; ii < ObjList.RelevantCarID.size(); ii++)
    ObjList.RelevantCarID[ii] = -1; // set to non active

  for (int obj = 0; obj < ObjList.ObjectData.size(); obj++)
  {
    int lane = round(ObjList.ObjectData[obj].LaneID[0]);
    double v_max = std::max(EgoData.v,ObjList.ObjectData[obj].v[0]);
    double v_diff = EgoData.v - ObjList.ObjectData[obj].v[0];
    double s = ObjList.ObjectData[obj].s[0];

    if (lane == round(EgoData.LaneID)) // on the same lane
    {
      if (s >= 0 && (s <= +d_f || s <=  t_diff_f*std::max(0.0,v_diff) || s <= +t_max_f*v_max) && s <= s_max)
      {
        if (ObjList.RelevantCarID[3] < 0 || ObjList.ObjectData[obj].s[0] < ObjList.ObjectData[std::max(0,ObjList.RelevantCarID[3])].s[0])
          ObjList.RelevantCarID[3] = obj;
      }
      else if (s <= 0 && (s >= -d_r || s >=  t_diff_r*std::min(0.0,v_diff) || s >= -t_max_r*v_max)&& s >= -s_max)
      {
        if (ObjList.RelevantCarID[7] < 0 || ObjList.ObjectData[obj].s[0] < ObjList.ObjectData[std::max(0,ObjList.RelevantCarID[7])].s[0])
          ObjList.RelevantCarID[7] = obj;
      }
    }
    else if (lane == round(EgoData.LaneID) + 1) // on lane on the left side
    {
      if ((s <= 0 && (s >= -d_s -d_s_r || s >=  t_diff_r*v_diff-t_max_s_r*v_max-d_s_r) && s >= -s_max) ||
          (s >= 0 && (s <= +d_s || s <=  t_diff_s*std::max(0.0,v_diff)  || s <= +t_max_s*v_max) && s <= s_max))
      {
        if (ObjList.RelevantCarID[1] < 0 || fabs(ObjList.ObjectData[obj].s[0]) < fabs(ObjList.ObjectData[std::max(0,ObjList.RelevantCarID[1])].s[0]))
          ObjList.RelevantCarID[1] = obj;
      }
      else if (s >= 0 && (s <= +d_f || s <=  t_diff_f*std::max(0.0,v_diff)  || s <= t_max_f_sec*v_max)&& s <= s_max)
      {
        if (ObjList.RelevantCarID[2] < 0 || ObjList.ObjectData[obj].s[0] < ObjList.ObjectData[std::max(0,ObjList.RelevantCarID[2])].s[0])
          ObjList.RelevantCarID[2] = obj;
      }
      else if (s <= 0 && (s >= -d_r || s >=  t_diff_r*std::min(0.0,v_diff) || s >= -t_max_r_sec*v_max)&& s >= -s_max)
      {
        if (ObjList.RelevantCarID[0] < 0 || ObjList.ObjectData[obj].s[0] < ObjList.ObjectData[std::max(0,ObjList.RelevantCarID[0])].s[0])
          ObjList.RelevantCarID[0] = obj;
      }
    }
    else if (lane == round(EgoData.LaneID) - 1) // on lane on the right side
    {

      if ((s <= 0 && (s >= -d_s-d_s_r || s >=  t_diff_r*v_diff-t_max_s_r*v_max-d_s_r) && s >= -s_max) ||
          (s >= 0 && (s <= +d_s || s <=  t_diff_s*std::max(0.0,v_diff)  || s <=  t_max_s*v_max) && s <= s_max))
      {
        if (ObjList.RelevantCarID[5] < 0 || fabs(ObjList.ObjectData[obj].s[0]) < fabs(ObjList.ObjectData[std::max(0,ObjList.RelevantCarID[5])].s[0]))
          ObjList.RelevantCarID[5] = obj;
      }
      else if (s >= 0 && (s <= +d_f || s <=  t_diff_f*std::max(0.0,v_diff) || s <= t_max_f_sec*v_max) && s <= s_max)
      {
        if (ObjList.RelevantCarID[4] < 0 || ObjList.ObjectData[obj].s[0] < ObjList.ObjectData[std::max(0,ObjList.RelevantCarID[4])].s[0])
          ObjList.RelevantCarID[4] = obj;
      }
      else if (s <= 0 && (s >= -d_r || s >=  t_diff_r*std::min(0.0,v_diff) || s >= -t_max_r_sec*v_max) && s >= -s_max)
      {
        if (ObjList.RelevantCarID[6] < 0 || ObjList.ObjectData[obj].s[0] < ObjList.ObjectData[std::max(0,ObjList.RelevantCarID[6])].s[0])
          ObjList.RelevantCarID[6] = obj;
      }
    }
  }



  return;
}


void SensorFusion::calcOutput(const vector<sObjData> &ObjData, const sEgoData &EgoData,
                              sFusionData& DataOut)
{
  FusionData.egoData.s = 0; // center of calculation
  FusionData.egoData.d = EgoData.d;
  FusionData.egoData.e_yaw = EgoData.e_yaw;
  FusionData.egoData.v = EgoData.v;
  FusionData.egoData.LaneID = associateLane(EgoData.d);

  FusionData.ObjectList.t.clear();
  FusionData.ObjectList.ObjectData.clear();

  FusionData.ObjectList.t.push_back(0.0); // first element is t = 0;

  const int iter_pred = floor(__SF_T_Pred/__SF_dT_Pred) + 1;
  for (int obj = 0; obj < ObjData.size(); obj++)
  {
    FusionData.ObjectList.ObjectData.push_back({0});
    FusionData.ObjectList.ObjectData[obj].id = (ObjData[obj].id);
    FusionData.ObjectList.ObjectData[obj].s.push_back(ObjData[obj].s);
    FusionData.ObjectList.ObjectData[obj].d.push_back(ObjData[obj].d);
    FusionData.ObjectList.ObjectData[obj].v.push_back(ObjData[obj].v);
    FusionData.ObjectList.ObjectData[obj].LaneID.push_back(associateLane(ObjData[obj].d));
  }

  // Prediction
  for (int iter = 1; iter < iter_pred; iter ++)
  {
    FusionData.ObjectList.t.push_back(FusionData.ObjectList.t[iter-1]  + __SF_dT_Pred); // first element is t = 0;
    for (int obj = 0; obj < FusionData.ObjectList.ObjectData.size(); obj++)
    {
      FusionData.ObjectList.ObjectData[obj].s.push_back(
          FusionData.ObjectList.ObjectData[obj].s[iter-1]
                                                  + __SF_dT_Pred*FusionData.ObjectList.ObjectData[obj].v[iter-1]);
      FusionData.ObjectList.ObjectData[obj].d.push_back(FusionData.ObjectList.ObjectData[obj].d[iter-1]);
      FusionData.ObjectList.ObjectData[obj].v.push_back(FusionData.ObjectList.ObjectData[obj].v[iter-1]);
      FusionData.ObjectList.ObjectData[obj].LaneID.push_back(associateLane(FusionData.ObjectList.ObjectData[obj].d[iter-1]));
    }
  }

  getRelevantCarIDMatrix(FusionData.egoData,FusionData.ObjectList);

  // ok finished.. sending it out
  DataOut = FusionData;
  return;
}
