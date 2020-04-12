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
static void getRelevantCarIDMatrix(const sFusionEgoData &EgoData, const sFusionObjectList& ObjList,
                                   int RelevantCarID[3][3])
{
  double t_threshold_front = 5;
  double t_threshold_rear  = -5;
  double t_threshold_side  = 0.5;
  double d_threshold_front = 10;
  double d_threshold_rear  = -10;
  double d_threshold_side  = 5;

  for (int ii = 0; ii < 3; ii++)
    for (int jj = 0; jj < 3; jj++)
      RelevantCarID[ii][jj] = -1; // set to non active

  for (int obj = 0; obj < ObjList.ObjectData.size(); obj++)
  {
    int lane = round(ObjList.ObjectData[obj].LaneID);
    if (lane == round(EgoData.LaneID)) // on the same lane
    {
      if (ObjList.ObjectData[obj].s > 0 && (ObjList.ObjectData[obj].s < d_threshold_front ||
          ObjList.ObjectData[obj].s < t_threshold_front*std::max(EgoData.v,ObjList.ObjectData[obj].v)))
      {
        if (RelevantCarID[1][2] < 0 || ObjList.ObjectData[obj].s < ObjList.ObjectData[RelevantCarID[1][2]].s)
          RelevantCarID[1][2] = obj;
      }
      else if (ObjList.ObjectData[obj].s < 0 && (ObjList.ObjectData[obj].s > d_threshold_rear ||
          ObjList.ObjectData[obj].s > t_threshold_rear*std::max(EgoData.v,ObjList.ObjectData[obj].v)))
      {
        if (RelevantCarID[1][0] < 0 || ObjList.ObjectData[obj].s < ObjList.ObjectData[RelevantCarID[1][2]].s)
          RelevantCarID[1][0] = obj;
      }
    }
    else if (lane == round(EgoData.LaneID) + 1) // on lane on the left side
    {

    }
    else if (lane == round(EgoData.LaneID) - 1) // on lane on the right side
    {

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

  FusionData.ObjectList.clear();
  FusionData.ObjectList.push_back({0}); // insert a dummy first object
  FusionData.ObjectList[0].t = 0.0; // first element is t = 0;
  for (int obj = 0; obj < ObjData.size(); obj++)
  {
    FusionData.ObjectList[0].ObjectData.push_back({0});
    FusionData.ObjectList[0].ObjectData[obj].id = ObjData[obj].id;
    FusionData.ObjectList[0].ObjectData[obj].s = ObjData[obj].s;
    FusionData.ObjectList[0].ObjectData[obj].d = ObjData[obj].d;
    FusionData.ObjectList[0].ObjectData[obj].v = ObjData[obj].v;
    FusionData.ObjectList[0].ObjectData[obj].LaneID = associateLane(ObjData[obj].d);
  }

  getRelevantCarIDMatrix(FusionData.egoData,FusionData.ObjectList[0],FusionData.ObjectList[0].RelevantCarID);

  // predict into the future
  int iter_pred = floor(__SF_T_Pred/__SF_dT_Pred) + 1;
  for (int iter = 1; iter < iter_pred; iter ++)
  {
    FusionData.ObjectList.push_back({0}); // insert a dummy first object
    FusionData.ObjectList[iter].t = FusionData.ObjectList[iter-1].t  + __SF_dT_Pred; // first element is t = 0;
    for (int obj = 0; obj < ObjData.size(); obj++)
    {
      FusionData.ObjectList[iter].ObjectData.push_back({0});
      FusionData.ObjectList[iter].ObjectData[obj].id = FusionData.ObjectList[iter-1].ObjectData[obj].id;
      FusionData.ObjectList[iter].ObjectData[obj].s =
          FusionData.ObjectList[iter-1].ObjectData[obj].s
          + __SF_dT_Pred*FusionData.ObjectList[iter-1].ObjectData[obj].v;
      FusionData.ObjectList[iter].ObjectData[obj].d = FusionData.ObjectList[iter-1].ObjectData[obj].d;
      FusionData.ObjectList[iter].ObjectData[obj].v = FusionData.ObjectList[iter-1].ObjectData[obj].v;
      FusionData.ObjectList[iter].ObjectData[obj].LaneID = associateLane(ObjData[obj].d);
    }
    // Unncessarcy without ego vehicle predicition:
    // getRelevantCarIDMatrix(FusionData.egoData,FusionData.ObjectList[iter],FusionData.ObjectList[iter].RelevantCarID);
  }


  // ok finished.. sending it out
  DataOut = FusionData;
  return;
}
