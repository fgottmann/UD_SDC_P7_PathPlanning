#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include "external/Eigen-3.3/Eigen/Core"
#include "external/Eigen-3.3/Eigen/QR"
#include "external/Eigen-3.3/Eigen/Dense"
#include "helpers.hpp"
#include "json.hpp"
#include "sensorfusion.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

using Eigen::VectorXd;
using Eigen::MatrixXd;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  vector<double> map_waypoints_ddx;
  vector<double> map_waypoints_ddy;
  vector<double> map_waypoints_curv;
  vector<double> map_waypoints_yaw;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  const double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  // get additional data
  for (int ii = 0; ii < map_waypoints_dy.size();ii++)
  {
    int ind_p1 = (ii+1)%map_waypoints_dx.size();
    int ind_m1 = (ii-1)%map_waypoints_dx.size();
    // get averaged second derivatives
    map_waypoints_ddx.push_back((
        (map_waypoints_dx[ii] - map_waypoints_dx[ind_m1])/
        fmod(map_waypoints_s[ii] - map_waypoints_s[ind_m1],max_s)
        +
        (map_waypoints_dx[ind_p1] - map_waypoints_dx[ii])/
        fmod(map_waypoints_s[ind_p1] - map_waypoints_s[ii],max_s))*0.5);
    map_waypoints_ddy.push_back((
        (map_waypoints_dy[ii] - map_waypoints_dy[ind_m1])/
        (map_waypoints_s[ii] - map_waypoints_s[ind_m1],max_s)
        +
        (map_waypoints_dy[ind_p1] - map_waypoints_dy[ii])/
        (map_waypoints_s[ind_p1] - map_waypoints_s[ii]))*0.5);

    // simplified curvature calculation (as sqrt(dx²+dy²) == 1)
    map_waypoints_curv.push_back(
        map_waypoints_dx[ii]*map_waypoints_ddy[ii] - map_waypoints_ddx[ii]*map_waypoints_dy[ii]);
    map_waypoints_yaw.push_back(atan2(map_waypoints_dy[ii],map_waypoints_dx[ii]));

  }

  // init classes
  SensorFusion sensorfusion_ = SensorFusion();

  //std::cout << (map_waypoints_x[0] - map_waypoints_x[map_waypoints_x.size()-1]) << std::endl;
  //std::cout << (map_waypoints_y[0] - map_waypoints_y[map_waypoints_y.size()-1]) << std::endl;
  //std::cout << (map_waypoints_s[map_waypoints_y.size()-1]) << std::endl;


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,
               &map_waypoints_curv,&map_waypoints_yaw,&max_s,
               &sensorfusion_]
               (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                   uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      static bool initialized = false;
      static double last_s_base = 0;
      static vector<double> last_s_vals;
      static vector<double> last_s_dot_vals;
      static vector<double> last_s_ddot_vals;
      static vector<double> last_d_vals;
      static vector<double> last_d_dot_vals;
      static vector<double> last_d_ddot_vals;

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          sEgoData EgoData;
          EgoData.x   = j[1]["x"];
          EgoData.y   = j[1]["y"];
          EgoData.yaw = deg2rad(double(j[1]["yaw"]));
          EgoData.v   = j[1]["speed"];

          // Recalculate with own data to ensure consistency
          vector<double> frenet_data = getFrenet(EgoData.x, EgoData.y, EgoData.yaw,
                                                 map_waypoints_x, map_waypoints_y,
                                                 map_waypoints_yaw, map_waypoints_s, max_s);

          EgoData.s       = frenet_data[0];
          EgoData.d       = frenet_data[1];
          EgoData.e_yaw   = frenet_data[2];

          //std::cout << "s: " << EgoData.s << ", d: " << EgoData.d << ", e_yaw: " << EgoData.e_yaw << std::endl;

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];


          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          std::vector<sObjData> sensor_fusion;
          for (int ii = 0; ii <j[1]["sensor_fusion"].size();ii++)
          {
            sObjData Obj;
            Obj.id = int(j[1]["sensor_fusion"][ii][0]);
            Obj.d = -double(j[1]["sensor_fusion"][ii][6]); // right of reference is negative here
            Obj.v = sqrt(pow(double(j[1]["sensor_fusion"][ii][3]),2) + pow(double(j[1]["sensor_fusion"][ii][4]),2));
            Obj.s = fmod(double(j[1]["sensor_fusion"][ii][5])- EgoData.s + 0.3*Obj.v,max_s); // norm to vehicle position
            sensor_fusion.push_back(Obj);
          }


          // resync with last path after 1s
          if (previous_path_x.size() > 0)
          {
            vector<double> frenet_data = getFrenet(previous_path_x[std::min(24,int(previous_path_x.size()-1))], previous_path_y[std::min(24,int(previous_path_x.size()-1))], EgoData.yaw,
                                                   map_waypoints_x, map_waypoints_y,
                                                   map_waypoints_yaw, map_waypoints_s, max_s);
            EgoData.x = previous_path_x[std::min(50,int(previous_path_x.size()-1))];
            EgoData.y = previous_path_y[std::min(50,int(previous_path_x.size()-1))];
            EgoData.s = frenet_data[0];
            EgoData.d = frenet_data[1];
          }

          sFusionData FusionData;
          sensorfusion_.calcOutput(sensor_fusion, EgoData, FusionData);


          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          // get a polynomial from the lane data
          double lookahead = std::max(30.0,std::min(50.0,EgoData.v)*4.0);
          int breakpoint_nb = 10;
          double delta_BP = (lookahead)/double(breakpoint_nb-1);

          vector<double> map_s;
          vector<double> map_x;
          vector<double> map_y;
          vector<double> map_x_filt;
          vector<double> map_d_filt;
          vector<double> map_y_filt;

          if (previous_path_x.size() > 1)
          {
            for (int ii = std::max(0,std::min(22,int(previous_path_x.size()-3))); ii <= std::min(24,int(previous_path_x.size()-1));ii++)
            {
              map_x.push_back(previous_path_x[ii]);
              map_y.push_back(previous_path_y[ii]);
              vector<double> frenet_data = getFrenet(previous_path_x[ii], previous_path_y[ii], EgoData.yaw,
                                                     map_waypoints_x, map_waypoints_y,
                                                     map_waypoints_yaw, map_waypoints_s, max_s);
              map_s.push_back(fmod(frenet_data[0] - EgoData.s,max_s));
            }
          }
          double x_ref,y_ref,phi_ref;
          if (previous_path_x.size() > 1)
          {
            x_ref = map_x[map_x.size()-1];
            y_ref = map_y[map_y.size()-1];
            phi_ref = atan2(map_y[map_s.size()-1]-map_y[map_s.size()-2],map_x[map_s.size()-1]-map_x[map_s.size()-2]);
          }
          else
          {
            x_ref = EgoData.x;
            y_ref = EgoData.y;
            phi_ref = EgoData.yaw;
            map_x.push_back(x_ref - cos(phi_ref));
            map_y.push_back(y_ref - sin(phi_ref));
            map_x.push_back(x_ref);
            map_y.push_back(y_ref);
            map_s.push_back(-1);
            map_s.push_back(0);
          }

          for (int ii = 1; ii < breakpoint_nb;ii++)
          {
            double BP = double(ii)*delta_BP;
            map_s.push_back(BP);
            vector<double> pos = getXY(fmod(BP + EgoData.s,max_s),6,map_waypoints_s,map_waypoints_x,map_waypoints_y);
            map_x.push_back(pos[0]);
            map_y.push_back(pos[1]);
          }

          MatrixXd LHS = MatrixXd::Zero(map_s.size(),4);
          VectorXd RHS_x = VectorXd::Zero(map_s.size());
          VectorXd RHS_y = VectorXd::Zero(map_s.size());
          for (int ii = 0; ii < map_s.size();ii++)
          {
            double BP = map_s[ii];
            LHS(ii,0) = BP*BP;
            LHS(ii,1) = BP*BP*BP;
            LHS(ii,2) = BP*BP*BP*BP;
            LHS(ii,3) = BP*BP*BP*BP*BP;
            RHS_x(ii) = map_x[ii]-x_ref - cos(phi_ref)*BP;
            RHS_y(ii) = map_y[ii]-y_ref - sin(phi_ref)*BP;
          }

          VectorXd Coefs_x = LHS.colPivHouseholderQr().solve(RHS_x);
          VectorXd Coefs_y = LHS.colPivHouseholderQr().solve(RHS_y);
          //std::cout <<  Coefs_x.transpose() << std::endl;
          //std::cout <<  Coefs_y.transpose() << std::endl;

          vector<double> map_s_interp;
          double BP = 0;
          while(BP < lookahead)
          {
            map_s_interp.push_back(BP);
            map_x_filt.push_back(x_ref + BP*(cos(phi_ref) + BP*(Coefs_x(0)  + BP*(Coefs_x(1) + BP*(Coefs_x(2) + BP*Coefs_x(3))))));
            map_y_filt.push_back(y_ref + BP*(sin(phi_ref) + BP*(Coefs_y(0)  + BP*(Coefs_y(1) + BP*(Coefs_y(2) + BP*Coefs_y(3))))));
            vector<double> frenet_data = getFrenet(map_x_filt[map_x_filt.size()-1], map_y_filt[map_y_filt.size()-1], EgoData.yaw ,
                                                   map_waypoints_x, map_waypoints_y,
                                                   map_waypoints_yaw, map_waypoints_s, max_s);
            map_d_filt.push_back(frenet_data[1]);
            BP += 0.2;

          }

          if (EgoData.s > 6900)
            double a = 0;
          else
            double b =0;

          vector<double> next_s_vals;
          vector<double> next_s_dot_vals;
          vector<double> next_s_ddot_vals;
          vector<double> next_d_vals;


          double s_0,s_dot_0,s_ddot_0, d_0;

          if (false == initialized)
          {
            next_s_vals.push_back(0); // calculating everything relative to s=0;
            next_s_dot_vals.push_back(EgoData.v);
            next_s_ddot_vals.push_back(0);
            next_d_vals.push_back(EgoData.d);
          }
          else
          {
            next_s_vals.push_back(0); // calculating everything relative to s=0;
            next_s_dot_vals.push_back(interpolate_linear(last_s_vals, last_s_dot_vals,EgoData.s - last_s_base));
            next_s_ddot_vals.push_back(interpolate_linear(last_s_vals, last_s_ddot_vals,EgoData.s - last_s_base));
            next_d_vals.push_back(interpolate_linear(last_s_vals, last_d_vals,EgoData.s - last_s_base));
          }

          double t=0;
          const double t_max = 2;
          double dt_out = 0.02;
          double dt_incr = 0.02;
          double v_ref = 49.5*1.6/3.6;
          int incr = 0;
          while(t < t_max)
          {
            t+= 0.02;
            incr += 1;

            next_d_vals.push_back(next_d_vals[next_d_vals.size()-1]);
            int incr_obj = std::max(int(0),std::min(int(incr),int(FusionData.ObjectList.size()-1)));
            int relevant_ob_id = FusionData.ObjectList[0].RelevantCarID[1][2];
            double j = 0;
            if (relevant_ob_id > 0 && FusionData.ObjectList[0].ObjectData[relevant_ob_id].v < v_ref)
            {
              double dist_front = FusionData.ObjectList[incr].ObjectData[relevant_ob_id].s;
              double v_front = FusionData.ObjectList[incr].ObjectData[relevant_ob_id].v;
              dist_front -= 1.0*std::max(v_front,next_s_dot_vals[next_s_dot_vals.size()-1]);

              j = -2.0*next_s_ddot_vals[next_s_dot_vals.size()-1]
                                        -1.0*(next_s_dot_vals[next_s_dot_vals.size()-1] -
                                            std::min(v_ref,v_front - 0.5*(next_s_vals[next_s_vals.size()-1] - dist_front)));

            }
            else
            {
              j = -2.0*next_s_ddot_vals[next_s_dot_vals.size()-1]
                                        -0.5*(next_s_dot_vals[next_s_dot_vals.size()-1] - v_ref);
            }

            j = std::max(-4.0,std::min(4.0,j));
            next_s_ddot_vals.push_back(std::min(2.0,std::max(-5.0,next_s_ddot_vals[next_s_ddot_vals.size()-1] + dt_incr*j)));
            next_s_dot_vals.push_back(std::min(v_ref,std::max(0.0,next_s_dot_vals[next_s_dot_vals.size()-1] + dt_incr*next_s_ddot_vals[next_s_ddot_vals.size()-1])));
            next_s_vals.push_back(next_s_vals[next_s_vals.size()-1] + dt_incr*next_s_dot_vals[next_s_dot_vals.size()-1]);

          }
int relevant_ob_id = std::max(0,int(FusionData.ObjectList[0].RelevantCarID[1][2]));
          std::cout << "RelObjID-front: " << FusionData.ObjectList[0].RelevantCarID[1][2] << "; vel: "<< FusionData.ObjectList[0].ObjectData[relevant_ob_id].v << std::endl;


          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // add some old points for latency compensation -> 50*0.02 = 1.0s
          if (previous_path_x.size() > 0)
          {
            for (int ii = 0; ii < std::min(24,int(previous_path_x.size()-1));ii++)
            {
              next_x_vals.push_back(previous_path_x[ii]);
              next_y_vals.push_back(previous_path_y[ii]);
            }

            frenet_data = getFrenet(next_x_vals[next_x_vals.size()-1], next_y_vals[next_y_vals.size()-1], EgoData.yaw ,
                                    map_waypoints_x, map_waypoints_y,
                                    map_waypoints_yaw, map_waypoints_s, max_s);
          }

          for (int ii = 0; next_x_vals.size() < 50; ii++)
          {
            double x = interpolate_linear(map_s_interp, map_x_filt,next_s_vals[ii]);
            double y = interpolate_linear(map_s_interp, map_y_filt,next_s_vals[ii]);

            next_x_vals.push_back(x);
            next_y_vals.push_back(y);
          }

          double max_v = 0;
          int max_v_ind = -1;
          for (int ii = 0; ii < next_x_vals.size()-1;ii++)
          {
            double v_act = sqrt((next_x_vals[ii] - next_x_vals[ii+1])*(next_x_vals[ii] - next_x_vals[ii+1]) +
                     (next_y_vals[ii] - next_y_vals[ii+1])*(next_y_vals[ii] - next_y_vals[ii+1]))/0.02;
            if (fabs(v_act) > fabs(max_v))
            {
              max_v = v_act;
              max_v_ind = ii;
            }
          }

          double max_a = 0;
          double max_v1 = 0;
          double max_v2 = 0;
          int max_a_ind = -1;
          for (int ii = 0; ii < next_x_vals.size()-2;ii++)
          {
            double v_act_1 = sqrt((next_x_vals[ii] - next_x_vals[ii+1])*(next_x_vals[ii] - next_x_vals[ii+1]) +
                     (next_y_vals[ii] - next_y_vals[ii+1])*(next_y_vals[ii] - next_y_vals[ii+1]))/0.02;
            double v_act_2 = sqrt((next_x_vals[ii+1] - next_x_vals[ii+2])*(next_x_vals[ii+1] - next_x_vals[ii+2]) +
                     (next_y_vals[ii+1] - next_y_vals[ii+2])*(next_y_vals[ii+1] - next_y_vals[ii+2]))/0.02;
            double a_act = (v_act_1 - v_act_2)/0.02;

            if (fabs(a_act) > fabs(max_a))
            {
              max_a = a_act;
              max_a_ind = ii;
              max_v1 = v_act_1;
              max_v2 = v_act_2;
            }
          }

          //std::cout << "Old: " << std::min(49,int(previous_path_x.size()-1)) << "; max_a: " << max_a << "; max_v1: " << max_v1 << "; max_v2: " << max_v2 << "; max_a_ind: " << \
              max_a_ind << "; max_v: " << max_v << "; max_v_ind: " << max_v_ind << std::endl;


          //std::cout << std::endl;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          last_s_base = EgoData.s;
          last_s_vals = next_s_vals;
          last_s_dot_vals =  next_s_dot_vals;
          last_s_ddot_vals = next_s_ddot_vals;
          last_d_vals = next_d_vals;
          initialized = true;

        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
      char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
