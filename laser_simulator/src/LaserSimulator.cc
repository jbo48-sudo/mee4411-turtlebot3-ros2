/*
  Copyright (C) 2013 Nathan Michael

  This file is part of laser_simulator, a laser simulator for ROS.

  mesh80211s is free software: you can redistribute it and/or modify it under
  the terms of the GNU General Public License as published by the Free Software
  Foundation, either version 3 of the License, or (at your option) any later
  version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
  details.

  You should have received a copy of the GNU General Public License along with
  this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "laser_simulator/LaserSimulator.h"

#include <cmath>
#include <time.h>
#include <armadillo>
#include <boost/tuple/tuple.hpp>

LaserSimulator::LaserSimulator()
: initialized(false), occupied_threshold(0), noise_sd(0.0)
{
  return;
}

LaserSimulator::~LaserSimulator()
{
  for (std::map < std::string, model_t * > ::iterator i = models.begin();
    i != models.end(); ++i)
  {
    delete i->second;
  }

  return;
}

void LaserSimulator::SetPose(const geometry_msgs::msg::Pose & pose_)
{
  this->pose = pose_;
}

void LaserSimulator::SetFrameID(const std::string & frame_id_)
{
  this->frame_id = frame_id_;
}

void LaserSimulator::SetLaserOffset(const geometry_msgs::msg::Pose & offset_)
{
  this->offset = offset_;
}

void LaserSimulator::LoadOccupancyGrid(
  const nav_msgs::msg::OccupancyGrid & map,
  double depth)
{
  this->triangles.clear();

  double x = map.info.origin.position.x;
  double y = map.info.origin.position.y;
  double z = map.info.origin.position.z;

  double delta = 0.5 * map.info.resolution;
  double resolution = map.info.resolution;

#ifdef DEBUG
  std::cout << map.data.size() << std::endl;
  std::cout << x << ", " << y << ", " << z << ", " << std::endl;
  std::cout << map.info.resolution << std::endl;
#endif

  // Note we do not clear the triangles here as we may wish
  // to load multiple occupancy grids
  std::vector < signed char > ::const_iterator k = map.data.begin();
  for (unsigned int i = 0; i < map.info.height; i++) {
    for (unsigned int j = 0; j < map.info.width; j++, ++k) {
      if (*k > this->occupied_threshold) {
        double center_x = x + j * resolution + delta;
        double center_y = y + i * resolution + delta;

        Point face_1_1(center_x - delta, center_y + delta, z);
        Point face_1_2(center_x - delta, center_y - delta, z);
        Point face_1_3(center_x - delta, center_y - delta, z + depth);
        Point face_1_4(center_x - delta, center_y + delta, z + depth);

        this->triangles.push_back(Triangle(face_1_1, face_1_3, face_1_4));
        this->triangles.push_back(Triangle(face_1_1, face_1_2, face_1_3));

        Point face_2_1(center_x - delta, center_y - delta, z);
        Point face_2_2(center_x + delta, center_y - delta, z);
        Point face_2_3(center_x + delta, center_y - delta, z + depth);
        Point face_2_4(center_x - delta, center_y - delta, z + depth);

        this->triangles.push_back(Triangle(face_2_1, face_2_3, face_2_4));
        this->triangles.push_back(Triangle(face_2_1, face_2_2, face_2_3));

        Point face_3_1(center_x + delta, center_y - delta, z);
        Point face_3_2(center_x + delta, center_y + delta, z);
        Point face_3_3(center_x + delta, center_y + delta, z + depth);
        Point face_3_4(center_x + delta, center_y - delta, z + depth);

        this->triangles.push_back(Triangle(face_3_1, face_3_3, face_3_4));
        this->triangles.push_back(Triangle(face_3_1, face_3_2, face_3_3));

        Point face_4_1(center_x + delta, center_y + delta, z);
        Point face_4_2(center_x - delta, center_y + delta, z);
        Point face_4_3(center_x - delta, center_y + delta, z + depth);
        Point face_4_4(center_x + delta, center_y + delta, z + depth);

        this->triangles.push_back(Triangle(face_4_1, face_4_3, face_4_4));
        this->triangles.push_back(Triangle(face_4_1, face_4_2, face_4_3));
      }
    }
  }
  this->tree.rebuild(this->triangles.begin(), this->triangles.end());

#ifdef DEBUG
  std::cout << this->tree.bbox() << std::endl;
  std::cout << this->tree.size() << std::endl;
#endif

  return;
}

visualization_msgs::msg::Marker LaserSimulator::GetMarker()
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = this->frame_id;
  marker.ns = "laser_simulator";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 0.5; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  for (std::list < Triangle > ::iterator it = this->triangles.begin();
    it != this->triangles.end(); ++it)
  {
    geometry_msgs::msg::Point p;

    p.x = (*it).vertex(0).x();
    p.y = (*it).vertex(0).y();
    p.z = (*it).vertex(0).z();
    marker.points.push_back(p);

    p.x = (*it).vertex(1).x();
    p.y = (*it).vertex(1).y();
    p.z = (*it).vertex(1).z();
    marker.points.push_back(p);

    p.x = (*it).vertex(2).x();
    p.y = (*it).vertex(2).y();
    p.z = (*it).vertex(2).z();
    marker.points.push_back(p);
  }

  return marker;
}

int param_exit(const std::string & error)
{
  RCLCPP_WARN(
    rclcpp::get_logger("rclcpp"), "%s: Unable to find parameter: %s",
    "laser_simulator", error.c_str());

  return -1;
}

int LaserSimulator::LoadLaserModel(std::shared_ptr < rclcpp::Node > n)
{
  n->declare_parameter("min_angle_deg", rclcpp::ParameterType::PARAMETER_DOUBLE);
  n->declare_parameter("max_angle_deg", rclcpp::ParameterType::PARAMETER_DOUBLE);
  n->declare_parameter("rate", rclcpp::ParameterType::PARAMETER_DOUBLE);
  n->declare_parameter("scan_count", rclcpp::ParameterType::PARAMETER_INTEGER);
  n->declare_parameter("min_range", rclcpp::ParameterType::PARAMETER_DOUBLE);
  n->declare_parameter("max_range", rclcpp::ParameterType::PARAMETER_DOUBLE);

  if (!n->has_parameter("min_angle_deg")) {return param_exit("min_angle_deg");}
  if (!n->has_parameter("max_angle_deg")) {return param_exit("max_angle_deg");}
  if (!n->has_parameter("rate")) {return param_exit("rate");}
  if (!n->has_parameter("scan_count")) {return param_exit("scan_count");}
  if (!n->has_parameter("min_range")) {return param_exit("min_range");}
  if (!n->has_parameter("max_range")) {return param_exit("max_range");}

  double minimum_angle_deg = n->get_parameter("min_angle_deg").as_double();
  double maximum_angle_deg = n->get_parameter("max_angle_deg").as_double();
  double scan_rate = n->get_parameter("rate").as_double();
  this->number_of_ranges = n->get_parameter("scan_count").as_int();
  assert(this->number_of_ranges > 0);

  this->scan_time = 1.0 / scan_rate;
  this->time_increment = scan_time / double(this->number_of_ranges);
  this->minimum_angle = minimum_angle_deg * M_PI / 180;
  this->maximum_angle = maximum_angle_deg * M_PI / 180;

  this->angle_increment = (maximum_angle - minimum_angle) / double(this->number_of_ranges - 1);

  this->minimum_range = n->get_parameter("min_range").as_double();
  this->maximum_range = n->get_parameter("max_range").as_double();

  for (int i = 0; i < this->number_of_ranges; i++) {
    this->scan_points.push_back(
      std::make_pair < float, float > (
        this->maximum_range * cos(this->minimum_angle + i * this->angle_increment),
        this->maximum_range * sin(this->minimum_angle + i * this->angle_increment)
      )
    );
  }

  this->initialized = true;

  return 0;
}

/*
int LaserSimulator::LoadDynamicModels(const rclcpp::Node& n)
{
  std::map<std::string, boost::tuple<double, double, double> > types;

  if (n.has_parameter("types"))
    {
      XmlRpc::XmlRpcValue t = n.get_parameter("types");
      if (t.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "types parameter needs to be an array");
          return -1;
        }
      else
        {
          if (t.size() == 0)
            {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "no values in types array");
              return -1;
            }
          else
            {
              for (int i = 0; i < t.size(); i++)
                {
                  if (t[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
                    {
                      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "types entry %d is not a structure, stopping", i);
                      return -1;
                    }
                  if (!t[i].hasMember("id"))
                    {
                      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "types entry %d has no 'id' member", i);
                      return -1;
                    }
                  if (!t[i].hasMember("xdim"))
                    {
                      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "models entry %d has no 'xdim' member", i);
                      return -1;
                    }
                  if (!t[i].hasMember("ydim"))
                    {
                      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "types entry %d has no 'ydim' member", i);
                      return -1;
                    }
                  if (!t[i].hasMember("zdim"))
                    {
                      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "types entry %d has no 'zdim' member", i);
                      return -1;
                    }

                  std::string id = (std::string)t[i]["id"];
                  double xdim = (double)t[i]["xdim"];
                  double ydim = (double)t[i]["ydim"];
                  double zdim = (double)t[i]["zdim"];

                  if (types.count(id) > 0)
                    {
                      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "type %s is not unique", id.c_str());
                      return -1;
                    }

                  //types.insert(std::make_pair<std::string,
                  //             boost::tuple<double, double, double> >
                  //             (id, boost::make_tuple(xdim, ydim, zdim)));
                  types.insert({id, boost::make_tuple(xdim, ydim, zdim)});
                }
            }
        }
    }
  else
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "no types parameter provided");
      return -1;
    }

  if (n.has_parameter("models"))
    {
      XmlRpc::XmlRpcValue m = n.get_parameter("models");
      if (m.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "models parameter needs to be an array");
          return -1;
        }
      else
        {
          if (m.size() == 0)
            {
              RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "no values in models array");
              return -1;
            }
          else
            {
              for (int i = 0; i < m.size(); i++)
                {
                  if (m[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
                    {
                      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "models entry %d is not a structure, stopping", i);
                      return -1;
                    }
                  if (!m[i].hasMember("id"))
                    {
                      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "models entry %d has no 'id' member", i);
                      return -1;
                    }
                  if (!m[i].hasMember("type"))
                    {
                      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "models entry %d has no 'type' member", i);
                      return -1;
                    }

                  std::string id = (std::string)m[i]["id"];
                  std::string type = (std::string)m[i]["type"];

                  if (models.count(id) > 0)
                    {
                      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "model %s is not unique", id.c_str());
                      return -1;
                    }

                  if (types.count(type) == 0)
                    {
                      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "model %s has unknown type %s",
                                id.c_str(), type.c_str());
                      return -1;
                    }

                  models[id] = new Model(types[type].get<0>(),
                                         types[type].get<1>(),
                                         types[type].get<2>());
                }
            }
        }
    }
  else
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "no models parameter provided");
      return -1;
    }

  return 0;
}
*/


void LaserSimulator::GetScan(std::vector < float > & ranges)
{
  srand(time(NULL)); // initialize random number generator

  arma::colvec s(3);
  arma::colvec t(3);
  t(0) = pose.position.x + offset.position.x;
  t(1) = pose.position.y + offset.position.y;
  t(2) = pose.position.z + offset.position.z;

  double a = pose.orientation.w;
  double b = pose.orientation.x;
  double c = pose.orientation.y;
  double d = pose.orientation.z;

  arma::mat R(3, 3);
  R(0, 0) = a * a + b * b - c * c - d * d;
  R(0, 1) = 2.0 * b * c - 2.0 * a * d;
  R(0, 2) = 2.0 * b * d + 2.0 * a * c;
  R(1, 0) = 2.0 * b * c + 2.0 * a * d;
  R(1, 1) = a * a - b * b + c * c - d * d;
  R(1, 2) = 2.0 * c * d - 2.0 * a * b;
  R(2, 0) = 2.0 * b * d - 2.0 * a * c;
  R(2, 1) = 2.0 * c * d + 2.0 * a * b;
  R(2, 2) = a * a - b * b - c * c + d * d;

  arma::colvec p(3);
  p(2) = 0;

  arma::colvec pi(3); // intersection point
  Point p1(t(0), t(1), t(2)); // sensor origin
  unsigned int k = 0; // ray count
  for (auto i = scan_points.begin(); i != scan_points.end(); ++i, k++) {
    // Get ray direction
    p(0) = (*i).first;
    p(1) = (*i).second;
    s = R * p;
    Vector v(s(0), s(1), s(2));
    Ray ray_query(p1, v);

    // Initialize range
    double range = maximum_range;

    // Find first intersection along the ray
    Ray_intersection intersection = tree.first_intersection(ray_query);
    if (intersection) {
      if (boost::get < Point > (&(intersection->first))) {
        const Point * pint = boost::get < Point > (&(intersection->first));
        pi(0) = pint->x();
        pi(1) = pint->y();
        pi(2) = pint->z();
        range = arma::norm(pi - t, 2);
      }
    }

    // Set the output range
    if (noise_sd > 0.0 && range < maximum_range) {
      // Add noise if desired
      double U = rand() / double(RAND_MAX);
      double V = rand() / double(RAND_MAX);
      // Box-Muller method
      double sn_var = sqrt(-2.0 * log(U)) * cos(2.0 * M_PI * V);
      ranges[k] = range + noise_sd * sn_var;
    } else {
      ranges[k] = range;
    }
  }

  // std::list<Object_and_primitive_id> intersections;
  // for (std::list<std::pair<float, float> >::iterator i = scan_points.begin();
  //      i != scan_points.end(); ++i, k++)
  //   {
  //     p(0) = (*i).first;
  //     p(1) = (*i).second;

  //     s = R*p + t;

  //     Point p2(s(0), s(1), s(2));

  //     Segment segment_query(p1, p2);
  //     double max_range = maximum_range;

  //     if (tree.do_intersect(segment_query))
  //       {
  //         intersections.clear();
  //         tree.all_intersections(segment_query,
  //                                std::back_inserter(intersections));

  //         for (std::list<Object_and_primitive_id>::iterator j = intersections.begin();
  //              j != intersections.end(); ++j)
  //           {
  //             Point point;
  //             if (!CGAL::assign(point, (*j).first))
  //               continue;
  //             pi(0) = point.x();
  //             pi(1) = point.y();
  //             pi(2) = point.z();
  //             double range = arma::norm(pi - t, 2);
  //             if (range < max_range)
  //               max_range = range;
  //           }
  //       }

  //     if (!dynamic_tree.empty())
  //       {
  //         if (dynamic_tree.do_intersect(segment_query))
  //           {
  //             intersections.clear();
  //             dynamic_tree.all_intersections(segment_query,
  //                                            std::back_inserter(intersections));

  //             for (std::list<Object_and_primitive_id>::iterator j =
  //                    intersections.begin(); j != intersections.end(); ++j)
  //               {
  //                 Point point;
  //                 if (!CGAL::assign(point, (*j).first))
  //                   continue;
  //                 pi(0) = point.x();
  //                 pi(1) = point.y();
  //                 pi(2) = point.z();
  //                 double range = arma::norm(pi - t, 2);
  //                 if (range < max_range)
  //                   max_range = range;
  //               }
  //           }
  //       }


  //     if (noise_sd > 0.0 && max_range < maximum_range)
  //       {
  //         double U = rand() / double(RAND_MAX);
  //         double V = rand() / double(RAND_MAX);
  //         // Box-Muller method
  //         double sn_var = sqrt (-2.0*log(U)) * cos(2.0*M_PI*V);
  //         ranges[k] = max_range + noise_sd * sn_var;
  //       }
  //     else
  //       {
  //         ranges[k] = max_range;
  //       }
  //   }

  return;
}

void LaserSimulator::UpdatePoseArray(const laser_simulator::msg::PoseStampedNamedArray & pose_array)
{
  std::list < Triangle > dynamic_triangles;

  for (unsigned int i = 0; i < pose_array.poses.size(); ++i) {
    if (pose_array.poses[i].child_frame_id == frame_id) {
      continue;
    }

    if (models.count(pose_array.poses[i].child_frame_id) == 0) {
      printf(
        "%s: Unknown model in odom_array message (%s) - skipping\n",
        "laser_simulator", pose_array.poses[i].child_frame_id.c_str());
      continue;
    }

    double x_delta = 0.5 * models[pose_array.poses[i].child_frame_id]->xdim;
    double y_delta = 0.5 * models[pose_array.poses[i].child_frame_id]->ydim;
    double z_delta = 0.5 * models[pose_array.poses[i].child_frame_id]->zdim;

    double center_x = pose_array.poses[i].pose.position.x;
    double center_y = pose_array.poses[i].pose.position.y;
    double center_z = pose_array.poses[i].pose.position.z;

    double dxm = center_x - x_delta;
    double dxp = center_x + x_delta;

    double dym = center_y - y_delta;
    double dyp = center_y + y_delta;

    double dzm = center_z - z_delta;
    double dzp = center_z + z_delta;

    Point face_1_1(dxm, dyp, dzm);
    Point face_1_2(dxm, dym, dzm);
    Point face_1_3(dxm, dym, dzp);
    Point face_1_4(dxm, dyp, dzp);

    dynamic_triangles.push_back(Triangle(face_1_1, face_1_3, face_1_4));
    dynamic_triangles.push_back(Triangle(face_1_1, face_1_2, face_1_3));

    Point face_2_1(dxm, dym, dzm);
    Point face_2_2(dxp, dym, dzm);
    Point face_2_3(dxp, dym, dzp);
    Point face_2_4(dxm, dym, dzp);

    dynamic_triangles.push_back(Triangle(face_2_1, face_2_3, face_2_4));
    dynamic_triangles.push_back(Triangle(face_2_1, face_2_2, face_2_3));

    Point face_3_1(dxp, dym, dzm);
    Point face_3_2(dxp, dyp, dzm);
    Point face_3_3(dxp, dyp, dzp);
    Point face_3_4(dxp, dym, dzp);

    dynamic_triangles.push_back(Triangle(face_3_1, face_3_3, face_3_4));
    dynamic_triangles.push_back(Triangle(face_3_1, face_3_2, face_3_3));

    Point face_4_1(dxp, dyp, dzm);
    Point face_4_2(dxm, dyp, dzm);
    Point face_4_3(dxm, dyp, dzp);
    Point face_4_4(dxp, dyp, dzp);

    dynamic_triangles.push_back(Triangle(face_4_1, face_4_3, face_4_4));
    dynamic_triangles.push_back(Triangle(face_4_1, face_4_2, face_4_3));
  }

  dynamic_tree.clear();
  if (dynamic_triangles.size() > 0) {
    dynamic_tree.rebuild(dynamic_triangles.begin(), dynamic_triangles.end());
  }
}
