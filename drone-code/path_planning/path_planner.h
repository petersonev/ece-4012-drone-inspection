#pragma once

#include <math.h>
#include <stdio.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace path {

struct MatlabPoint {
    double heading;
    double dist;
    double image_heading;
};
std::ostream& operator<<(std::ostream& stream, const MatlabPoint& coord);

struct GeoCoord {
    GeoCoord() : lat(0), lon(0){};
    GeoCoord(double lat, double lon) : lat(lat), lon(lon){};
    double lat;  // latitude
    double lon;  // longitude
};
std::ostream& operator<<(std::ostream& stream, const GeoCoord& coord);

// Pose -> Position and Orientation of Drone
struct Pose {
    Pose() : heading(0), alt(0), roll(0), pitch(0){};
    Pose(GeoCoord coord, double heading, double alt) : coord(coord), heading(heading), alt(alt), roll(0), pitch(0){};
    GeoCoord coord;  // lat/lon
    double heading;  // relative to north, positive clockwise (?), radians (aka yaw)
    double alt;      // height above ground in meters
    double roll;
    double pitch;

    static std::string get_field_names() { return "lat lon heading alt roll pitch"; }
};
std::ostream& operator<<(std::ostream& stream, const Pose& pose);

struct Waypoint {
    Waypoint() : take_pic(false){};
    Waypoint(double lat, double lon, double heading, double alt, bool take_pic)
        : pose(GeoCoord(lat, lon), heading, alt), take_pic(take_pic){};
    Waypoint(Pose pose, bool take_pic) : pose(pose), take_pic(take_pic){};
    Pose pose;      // position/orientation at the waypoint
    bool take_pic;  // whether to take a picture at this wp or not

    // some nice short-hand accessors
    double lat() const { return pose.coord.lat; };
    double lon() const { return pose.coord.lon; };
    double heading() const { return pose.heading; };
    double alt() const { return pose.alt; };
};
std::ostream& operator<<(std::ostream& stream, const Waypoint& wp);

/*
 * Method leverage by flight control software, drone tells it an initial
 * position, and it returns a set path in terms of lat/lon/heading poses to hit.
 *
 * @arg filename filename of matlab planned path file
 * @arg init_pose initial pose of drone, assuming facing aircraft at front edge
 * @arg standoff_dist - initial distance to front of aircraft?
 */
std::vector<Waypoint> plan_path(const std::string& filename, const Pose& init_pose, double standoff_dist);

std::vector<Waypoint> plan_path(const std::string& filename, const GeoCoord& front, const GeoCoord& tail, double alt,
                                double standoff_dist);

GeoCoord calc_new_coord(double d, double brng, GeoCoord old_lat_lon);

double calc_bearing(GeoCoord a, GeoCoord b);

std::vector<std::string> csv_read_line(std::istream& str);

std::vector<Waypoint> convert_matlab_points(const Pose& drone_loc, std::vector<MatlabPoint> points_m,
                                            double standoff_dist);

std::vector<MatlabPoint> read_in_matlab(const std::string& filename);

void print_waypoints(std::vector<Waypoint> points);
void print_matlab_points(std::vector<MatlabPoint> points);

double wrap_angle_rads(double angle);
double wrap_angle_360(double angle);
}  // namespace path
