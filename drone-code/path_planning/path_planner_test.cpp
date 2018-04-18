#include "path_planner.h"

#include <iostream>

using namespace path;

int main(int argc, char* argv[]) {
    std::string path_filename;
    if (argc > 1) {
        path_filename = argv[1];
    } else {
        std::cout << "Please pass in matlab_path.csv";
        return 0;
    }

    Pose drone_pose;  // initially set to burger bowl
    drone_pose.coord.lat = 33.779022;
    drone_pose.coord.lon = -84.402975;
    drone_pose.heading = 270.0;
    // roll and pitch make no difference to planner
    drone_pose.roll = 20.0;
    drone_pose.pitch = -30.0;

    // test out stream operator for fun
    std::cout << "Drone pose output test: " << drone_pose << std::endl
              << std::endl;

    // Now test out planning
    double waypoint_alt = 15;
    double standoff_dst = 2;

    // note, tail_coord is north-west-ish of front_coord
    GeoCoord front_coord(33.778745, -84.402746);
    GeoCoord tail_coord(33.779229, -84.403151);

    auto wps = plan_path(path_filename, front_coord, tail_coord, waypoint_alt, standoff_dst);

    print_waypoints(wps);

    return 0;
}
