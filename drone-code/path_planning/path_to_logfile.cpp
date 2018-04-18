#include "path_planner.h"

#include <iostream>

using namespace path;

int main(int argc, char* argv[]) {
    std::string path_filename, output_log_filename;
    if (argc > 2) {
        path_filename = argv[1];
        output_log_filename = argv[2];
    } else {
        std::cout << "Please pass in matlab_path.csv and output file name";
        return 0;
    }

    auto ideal_pose_log = std::ofstream(output_log_filename, std::ofstream::out);

    // Now test out planning
    double waypoint_alt = 15;
    double standoff_dst = 2;

    // note, tail_coord is north-west-ish of front_coord
    GeoCoord front_coord(33.778745, -84.402746);
    GeoCoord tail_coord(33.779229, -84.403151);

    auto wps = plan_path(path_filename, front_coord, tail_coord, waypoint_alt, standoff_dst);

    ideal_pose_log << "t " + path::Pose::get_field_names() << std::endl;

    for (std::size_t i = 0; i < wps.size(); ++i) {
        ideal_pose_log << i << " " << wps[i].pose << std::endl;
    }

    return 0;
}
