#include "path_planner.h"

namespace path {

std::ostream& operator<<(std::ostream& stream, const MatlabPoint& pt) {
    return stream << std::setprecision(5) << pt.heading << " " << std::setprecision(9) << pt.dist << " "
                  << std::setprecision(5) << pt.image_heading;
}

std::ostream& operator<<(std::ostream& stream, const GeoCoord& coord) {
    return stream << std::setprecision(9) << coord.lat << " " << coord.lon;
}

std::ostream& operator<<(std::ostream& stream, const Pose& pose) {
    return stream << pose.coord << " " << std::setprecision(5) << pose.alt << " " << pose.heading << " " << pose.roll
                  << " " << pose.pitch;
}

std::ostream& operator<<(std::ostream& stream, const Waypoint& wp) { return stream << wp.pose << " " << wp.take_pic; }

std::vector<Waypoint> plan_path(const std::string& filename, const Pose& init_pose, double standoff_dist) {
    // read in csv output by matlab for relative points
    std::vector<MatlabPoint> matlab_points = read_in_matlab(filename.c_str());

    // convert matlab points using initial drone loc and standoff dist
    std::vector<Waypoint> waypoints = convert_matlab_points(init_pose, matlab_points, standoff_dist);

    return waypoints;
}

std::vector<Waypoint> plan_path(const std::string& filename, const GeoCoord& front, const GeoCoord& tail, double alt,
                                double standoff_dist) {
    path::Pose init_pose;
    init_pose.alt = alt;
    init_pose.coord = front;
    init_pose.heading = calc_bearing(front, tail);

    std::cout << "Init pose: " << init_pose << std::endl;

    std::cout << "Actual heading: " << init_pose.heading << std::endl;

    // std::cout << "Pose heading is : " << init_pose.heading << std::endl;
    // these aren't used for planning, so they really don't matter
    init_pose.roll = 0;
    init_pose.pitch = 0;

    std::vector<MatlabPoint> matlab_points = read_in_matlab(filename.c_str());
    std::vector<Waypoint> waypoints = convert_matlab_points(init_pose, matlab_points, standoff_dist);

    return waypoints;
}

GeoCoord calc_new_coord(double d, double brng, GeoCoord old_lat_lon) {
    double R = 6378100;  // earths radius, meters
    // brng *= 180/ M_PI;
    old_lat_lon.lat = old_lat_lon.lat / 180.0 * M_PI;
    old_lat_lon.lon = old_lat_lon.lon / 180.0 * M_PI;

    std::cout << old_lat_lon.lat << ", " << old_lat_lon.lon << ", " << d << ", " << brng * 180 / M_PI << "\n";

    struct GeoCoord res;
    res.lat = asin(sin(old_lat_lon.lat) * cos(d / R) + cos(old_lat_lon.lat) * sin(d / R) * cos(brng));
    double y = sin(brng) * sin(d / R) * cos(old_lat_lon.lat);
    double x = cos(d / R) - sin(old_lat_lon.lat) * sin(res.lat);
    // std::cout << "y: " << y << ", x: " << x << "\n";
    res.lon = old_lat_lon.lon + atan2(y, x);
    res.lat *= 180.0 / M_PI;
    res.lon *= 180.0 / M_PI;
    return res;
}

double calc_bearing(GeoCoord a, GeoCoord b) {
    // convert to radians
    a.lat = a.lat / 180.0 * M_PI;
    a.lon = a.lon / 180.0 * M_PI;
    b.lat = b.lat / 180.0 * M_PI;
    b.lon = b.lon / 180.0 * M_PI;

    double dLon = (b.lon - a.lon);

    double y = sin(dLon) * cos(b.lat);
    double x = cos(a.lat) * sin(b.lat) - sin(a.lat) * cos(b.lat) * cos(dLon);

    double brng = atan2(y, x);

    brng *= 180.0 / M_PI;
    // brng = toDegrees(brng);
    // brng = (brng + 360) % 360;
    // brng = 360 - brng; // count degrees counter-clockwise - remove to make clockwise

    return wrap_angle_360(brng);
}

std::vector<struct Waypoint> convert_matlab_points(const Pose& drone_pose, std::vector<MatlabPoint> points_m,
                                                   double standoff_dist) {
    std::vector<Waypoint> waypoints;
    double drone_heading_rads = drone_pose.heading / 180 * M_PI;
    double prev_x = 0;
    double prev_y = 0;
    double prev_heading = drone_heading_rads;

    for (const auto point : points_m) {
        double hdg_rads = point.heading / 180 * M_PI;
        double phi = wrap_angle_rads(hdg_rads + drone_heading_rads);

        double x_f = standoff_dist * sin(drone_heading_rads) + point.dist * sin(phi);
        double y_f = standoff_dist * cos(drone_heading_rads) + point.dist * cos(phi);
        std::cout << "x_f: " << x_f << ", y_f: " << y_f << "\n";

        double new_heading = atan2(x_f, y_f);
        double new_dist = sqrt(x_f * x_f + y_f * y_f);

        struct Waypoint new_waypoint;
        // new_waypoint.dn = y_f;
        // new_waypoint.de = x_f;

        double dx = x_f - prev_x;
        double dy = y_f - prev_y;
        double dist_to_prev = sqrt(dx * dx + dy * dy);
        double theta = atan2(dx, dy);
        double psi = theta - prev_heading;
        // new_waypoint.d_fow = dist_to_prev * cos(psi);
        // new_waypoint.d_right = dist_to_prev * sin(psi);
        double i_hat = sin(prev_heading);
        double j_hat = cos(prev_heading);
        double fow = dx * i_hat + dy * j_hat;
        double right = sqrt(dist_to_prev * dist_to_prev - fow * fow);
        psi = wrap_angle_rads(psi);
        // std::cout << prev_heading << ", " << theta << ", " << psi << std::endl;
        if (psi < 0) {
            right = -1 * right;
        }

        Pose pose(calc_new_coord(new_dist, new_heading, drone_pose.coord),
                  wrap_angle_360(drone_pose.heading + point.image_heading), drone_pose.alt);

        prev_x = x_f;
        prev_y = y_f;
        prev_heading = wrap_angle_rads(pose.heading / 180 * M_PI);

        waypoints.push_back(Waypoint(pose, true));
    }
    return waypoints;
}

// reads in file of form: distance from f/c, heading from f/c, image heading
std::vector<MatlabPoint> read_in_matlab(const std::string& filename) {
    std::vector<MatlabPoint> points;
    std::ifstream input(filename);
    std::stringstream buffer;
    if (input) {
        buffer << input.rdbuf();
    } else {
        std::cout << "ERROR PARSING CSV POINTS" << std::endl;
    }
    std::string dist_s;
    while (std::getline(buffer, dist_s, ',')) {
        std::string heading_s;
        std::string img_hdg_s;
        std::getline(buffer, heading_s, ',');
        std::getline(buffer, img_hdg_s, '\n');
        MatlabPoint p;
        p.heading = std::stod(heading_s);
        p.dist = std::stod(dist_s);
        p.image_heading = std::stod(img_hdg_s);
        points.push_back(p);
    }
    input.close();
    return points;
}

void print_waypoints(std::vector<Waypoint> points) {
    std::cout << "lat,lon,heading" << std::endl;
    for (std::size_t i = 0; i < points.size(); ++i) {
        std::cout << i << " " << points[i] << std::endl;
    }
}

void print_MatlabPoints(std::vector<MatlabPoint> points) {
    std::cout << "heading, dist" << std::endl;
    for (const auto& matlab_wp : points) std::cout << matlab_wp << std::endl;
}

double wrap_angle_rads(double angle) {
    while (angle < -1 * M_PI) {
        angle += M_PI * 2;
    }
    while (angle > M_PI) {
        angle -= M_PI * 2;
    }
    return angle;
}

double wrap_angle_360(double angle) {
    while (angle > 360.0) {
        angle -= 360.0;
    }
    while (angle < 0) {
        angle += 360.0;
    }
    return angle;
}
}  // namespace path
