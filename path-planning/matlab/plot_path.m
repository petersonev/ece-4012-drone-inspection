% Written by Eric Klawitter for Georgia Tech Senior Design

% main runner function for calculating waypoints

%%% args:
% image_file: string, path to image of object to plot path for
% color_image_file:string, path to color image file used for tagging images at certain
% waypoints
% color_map_file: string, path to csv that maps colors in the above image
% to descriptors of the plane
% range_m: num, the distance the camera can see
% safety_radius_m: num, distance in meters to stay away from the object
% fov: num, camera FOV degrees
% px_to_m: num, scaling factor for image to meters
% minoverlap: num, required overlap between two images (useful for
% photostitching)

%%% returns:
% points: Nx5 matrix of waypoints, contents each row is a point as defined
% below.  All waypoints are described as a distance and angle from the
% leftmost centermost inner perimeter point
    % Index 1. distance from leftmost point to waypoint
    % 2. heading from leftmost point to waypoint, clockwise, right is 0
    % 3. heading the drone should face when taking the image (cw, right is
    % 0)
    % 4,5: forward and right distance to get to next point from current
    % point, currently unused
% tags: Nx1 cell array of strings describing the images taken at each point

function [points, tags] = plot_path(image_file, color_image_file, color_map_file, range_m, safety_radius_m, fov, px_to_m, minoverlap);


% do some conversions from meters and degrees to pixels and rads
range_px = floor(range_m / px_to_m);
safety_radius_px = ceil(safety_radius_m / px_to_m);
fov_rad = fov / 180 * pi;

% read in the images and color map
img = imread(image_file);
img = im2bw(img, .5);
color_img = imread(color_image_file);
color_img_map = read_in_color_map(color_map_file);

% invert if image uses black to represent object
if (img(1,1) == 1)
    img = ~img;
end


% get the perimeters of the inner object and safety perimter
[imgbound, dilatedbound, innerboundimg, outerboundimg, dilated] = initial_processing(img, safety_radius_px);


% actually do the calculation, can take multiple minutes
[points, tags] = calculate_waypoints(fov_rad, range_px, safety_radius_px, imgbound, dilatedbound, innerboundimg, outerboundimg, minoverlap, color_img, color_img_map, px_to_m);

% produce a diagram showing the object, safety radius, and
% waypoints/headings
produce_diagram(points,img, dilated);

% normalize the points back to meters by the scale factor
points = normalize_waypoints(img, points, px_to_m);

end