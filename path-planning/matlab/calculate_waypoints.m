% Written by Eric Klawitter for Georgia Tech Senior Design

% calculates a set of symmetric waypoints around an object to fully
% photograph the perimeter of an object

%%% Notes:
% The code will start the waypoint planning from the leftmost
% point and will produce a vertically symmetric path

%%% Params:
% fov : number, the camera FOV
% range : number (meters), distance from the camera that the object can be seen
% safety_radius: number (meters), the distance from the object to maintain
% innerpts: Nx2 matrix of row,column points that represent the perimeter to
% be captured
% outerpts: Nx2 matrix representing the dilated object, also the curve that
% all waypoints will be located in
% innerimg: NxM logical, represents with 1 where (R,C) is in innerpts
% outerimg: NxM logical, with a 1 where (R,C) is in outerpts
% minoverlap: number, threshhold of overlap for each image (useful if
% intending to photostitch
% color_img: NxMx3 color image used for image tagging purposes
% color_img_map: 1x2 Cell array where first item is an array of strings
% describing a part of the image, second item is an Nx3 matrix of rgb
% values that correspond to the respective string in the first item, and
% the rgb values correspond to a color in the colr image passed in
% px_to_m: number, scale of the image (pixels per meter)

%%% Returns:
% points: Nx5 Matrix representing each waypoint, the 5 fields are:
%   1. the row in the image for the waypoint
%   2. the column in the image for the waypoint
%   3. the angle heading to take the image at (clockwise angle, direct right=0
%   4. central point on the object perimeter, row
%   5. central point on the object perimeter, col

function [points, tags] = calculate_waypoints(fov, range, safety_radius, innerpts, outerpts, innerimg, outerimg, minoverlap, color_img, color_img_map, px_to_m)


% do some visualization initialization
bothimg= innerimg | outerimg;
subplot(1,2,1)
imshow(bothimg)
innerpts = rotate_inner(innerpts);
coverage = logical(zeros(size(innerpts, 1),1));
blank = logical(zeros(size(innerimg)));


% initialize our variables
curRow = 1;
derivativelen = 20; % number of points used to take derivative
points = [];
tags = {};
prev_bounds = [];
temprange = 0;

while(curRow < size(innerpts,1))
    % get first (row,col) pair in the image on the inside perimeter
    crow = innerpts(curRow, 1);
    ccol = innerpts(curRow, 2);
    
    % calculate orthogonal angle at that point on the inner perimeter
    angle = calc_normal(curRow, innerpts, derivativelen);
    
    % draw line at this angle through that point in both directions
    % draws line at slightly more than safety radius
    [srow, scol] = calculate_endpoint(crow, ccol, angle, safety_radius * 1.5+temprange);
    [frow, fcol] = calculate_endpoint(crow, ccol, mod(angle+pi, 2*pi), safety_radius * 1.5+temprange);
    
    % intersect new line with the outer safety boundary
    % intersection point will be used to calculate in view points
    intersections = intersect_line_on_image_wide(srow, frow, scol, fcol, blank, outerimg);
    intersections = remove_invalid_intersections(intersections, crow, ccol, angle, innerimg, blank, safety_radius * 1.5 + temprange);
    
    
    % if no intersections were found, may be in a concave part of plane
    % find alternate angles by sweeping clockwise and counterclockwise from
    % true orthogonal angle
    % temprange gets incremented if no angle sucessfully intersects
    if (size(intersections, 1) < 1)
        % alternative angles: angle+/- 2 degrees up to +/- 20 degrees
        [angle, rowfocal, colfocal, temprange] = find_alternate_angles(angle, crow, ccol, safety_radius, temprange, innerimg, blank, outerimg, px_to_m);
        % if rowfocal returns -1, no intersection was found
        if (rowfocal == -1)
            continue
        end
    else
        % else if we did find intersections, take the focal point as the first one
        rowfocal = intersections(1,1);
        colfocal = intersections(1,2);
        
        % since angle could be off by 180, calculate 4 quadrant arctan to
        % get actual angle
        angle = 2*pi - wrapTo2Pi(atan2(rowfocal - crow, ccol-colfocal));
    end
    
    % now using the focal point and angle, find all points in view
    bounds = sweep_out_vision(rowfocal, colfocal, angle, fov, range+temprange, blank, innerimg, crow, ccol, temprange);
    
    % then get indices of points in view
    inviewidx = sort(find(ismember(innerpts, bounds, 'rows')));
    
    finish = false;
    
    % if we aren't at the starting point and currently at the starting row,
    % we have reached the back tip of the plane and can finish
    if (crow == innerpts(1,1) && size(points, 1) > 2)
        finish=true;
    end
    
    % the first point and finish are always waypoints
    % desireable to have head-on and tail-on images
    if (curRow == 1 || finish)
        
        % set the currently visible points to covered
        coverage(inviewidx) = 1;
        % add the current point to the list of waypoints
        points = [points;rowfocal, colfocal, angle, crow, ccol];
        
        % reset the temporary range
        temprange = 0;
        
        tags{end+1} = get_image_tags(bounds, color_img, color_img_map);
        live_update(points, range, coverage, innerpts, bothimg, inviewidx);
        
        % persist previous point for reasons explained below
        prevpoint = [rowfocal, colfocal, angle, crow, ccol];
        
        % code necessary for live updating display
        previnviewidx = inviewidx;
        live_update(points, range, coverage, innerpts, bothimg, previnviewidx);
        if (finish)
            break
        end
    else
        
        % otherwise need to determine if we need to save a waypoint
        % to do this, calculate two heuristics, both will indicate whether
        % the previous point should be a waypoint
        
        % first heuristic: determine what percentage of points in view have
        % previously been covered
        % if this is less than the desired overlap %, prev point is
        % waypoint
        coveredpct = sum(coverage(inviewidx))/ size(inviewidx, 1);
        
        % second heuristic: calculate total distance of new points covered
        % if this distance is above a certain threshold, save it
        % useful for curves where may be capturing lots of new perimeter at
        % oblique angles
        new_px = size(inviewidx, 1) - sum(coverage(inviewidx));
        new_dist_covered = new_px * px_to_m;
        percent_fov_new = new_dist_covered / (tan(fov/2) * safety_radius * px_to_m);
        
        % if either of these heuristics trigger, save the point
        if (coveredpct < minoverlap || percent_fov_new > .85)
            tags{end+1} = get_image_tags(prev_bounds, color_img, color_img_map);
            prev_bounds = bounds;
            % save current point similar to above
            points = [points; prevpoint];
            temprange = 0;
            coverage(previnviewidx) = 1;
            
            live_update(points, range, coverage, innerpts, bothimg, previnviewidx);
            previnviewidx= inviewidx;
        else
            % otherwise, save the current set of data
            prevpoint = [rowfocal, colfocal, angle, crow, ccol];
            previnviewidx = inviewidx;
            prev_bounds = bounds;
        end
    end
    % increment to next point and loop again
    curRow = curRow + 1;
end
tags = tags';
[points, tags] = mirror_waypoints(points, tags);
end


% function used to determine what points on the inner perimeter are visible
% draws a series of lines from the focal point at angles between +/- fov/2
% returns set of points currently in view
function points = sweep_out_vision(rf, cf, angle, fov, range, blank, innerimg, crow, ccol, temprange)

% if temprange is nonzero, ie having trouble seeing plane from pt, limit
% range of angles swept through
if (temprange ~= 0)
    fov = fov / 2;
end

% determine set of angles to sweep through
minangle = mod(angle - fov/2, 2 * pi);
maxangle = mod(angle + fov/2, 2* pi);
curangle = minangle;

% by definition, intersection must include the current row/col
intersected = [crow, ccol];

% this can be varied, but should be proportional to how far the range
stepangle = 2/(range);

% initadjacent avoids conical effect of fov, make it triangular
initadjacent = cos(fov/2) * range;
while (abs(curangle- maxangle) > stepangle/2)
    
    % calculate endpoint at current step
    [frow, fcol] = calculate_endpoint(rf, cf, curangle, 1/cos(abs(curangle - angle)) * initadjacent);
    
    % get narrow line and fat line intersections
    intersections = intersect_line_on_image_narrow(rf, frow, cf, fcol, blank, innerimg);
    intersection_wide = intersect_line_on_image_wide(rf, frow, cf, fcol, blank, innerimg);
    
    % if narrow line intersects a point
    if (size(intersections, 1) > 0)
        % find point closest to focal
        % if the narrow point is further than the closest wide point,
        % the line passed through the plane and intersected an unseeable
        % point, in this case discard - helps avoid seeing too many oblique
        % points
        [closest, narrow_dist] = find_closest_point([rf, cf], intersections);
        [closeset_wide, wide_dist] = find_closest_point([rf,cf], intersection_wide);
        if (narrow_dist - wide_dist) < 6
            % if we haven't already intersected this point, add it to the
            % list
            if (~ismember(closest, intersected, 'rows'))
                intersected = [intersected; closest];
            end
        end
    end
    % update the angle
    curangle = curangle + stepangle;
    curangle = mod(curangle, 2* pi);
end
points = intersected;
end

% find point in set closest to a specific point
function [closest, dist] = find_closest_point(start, points)
total = sum(abs(points - start).^2, 2);
closest = points(find(total == min(total)), :);
dist = sqrt(min(total));
end

% calculates orthogonal angle to inner perimeter for a specific point
function angle = calc_normal(row, innerpts, dp)

% dp: num of points for deriv should be somewhate related to image size
p1 = innerpts(mod(row-dp-1, size(innerpts, 1))+1,:);
p2 = innerpts(mod(row+dp-1, size(innerpts, 1))+1,:);
d = p2 - p1; % of form [dy, dx]
angle = atan(-1*d(2)/d(1));
end

% function to calculate the endpoint of a line given start, angle, and len
function [frow,fcol] = calculate_endpoint(srow, scol, angle, len)
frow = round(srow + len * sin(angle));
fcol = round(scol + len * cos(angle));
end

% gets intersection of two images, currently unused but is useful for
% debugging intersection based logic
function intersections = get_intersection(img1, img2)
[row, col] = find(img1 & img2);
intersections = [row,col];
end

% live update function, draws two windows of newly covered points and
% current set of intersections
function live_update(points, len, coverage, innerpts, img, newcoverage)
subplot(1,2,1)
for(i=size(points,1))
    [frow, fcol] = calculate_endpoint(points(i,1), points(i,2), points(i,3), len);
    line([fcol, points(i,2)], [frow, points(i,1)])
end
subplot(1,2,2)
plot(innerpts(coverage,2), -innerpts(coverage,1), 'r*');
hold on
plot(innerpts(newcoverage,2), -innerpts(newcoverage,1), 'g*');
drawnow;
end


% interpolates a line onto an image given two ordered pairs
function img = draw_line_on_image(y1, y2, x1, x2, img)
if (y1 < 1)
    y1 = 1;
end
if (y2 < 1)
    y2 = 1;
end
if (y1 > size(img, 1))
    y1 = size(img, 1);
end
if (y2 > size(img, 1))
    y2 = size(img, 1);
end
if (x1 < 1)
    x1 = 1;
end
if (x2 < 1)
    x2 = 1;
end
if (x2 > size(img, 2))
    x2 = size(img, 2);
end
if (x1 > size(img, 2))
    x1 = size(img, 2);
end
x1=int16(x1); x2=int16(x2); y1=int16(y1); y2=int16(y2);
% distances according to both axes
xn = double(x2-x1);
yn = double(y2-y1);

% interpolate against axis with greater distance between points;
% this guarantees statement in the under the first point!
if (abs(xn) > abs(yn))
    xc = x1 : sign(xn) : x2;
    if yn==0
        yc = y1+zeros(1, abs(xn)+1, 'int16');
    else
        yc = int16(double(y1):abs(yn/xn)*sign(yn):double(y2));
    end
else
    yc = y1 : sign(yn) : y2;
    if xn==0
        xc = x1+zeros(1, abs(yn)+1, 'int16');
    else
        xc = int16(double(x1):abs(xn/yn)*sign(xn):double(x2));
    end
end

% 2-D indexes of line are saved in (xc, yc), and
% 1-D indexes are calculated here:
ind = sub2ind(size(img), yc, xc);

% draw line on the image
img(ind) = 255;
end


% more efficient form of drawing line on image then getting intersection
% same as draw line on image, but instead of drawing, checks if point on
% image is logical 1
% wide version, draws line of width 2px
function intersections = intersect_line_on_image_wide(y1, y2, x1, x2, img, to_intersect)
if (y1 < 1)
    y1 = 1;
end
if (y2 < 1)
    y2 = 1;
end
if (y1 > size(img, 1))
    y1 = size(img, 1);
end
if (y2 > size(img, 1))
    y2 = size(img, 1);
end
if (x1 < 1)
    x1 = 1;
end
if (x2 < 1)
    x2 = 1;
end
if (x2 > size(img, 2))
    x2 = size(img, 2);
end
if (x1 > size(img, 2))
    x1 = size(img, 2);
end
x1=int16(x1); x2=int16(x2); y1=int16(y1); y2=int16(y2);
% distances according to both axes
xn = double(x2-x1);
yn = double(y2-y1);

% interpolate against axis with greater distance between points;
% this guarantees statement in the under the first point!
if (abs(xn) > abs(yn))
    xc = x1 : sign(xn) : x2;
    if yn==0
        yc = y1+zeros(1, abs(xn)+1, 'int16');
    else
        yc = int16(double(y1):abs(yn/xn)*sign(yn):double(y2));
    end
else
    yc = y1 : sign(yn) : y2;
    if xn==0
        xc = x1+zeros(1, abs(yn)+1, 'int16');
    else
        xc = int16(double(x1):abs(xn/yn)*sign(xn):double(x2));
    end
end

% 2-D indexes of line are saved in (xc, yc), and
% 1-D indexes are calculated here:
intersections = [];
for i=1:size(yc, 2)
    if (to_intersect(yc(i), xc(i)))
        intersections = [intersections; double(yc(i)), double(xc(i))];
    end
    try
        if (to_intersect(yc(i)+1, xc(i)))
            intersections = [intersections; double(yc(i)+1), double(xc(i))];
        end
        if (to_intersect(yc(i), xc(i)+1))
            intersections = [intersections; double(yc(i)), double(xc(i)+1)];
        end
    catch
    end
end

end

% narrow version of above, only draws a line 1px wide, will sometimes miss
% intersections due to intersect-4 methodology
function intersections = intersect_line_on_image_narrow(y1, y2, x1, x2, img, to_intersect)
if (y1 < 1)
    y1 = 1;
end
if (y2 < 1)
    y2 = 1;
end
if (y1 > size(img, 1))
    y1 = size(img, 1);
end
if (y2 > size(img, 1))
    y2 = size(img, 1);
end
if (x1 < 1)
    x1 = 1;
end
if (x2 < 1)
    x2 = 1;
end
if (x2 > size(img, 2))
    x2 = size(img, 2);
end
if (x1 > size(img, 2))
    x1 = size(img, 2);
end
x1=int16(x1); x2=int16(x2); y1=int16(y1); y2=int16(y2);
% distances according to both axes
xn = double(x2-x1);
yn = double(y2-y1);

% interpolate against axis with greater distance between points;
% this guarantees statement in the under the first point!
if (abs(xn) > abs(yn))
    xc = x1 : sign(xn) : x2;
    if yn==0
        yc = y1+zeros(1, abs(xn)+1, 'int16');
    else
        yc = int16(double(y1):abs(yn/xn)*sign(yn):double(y2));
    end
else
    yc = y1 : sign(yn) : y2;
    if xn==0
        xc = x1+zeros(1, abs(yn)+1, 'int16');
    else
        xc = int16(double(x1):abs(xn/yn)*sign(xn):double(x2));
    end
end

% 2-D indexes of line are saved in (xc, yc), and
% 1-D indexes are calculated here:
intersections = [];
for i=1:size(yc, 2)
    if (to_intersect(yc(i), xc(i)))
        intersections = [intersections; double(yc(i)), double(xc(i))];
    end
end

end


% gets tags associated with a current set of in view pixels
% uses the set of covered pixels with the color image and image map
function tags = get_image_tags(bounds, color_img, color_image_map)

% gets all colors currently visible
colors = [];
for i= 1:size(bounds, 1)
    colors = [colors;reshape((color_img(bounds(i, 1), bounds(i, 2), :)), [1,3])];
end

% reduce to only unique colors
unique_colors = unique(colors, 'rows');

% lookup index in color mapping
tag_indices = find(ismember(color_image_map{1}, unique_colors, 'rows'));

% create string of tags from indices
tags = '';
for i = 1: size(tag_indices, 1)
    tags = strcat(tags,',', color_image_map{2}{tag_indices(i)});
end
tags = extractAfter(tags, ',');
end


% rotates inner points such that the starting point is the front/center
% point
function points = rotate_inner(input)
row = input(:, 1);
col = input(:, 2);
l_col = min(col);
l_row = floor(median(row(col==l_col)));
[q, idx] = ismember([l_row, l_col], input, 'rows');
points = [input(idx:end, :); input(1:idx-1, :)];
end


% eliminates intersections that pass through the inner perimeter on the way
% to the outer perimeter
function reduced_intersections = remove_invalid_intersections(intersections, crow, ccol, angle, innerimg, blank, range)
reduced_intersections = intersections;
% if no interesctions no need to reduce, return
if (size(intersections, 1) == 0)
    return;
end

% intersects wide line on image
% if the double ended wide line intersects no points, no need to remove any
[srow, scol] = calculate_endpoint(crow, ccol,angle, range);
[frow, fcol] = calculate_endpoint(crow, ccol, mod(angle+pi, 2*pi), range);
to_remove = intersect_line_on_image_wide(srow, frow, scol, fcol, blank, innerimg);
if (size(to_remove, 1) == 0)
    return;
end

% will necessarily intersect the current row/col, ignore those
[q, idx] = ismember([crow, ccol], to_remove, 'rows');
if (idx ~= 0)
    to_remove(idx, :) = [];
end

% for each point identified to remove, if it is close to crow/ccol, ignore
c_idx = [];
for i = 1:size(to_remove, 1)
    dy = to_remove(i, 1) - crow;
    dx = to_remove(i,2) - ccol;
    dist = sqrt(dy * dy + dx * dx);
    if (dist < 4)
        c_idx = [c_idx, i];
    end
end
to_remove(c_idx, :) = [];

% now remove intersections which fall at a similar angle to the line that
% intersects the inner boundary before reaching the outer
reduced_intersections = remove_by_angle(intersections, to_remove, crow, ccol);
end


% function iterates through intersections and removes ones that fall within
% 15 degrees of points identified to remove with respect to crow/ccol
function reduced = remove_by_angle(intersections, to_remove, crow, ccol)
reduced = intersections;
remove_idx = [];

% calculates angle between points to remove and the original intersections
% n^2 because does it for each intersection for each to remove
for i = 1: size(to_remove, 1)
    dy = crow - to_remove(i,1);
    dx = ccol - to_remove(i, 2);
    angle = wrapTo2Pi(atan2(dy, dx));
    
    % calculate angle to the point to the intersection, if it is similar to
    % the point to remove's angle, remove it!
    for (j = 1: size(reduced))
        r_dy = crow - reduced(j, 1);
        r_dx = ccol - reduced(j , 2);
        angle_2 = wrapTo2Pi(atan2(r_dy, r_dx));
        angle_diff = min([(2*pi) - abs(angle - angle_2), abs(angle - angle_2)]);
        
        % if
        if (angle_diff < pi/180 * 15)
            remove_idx = [remove_idx, j];
        end
    end
end
reduced(remove_idx, :) = [];


end

% when the orthogonal line does not intersect, sweeps out left/right to
% find a point/angle which will intersect the safety boundary and have view
% of the pixel
function [angle, rowfocal, colfocal, temprange] = find_alternate_angles(angle, crow, ccol, safety_radius, temprange, innerimg, blank, outerimg, px_to_m)
% currently sweeps out an additional +/- 45 degrees - this is
% tunable
% primarily used when no point on the safety perimeter is close
% enough to the current point, thus increases temprange instead
aa1 = angle:pi/180 * 2: angle +pi/180 * 45;
aa2 = angle:-pi/180*2: angle - pi/180 * 45;

% setup the angles to sweep through
altangles = [aa2;aa1];
altangles = altangles(:)';
altangles = wrapToPi(altangles);

% setup initial vars
index = 1;
firstAngle = -1;
lastAngle = -1;
consec = 0;
rowfocal = -1;
colfocal = -1;
angle = 0;
while (index < size(altangles, 2))
    
    % essentially repeats the original process for finding
    % intersections but for each new angle
    tangle = altangles(index);
    [srow, scol] = calculate_endpoint(crow, ccol,tangle, safety_radius * 1.5 + temprange);
    [frow, fcol] = calculate_endpoint(crow, ccol, mod(tangle+pi, 2*pi), safety_radius * 1.5 + temprange);
    intersections = intersect_line_on_image_wide(srow, frow, scol, fcol, blank, outerimg);
    if (size(intersections, 1) > 0)
        
        intersections = remove_invalid_intersections(intersections, crow, ccol, tangle, innerimg, blank, safety_radius * 1.5 + temprange);
        
        % calculate true angle to adjust for +/- 180 issue
        if (size(intersections, 1) > 0)
            consec = 0;
            rowfocal = intersections(1,1);
            colfocal = intersections(1,2);
            tangle = 2*pi - wrapTo2Pi(atan2(rowfocal - crow, ccol-colfocal));
            
            % want to take average of first and last angle at end
            if firstAngle == -1
                firstAngle = wrapTo2Pi(tangle);
                lastAngle = wrapTo2Pi(tangle);
            else
                lastAngle = wrapTo2Pi(tangle);
            end
        end
    end
    
    % if no intersections for two angles in a row, we're done
    if (size(intersections, 1) == 0 && firstAngle ~= -1)
        consec = consec + 1;
        if (consec == 2)
            break;
        end
    end
    index = index + 1;
end

%if found a valid angle, get intersections for averaged angle
if (firstAngle ~= -1)
    angle = (firstAngle + lastAngle)/2;
    [srow, scol] = calculate_endpoint(crow, ccol, angle, safety_radius * 1.5 + temprange);
    [frow, fcol] = calculate_endpoint(crow, ccol, mod(angle+pi, 2*pi), safety_radius * 1.5 + temprange);
    
    intersections = intersect_line_on_image_wide(srow, frow, scol, fcol, blank, outerimg);
    
    intersections = remove_invalid_intersections(intersections, crow, ccol, angle, innerimg, blank, safety_radius * 1.5 + temprange);
    if (size(intersections, 1) == 0)
        temprange = temprange + (1/px_to_m);
        return;
    end
    rowfocal = intersections(1,1);
    colfocal = intersections(1,2);
    % if never found an angle, increase the range and try again
else
    temprange = temprange + (1/px_to_m);
    return;
end
end

% function used at end to mirror waypoints from top half of plane to bottom
% half

% method: columns stay the same, rows get flipped about the row of the
% first point
% angles get wrapped to +/-180 then flipped then converted back to 0/2Pi
function [points, tags] = mirror_waypoints(points, tags)
centerline = points(1, 4);
for i = size(points, 1)-1:-1:2
    new_row_focal = centerline + (centerline - points(i, 1));
    new_col_focal = points(i, 2);
    new_angle = wrapTo2Pi(wrapToPi(points(i,3)) * -1);
    new_crow = centerline + (centerline - points(i, 4));
    new_ccol = points(i,5);
    points = [points; new_row_focal, new_col_focal, new_angle, new_crow, new_ccol];
    tags(size(points, 1)) = tags(i);
end
end