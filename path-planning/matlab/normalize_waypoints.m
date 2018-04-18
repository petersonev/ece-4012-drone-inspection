% Written by Eric Klawitter for Georgia Tech Senior Design

% simple function for converting from pixel based waypoints to the method
% used by the drone control software
% points are fed in as output by calculate waypoints, and are returned in
% the form described by plot_path

function output = normalize_waypoints(img, points, scale)
    % first locate the reference point, always the center of the leftmost
    % points
    [row, col] = find(img == 1);
    l_col = min(col);
    l_row = median(row(col==l_col));
    % this point is then used to calculate the other waypoints in meters as
    % a heading and distance from that point
    output = [];
    
    for i = 1: size(points, 1)
        
        % for each point, calculate the distance and angle
        p_row = points(i,1);
        p_col = points(i,2);
        dy = p_row - l_row;
        dx = p_col - l_col;
        angle = wrapToPi(points(i, 3)) * 180 / pi;
        if (angle < 0)
            angle = angle + 360;
        end
        brng = atan2(dy, dx) * 180/ pi;
        if (brng < 0)
            brng = brng + 360;
        end
        dist = sqrt(dy * dy + dx * dx) * scale;
        output = [output; dist, brng, angle, dy*scale, dx*scale];

end