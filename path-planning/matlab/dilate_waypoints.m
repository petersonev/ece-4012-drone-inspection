function output = dilate_waypoints(points, dilation_dist)
        output = points
        for (i = 1: size(points, 1))
            theta = points(i,2) / 180 * pi;
            dist = points(i, 1);
            hdg = points(i,3)/ 180 * pi;
            x = dist * cos(theta);
            y = -1 * dist * sin(theta);
            y = y + dilation_dist * sin(hdg);
            x = x - dilation_dist * cos(hdg);
            psi = atan2(-y, x)*180/pi;
            new_dist = sqrt(x * x + y*y);
            output(i,:) = [new_dist, psi, points(i,3), points(i,3), points(i,4)]          

end