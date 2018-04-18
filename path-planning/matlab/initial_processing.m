% Written by Eric Klawitter for Georgia Tech Senior Design

% does initial processing on the passed in image to extract the inner
% perimeter, then dilate it for the safety radius

function [imgbound, dilatedbound, innerboundimg, outerboundimg, dilated] = initial_processing(img, safety_radius)
    
% dilate by safety radius in all directions
se = strel('disk', safety_radius);
dilated = imdilate(img, se);


% get the boundary matrix of the image in 8 neighbor manner
innerboundimg = get_bw_boundary_img(img, 8);

% get the safety radius border matrix in 4 neighbor manner
outerboundimg = get_bw_boundary_img(dilated, 4);
imgbound = bwboundaries(img, 8); % returns closed loop of perimeter pixels, in form (img row, img col)
imgbound = imgbound{1};
dilatedbound = bwboundaries(dilated, 8);
dilatedbound = dilatedbound{1};

end


% helper method for getting producing an image from the boundary
function boundaryImg = get_bw_boundary_img(img, type)
boundaryImg = logical(zeros(size(img)));
imgbound = bwboundaries(img, type);
boundaries = sub2ind(size(boundaryImg),imgbound{1}(:,1), imgbound{1}(:,2));
boundaryImg(boundaries)=1;
end