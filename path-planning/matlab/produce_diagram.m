function produce_diagram(points, img, dilated)
figure
diagram= zeros(size(img,1), size(img,2),3);
blank = zeros(size(img,1), size(img,2));
diagram(:,:,3) = blank | xor(dilated,img);
diagram(:,:,2) = blank | img;

imshow(diagram);
hold on
for(i=1:size(points,1))
    line([points(i,5), points(i,2)], [points(i,4), points(i,1)], 'Color', 'red')
%     line([points(i,5), points(i,2)], [points(i,4), points(i,1)], 'Color', 'm')
%     line([points(i,7), points(i,2)], [points(i,6), points(i,1)], 'Color', 'm')
%     line([points(i,7), points(i,5)], [points(i,6), points(i,4)], 'Color', 'm')
    plot(points(i,2), points(i,1), 'y*');
end

end
