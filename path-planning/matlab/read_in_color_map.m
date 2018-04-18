function map = read_in_color_map(filename)
file = fopen(filename, 'r');
format = '%d, %d, %d, %s';
map = {[], {}};
color = fscanf(file, format, 3);
while(size(color, 1) ~= 0)
    
    line = fgetl(file);
    map{end,1} = [map{end,1}; color'];
    map{end, 2}{end+1} = line;
    color = fscanf(file, format, 3);
end

end
