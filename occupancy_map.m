% Create a simple 2D occupancy map

% Define map dimensions (in meters)
mapWidth = 10;
mapHeight = 10;
resolution = 10; % cells per meter

% Create the occupancy map
map = binaryOccupancyMap(mapWidth, mapHeight, resolution);

% Add some obstacles (walls and objects)
% Rectangular wall along the top
walls = [0 8; 10 8; 10 9; 0 9; 0 8];
setOccupancy(map, walls, 1, 'polyline');

% Rectangular obstacle in the center
obstacle1 = [4 4; 6 4; 6 6; 4 6; 4 4];
setOccupancy(map, obstacle1, 1, 'polyline');

% Small obstacle in bottom-left
setOccupancy(map, [2 2; 3 2; 3 3; 2 3; 2 2], 1, 'polyline');

% Single-cell obstacles
setOccupancy(map, [7 2], 1);
setOccupancy(map, [8 3], 1);

% Display the map
figure;
show(map);
title('Binary Occupancy Map');
xlabel('X (meters)');
ylabel('Y (meters)');
