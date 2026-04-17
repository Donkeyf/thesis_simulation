% OCCUPANCY MAP
mapWidth = 10;
mapHeight = 10;
resolution = 5;

map = binaryOccupancyMap(mapWidth, mapHeight, resolution);

% Obstacles
% Circle parameters
cx = 6;      % center x
cy = 3;      % center y
r  = 0.5;      % radius

% Create grid around the circle
[x, y] = meshgrid(cx-r:0.05:cx+r, cy-r:0.05:cy+r);

% Keep only points inside the circle
inside = (x - cx).^2 + (y - cy).^2 <= r^2;

% Set occupancy
setOccupancy(map, [x(inside), y(inside)], 1);

[x, y] = meshgrid(0:0.1:4, 3.5:0.1:4);
setOccupancy(map, [x(:), y(:)], 1);

[x, y] = meshgrid(6:0.1:8, 7.5:0.1:8);
setOccupancy(map, [x(:), y(:)], 1);

[x, y] = meshgrid(3.5:0.1:4, 6:0.1:10);
setOccupancy(map, [x(:), y(:)], 1);


[x, y] = meshgrid(6:0.1:7, 5:0.1:6);
setOccupancy(map, [x(:), y(:)], 1);

[x, y] = meshgrid(3.5:0.1:4.5, 1:0.1:2);
setOccupancy(map, [x(:), y(:)], 1);

% Show map
figure;
show(map);
hold on;
title('UAV with Virtual Camera (Ray Casting)');
xlabel('X (m)');
ylabel('Y (m)');

% UAV STATE
uavPose = [1 1 pi/4];  % [x y theta] (ROW vector is important)

% CAMERA PARAMETERS
camRange = 10;
camFOV = pi/3;
numRays = 50;

angles = linspace(-camFOV/2, camFOV/2, numRays);

% ✅ Vectorized ray casting (clean + fast)
endPts = rayIntersection(map, uavPose, angles, camRange);

% Compute ranges
ranges = vecnorm(endPts - uavPose(1:2), 2, 2);

% Plot rays
for i = 1:size(endPts,1)
    plot([uavPose(1), endPts(i,1)], ...
         [uavPose(2), endPts(i,2)], 'r');
end

% Plot UAV
plot(uavPose(1), uavPose(2), 'bo', 'MarkerSize', 8, 'LineWidth', 2);

% Heading arrow
quiver(uavPose(1), uavPose(2), ...
       cos(uavPose(3)), sin(uavPose(3)), ...
       0.5, 'b', 'LineWidth', 2);