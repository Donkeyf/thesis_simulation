% OCCUPANCY MAP
mapWidth = 10;
mapHeight = 10;
resolution = 5;

map = binaryOccupancyMap(mapWidth, mapHeight, resolution);

% Obstacles
[x, y] = meshgrid(4:0.2:6, 4:0.2:6);
setOccupancy(map, [x(:), y(:)], 1);

[x, y] = meshgrid(1:0.2:2, 1:0.2:2);
setOccupancy(map, [x(:), y(:)], 1);

[x, y] = meshgrid(0:0.2:10, 8:0.2:9);
setOccupancy(map, [x(:), y(:)], 1);

setOccupancy(map, [7 2], 1);
setOccupancy(map, [8 3], 1);

% Show map
figure;
show(map);
hold on;
title('UAV with Virtual Camera (Ray Casting)');
xlabel('X (m)');
ylabel('Y (m)');

% UAV STATE
uavPose = [2 2 pi/4];  % [x y theta] (ROW vector is important)

% CAMERA PARAMETERS
camRange = 4;
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