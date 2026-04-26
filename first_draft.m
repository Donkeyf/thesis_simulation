% OCCUPANCY MAP
mapWidth = 10;
mapHeight = 10;
resolution = 5;

map = binaryOccupancyMap(mapWidth, mapHeight, resolution);

% Obstacles
% Circle parameters
cx = 7;      % center x
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

[x, y] = meshgrid(4:0.1:5, 1:0.1:2);
setOccupancy(map, [x(:), y(:)], 1);

occ = occupancyMatrix(map);   % extract grid (0 free, 1 occupied)


distOutside = bwdist(occ);
distInside  = bwdist(~occ);

sdf = distOutside - distInside;
sdf = sdf / map.Resolution;

imagesc(sdf);
axis equal tight;
colorbar;
title('Signed Distance Field');

% Show map
figure;
show(map);
hold on;
title('UAV with Virtual Camera (Ray Casting)');
xlabel('X (m)');
ylabel('Y (m)');

% UAV STATE
uavPose = [1 1 pi/4];  % [x y theta] (ROW vector is important)
start = [uavPose(1), uavPose(2)];

% TARGET POES
target = [9, 9];

% CAMERA PARAMETERS
camRange = 20;
camFOV = pi/3;
numRays = 20;

angles = linspace(-camFOV/2, camFOV/2, numRays);

% ✅ Vectorized ray casting (clean + fast)
endPts = rayIntersection(map, uavPose, angles, camRange);
rel = endPts - uavPose(1:2);
angles = atan2(rel(:,2), rel(:,1));

[anglesSorted, idx] = sort(angles);
endPts = endPts(idx,:);
fovPoly = [uavPose(1:2); endPts];

%%
% Dijkstras implementation
% [edges, nodes] = build_prm(500, camRange, uavPose, camFOV, uavPose(3), map, fovPoly, target);
% s = edges(:,1);
% t = edges(:,2);
% weights = vecnorm(nodes(s,:) - nodes(t,:), 2, 2);
% G = graph(s, t, weights);
% 
% startIdx = 1;
% goalIdx  = size(nodes,1);
% nodes(goalIdx)
% 
% [pathIdx, pathCost] = shortestpath(G, startIdx, goalIdx);
% path = nodes(pathIdx, :);
%%

% A* implementation
planner = plannerAStarGrid(map);

startGrid = world2grid(map, start);
targetGrid = world2grid(map, target);

pathGrid = plan(planner, startGrid, targetGrid);
path = grid2world(map, pathGrid);

% Plot only the first and last rays
idx = [1, size(endPts,1)];

for i = idx
    plot([uavPose(1), endPts(i,1)], ...
         [uavPose(2), endPts(i,2)], 'r', 'LineWidth', 2);
end

% Plot UAV
plot(uavPose(1), uavPose(2), 'bo', 'MarkerSize', 8, 'LineWidth', 2);

% Heading arrow
quiver(uavPose(1), uavPose(2), ...
       cos(uavPose(3)), sin(uavPose(3)), ...
       0.5, 'b', 'LineWidth', 2);

% Plot nodes
% plot(nodes(:,1), nodes(:,2), 'b.');
% 
% % Plot edges
% for k = 1:size(edges,1)
%     i = edges(k,1); j = edges(k,2);
%     plot([nodes(i,1), nodes(j,1)], ...
%          [nodes(i,2), nodes(j,2)], 'g');
% end

plot(target(1), target(2), 'ro');
plot(path(:,1), path(:,2), 'r', 'LineWidth', 2);