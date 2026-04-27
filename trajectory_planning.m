%% Build Occupancy Map
mapWidth = 10;
mapHeight = 10;
resolution = 5;

map = binaryOccupancyMap(mapWidth, mapHeight, resolution);

% Create a border around the map by setting occupancy to 1 along edges
% Determine map size in cells
mapGridSize = map.GridSize; % [Ny Nx]
Nx = mapGridSize(2);
Ny = mapGridSize(1);

% Create a logical grid and set border cells to occupied
occGrid = false(Ny, Nx);
occGrid(1, :) = true;    % top row
occGrid(end, :) = true;  % bottom row
occGrid(:, 1) = true;    % left column
occGrid(:, end) = true;  % right column

% Apply to occupancy map
setOccupancy(map, occGrid);

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

%% UAV Setting

% UAV STATE
uavPose = [1 1 pi/4];  % [x y theta] (ROW vector is important)
start = [uavPose(1), uavPose(2)];

% TARGET POS
target = [9, 9];

% CAMERA PARAMETERS
camRange = 10;
camFOV = pi/3;
numRays = 100;

angles = linspace(-camFOV/2, camFOV/2, numRays);

% ✅ Vectorized ray casting (clean + fast)
endPts = rayIntersection(map, uavPose, angles, camRange);
rel = endPts - uavPose(1:2);
angles = atan2(rel(:,2), rel(:,1));

[anglesSorted, idx] = sort(angles);
endPts = endPts(idx,:);
fovPoly = [uavPose(1:2); endPts];

%% ESDF
occ = occupancyMatrix(map);   % returns values in [0,1]
occ = occ > 0.5;              % convert to logical (important)

D_out = bwdist(occ);      % distance to nearest obstacle
D_in  = bwdist(~occ);     % distance to nearest free space

ESDF = D_out - D_in;

imagesc(ESDF);
colorbar;
axis equal tight;
title('ESDF (meters)');

%% Build PRM

[edges, nodes] = build_prm(150, uavPose, camFOV, uavPose(3), target, endPts);
% goal_idx = find(nodes == target)
% nodes(goal_idx,:)
% G = graph(edges(:,1), edges(:,2));
% bins = conncomp(G);
% 
% disp(bins(1))           % start component
% disp(bins(end))         % goal component
% bins(1) ~= bins(end)

path = a_star(edges, nodes, 1, size(nodes, 1));
path_nodes = nodes(path, :);

%% Plotting
figure;
show(map);
hold on;
title('UAV with Virtual Camera (Ray Casting)');
xlabel('X (m)');
ylabel('Y (m)');

% Plot UAV
plot(uavPose(1), uavPose(2), 'bo', 'MarkerSize', 8, 'LineWidth', 2);

% Heading arrow
quiver(uavPose(1), uavPose(2), ...
    cos(uavPose(3)), sin(uavPose(3)), ...
    0.5, 'b', 'LineWidth', 2);

% Plot only the first and last rays
idx = [1, size(endPts,1)];

for i = idx
    plot([uavPose(1), endPts(i,1)], ...
        [uavPose(2), endPts(i,2)], 'r', 'LineWidth', 2);
end

% Plot nodes
plot(nodes(:,1), nodes(:,2), 'b.');

% Plot edges
for k = 1:size(edges,1)
    i = edges(k,1); j = edges(k,2);
    plot([nodes(i,1), nodes(j,1)], ...
         [nodes(i,2), nodes(j,2)], 'g');
end

% Plot path
plot(path_nodes(:,1), path_nodes(:,2), '-o', 'LineWidth', 2);

% Plot visible area polygon (fovPoly) filled with semi-transparent color
% Ensure polygon is closed
poly = [fovPoly; fovPoly(1,:)];

% Create patch with transparency
hPatch = patch('XData', poly(:,1), 'YData', poly(:,2), ...
    'FaceColor', [1 0.6 0.6], 'EdgeColor', 'none', 'FaceAlpha', 0.25);

% Also plot the FOV boundary for clarity
plot(poly(:,1), poly(:,2), 'r-', 'LineWidth', 1.5);

% Optionally, mark the end points
plot(endPts(:,1), endPts(:,2), 'rx', 'MarkerSize', 6, 'LineWidth', 1.2);