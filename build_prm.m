function [edges, nodes] = build_prm(no_sample, range, uavPose, fovAngle, theta0, map, fovPoly)
    
    r =  range * sqrt(rand(no_sample,1));
    theta = (theta0 - fovAngle/2) + rand(no_sample,1)*fovAngle;

    x = uavPose(1) + r .* cos(theta);
    y = uavPose(2) + r .* sin(theta);

    nodes = [x y];

    isFree = getOccupancy(map, nodes) < 0.5;
    nodes = nodes(isFree, :);

    [in, ~] = inpolygon(nodes(:,1), nodes(:,2), ...
                    fovPoly(:,1), fovPoly(:,2));
    nodes = nodes(in,:);

    connectionDist = 1.5;
    edges = [];
    
    for i = 1:size(nodes,1)
        for j = i+1:size(nodes,1)
            d = norm(nodes(i,:) - nodes(j,:));
            
            if d < connectionDist
                % Check collision along edge
                pts = [linspace(nodes(i,1), nodes(j,1), 10)', ...
                       linspace(nodes(i,2), nodes(j,2), 10)'];
                
                if all(getOccupancy(map, pts) < 0.5)
                    edges = [edges; i j];
                end
            end
        end
    end
    
end
