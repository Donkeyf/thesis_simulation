function [edges, nodes] = build_prm(no_sample, uavPose, fovAngle, theta0, target, endPts, fovPoly)

    range = norm(target - uavPose(1,1:2));
    nodes = [];
    while size(nodes,1) < no_sample
        r = range * sqrt(rand);
        theta = (theta0 - fovAngle/2) + rand*fovAngle;

        x = uavPose(1) + r*cos(theta);
        y = uavPose(2) + r*sin(theta);

        % Check if inside FOV polygon
        in = inpolygon(x, y, fovPoly(:,1), fovPoly(:,2));

        if in
            nodes = [nodes; x y];
        end
    end
    
    nodes = [nodes; target];
    nodes = [uavPose(1:2); nodes];

    connectionDist = 2;
    edges = [];
    
    for i = 1:size(nodes,1)
        for j = i+1:size(nodes,1)
            d = norm(nodes(i,:) - nodes(j,:));
            
            if d < connectionDist
                % Check collision along edge
                if ~collision_check(endPts, nodes(i,:), nodes(j,:))
                    edges = [edges; i j];
                end
            end
        end
    end
end

function isCollision = collision_check(endPts, a, b)
    for k = 1:size(endPts, 1) - 1
        c = endPts(k,:);
        d = endPts(k + 1,:);

        if norm(c - d) > 1
            continue
        end
        
        isCollision = ccw(a, c, d) ~= ccw(b, c, d) & ccw(a, b, c) ~= ccw(a, b, d);
        if isCollision
            return
        end
    end
    return
end

function find_ccw = ccw(a, b, c)
    find_ccw = (c(2) - a(2)) * (b(1) - a(1)) > (b(2) - a(2)) * (c(1) - a(1));
end