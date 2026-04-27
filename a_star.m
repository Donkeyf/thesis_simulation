function path = a_star(edges, nodes, startIdx, goalIdx)

N = size(nodes,1);

% Build adjacency list
adj = cell(N,1);
for k = 1:size(edges,1)
    i = edges(k,1);
    j = edges(k,2);
    adj{i} = [adj{i}, j];
    adj{j} = [adj{j}, i];
end

% Initialize
openSet = startIdx;
inOpenSet = false(N,1);
inOpenSet(startIdx) = true;

closedSet = false(N,1);

cameFrom = zeros(N,1);

gScore = inf(N,1);
gScore(startIdx) = 0;

fScore = inf(N,1);
fScore(startIdx) = norm(nodes(startIdx,:) - nodes(goalIdx,:));

while ~isempty(openSet)

    % Get node with lowest fScore
    [~, idx] = min(fScore(openSet));
    current = openSet(idx);

    % Goal check
    if current == goalIdx
        path = reconstruct_path(cameFrom, current);
        return;
    end

    % Remove from open set
    openSet(idx) = [];
    inOpenSet(current) = false;
    closedSet(current) = true;

    % Explore neighbors
    for nb = adj{current}

        if closedSet(nb)
            continue;
        end

        tentative_g = gScore(current) + norm(nodes(current,:) - nodes(nb,:));

        if tentative_g < gScore(nb)
            cameFrom(nb) = current;
            gScore(nb) = tentative_g;
            fScore(nb) = gScore(nb) + norm(nodes(nb,:) - nodes(goalIdx,:));

            if ~inOpenSet(nb)
                openSet(end+1) = nb;
                inOpenSet(nb) = true;
            end
        end
    end
end

path = [];
end

function path = reconstruct_path(cameFrom, current)
path = current;
while cameFrom(current) ~= 0
    current = cameFrom(current);
    path = [current; path];
end
end