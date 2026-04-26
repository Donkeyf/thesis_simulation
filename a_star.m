% function [open] = a_star(edges, nodes, start, target)
%     open = [start, 0];
%     close = [];
%     curr_node = 1;
%     g = 0;
% 
%     while ~isempty(open)
%         neighbour_ind_1 = edges(:,1) == curr_node;
%         neighbour_ind_2 = edges(:,2) == curr_node;
% 
%         neighbour = nodes(edges(neighbour_ind_1, 2));
%         neighbour = [neighbour; nodes(edges(neighbour_ind_2, 1))];
% 
%         if ismember(target, neighbour)
%             open = [open; target];
%             break
%         end
% 
%         g_temp = g + sqrt((nodes(curr_node, 1) + neighbour(:, 1))^2 + (nodes(curr_node, 2) + neighbour(:, 2))^2);
%         h = sqrt((target(1) + neighbour(:, 1))^2 + (target(2) + neighbour(:, 2))^2);
%         f = g_temp + h;
% 
% 
% 
%     end
% 
% end

function path = a_star(edges, nodes, startIdx, goalIdx)

N = size(nodes,1);

% Build adjacency list
adj = cell(N,1);
for k = 1:size(edges,1)
    i = edges(k,1);
    j = edges(k,2);
    adj{i} = [adj{i}, j];
    adj{j} = [adj{j}, i]; % undirected
end

% Initialize
openSet = startIdx;
cameFrom = zeros(N,1);

gScore = inf(N,1);
gScore(startIdx) = 0;

fScore = inf(N,1);
fScore(startIdx) = norm(nodes(startIdx,:) - nodes(goalIdx,:)); % heuristic

while ~isempty(openSet)

    % Get node in openSet with lowest fScore
    [~, idx] = min(fScore(openSet));
    current = openSet(idx);

    % Goal check
    if current == goalIdx
        path = reconstruct_path(cameFrom, current);
        return;
    end

    % Remove current from openSet
    openSet(idx) = [];

    % Explore neighbors
    for nb = adj{current}

        tentative_g = gScore(current) + norm(nodes(current,:) - nodes(nb,:));

        if tentative_g < gScore(nb)
            cameFrom(nb) = current;
            gScore(nb) = tentative_g;
            fScore(nb) = gScore(nb) + norm(nodes(nb,:) - nodes(goalIdx,:));

            if ~ismember(nb, openSet)
                openSet(end+1) = nb;
            end
        end
    end
end

% If no path found
path = [];
end


function path = reconstruct_path(cameFrom, current)
path = current;
while cameFrom(current) ~= 0
    current = cameFrom(current);
    path = [current; path];
end
end