function J = objective_function(Q, pb, ESDF, dthr)

    %% smoothness cost
    start_id = pb - 1;
    end_id = size(Q) - pb + 1;

    % Extract shifted arrays for vectorized calculation
    Q_next = Q(start_idx+1 : end_idx+1, :);
    Q_curr = Q(start_idx   : end_idx,   :);
    Q_prev = Q(start_idx-1 : end_idx-1, :);

    diffs = Q_next - 2*Q_curr + Q_prev;
    fs = sum(diffs.^2, 'all');

    %% collision cost
    % start id is pb
    end_id = size(Q) - pb;
    
    d_Q = ESDF(floor(Q(:, 1)), floor(Q(:,2)));
    Fc = zeros(size(Q)- 2 * pb);
    
    for i = pb:size(Q)- 2 * pb
        if d_Q(i) <= dthr
            Fc(i) = d_Q(i) - dthr;
        else
            Fc(i) = 0;
        end
    end
    
    %% Velocity and Acceleration cost
     
    
end