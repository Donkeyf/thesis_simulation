function J = objective_function(Q, pb, ESDF, dthr, v_max, a_max, lambdas, delta_t)
    N = size(Q, 1) - 1;
    %% smoothness cost
    start_id = pb - 1;
    end_id = N - pb + 1;

    % Extract shifted arrays for vectorized calculation
    Q_next = Q(start_id+1:end_id+1, :);
    Q_curr = Q(start_id:end_id,   :);
    Q_prev = Q(start_id-1:end_id-1, :);

    diffs = Q_next - 2*Q_curr + Q_prev;
    fs = sum(diffs.^2, 'all');

    %% collision cost
    % start id is pb
    end_id = N - pb;
    
    d_Q = ESDF(floor(Q(pb:end_id, 1)), floor(Q(pb:end_id,2)));
    Fc = zeros(1, N- 2 * pb);
    
    for i = pb:end_id
        if d_Q(i) <= dthr
            Fc(i) = d_Q(i) - dthr;
        else
            Fc(i) = 0;
        end
    end

    fc = sum(Fc, 'all');
    
    %% Velocity and Acceleration cost
    % calculate vel and accel cp
    V = zeros(N - 1, 2);
    A = zeros(size(V, 1) - 1, 2);
    for i = 1:size(V, 1)
        V(i,:) = 1/delta_t * (Q(i + 1,:) - Q(i,:));
    end

    for i = 1:size(A, 1)
        A(i,:) = 1/delta_t * (V(i + 1,:) - V(i,:));
    end     
    
    % velocity cost
    start_id = pb - 1;
    end_id = N - pb;
    Fv = zeros(end_id - start_id, 2);

    for k = 1:2
        for j = start_id:end_id
            if V(j, k)^2 > v_max^2
                Fv(j, k) = (V(j, k)^2 - v_max^2)^2;
            else
                Fv(j, k) = 0;
            end
        end
    end

    fv = sum(Fv, 'all');

    % acceleration cost
    start_id = pb - 2;
    end_id = N - pb;
    Fa = zeros(end_id - start_id, 2);

    for k = 1:2
        for j = start_id:end_id
            if A(j, k)^2 > a_max^2
                Fa(j, k) = (A(j, k)^2 - a_max^2)^2;
            else
                Fa(j, k) = 0;
            end
        end
    end

    fa = sum(Fa, 'all');

    J = lambdas(1) * fs + lambdas(2) * fc + lambdas(3) * (fv + fa);

end