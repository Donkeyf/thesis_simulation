function path = bspline_opt(Q, p_b, delta_t)
    % calculate vel and accel cp
    V = zeros(size(Q) - 1, 2);
    A = zeros(size(V) - 1, 2);
    for i = 1:size(V)
        V(i,:) = 1/delta_t * (Q(i + 1,:) - Q(i,:));
    end

    for i = 1:size(A)
        A(i,:) = 1/delta_t * (V(i + 1,:) - V(i,:));
    end
    
    

end