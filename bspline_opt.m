function [path, fval] = bspline_opt(Q_0, pb, ESDF, delta_t)
    
    v_max = 10;
    a_max = 4;
    dthr = 0.2;
    lambdas = [10, 100, 0.01];

    Q = Q(2:end-1,:);

    f = @(Q)objective_function(Q, pb, ESDF, dthr, v_max, a_max, lambdas, delta_t);
    
    options = optimoptions("fminunc",Algorithm="quasi-newton");
    [path, fval] = fminunc(f,Q_0, options);
    path = [Q(1,:); path; Q(end,:)];
end