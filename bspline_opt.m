function [path, fval] = bspline_opt(Q_0, pb, ESDF, delta_t)
    
    v_max = 10;
    a_max = 4;
    dthr = 0.5;
    lambdas = [10, 5, 0.01];

    Q_1 = Q_0(2:end-1,:);

    f = @(Q)objective_function(Q_1, pb, ESDF, dthr, v_max, a_max, lambdas, delta_t);
    
    options = optimoptions("fminunc",Algorithm="quasi-newton");
    [path, fval] = fminunc(f,Q_0, options);
    path = [Q_0(1,:); path; Q_0(end,:)];
end