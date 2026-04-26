function [prim] = primitive_generation(tau, u_max, r, p_0, v_0)
    % generate set of primitives
    % tau = timespan of motion
    % n= 2 (double integrator)
    % u_max = limit of selected derivative
    % r = number of primitives
    
    % discretize control input and time
    u = linspace(-u_max, u_max, 2 * r + 1);
    
    % create prim vector
    prim = zeros((2 * r + 1)^2, 2);
    
    % loop control inputs
    k = 1;
    for ux = 1:size(u, 2)
        for uy = 1:size(u, 2)
            px = p_0(1) + v_0(1) * tau + 0.5 * u(ux) * tau^2;
            py = p_0(2) + v_0(2) * tau + 0.5 * u(uy) * tau^2;
            prim(k, :) = [px, py]';
            k = k + 1;
        end
    end
    

