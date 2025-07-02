function [A, B, C, D] = LinearizeHandle(f, t0, x0, u0, delta)
    arguments
        f
        t0
        x0
        u0
        delta = 1e-6;
    end

    n = length(x0);
    fx0u0 = f(t0, x0, u0);

    A = zeros(n);
    
    for i = [1: n]
        dx = zeros(n, 1);
        dx(i) = delta;
        A(:, i) = (f(t0, x0 + dx, u0) - fx0u0) / delta;
    end

    B = (f(t0, x0, u0 + delta) - fx0u0) / delta;

    C = eye(n);
    D = zeros(n, 1);
end