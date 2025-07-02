function dxdy = VanDerPol(t, x, a, u)
    
    dx1dy = x(2);
    dx2dy = a * x(2) * (1 - x(1) ^ 2) - x(1) + u;

    dxdy = [dx1dy; dx2dy];
end

a = 0.5;
u = 0; 

p = [0, 1];
T = [0, 30];
h = 0.1;
tRange = [T(1): h: T(2)];

VanDerPolParametric = @(t, x) VanDerPol(t, x, a, u);

[t_sol, S_sol] = ode45(VanDerPolParametric, tRange, p);
[t1, y1] = rk(VanDerPolParametric, T, p, h);
out = sim('VanDerPolSimulink.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(h));

hold on
plot(t_sol, S_sol(:, 1), 'k.');
plot(t1, y1(:,1), 'g');
plot(out.simout.Time, out.simout.Data, 'ro');
hold off