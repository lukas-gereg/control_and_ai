function dxdy = LinearDiffFunction(t, x, a, u)

    dx1dy = x(2);
    dx2dy = x(3);
    dx3dy = (3 * u - a(2) * x(3) - a(3) * x(2) - a(4) * x(1)) / a(1);

    dxdy = [dx1dy; dx2dy; dx3dy];
end

function x = LinearDiffFunctionAnalyt(t)
    x = - 0.8 .* exp(-3 .* t) + 2 .* exp(-2 .* t) - 3.2 .* exp(-0.5 .* t) + 2;
end

a = [1, 5.5, 8.5, 3];
u = 2; 

p = [0, 0, 0];
T = [0, 10];
h = 0.1;
tRange = [T(1): h: T(2)];

LinearDiffFunctionParametric = @(t, x) LinearDiffFunction(t, x, a, u);

[t_sol, S_sol] = ode45(LinearDiffFunctionParametric, tRange, p);
[t1, y1] = rk(LinearDiffFunctionParametric, T, p, h);
x = LinearDiffFunctionAnalyt(tRange);
out = sim('T3_A_Simulink.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(h));

subplot(3, 1, 1)

hold on
plot(t_sol, S_sol(:, 1), 'k.');
plot(t1, y1(:, 1), 'g');
plot(tRange, x, 'ro');
plot(out.tout, out.simout.signal1.Data, 'ms');
hold off

legend("Ode45", "RK", "Analytické riešenie", "Simulink");
title("LDR");
xlabel("t");
ylabel("y");

subplot(3, 1, 2)

hold on
plot(t_sol, S_sol(:, 2), 'k.');
plot(t1, y1(:, 2), 'g');
plot(out.tout, out.simout.signal2.Data, 'ms');
hold off

legend("Ode45", "RK", "Simulink");
title("LDR");
xlabel("t");
ylabel("y'");

subplot(3, 1, 3)

hold on
plot(t_sol, S_sol(:, 3), 'k.');
plot(t1, y1(:, 3), 'g');
plot(out.tout, out.simout.signal3.Data, 'ms');
hold off

legend("Ode45", "RK", "Simulink");
title("LDR");
xlabel("t");
ylabel("y''");