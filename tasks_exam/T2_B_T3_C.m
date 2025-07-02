%%
function dxdy = MatPendulum(t, x, l, B, g, m, u)

    dx1dy = x(2);
    dx2dy = (u / (m * l ^ 2)) - B / m * x(2) - g / l * sin(x(1));

    dxdy = [dx1dy; dx2dy];
end

l=0.6;
B=1e-2;
g=9.81;
m=0.2038;
u = 0;

p = [1, 0];
T = [0, 30];
h = 0.01;
tRange = [T(1): h: T(2)];

MatPendulumParametric = @(t, x) MatPendulum(t, x, l, B, g, m, u);

[t_sol, S_sol] = ode45(MatPendulumParametric, tRange, p);
[t1, y1] = rk(MatPendulumParametric, T, p, h);
out = sim('MatPendulum_T3_C_Simulink.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(h));

subplot(2, 1, 1)
hold on
plot(t_sol, S_sol(:, 1), 'k.');
plot(t1, y1(:, 1), 'g');
plot(out.tout, out.simout.signal1.Data, 'ro');
legend("Ode45", "RK", "Simulink");
ylabel("Odchýlenie od stredu")
xlabel("t")
hold off

subplot(2, 1, 2)
hold on
plot(t_sol, S_sol(:, 2), 'k.');
plot(t1, y1(:, 2), 'g');
plot(out.tout, out.simout.signal2.Data, 'ro');
legend("Ode45", "RK", "Simulink");
ylabel("Zmena vzdialenosti od stredu")
xlabel("t")
hold off

%%
function dxdy = VanDerPol(t, x, a,  u)
    
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

VanDerPolParametric = @(t, x) VanDerPol(t, x, a,  u);

[t_sol, S_sol] = ode45(VanDerPolParametric, tRange, p);
[t1, y1] = rk(VanDerPolParametric, T, p, h);
out = sim('VanDerPol_T3_C_Simulink.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(h));

subplot(2, 1, 1)
hold on
plot(t_sol, S_sol(:, 1), 'k.');
plot(t1, y1(:, 1), 'g');
plot(out.tout, out.simout.signal1.Data, 'ro');
legend("Ode45", "RK", "Simulink");
ylabel("Poloha")
xlabel("t")
hold off

subplot(2, 1, 2)
hold on
plot(t_sol, S_sol(:, 2), 'k.');
plot(t1, y1(:, 2), 'g');
plot(out.tout, out.simout.signal2.Data, 'ro');
legend("Ode45", "RK", "Simulink");
ylabel("Rýchlosť")
xlabel("t")
hold off