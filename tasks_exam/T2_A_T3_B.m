%%
function dxdy = RLC(t, x, R, L, C, u)
    
    dx1dy = x(2) / C;
    dx2dy = C * (- R * x(2) - x(1) + u) / L;

    dxdy = [dx1dy; dx2dy];
end

R = 5;
L = 0.1;
C = 1e-4;
u = 10; 

p = [0, 0];
T = [0, 0.2];
h = 0.01;
tRange = [T(1): h: T(2)];

RLCParametric = @(t, x) RLC(t, x, R, L, C, u);

[t_sol, S_sol] = ode45(RLCParametric, tRange, p);
[t1, y1] = rk(RLCParametric, T, p, h);
out = sim('RLC_T3_B_Simulink.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(h));

subplot(2, 1, 1);
hold on
plot(t_sol, S_sol(:, 1), 'k.');
plot(t1, y1(:, 1), 'g');
plot(out.tout, out.simout.signal1.Data, 'ro');
xlabel("t");
ylabel("Úbytky napätia");
legend("Ode45", "RK", "Simulink");
hold off

subplot(2, 1, 2);
hold on
plot(t_sol, S_sol(:, 2), 'k.');
plot(t1, y1(:, 2), 'g');
plot(out.tout, out.simout.signal2.Data, 'ro');
xlabel("t");
ylabel("Prúd");
legend("Ode45", "RK", "Simulink");
hold off

%%
function dxdy = DCMotor(t, x, Cu, L, R, B, J, u)

    dx1dy = x(2);
    dx2dy = (Cu * u - (B * L + J * R) * x(2) - (J * L) * x(1)) / (Cu ^ 2 + B * R);

    dxdy = [dx1dy; dx2dy];
end

Cu = 1;
L = 1;
R = 1;
B = 1;
J = 1;
u = 1;

p = [0, 0];
T = [0, 20];
h = 0.1;
tRange = [T(1): h: T(2)];

DCMotorParametric = @(t, x) DCMotor(t, x, Cu, L, R, B, J, u);

[t_sol, S_sol] = ode45(DCMotorParametric, tRange, p);
[t1, y1] = rk(DCMotorParametric, T, p, h);
out = sim('DCMotor_T3_B_Simulink.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(h));

subplot(2, 1, 1);
hold on
plot(t_sol, S_sol(:, 1), 'k.');
plot(t1, y1(:, 1), 'g');
plot(out.tout, out.simout.signal1.Data, 'ro');
xlabel("t");
ylabel("y");
legend("Ode45", "RK", "Simulink");
hold off

subplot(2, 1, 2);
hold on
plot(t_sol, S_sol(:, 2), 'k.');
plot(t1, y1(:, 2), 'g');
plot(out.tout, out.simout.signal2.Data, 'ro');
xlabel("t");
ylabel("y'");
legend("Ode45", "RK", "Simulink");
hold off