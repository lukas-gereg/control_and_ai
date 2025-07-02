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

p = [0; 0];
T = [0, 20];
h = 0.01;
tRange = [T(1): h: T(2)];

MatPendulumParametric = @(t, x) MatPendulum(t, x, l, B, g, m, u);

[t_sol, S_sol] = ode45(MatPendulumParametric, tRange, p);

odeHandleSimulink = @(t, x, u) MatPendulum(t, x, l, B, g, m, u);
x0 = GetHandleX0(MatPendulumParametric, tRange, p);
[A, B, C, D] = LinearizeHandle(odeHandleSimulink, 0, x0, u);

Ts = 0.1;
outIdx = [1];

sys = tf(ss(A, B, C, D));
sys = sys(1);

sys_d = c2d(sys, 0.1);

[Q, P, Ts] = PolesPlacement(sys_d, [0.1, 0.2, 0.3, 0.4]);

discrete_regulator = tf(Q, P, Ts);
% K = place(A, B, [-4, -10]);
% N = 1 / ( C * ( - (A - B * K)) ^ -1 * B);

u_control = 0;
z = 1;
disorder_times = [12, 16];

p = [1; 0];

out_pp = sim('NLE_DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(Ts));

figure(Name="Poles Placement")

subplot(2, 1, 1)

stairs(out_pp.simout.y_t_.Time, out_pp.simout.y_t_.Data)

subplot(2, 1, 2)

stairs(out_pp.simout.u_t_.Time, out_pp.simout.u_t_.Data)

% outppnle = sim('PP_NLE.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(h));
% outpp = sim('PP_LE.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(h));
% 
% figure(Name='Poles Placement Non Linear version')
% 
% subplot(2, 1, 1)
% 
% hold on
% 
% plot(outppnle.simout.y_t_.Time, outppnle.simout.y_t_.Data(:, 2), DisplayName='Poles Placement y(t)')
% stairs(outppnle.simout.w_t_.Time, outppnle.simout.w_t_.Data, DisplayName='w(t)')
% 
% hold off
% 
% legend(gca)
% 
% subplot(2, 1, 2)
% 
% hold on
% 
% plot(outppnle.simout.u_t_.Time, outppnle.simout.u_t_.Data, DisplayName='Poles Placement u(t)')
% % stairs(outppnle.simout.w_t_.Time, outppnle.simout.w_t_.Data, DisplayName='w(t)')
% 
% hold off
% 
% legend(gca)
% 
% figure(Name='Poles Placement Linear version')
% 
% subplot(2, 1, 1)
% 
% hold on
% 
% plot(outpp.simout.y_t_.Time, outpp.simout.y_t_.Data(:, 2), DisplayName='Poles Placement y(t)')
% % stairs(outpp.simout.w_t_.Time, outpp.simout.w_t_.Data, DisplayName='w(t)')
% 
% hold off
% 
% legend(gca)
% 
% subplot(2, 1, 2)
% 
% hold on
% 
% plot(outpp.simout.u_t_.Time, outpp.simout.u_t_.Data, DisplayName='Poles Placement u(t)')
% stairs(outpp.simout.w_t_.Time, outpp.simout.w_t_.Data, DisplayName='w(t)')
% 
% hold off
% 
% legend(gca)