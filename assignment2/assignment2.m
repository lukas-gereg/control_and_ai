function dxdy = HillEquation(t, x, n, gamma, u)
    dx1dy = 1 - x(1) + u;
    dx2dy =  (x(1) ^ n) / (1 + x(1) ^ n) - gamma * x(2);

    dxdy = [dx1dy; dx2dy];
end

gamma = 1;
n = 2;
u = 0;

HillEquationParametrized = @(t, x) HillEquation(t, x, n, gamma, u);

p = [0; 1];
T = [0, 100];
h = 0.01;
tRange = [T(1): h: T(2)];
outIdx = [2];

odeHandleSimulink = @(t, x, u) HillEquation(t, x, n, gamma, u);
x0 = GetHandleX0(HillEquationParametrized, tRange, p);
[A, B, C, D] = LinearizeHandle(odeHandleSimulink, 0, x0, u);

Ts = 0.75;

sys = tf(ss(A, B, C, D));
sys = sys(end);

sys_d = c2d(sys, Ts);

uTrainFun = @(t) 0.1 * idinput(length(t), 'prbs', [0, 0.2], [-1 1]);
uTestFun  = @(t) 0.05 * sin(2 * pi * 0.1 * t);
noise_std = 0.05;

[data_train, data_test, t0, x0eq] = collectDataAfterStab(odeHandleSimulink, p, u, Ts, 200, 1e-4, 100, 20, uTrainFun, uTestFun, noise_std);

t_train = (0: size(data_train.y, 1) - 1)' * Ts;

x0 = GetHandleX0(HillEquationParametrized, tRange, p);
[A, B, C, D] = LinearizeHandle(odeHandleSimulink, 0, x0, u);

[na, nb, nc, nk] = tf2arxorders(sys_d);

arx_model   = arx(data_train, [na nb nk]);
armax_model = armax(data_train, [na nb nc nk]);

arx_tf = tf(arx_model.B, arx_model.A, sys_d.Ts);
armax_tf = tf(armax_model.B, armax_model.A, sys_d.Ts);

figure(Name="Training data")

subplot(2,1,1);

stairs(t_train, data_train.u, 'b', 'LineWidth', 1.5);

title('u(t)');
xlabel('Time');
ylabel('Amplitude');

grid on;

subplot(2,1,2);

stairs(t_train, data_train.y, 'r', 'LineWidth', 1.5);

title('y(t)');
xlabel('Time');
ylabel('Amplitude');

grid on;

[y_arx, fit_arx] = compare(data_test, arx_model);
[y_armax, fit_armax] = compare(data_test, armax_model);

fprintf('ARX fit:   %.2f %%\n', fit_arx);
fprintf('ARMAX fit: %.2f %%\n', fit_armax);

t_test = (0: size(data_test.y, 1) - 1)' * Ts;

figure(Name="Comparison of identifications");

hold on

plot(t_test, data_test.y, 'k-', DisplayName="Test data");
plot(t_test, squeeze(y_arx.y), 'b--', DisplayName=sprintf("ARX: %.2f%%", fit_arx));
plot(t_test, squeeze(y_armax.y),  'r:', DisplayName=sprintf("ARMAX: %.2f%%", fit_armax), LineWidth=1.5);

hold off

xlabel('Time (s)');
ylabel('y(k)');

grid on

legend(gca);

%%

u_control = 0.5;
z = 0.1;
disorder_times = [15, 16];
T = [0, 30];

original_sys = sys_d;

[Q, P, Ts] = DeadBeat(sys_d);
discrete_regulator = tf(Q, P, Ts);

out_orig_d = ...
    sim('NLE_DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

[Q, P, Ts] = DeadBeatRestrained(sys_d, 3.5);
discrete_regulator = tf(Q, P, Ts);

out_orig_dr = ...
    sim('NLE_DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

[Q, P, Ts] = PolesPlacement(sys_d, [0.7, 0.9, 0.495, 0.65]);
discrete_regulator = tf(Q, P, Ts);

out_orig_pp = ...
    sim('NLE_DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

sys_d = arx_tf;

[Q, P, Ts] = DeadBeat(sys_d);
discrete_regulator = tf(Q, P, Ts);

out_arx_d = ...
    sim('NLE_DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

[Q, P, Ts] = DeadBeatRestrained(sys_d, 3.5);
discrete_regulator = tf(Q, P, Ts);

out_arx_dr = ...
    sim('NLE_DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

[Q, P, Ts] = PolesPlacement(sys_d, [0.1, 0.2, 0.3, 0.4]);
discrete_regulator = tf(Q, P, Ts);

out_arx_pp = ...
    sim('NLE_DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

sys_d = armax_tf;

[Q, P, Ts] = DeadBeat(sys_d);
discrete_regulator = tf(Q, P, Ts);

out_armax_d = ...
    sim('NLE_DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

[Q, P, Ts] = DeadBeatRestrained(sys_d, 3.5);
discrete_regulator = tf(Q, P, Ts);

out_armax_dr = ...
    sim('NLE_DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

[Q, P, Ts] = PolesPlacement(sys_d, [0.1, 0.2, 0.3, 0.4]);
discrete_regulator = tf(Q, P, Ts);

out_armax_pp = ...
    sim('NLE_DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

figure(Name="Dead Beat")

subplot(2, 1, 1)

hold on

stairs(out_orig_d.simout.y_t_.Time, out_orig_d.simout.y_t_.Data, DisplayName='Original DeadBeat y(t)')
stairs(out_arx_d.simout.y_t_.Time, out_arx_d.simout.y_t_.Data, DisplayName='ARX DeadBeat y(t)')
stairs(out_armax_d.simout.y_t_.Time, out_armax_d.simout.y_t_.Data, DisplayName='ARMAX DeadBeat y(t)')
stairs(out_orig_d.simout.z_t_.Time, out_orig_d.simout.z_t_.Data, DisplayName='z(t)')

hold off

legend(gca)

subplot(2, 1, 2)

hold on

stairs(out_orig_d.simout.u_t_.Time, out_orig_d.simout.u_t_.Data, DisplayName='Original DeadBeat u(t)')
stairs(out_arx_d.simout.u_t_.Time, out_arx_d.simout.u_t_.Data, DisplayName='ARX DeadBeat u(t)')
stairs(out_armax_d.simout.u_t_.Time, out_armax_d.simout.u_t_.Data, DisplayName='ARMAX DeadBeat u(t)')
stairs(out_orig_d.simout.z_t_.Time, out_orig_d.simout.z_t_.Data, DisplayName='z(t)')

hold off

legend(gca)

figure(Name="Dead Beat Restricted")

subplot(2, 1, 1)

hold on

stairs(out_orig_dr.simout.y_t_.Time, out_orig_dr.simout.y_t_.Data, DisplayName='Original DeadBeat Restricted y(t)')
stairs(out_arx_dr.simout.y_t_.Time, out_arx_dr.simout.y_t_.Data, DisplayName='ARX DeadBeat Restricted y(t)')
stairs(out_armax_dr.simout.y_t_.Time, out_armax_dr.simout.y_t_.Data, DisplayName='ARMAX DeadBeat Restricted y(t)')
stairs(out_orig_dr.simout.z_t_.Time, out_orig_dr.simout.z_t_.Data, DisplayName='z(t)')

hold off

legend(gca)

subplot(2, 1, 2)

hold on

stairs(out_orig_dr.simout.u_t_.Time, out_orig_dr.simout.u_t_.Data, DisplayName='Original DeadBeat Restricted u(t)')
stairs(out_arx_dr.simout.u_t_.Time, out_arx_dr.simout.u_t_.Data, DisplayName='ARX DeadBeat Restricted u(t)')
stairs(out_armax_dr.simout.u_t_.Time, out_armax_dr.simout.u_t_.Data, DisplayName='ARMAX DeadBeat Restricted u(t)')
stairs(out_orig_dr.simout.z_t_.Time, out_orig_dr.simout.z_t_.Data, DisplayName='z(t)')

hold off

legend(gca)

figure(Name="Poles Placement")

subplot(2, 1, 1)

hold on

stairs(out_orig_pp.simout.y_t_.Time, out_orig_pp.simout.y_t_.Data, DisplayName='Original Poles Placement y(t)')
stairs(out_arx_pp.simout.y_t_.Time, out_arx_pp.simout.y_t_.Data, DisplayName='ARX Poles Placement y(t)')
stairs(out_armax_pp.simout.y_t_.Time, out_armax_pp.simout.y_t_.Data, DisplayName='ARMAX Poles Placement y(t)')
stairs(out_orig_pp.simout.z_t_.Time, out_orig_pp.simout.z_t_.Data, DisplayName='z(t)')

hold off

legend(gca)

subplot(2, 1, 2)

hold on

stairs(out_orig_pp.simout.u_t_.Time, out_orig_pp.simout.u_t_.Data, DisplayName='Original Poles Placement u(t)')
stairs(out_arx_pp.simout.u_t_.Time, out_arx_pp.simout.u_t_.Data, DisplayName='ARX Poles Placement u(t)')
stairs(out_armax_pp.simout.u_t_.Time, out_armax_pp.simout.u_t_.Data, DisplayName='ARMAX Poles Placement u(t)')
stairs(out_orig_pp.simout.z_t_.Time, out_orig_pp.simout.z_t_.Data, DisplayName='z(t)')

hold off

legend(gca)