function dxdy = HillEquation(t, x, n, gamma, u)
    dx1dy = 1 - x(1) + u;
    dx2dy =  (x(1) ^ n) / (1 + x(1) ^ n) - gamma * x(2);

    dxdy = [dx1dy; dx2dy];
end

gamma = 1;
n = 2;
u = 0;

HillEquationParametrized = @(t, x) HillEquation(t, x, n, gamma, u);

p = [0, 1];
T = [0, 60];
h = 0.1;
tRange = [T(1): h: T(2)];

[t_sol, S_sol] = ode45(HillEquationParametrized, tRange, p);

odeHandleSimulink = @(t, x, u) HillEquation(t, x, n, gamma, u);
x0 = GetHandleX0(HillEquationParametrized, tRange, p);
[A, B, C, D] = LinearizeHandle(odeHandleSimulink, 0, x0, u);

sys = tf(ss(A, B, C, D));

outIdx = [length(sys)];

sys = sys(end);
Ts = 0.75;

sys_d = c2d(sys, Ts);

uTrainFun = @(t) 0.1 * idinput(length(t), 'prbs', [0, 0.2], [-1 1]);
uTestFun  = @(t) 0.05 * sin(2 * pi * 0.1 * t);
noise_std = 0.05;

[data_train, data_test, t0, x0eq] = collectDataAfterStab(odeHandleSimulink, p, u, Ts, 200, 1e-4, 300, 100, uTrainFun, uTestFun, noise_std);

t_train = (0: size(data_train.y, 1) - 1)' * Ts;

[na, nb, nc, nk] = tf2arxorders(sys_d);

armax_model = armax(data_train, [na nb nc nk]);

armax_tf = tf(armax_model.B, armax_model.A, sys_d.Ts);

u_control = 0.5;
z = 0.1;
disorder_times = [30, 40];

original_sys = sys_d;

[Q, P, Ts] = PolesPlacement(sys_d, [0.7, 0.9, 0.495, 0.65]);
discrete_regulator = tf(Q, P, Ts);

out_orig_pp = sim('NLE_DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(Ts));

out_ffd_pp = sim("NLE_FFD.slx", StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(Ts));

figure(Name="Poles Placement")

subplot(2, 1, 1)

hold on

stairs(out_orig_pp.simout.y_t_.Time, out_orig_pp.simout.y_t_.Data, DisplayName='NLE Poles Placement y(k)')
stairs(out_ffd_pp.simout.y_t_.Time, out_ffd_pp.simout.y_t_.Data, DisplayName='FFD Poles Placement y(k)')
stairs(out_orig_pp.simout.z_t_.Time, out_orig_pp.simout.z_t_.Data, DisplayName='z(k)')

hold off

legend(gca)

subplot(2, 1, 2)

hold on

stairs(out_orig_pp.simout.u_t_.Time, out_orig_pp.simout.u_t_.Data, DisplayName='NLE Poles Placement u(k)')
stairs(out_ffd_pp.simout.u_t_.Time, out_ffd_pp.simout.u_t_.Data, DisplayName='FFD Poles Placement u(k)')
stairs(out_orig_pp.simout.z_t_.Time, out_orig_pp.simout.z_t_.Data, DisplayName='z(k)')

hold off

legend(gca)