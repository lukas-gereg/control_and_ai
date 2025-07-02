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
T = [0, 24];
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

[PC, IC, DC] = Naslin(sys, "PI", 2);
PC = PC * 0.86;
IC = IC * 1.065;
stable = VerifyPID(sys, PC, IC, DC);

if ~stable
    error('PID is not stable. Coefficients: P: %d, I: %d, D: %d.', PC, IC, DC);
end

[PD, SD, DD] = PID2PSDTrapezoids(PC, IC, DC, Ts);
stable = VerifyPSD(PD, SD, DD);

if ~stable
    error('PID is not stable. Coefficients: P: %d, S: %d, D: %d.', PD, SD, DD);
end

discrete_regulator = tf([PD, SD, DD], [1, -1, 0], Ts);

trainTime = 1800;
testTime = 600;

t_train = (0: Ts: trainTime)';
t_test = (0: Ts: testTime)' + trainTime;

w_train = prbsRandomAmplitude(t_train, x0(end), [-0.1, 0.1], [0, 0.2])';
w_test = generateTestSignal(t_test, x0(end), [-0.05, 0.05], [-0.05, 0.05], 0.1)';

u_control = [w_train, w_test];

outd = sim('NLE_DataCollection.slx', StartTime=num2str(0), StopTime=num2str(testTime + trainTime), SolverType='Fixed-step', FixedStep=num2str(Ts));

y = outd.simout.y_t_.Data';
y_train = y(1: length(w_train));
y_test = y(length(w_train): end);

u_base = outd.simout.u_t_.Data;
u_train = u_base(1: length(w_train));
u_test = u_base(length(w_train): end);

figure(Name="Train data")

subplot(3, 1, 1)

plot((0: Ts: trainTime), y_train, DisplayName="y(k)")

legend(gca)

subplot(3, 1, 2)

plot((0: Ts: trainTime), w_train, DisplayName="w(k)")

legend(gca)

subplot(3, 1, 3)

plot((0: Ts: trainTime), u_train, DisplayName="u(k)")

legend(gca)

figure(Name="Test data")

subplot(3, 1, 1)

plot((0: Ts: testTime), y_test, DisplayName="y(k)")

legend(gca)

subplot(3, 1, 2)

plot((0: Ts: testTime), w_test, DisplayName="w(k)")

legend(gca)

subplot(3, 1, 3)

plot((0: Ts: testTime), u_test, DisplayName="u(k)")

legend(gca)

%% Train FFD network

X_fwd = [y_train(1:end-1); u_train(1:end-1)'];
Y_fwd = y_train(2:end);

net_fwd = train(feedforwardnet([10 10]), X_fwd, Y_fwd);

%% Train INV network

X_inv = [
    w_train(4: end);
    y_train(3: end - 1);
    y_train(2: end - 2);
    y_train(1: end - 3);
];

Y_inv = u_train(4:end)';

net_inv = train(feedforwardnet([10 10]), X_inv, Y_inv);

%% Generate simulink FFD Network

gensim(net_fwd)

%% Generate simulink INV Network

gensim(net_inv)

%% Applied Feedforward model

u_control = w_test;

outsim = sim("NLE_FFD.slx", StartTime=num2str(0), StopTime=num2str(testTime), SolverType='Fixed-step', FixedStep=num2str(Ts));

mse_y_pred = mean((outsim.simout.y_t_.Data - w_test') .^ 2);

figure(Name="Feedforward model response to sinusoid signal");

hold on

stairs((0: Ts: testTime)', outsim.simout.y_t_.Data, DisplayName=sprintf("y_{pred}, (MSE=%.4f)", mse_y_pred));
stairs((0: Ts: testTime)', w_test, DisplayName="y_{ref}");

hold off
grid on

legend(gca)

xlabel("Time");
ylabel("Response");

%% Applied INV model

u_control = w_test;

outsim = sim("NLE_DIC.slx", StartTime=num2str(0), StopTime=num2str(testTime), SolverType='Fixed-step', FixedStep=num2str(Ts));

mse_y_pred = mean((outsim.simout.y_t_.Data - w_test') .^ 2);

figure(Name="DIC model response to sinusoid signal");

hold on

stairs((0: Ts: testTime)', outsim.simout.y_t_.Data, DisplayName=sprintf("y_{pred}, (MSE=%.4f)", mse_y_pred));
stairs((0: Ts: testTime)', w_test, DisplayName="y_{ref}");

hold off
grid on

legend(gca)

xlabel("Time");
ylabel("Response");

%% Applied INV FFC model

PC = PC / 0.86;
IC = IC / 1.065;

stable = VerifyPID(sys, PC, IC, DC);

if ~stable
    error('PID is not stable. Coefficients: P: %d, I: %d, D: %d.', PC, IC, DC);
end

[PD, SD, DD] = PID2PSDTrapezoids(PC, IC, DC, Ts);
stable = VerifyPSD(PD, SD, DD);

if ~stable
    error('PID is not stable. Coefficients: P: %d, S: %d, D: %d.', PD, SD, DD);
end

discrete_regulator = tf([PD, SD, DD], [1, -1, 0], Ts);

u_control = w_test;

outsim = sim("NLE_FFC.slx", StartTime=num2str(0), StopTime=num2str(testTime), SolverType='Fixed-step', FixedStep=num2str(Ts));

mse_y_pred = mean((outsim.simout.y_t_.Data - w_test') .^ 2);

figure(Name="FFC response to sinusoid signal");

hold on

stairs((0: Ts: testTime)', outsim.simout.y_t_.Data, DisplayName=sprintf("y_{pred}, (MSE=%.4f)", mse_y_pred));
stairs((0: Ts: testTime)', w_test, DisplayName="y_{ref}");

hold off
grid on

legend(gca)

xlabel("Time");
ylabel("Response");

%% NARX model

sys_d = c2d(sys, Ts);

uTrainFun = @(t) 0.1 * idinput(length(t), 'prbs', [0, 0.2], [-1 1]);
uTestFun  = @(t) 0.05 * sin(2 * pi * 0.1 * t);
noise_std = 0.05;

[data_train, data_test, t0, x0eq] = collectDataAfterStab(odeHandleSimulink, p, u, Ts, 200, 1e-4, 300, 100, uTrainFun, uTestFun, noise_std);

t_train = (0: size(data_train.y, 1) - 1)' * Ts;

[na, nb, nc, nk] = tf2arxorders(sys_d);

arx_model   = arx(data_train, [na nb nk]);
narx_model = nlarx(data_train, [na nb nk], idSigmoidNetwork);

arx_tf = tf(arx_model.B, arx_model.A, sys_d.Ts);

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
[y_narx, fit_narx] = compare(data_test, narx_model);

fprintf('ARX fit:   %.2f %%\n', fit_arx);
fprintf('NARX fit: %.2f %%\n', fit_narx);

t_test = (0: size(data_test.y, 1) - 1)' * Ts;

figure(Name="Comparison of identifications");

hold on

plot(t_test, data_test.y, 'k-', DisplayName="Test data");
plot(t_test, squeeze(y_arx.y), 'b--', DisplayName=sprintf("ARX: %.2f%%", fit_arx));
plot(t_test, squeeze(y_narx.y),  'r:', DisplayName=sprintf("NARX: %.2f%%", fit_narx), LineWidth=1.5);

hold off

xlabel('Time (s)');
ylabel('y(k)');

grid on

legend(gca);

u_control = 0.5;
z = 0.1;
disorder_times = [15, 16];
T = [0, 30];

original_sys = sys_d;

[Q, P, Ts] = PolesPlacement(sys_d, [0.7, 0.9, 0.495, 0.65]);
discrete_regulator = tf(Q, P, Ts);

out_orig_pp = sim('DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(Ts));

out_narx_pp = sim('NARXSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(Ts));

figure(Name="Poles Placement")

subplot(2, 1, 1)

hold on

stairs(out_orig_pp.simout.y_t_.Time, out_orig_pp.simout.y_t_.Data, DisplayName='ARX Poles Placement y(k)')
stairs(out_narx_pp.simout.y_t_.Time, out_narx_pp.simout.y_t_.Data, DisplayName='NARX Poles Placement y(k)')
stairs(out_orig_pp.simout.z_t_.Time, out_orig_pp.simout.z_t_.Data, DisplayName='z(k)')

hold off

legend(gca)

subplot(2, 1, 2)

hold on

stairs(out_orig_pp.simout.u_t_.Time, out_orig_pp.simout.u_t_.Data, DisplayName='ARX Poles Placement u(k)')
stairs(out_narx_pp.simout.u_t_.Time, out_narx_pp.simout.u_t_.Data, DisplayName='NARX Poles Placement u(k)')
stairs(out_orig_pp.simout.z_t_.Time, out_orig_pp.simout.z_t_.Data, DisplayName='z(k)')

hold off

legend(gca)