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
T = [0, 24];
h = 0.01;
tRange = [T(1): h: T(2)];

[t_sol, S_sol] = ode45(HillEquationParametrized, tRange, p);

odeHandleSimulink = @(t, x, u) HillEquation(t, x, n, gamma, u);
x0 = GetHandleX0(HillEquationParametrized, tRange, p);
[A, B, C, D] = LinearizeHandle(odeHandleSimulink, 0, x0, u);

sys = tf(ss(A, B, C, D));

[y1, t1] = step(sys, T(2));

figure(Name='Non Linear Equation vs Linearized Equation')

subplot(2, 1, 1)

hold on

plot(t_sol, S_sol(:, 1), 'k', DisplayName='Ode45');
plot(t1, y1(:, 1), 'go', DisplayName='Linearized');

hold off

legend(gca);

subplot(2, 1, 2)

hold on

plot(t_sol, S_sol(:, 2), 'k', DisplayName='Ode45');
plot(t1, y1(:, 2), 'go', DisplayName='Linearized');

hold off

legend(gca);

%%

u_control = 0.5;
z = 0.1;
disorder_times = [10, 14];
outIdx = [length(sys)];
sys = sys(end);

Ts = 0.75;
sys_d = c2d(sys, Ts);

[PC, IC, DC] = Naslin(sys, "PI", 2)
stable = VerifyPID(sys, PC, IC, DC);

if ~stable
    error('PID is not stable. Coefficients: P: %d, I: %d, D: %d.', PC, IC, DC);
end

y_inf_naslin = y_inf(sys, tf([DC, PC, IC], [1, 0]))
u_inf_naslin = u_inf(sys, tf([DC, PC, IC], [1, 0]))

[PD, SD, DD] = PID2PSDSquares(PC, IC, DC, sys_d.Ts);
stable = VerifyPSD(PD, SD, DD);

if ~stable
    error('PSD is not stable. Coefficients: P: %d, S: %d, D: %d.', PD, SD, DD);
end

discrete_regulator = tf([PD, SD, DD], [1, -1, 0], sys_d.Ts);

outc = sim('NLE_PIDSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(h));
outd = sim('NLE_DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

figure(Name='Naslin PID vs PSD')

subplot(2, 1, 1)

hold on

stairs(outd.simout.y_t_.Time, outd.simout.y_t_.Data, DisplayName='PSD y(t)')
plot(outc.simout.y_t_.Time, outc.simout.y_t_.Data, DisplayName='PID y(t)')
stairs(outc.simout.z_t_.Time, outc.simout.z_t_.Data, DisplayName='z(t)')

hold off

legend(gca)

subplot(2, 1, 2)

hold on

stairs(outd.simout.u_t_.Time, outd.simout.u_t_.Data, DisplayName='PSD u(t)')
plot(outc.simout.u_t_.Time, outc.simout.u_t_.Data, DisplayName='PID u(t)')
stairs(outc.simout.z_t_.Time, outc.simout.z_t_.Data, DisplayName='z(t)')

hold off

legend(gca)

[PC, IC, DC] = MOM(sys, "PI")
stable = VerifyPID(sys, PC, IC, DC);

if ~stable
    error('PID is not stable. Coefficients: P: %d, I: %d, D: %d.', PC, IC, DC);
end

y_inf_mom = y_inf(sys, tf([DC, PC, IC], [1, 0]))
u_inf_mom = u_inf(sys, tf([DC, PC, IC], [1, 0]))

[PD, SD, DD] = PID2PSDTrapezoids(PC, IC, DC, sys_d.Ts);
stable = VerifyPSD(PD, SD, DD);

if ~stable
    error('PID is not stable. Coefficients: P: %d, S: %d, D: %d.', PD, SD, DD);
end

discrete_regulator = tf([PD, SD, DD], [1, -1, 0], sys_d.Ts);

% outc = sim('PIDSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(h));
outc = sim('NLE_PIDSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(h));
outd = sim('NLE_DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

figure(Name='MOM PID vs PSD')

subplot(2, 1, 1)

hold on

stairs(outd.simout.y_t_.Time, outd.simout.y_t_.Data, DisplayName='PSD y(t)')
plot(outc.simout.y_t_.Time, outc.simout.y_t_.Data, DisplayName='PID y(t)')
stairs(outc.simout.z_t_.Time, outc.simout.z_t_.Data, DisplayName='z(t)')

hold off

legend(gca)

subplot(2, 1, 2)

hold on

stairs(outd.simout.u_t_.Time, outd.simout.u_t_.Data, DisplayName='PSD u(t)')
plot(outc.simout.u_t_.Time, outc.simout.u_t_.Data, DisplayName='PID u(t)')
stairs(outc.simout.z_t_.Time, outc.simout.z_t_.Data, DisplayName='z(t)')

hold off

legend(gca)

%%

[Q, P, Ts] = DeadBeat(sys_d);
discrete_regulator = tf(Q, P, Ts);

outd = sim('NLE_DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

[Q, P, Ts] = DeadBeatRestrained(sys_d, 3.5);
discrete_regulator = tf(Q, P, Ts);

outdr = sim('NLE_DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

figure(Name='DeadBeat vs DeadBeatRestrained')

subplot(2, 1,  1)

hold on

stairs(outd.simout.y_t_.Time, outd.simout.y_t_.Data, DisplayName='DeadBeat y(t)')
stairs(outdr.simout.y_t_.Time, outdr.simout.y_t_.Data, DisplayName='DeadBeat Restrained y(t)')
stairs(outd.simout.z_t_.Time, outd.simout.z_t_.Data, DisplayName='z(t)')

hold off

legend(gca)

subplot(2, 1,  2)

hold on

stairs(outd.simout.u_t_.Time, outd.simout.u_t_.Data, DisplayName='DeadBeat u(t)')
stairs(outdr.simout.u_t_.Time, outdr.simout.u_t_.Data, DisplayName='DeadBeat Restrained u(t)')
stairs(outd.simout.z_t_.Time, outd.simout.z_t_.Data, DisplayName='z(t)')

hold off

legend(gca)

%%

[Q, P, Ts] = PolesPlacement(sys_d, [0.7, 0.9, 0.495, 0.65]);
discrete_regulator = tf(Q, P, Ts);

outpp = sim('NLE_DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

figure(Name='Poles Placement')

subplot(2, 1,  1)

hold on

stairs(outpp.simout.y_t_.Time, outpp.simout.y_t_.Data, DisplayName='Poles Placement y(t)')
stairs(outpp.simout.z_t_.Time, outpp.simout.z_t_.Data, DisplayName='z(t)')

hold off

legend(gca)

subplot(2, 1,  2)

hold on

stairs(outpp.simout.u_t_.Time, outpp.simout.u_t_.Data, DisplayName='Poles Placement u(t)')
stairs(outpp.simout.z_t_.Time, outpp.simout.z_t_.Data, DisplayName='z(t)')

hold off

legend(gca)

%%

Qc = 1;
Qr= 0.01;
outIdx = [];
lqr_stop_time = 15;

[A, B, C, D] = tf2ss(cell2mat(sys.Numerator), cell2mat(sys.Denominator));

[Q, P, K, N] = LQR(sys, Qc, Qr);
lqr_controller = tf(Q, P);

out_lqr_le = sim('LQR_LE.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(h));
out_lqr_nle = sim('LQR_NLE.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(h));

figure(Name='LQR')

hold on

plot(out_lqr_le.simout.y_t_.Time, out_lqr_le.simout.y_t_.Data, DisplayName='LQR y(t)')
plot(out_lqr_nle.simout.y_t_.Time, out_lqr_nle.simout.y_t_.Data(:, 2), DisplayName='LQR y(t) NLE')
stairs(out_lqr_le.simout.w_z_.Time, out_lqr_le.simout.w_z_.Data, DisplayName='w(t)')
stairs(outpp.simout.z_t_.Time, outpp.simout.z_t_.Data, DisplayName='z(t)')

hold off

legend(gca)