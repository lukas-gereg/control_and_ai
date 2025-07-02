%%

T = [0, 50];
h = 0.1;
z = 0.1;
disorder_times = [30, 34];
u_control = 1;

roots = poly([-3, -0.5])
sys = tf([2], roots); % Motor
sys_d = c2d(sys, 0.1) % Motor

[PC, IC, DC] = Naslin(sys, "PI", 2)

stable = VerifyPID(sys, PC, IC, DC);
 
if ~stable
    error('PID is not stable. Coefficients: P: %d, I: %d, D: %d.', PC, IC, DC);
end

[PD, SD, DD] = PID2PSDTrapezoids(PC, IC, DC, sys_d.Ts)
stable = VerifyPSD(PD, SD, DD);

if ~stable
    error('PSD is not stable. Coefficients: P: %d, S: %d, D: %d.', PD, SD, DD);
end

discrete_regulator = tf([PD, SD, DD], [1, -1, 0], sys_d.Ts);


outcn = sim('PIDSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(h));
outdn = sim('DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

[PC, IC, DC] = GrahamLathrop(sys, "PI")

stable = VerifyPID(sys, PC, IC, DC);

if ~stable
    error('PID is not stable. Coefficients: P: %d, I: %d, D: %d.', PC, IC, DC);
end

[PD, SD, DD] = PID2PSDTrapezoids(PC, IC, DC, sys_d.Ts)
stable = VerifyPSD(PD, SD, DD);

if ~stable
    error('PSD is not stable. Coefficients: P: %d, S: %d, D: %d.', PD, SD, DD);
end

discrete_regulator = tf([PD, SD, DD], [1, -1, 0], sys_d.Ts);

outcgl = sim('PIDSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(h));
outdgl = sim('DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

figure(Name='Naslin PID vs PSD')

subplot(2, 1, 1)

hold on

stairs(outdn.simout.y_t_.Time, outdn.simout.y_t_.Data, DisplayName='Naslin PS y(t)')
plot(outcn.simout.y_t_.Time, outcn.simout.y_t_.Data, DisplayName='Naslin PI y(t)')
% plot(outcgl.simout.y_t_.Time, outcgl.simout.y_t_.Data, DisplayName='GL PID y(t)')
% stairs(outc.simout.z_t_.Time, outc.simout.z_t_.Data, DisplayName='z(t)')

hold off

legend(gca)

subplot(2, 1, 2)

hold on

stairs(outdn.simout.u_t_.Time, outdn.simout.u_t_.Data, DisplayName='Naslin PS u(t)')
plot(outcn.simout.u_t_.Time, outcn.simout.u_t_.Data, DisplayName='Naslin PI u(t)')
% plot(outcgl.simout.u_t_.Time, outcgl.simout.u_t_.Data, DisplayName='GL PID u(t)')
hold off

legend(gca)