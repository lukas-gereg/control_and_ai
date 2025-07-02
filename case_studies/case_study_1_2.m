%%

T = [0, 50];
h = 0.1;
z = 0.1;
disorder_times = [30, 34];
u_control = 1;

sys = tf([100], [0.15, 1, 0]); % Motor
sys_d = c2d(sys, 0.08) % Motor

[PC, IC, DC] = Naslin(sys, "PI", 2)
% [PC, IC, DC] = GrahamLathrop(sys, "PI")
% [PC, IC, DC] = Butterworth(sys, "PI")
% [PC, IC, DC] = ZieglerNicholsMichajlov(sys, "PI") % Will crash
% [PC, IC, DC] = ZieglerNicholsExperimental(sys, "PI") % Will crash
% [PC, IC, DC] = MOM(sys, "PI") % Will Crash
stable = VerifyPID(sys, PC, IC, DC);

if ~stable
    error('PID is not stable. Coefficients: P: %d, I: %d, D: %d.', PC, IC, DC);
end

y_inf_motor = y_inf(sys, tf([DC, PC, IC], [1, 0]))
u_inf_motor = u_inf(sys, tf([DC, PC, IC], [1, 0]))


[PD, SD, DD] = PID2PSDTrapezoids(PC, IC, DC, sys_d.Ts)
stable = VerifyPSD(PD, SD, DD);

if ~stable
    error('PSD is not stable. Coefficients: P: %d, S: %d, D: %d.', PD, SD, DD);
end

discrete_regulator = tf([PD, SD, DD], [1, -1, 0], sys_d.Ts);

y_inf_motor_psd = y_inf(sys_d, discrete_regulator)
u_inf_motor_psd = u_inf(sys_d, discrete_regulator)

outc = sim('PIDSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(h));
outd = sim('DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

figure(Name='DC Motor Naslin PID vs PSD')
% figure(Name='DC Motor GL PID vs PSD')
% figure(Name='DC Motor BW PID vs PSD')
% figure(Name='DC Motor Ziegler Nichols - Michajlov PID vs PSD')
% figure(Name='DC Motor Ziegler Nichols - Experimental PID vs PSD')
% figure(Name='DC Motor MOM PID vs PSD')

subplot(3, 1, 1)

hold on

stairs(outd.simout.y_t_.Time, outd.simout.y_t_.Data, DisplayName='PSD y(t)')
plot(outc.simout.y_t_.Time, outc.simout.y_t_.Data, DisplayName='PID y(t)')
stairs(outc.simout.z_t_.Time, outc.simout.z_t_.Data, DisplayName='z(t)')

hold off

legend(gca)

subplot(3, 1, 2)

hold on

stairs(outd.simout.u_t_.Time, outd.simout.u_t_.Data, DisplayName='PSD u(t)')
plot(outc.simout.u_t_.Time, outc.simout.u_t_.Data, DisplayName='PID u(t)')
stairs(outc.simout.z_t_.Time, outc.simout.z_t_.Data, DisplayName='z(t)')

hold off

legend(gca)

subplot(3, 1, 3)

hold on

stairs(outd.simout.e_t_.Time, outd.simout.e_t_.Data, DisplayName='PSD e(t)')
plot(outc.simout.e_t_.Time, outc.simout.e_t_.Data, DisplayName='PID e(t)')
stairs(outc.simout.z_t_.Time, outc.simout.z_t_.Data, DisplayName='z(t)')

hold off

legend(gca)

%%

T = [0, 50];
h = 0.1;
z = 0.1;
disorder_times = [30, 34];
u_control = 1;

sys = tf([0.1], [1, 1, 0.2]); % Electric Furnace
sys_d = c2d(sys, 2); % Electric Furnace

[PC, IC, DC] = Naslin(sys, "PI", 2)
% [PC, IC, DC] = GrahamLathrop(sys, "PI")
% [PC, IC, DC] = Butterworth(sys, "PI")
% [PC, IC, DC] = ZieglerNicholsMichajlov(sys, "PI") % Will crash
% [PC, IC, DC] = ZieglerNicholsExperimental(sys, "PI") % Will not stabilize with PSD, works fine with PID though
% [PC, IC, DC] = MOM(sys, "PI")
stable = VerifyPID(sys, PC, IC, DC);

if ~stable
    error('PID is not stable. Coefficients: P: %d, I: %d, D: %d.', PC, IC, DC);
end

y_inf_motor = y_inf(sys, tf([DC, PC, IC], [1, 0]))
u_inf_motor = u_inf(sys, tf([DC, PC, IC], [1, 0]))

[PD, SD, DD] = PID2PSDTrapezoids(PC, IC, DC, sys_d.Ts)
stable = VerifyPSD(PD, SD, DD);

if ~stable
    error('PSD is not stable. Coefficients: P: %d, S: %d, D: %d.', PD, SD, DD);
end

discrete_regulator = tf([PD, SD, DD], [1, -1, 0], sys_d.Ts);

y_inf_motor_psd = y_inf(sys_d, discrete_regulator)
u_inf_motor_psd = u_inf(sys_d, discrete_regulator)

outc = sim('PIDSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(h));
outd = sim('DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

figure(Name='Electric Furnace Naslin PID vs PSD')
% figure(Name='Electric Furnace GL PID vs PSD')
% figure(Name='Electric Furnace BW PID vs PSD')
% figure(Name='Electric Furnace Ziegler Nichols - Michajlov PID vs PSD')
% figure(Name='Electric Furnace Ziegler Nichols - Experimental PID vs PSD')
% figure(Name='Electric Furnace MOM PID vs PSD')

subplot(3, 1, 1)

hold on

stairs(outd.simout.y_t_.Time, outd.simout.y_t_.Data, DisplayName='PSD y(t)')
plot(outc.simout.y_t_.Time, outc.simout.y_t_.Data, DisplayName='PID y(t)')
stairs(outc.simout.z_t_.Time, outc.simout.z_t_.Data, DisplayName='z(t)')

hold off

legend(gca)

subplot(3, 1, 2)

hold on

stairs(outd.simout.u_t_.Time, outd.simout.u_t_.Data, DisplayName='PSD u(t)')
plot(outc.simout.u_t_.Time, outc.simout.u_t_.Data, DisplayName='PID u(t)')
stairs(outc.simout.z_t_.Time, outc.simout.z_t_.Data, DisplayName='z(t)')

hold off

legend(gca)

subplot(3, 1, 3)

hold on

stairs(outd.simout.e_t_.Time, outd.simout.e_t_.Data, DisplayName='PSD e(t)')
plot(outc.simout.e_t_.Time, outc.simout.e_t_.Data, DisplayName='PID e(t)')
stairs(outc.simout.z_t_.Time, outc.simout.z_t_.Data, DisplayName='z(t)')

hold off

legend(gca)