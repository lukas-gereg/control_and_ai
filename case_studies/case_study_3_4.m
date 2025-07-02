%%

T = [0, 50];
z = 0.1;
disorder_times = [30, 34];
u_control = 1;

sys = tf([100], [0.15, 1, 0]); % Motor
sys_d = c2d(sys, 0.08); % Motor

[Q, P, Ts] = DeadBeat(sys_d);
discrete_regulator = tf(Q, P, Ts)

y_inf_motor_db = y_inf(sys_d, discrete_regulator)
u_inf_motor_db = u_inf(sys_d, discrete_regulator)

outd = sim('DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

[Q, P, Ts] = DeadBeatRestrained(sys_d, 0.2);
discrete_regulator = tf(Q, P, Ts)

y_inf_motor_dbr = y_inf(sys_d, discrete_regulator)
u_inf_motor_dbr = u_inf(sys_d, discrete_regulator)

outdr = sim('DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

figure(Name='Motor DeadBeat vs DeadBeatRestrained')

subplot(3, 1, 1)

hold on

stairs(outd.simout.y_t_.Time, outd.simout.y_t_.Data, DisplayName='DeadBeat y(t)')
stairs(outdr.simout.y_t_.Time, outdr.simout.y_t_.Data, DisplayName='DeadBeat Restrained y(t)')
stairs(outd.simout.z_t_.Time, outd.simout.z_t_.Data, '--', DisplayName='z(t)')

hold off

legend(gca)

subplot(3, 1, 2)

hold on

stairs(outd.simout.u_t_.Time, outd.simout.u_t_.Data, DisplayName='DeadBeat u(t)')
stairs(outdr.simout.u_t_.Time, outdr.simout.u_t_.Data, DisplayName='DeadBeat Restrained u(t)')
stairs(outd.simout.z_t_.Time, outd.simout.z_t_.Data, '--', DisplayName='z(t)')

hold off

legend(gca)

subplot(3, 1, 3)

hold on

stairs(outd.simout.e_t_.Time, outd.simout.e_t_.Data, DisplayName="DeadBeat e(t)")
stairs(outdr.simout.e_t_.Time, outdr.simout.e_t_.Data, DisplayName="DeadBeat Restrained e(t)")
stairs(outd.simout.z_t_.Time, outd.simout.z_t_.Data, '--', DisplayName='z(t)')

hold off

legend(gca)

[Q, P, Ts] = PolesPlacement(sys_d, [0.1, 0.2, 0.3, 0.4]);
discrete_regulator = tf(Q, P, Ts)

outpp = sim('DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));


figure(Name='Motor Poles Placement')

subplot(3, 1, 1)

hold on

stairs(outpp.simout.y_t_.Time, outpp.simout.y_t_.Data, DisplayName='Poles Placement y(t)')
stairs(outpp.simout.z_t_.Time, outpp.simout.z_t_.Data, '--', DisplayName='z(t)')

hold off

legend(gca)

subplot(3, 1, 2)

hold on

stairs(outpp.simout.u_t_.Time, outpp.simout.u_t_.Data, DisplayName='Poles Placement u(t)')
stairs(outpp.simout.z_t_.Time, outpp.simout.z_t_.Data, '--', DisplayName='z(t)')

hold off

legend(gca)

subplot(3, 1, 3)

hold on

stairs(outpp.simout.e_t_.Time, outpp.simout.e_t_.Data, DisplayName="Poles Placement e(t)")
stairs(outpp.simout.z_t_.Time, outpp.simout.z_t_.Data, '--', DisplayName='z(t)')

hold off

legend(gca)

%%

T = [0, 50];
z = 0.1;
disorder_times = [30, 34];
u_control = 1;

sys = tf([0.1], [1, 1, 0.2]); % Electric Furnace
sys_d = c2d(sys, 2); % Electric Furnace

[Q, P, Ts] = DeadBeat(sys_d);
discrete_regulator = tf(Q, P, Ts)

y_inf_motor_db = y_inf(sys_d, discrete_regulator)
u_inf_motor_db = u_inf(sys_d, discrete_regulator)

outd = sim('DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

[Q, P, Ts] = DeadBeatRestrained(sys_d, 0.2);
discrete_regulator = tf(Q, P, Ts)

y_inf_motor_dbr = y_inf(sys_d, discrete_regulator)
u_inf_motor_dbr = u_inf(sys_d, discrete_regulator)

outdr = sim('DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

figure(Name='Electric Furnace DeadBeat vs DeadBeatRestrained')

subplot(3, 1, 1)

hold on

stairs(outd.simout.y_t_.Time, outd.simout.y_t_.Data, DisplayName='DeadBeat y(t)')
stairs(outdr.simout.y_t_.Time, outdr.simout.y_t_.Data, DisplayName='DeadBeat Restrained y(t)')
stairs(outd.simout.z_t_.Time, outd.simout.z_t_.Data, '--', DisplayName='z(t)')

hold off

legend(gca)

subplot(3, 1, 2)

hold on

stairs(outd.simout.u_t_.Time, outd.simout.u_t_.Data, DisplayName='DeadBeat u(t)')
stairs(outdr.simout.u_t_.Time, outdr.simout.u_t_.Data, DisplayName='DeadBeat Restrained u(t)')
stairs(outd.simout.z_t_.Time, outd.simout.z_t_.Data, '--', DisplayName='z(t)')

hold off

legend(gca)

subplot(3, 1, 3)

hold on

stairs(outd.simout.e_t_.Time, outd.simout.e_t_.Data, DisplayName="DeadBeat e(t)")
stairs(outdr.simout.e_t_.Time, outdr.simout.e_t_.Data, DisplayName="DeadBeat Restrained e(t)")
stairs(outd.simout.z_t_.Time, outd.simout.z_t_.Data, '--', DisplayName='z(t)')

hold off

legend(gca)

%%

T = [0, 50];
z = 0.1;
disorder_times = [30, 34];
u_control = 1;

sys = tf([3], [27, 27, 9, 1]); % Heat Exchanger
sys_d = c2d(sys, 1.5); % Heat Exchanger

[Q, P, Ts] = DeadBeat(sys_d);
discrete_regulator = tf(Q, P, Ts)

y_inf_motor_db = y_inf(sys_d, discrete_regulator)
u_inf_motor_db = u_inf(sys_d, discrete_regulator)

outd = sim('DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

[Q, P, Ts] = DeadBeatRestrained(sys_d, 3.5);
discrete_regulator = tf(Q, P, Ts)

y_inf_motor_dbr = y_inf(sys_d, discrete_regulator)
u_inf_motor_dbr = u_inf(sys_d, discrete_regulator)

outdr = sim('DiscreteSystem.slx', StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(sys_d.Ts));

figure(Name='Heat Exchanger DeadBeat vs DeadBeatRestrained')

subplot(3, 1, 1)

hold on

stairs(outd.simout.y_t_.Time, outd.simout.y_t_.Data, DisplayName='DeadBeat y(t)')
stairs(outdr.simout.y_t_.Time, outdr.simout.y_t_.Data, DisplayName='DeadBeat Restrained y(t)')
stairs(outd.simout.z_t_.Time, outd.simout.z_t_.Data, '--', DisplayName='z(t)')

hold off

legend(gca)

subplot(3, 1, 2)

hold on

stairs(outd.simout.u_t_.Time, outd.simout.u_t_.Data, DisplayName='DeadBeat u(t)')
stairs(outdr.simout.u_t_.Time, outdr.simout.u_t_.Data, DisplayName='DeadBeat Restrained u(t)')
stairs(outd.simout.z_t_.Time, outd.simout.z_t_.Data, '--', DisplayName='z(t)')

hold off

legend(gca)

subplot(3, 1, 3)

hold on

stairs(outd.simout.e_t_.Time, outd.simout.e_t_.Data, DisplayName="DeadBeat e(t)")
stairs(outdr.simout.e_t_.Time, outdr.simout.e_t_.Data, DisplayName="DeadBeat Restrained e(t)")
stairs(outd.simout.z_t_.Time, outd.simout.z_t_.Data, '--', DisplayName='z(t)')

hold off

legend(gca)