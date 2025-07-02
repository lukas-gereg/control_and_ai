a = poly([-3, -3, -3]);

A = [ 0      1       0;
      0      0       1;
  -a(4)/a(1)  -a(3)/a(1)  -a(2)/a(1)];

B = [0; 0; 2/a(1)];
C = eye(3);
D = 0;

sys = tf(ss(A, B, C, D));
sys = sys(1);

Ts = 0.5;
T = [0, 20];
u_control = 1;
z = 0.1;
disorder_times = [15, 16];

sys_d = c2d(sys, Ts);

[Q, P] = DeadBeat(sys_d);

discrete_regulator = tf(Q, P, Ts);

y_inf_val_db = y_inf(sys_d, discrete_regulator)
u_inf_val_db = u_inf(sys_d, discrete_regulator)

outdb = sim("DiscreteSystem.slx", StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(Ts));

[Q, P] = DeadBeatRestrained(sys_d, 15);

discrete_regulator = tf(Q, P, Ts);

y_inf_val_dbr = y_inf(sys_d, discrete_regulator)
u_inf_val_dbr = u_inf(sys_d, discrete_regulator)

outdbr = sim("DiscreteSystem.slx", StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(Ts));

figure(Name="DeadBeat vs DeadBeat Restricted")

subplot(2, 1, 1)

hold on

stairs(outdbr.simout.y_t_.Time, outdbr.simout.y_t_.Data, DisplayName="DeadBeat Restricted y(k)")
stairs(outdb.simout.y_t_.Time, outdb.simout.y_t_.Data, DisplayName="DeadBeat y(k)")
stairs(outdb.simout.z_t_.Time, outdb.simout.z_t_.Data, DisplayName="z(k)")

hold off

legend(gca)

subplot(2, 1, 2)


hold on

stairs(outdbr.simout.u_t_.Time, outdbr.simout.u_t_.Data, DisplayName="DeadBeat Restricted u(k)")
stairs(outdb.simout.u_t_.Time, outdb.simout.u_t_.Data, DisplayName="DeadBeat u(k)")
stairs(outdb.simout.z_t_.Time, outdb.simout.z_t_.Data, DisplayName="z(k)")

hold off

legend(gca)