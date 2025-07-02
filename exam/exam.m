a = poly([-3, -3, -3]);

function dxdy = LDS(t, x, u, a)
    dx1dy = x(2);
    dx2dy = x(3);
    dx3dy = (2 * u - a(4) * x(1) - a(3) * x(2) - a(2) * x(3)) / a(1);

    dxdy = [dx1dy; dx2dy; dx3dy];
end

u = 0;
LDSParam = @(t, x) LDS(t, x, u, a);

p = [1; 1; 1];
T = [0, 20];
h = 0.01;
tRange = [T(1): h: T(2)];

function y = analyt(t)
    y = exp(-3.*t) + 4 .* t .* exp(-3.*t) + 8 .* t .^ 2 .* exp(-3.*t);
end

[t_sol, S_sol] = ode45(LDSParam, tRange, p);
out = sim("a1.slx", StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(h));

figure(Name="System")

hold on

plot(out.simout, DisplayName="Simulink y")
plot(t_sol, S_sol(:, 1), DisplayName="ODE y")
plot(tRange, analyt(tRange), DisplayName="Analytical y")

hold off

legend(gca)

A = [ 0      1       0;
      0      0       1;
  -a(4)/a(1)  -a(3)/a(1)  -a(2)/a(1)];

B = [0; 0; 2/a(1)];
C = eye(3);
D = 0;

sys = tf(ss(A, B, C, D));
sys = sys(1);
[PM, IM, DM] = MOM(sys, "PID");

[PG, IG, DG] = GrahamLathrop(sys, "PID");
[PZ, IZ, DZ] = ZieglerNicholsMichajlov(sys, "PID");

PC = PZ;
IC = IZ;
DC = DZ;

y_inf_val = y_inf(sys, tf([DC, PC, IC], [1, 0]));

u_control = 1;
z = 0.1;
disorder_times = [15, 16];

outzn = sim("PIDSystem.slx", StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(h));

PC = PG;
IC = IG;
DC = DG;

outgl = sim("PIDSystem.slx", StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(h));

figure(Name="Ziegler Nichols vs Graham Lathrop PID")

subplot(2, 1, 1)

hold on

plot(outzn.simout.y_t_.Time, outzn.simout.y_t_.Data, DisplayName="Ziegler-Nichols y(t)")
plot(outgl.simout.y_t_.Time, outgl.simout.y_t_.Data, DisplayName="Graham Lathrop y(t)")
stairs(outgl.simout.z_t_.Time, outgl.simout.z_t_.Data, DisplayName="z(t)")

hold off

legend(gca)

subplot(2, 1, 2)

hold on

plot(outzn.simout.u_t_.Time, outzn.simout.u_t_.Data, DisplayName="Ziegler-Nichols u(t)")
plot(outgl.simout.u_t_.Time, outgl.simout.u_t_.Data, DisplayName="Graham Lathrop u(t)")
stairs(outgl.simout.z_t_.Time, outgl.simout.z_t_.Data, DisplayName="z(t)")

hold off

legend(gca)

PC = PG;
IC = IG;
DC = DG;

outmom = sim("PIDSystem.slx", StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(h));


Ts = 0.1;

[PDZ, SDZ, DDZ] = PID2PSDTrapezoids(PZ, IZ, DZ, Ts);

[PDM, SDM, DDM] = PID2PSDTrapezoids(PM, IM, DM, Ts);

discrete_regulator = tf([PDZ, SDZ, DDZ], [1, -1, 0], Ts);

sys_d = c2d(sys, Ts);

outdzn = sim("DiscreteSystem.slx", StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(Ts));

discrete_regulator = tf([PDM, SDM, DDM], [1, -1, 0], Ts);

outdmom = sim("DiscreteSystem.slx", StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(Ts));

figure(Name="MOM PID vs PSD")

subplot(2, 1, 1)

hold on

plot(outmom.simout.y_t_.Time, outmom.simout.y_t_.Data, DisplayName="MOM PID y(t)")
stairs(outdmom.simout.y_t_.Time, outdmom.simout.y_t_.Data, DisplayName="MOM PSD y(k)")
stairs(outmom.simout.z_t_.Time, outmom.simout.z_t_.Data, DisplayName="z(t)")

hold off

legend(gca)

subplot(2, 1, 2)

hold on

plot(outmom.simout.u_t_.Time, outmom.simout.u_t_.Data, DisplayName="MOM PID u(t)")
stairs(outdmom.simout.u_t_.Time, outdmom.simout.u_t_.Data, DisplayName="MOM PSD u(k)")
stairs(outmom.simout.z_t_.Time, outmom.simout.z_t_.Data, DisplayName="z(t)")

hold off

legend(gca)

figure(Name="Ziegler Nichols PID vs PSD")

subplot(2, 1, 1)

hold on

plot(outzn.simout.y_t_.Time, outzn.simout.y_t_.Data, DisplayName="ZN PID y(t)")
stairs(outdzn.simout.y_t_.Time, outdzn.simout.y_t_.Data, DisplayName="ZN PSD y(k)")
stairs(outzn.simout.z_t_.Time, outzn.simout.z_t_.Data, DisplayName="z(t)")

hold off

legend(gca)

subplot(2, 1, 2)

hold on

plot(outzn.simout.u_t_.Time, outzn.simout.u_t_.Data, DisplayName="ZN PID u(t)")
stairs(outdzn.simout.u_t_.Time, outdzn.simout.u_t_.Data, DisplayName="ZN PSD u(k)")
stairs(outzn.simout.z_t_.Time, outzn.simout.z_t_.Data, DisplayName="z(t)")

hold off

legend(gca)
