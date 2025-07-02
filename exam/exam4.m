sys = tf([2], [0.01, 1, 0, 0]);

sys_d = c2d(sys, 0.01);

[Q, P, Ts] = PolesPlacement(sys_d, [0.999, 0.988, 0.968, 0.703, 0.68, 0.388]);

T = [0, 60];
u_control = 1;
z = 0.1;
disorder_times = [15, 25];

discrete_regulator = tf(Q, P, Ts);

outdpp = sim("DiscreteSystem.slx", StartTime=num2str(T(1)), StopTime=num2str(T(2)), SolverType='Fixed-step', FixedStep=num2str(Ts));

figure(Name="Ball on plane")

subplot(2, 1, 1)

hold on

stairs(outdpp.simout.y_t_.Time, outdpp.simout.y_t_.Data, DisplayName="Ball PP y(k)")
stairs(outdpp.simout.z_t_.Time, outdpp.simout.z_t_.Data, DisplayName="z(k)")

hold off

legend(gca)

subplot(2, 1, 2)

hold on

stairs(outdpp.simout.u_t_.Time, outdpp.simout.u_t_.Data, DisplayName="Ball PP u(k)")
stairs(outdpp.simout.z_t_.Time, outdpp.simout.z_t_.Data, DisplayName="z(k)")

hold off

legend(gca)