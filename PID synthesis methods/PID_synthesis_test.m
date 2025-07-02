%sys = tf([0.1], [1, 1, 0.2])
%[P, I, D] = MOM(sys, "PI")
%[P, I, D] = Naslin(sys, "PI", 2)
%dsys = c2d(sys, 0.08)
%[P, I, D] = GrahamLathrop(sys, "PI")
%[P, I, D] = Butterworth(sys, "PI")

%sys = tf([100], [0.15, 1, 0]);

sys = tf([3], [27, 27, 9, 1])
%[P, I, D] = ZieglerNicholsExperimental(sys, "PI")
[P, I, D] = ZieglerNicholsMichajlov(sys, "PI")


%dsys = tf([0, 0.1073, 0.055], [1, - 0.8105,  0.1353], 0.1)
%sys = tf([100], [0.15, 1, 0]);
%[P, I, D] = Naslin(sys, "PI", 2)
%dsys = c2d(sys, 0.08)
%[Q, P, Tvz] = DeadBeat(dsys)
%tf(Q, P, Tvz)
%[Q, P, Tvz] = DeadBeatRestrained(dsys, 0.2)
%tf(Q, P, Tvz)
%dsys = tf([0.15, 0.09], [1, -0.97, 0.22], 0.08)
%[Q, P, Tvz] = PolesPlacement(dsys, [0.1, 0.2, 0.3, 0.4])
%tf(Q, P, Tvz)
%roots([-75, -150, -110, -35, -4])

sys = tf([3], [27, 27, 9, 1])

dsys = c2d(sys, 1.5)

%[Q, P, Tvz] = DeadBeat(dsys)
%[Q, P, Tvz] = DeadBeatRestrained(dsys, 3.5)
%a = [[1.73, 0, 0, 1, 0];[1.507, 1.73, 0, -1.5, 1];[0, 1.507, 1.73, 0.58, -1.5];[0, 0, 1.507, 0, 0.58];[0, 0, 0, 1, 1]];
%b = [0.5; -0.23; -0.05; 0.024; -1];

%a\b