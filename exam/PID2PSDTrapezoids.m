function [P, S, D] = PID2PSDTrapezoids(KP, I, D, Tvz)
    Kp = KP;
    Ti = Kp / I;
    Td = D / Kp;

    P = Kp * (1 + (Tvz / (2 * Ti)) + (Td / Tvz));
    S = - Kp * (1 + 2 * (Td / Tvz) - (Tvz / (2 * Ti)));
    D = Kp* (Td / Tvz);
end