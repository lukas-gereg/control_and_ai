function [P, S, D] = PID2PSDSquares(KP, I, D, Tvz)
    Kp = KP
    Ti = I / Kp
    Td = D * Kp

    P = Kp * (1 + (Td / Tvz));
    S = - Kp * (1 + 2 * (Td / Tvz) - (Tvz / Ti));
    D = Kp * (Td / Tvz);
end