function [P, I, D] = ZieglerNicholsMichajlov(sys, type)
    % Only used for stable systems
    % Cannot be used for stable 2nd order differential equations
    omega = sym("omega", ["real", "positive"]);
    Pk = sym("Pk","real");

    denum = flip(cell2mat(sys.Denominator), 2);
    num = flip(cell2mat(sys.Numerator), 2);
    num = num(1);
    equation = num * Pk + denum(1);

    for idx = [2: length(denum)]
        equation = equation + (omega * 1i) ^ (idx - 1) * denum(idx);
    end
    
    omegak = solve(imag(equation) == 0);
    Tk = (2 * pi) / omegak;

    P_equation = subs(real(equation) == 0, omega, omegak);
    Pk = solve(P_equation);

    I = 0;
    D = 0;

    if type == "P"
        P = 0.5 * Pk;
    elseif type == "PI"
        P = 0.45 * Pk;
        I = P / (0.85 * Tk);
    elseif type == "PD"
        P = 0.6 * Pk;
        D = 0.06 * Tk * P;
    elseif type == "PID"
        P = 0.6 * Pk;
        I = P / (0.5 * Tk);
        D = 0.125 * Tk * P;
    end

    P = double(P);
    I = double(I);
    D = double(D);
end