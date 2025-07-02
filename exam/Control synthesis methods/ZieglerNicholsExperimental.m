function [P, I, D] = ZieglerNicholsExperimental(sys, type, tolerance)
    % Only used for stable systems
    arguments
        sys;
        type;
        tolerance = 0.00025;
    end
    
    discrete_sys = c2d(sys, tolerance * 2);
    
    [y, t] = step(discrete_sys);
    
    d1y = gradient(y, t);
    
    t_infl = interp1(d1y, t, max(d1y));
    y_infl = interp1(t, y, t_infl);
    
    slope  = interp1(t, d1y, t_infl);
    intcpt = y_infl - slope * t_infl;
    tngt = slope * t + intcpt;

    [~, TuIdx] = min(abs(tngt));
    [~, TpIdx] = min(abs(tngt - max(y)));

    denumerator = cell2mat(sys.Denominator);
    numerator = flip(cell2mat(sys.Numerator) / denumerator(end), 2);

    Kp = numerator(1);
    Tu = t(TuIdx);
    Tp = t(TpIdx);
    Tn = Tp - Tu;
    
    I = 0;
    D = 0;
    
    if type == "P"
        P = Tn / (Kp * Tu);
    elseif type == "PI"
        P = (0.9 * Tn) / (Kp * Tu);
        I = P / (3.5 * Tu);
    elseif type == "PD"
        P = (1.2 * Tn) / (Kp * Tu);
        D = 0.25 * Tu * P;
    elseif type == "PID"
        P = (1.25 * Tn) / (Kp * Tu);
        I = P / (2 * Tu);
        D = 0.05 * Tu * P;
    end
    
end