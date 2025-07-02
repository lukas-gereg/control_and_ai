function y_inf = y_inf(Gp, Gr)
    if Gp.Ts == 0 && Gp.Ts > 0
        Gp = c2d(Gp, Gr.Ts);
    elseif Gp.Ts > 0 && Gr.Ts == 0
        Gr = c2d(Gr, Gp.Ts);
    elseif Gp.Ts ~= Gr.Ts
        error('Plant Ts = %g, controller Ts = %g: incompatible sample times.', Gp.Ts, Gr.Ts);
    end
    
    L = Gp * Gr;
    T = feedback(L, 1);
    T_clean = minreal(T, 1e-6);
    
    y_inf = dcgain(T_clean);
end
