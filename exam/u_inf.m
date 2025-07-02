function u_inf = u_inf(Gp, Gr)
    if Gp.Ts == 0 && Gp.Ts > 0
        Gp = c2d(Gp, Gr.Ts);
    elseif Gp.Ts > 0 && Gr.Ts == 0
        Gr = c2d(Gr, Gp.Ts);
    elseif Gp.Ts ~= Gr.Ts
        error('Plant Ts = %g, controller Ts = %g: incompatible sample times.', Gp.Ts, Gr.Ts);
    end
    
    U = feedback(Gr, Gp);
    U_clean = minreal(U, 1e-6);

    u_inf = dcgain(U_clean);
end