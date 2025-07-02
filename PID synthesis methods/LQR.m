function [P, Q] = LQR(sys, Qc, Rc)
    arguments
        sys;
        Qc = 1;
        Rc = 1;
    end
    [A, B, C, D] = tf2ss(sys);

    Q = Qc * (C' * C);
    R = Rc;

    if rank(ctrb(A, B)) < size(A,1)
        error('System is not controllable, LQR not usable.');
    end

    K = lqr(A, B, Q, R);
    
    N = - dcgain(ss(A - B * K, B, C, D)); 

    Acl = A - B * K;
    Bcl = B * N;

    [P, Q] = ss2tf(Acl, Bcl, C, D);
end

