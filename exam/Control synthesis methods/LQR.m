function [Q, P, K, N] = LQR(sys, Qc, Rc)
    arguments
        sys;
        Qc = 1;
        Rc = 1;
    end
    
    [A, B, C, D] = tf2ss(cell2mat(sys.Numerator), cell2mat(sys.Denominator));

    Q = Qc * (C' * C);
    R = Rc;

    if rank(ctrb(A, B)) < size(A,1)
        error('System is not controllable, LQR not usable.');
    end

    K = lqr(A, B, Q, R);
    
    N = - inv(C * inv(A - B*K) * B);

    Acl = A - B * K;
    Bcl = B * N;

    [Q, P] = ss2tf(Acl, Bcl, C, D);
end

