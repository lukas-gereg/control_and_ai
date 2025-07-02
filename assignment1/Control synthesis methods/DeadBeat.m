function [Q, P,Ts] = DeadBeat(sys)
    if ~isdt(sys)
        error("System for Dead Beat regulator must be discrete.");
    end

    Ts = sys.Ts;

    num = cell2mat(sys.Numerator);
    denum = cell2mat(sys.Denominator);

    num = num / denum(1);
    num = num(2: end);
    denum = denum / denum(1);
    denum = denum(2: end);
    
    num_order = length(num);
    denum_order = length(denum);
    
    q0 = 1 / sum(num);
    Q = [q0, zeros(1, denum_order)];
    P = [1, zeros(1, denum_order)];
    

    for idx = [1: denum_order]
        Q(idx + 1) = denum(idx) * q0;

        if (idx <= num_order)
            pidx = num(idx);
        else
            pidx = 0;
        end

        P(idx + 1) = - pidx * q0;
    end
end