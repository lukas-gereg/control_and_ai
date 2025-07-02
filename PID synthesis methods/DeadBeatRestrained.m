function [Q, P, Ts] = DeadBeatRestrained(sys, restraint)
    function value = CalculateCoefficient(coeffs, idx, restraint, bsum)
        if idx > length(coeffs)
            coeff1 = 0;
        else 
            coeff1 = coeffs(idx);
        end

        coeff2 = coeffs(idx - 1);

        value = (coeff1 - coeff2) * restraint + (coeff2 / bsum);
    end

    if ~isdt(sys)
        error("System for Dead Beat regulator must be discrete.");
    end

    Ts = sys.Ts;

    num = cell2mat(sys.Numerator);
    denum = cell2mat(sys.Denominator);

    num = num / denum(1);
    denum = denum / denum(1);
    
    denum_order = length(denum);
    num_order = length(num);

    Q = [restraint];
    P = [1];
    
    bsum = sum(num);

    for idx = [2: denum_order + 1]
        Q(end + 1) = CalculateCoefficient(denum, idx, restraint, bsum);

        if (idx <= num_order + 1)
            value = CalculateCoefficient(num, idx, restraint, bsum);
        else
            value = 0;
        end

        P(end + 1) = - value;
    end
end