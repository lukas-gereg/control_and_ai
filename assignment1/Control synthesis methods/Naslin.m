function [P, I, D] = Naslin(sys, type, alpha)
    numerator = flip(cell2mat(sys.Numerator), 2);
    denumerator = flip(cell2mat(sys.Denominator), 2);
    
    denumerator = denumerator / numerator(1);

    if contains(type, "D")
        D = ((denumerator(3) ^ 2) / (alpha * denumerator(4))) - denumerator(2);
    else
        D = 0;
    end

    P = ((denumerator(2) + D) ^ 2 / (alpha * denumerator(3))) - denumerator(1);

    if contains(type, "I")
        I = ((denumerator(1) + P) ^ 2 / (alpha * (denumerator(2) + D)));
    else
        I = 0;
    end
end