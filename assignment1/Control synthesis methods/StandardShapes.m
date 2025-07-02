function [P, I, D] = StandardShapes(sys, type, versionCoefficients)
    numerator = flip(cell2mat(sys.Numerator), 2);
    denumerator = cell2mat(sys.Denominator);
    
    numerator = numerator(1) / denumerator(1);
    denumerator = denumerator / denumerator(1);
    
    [~, order] = size(denumerator);
    
    if contains(type, "I")
        coefficients = cell2mat(versionCoefficients(order));
        coeffLength = length(coefficients);
    else
        coefficients = cell2mat(versionCoefficients(order - 1));
        coeffLength = length(coefficients) + 1;
    end
    
    freeCoefficientsIdx = [2:coeffLength - 2];

    if contains(type, "D")
        freeCoefficientsIdx = freeCoefficientsIdx(freeCoefficientsIdx ~= coeffLength - 2);
    end

    omegaCandidates = zeros(1, 2 * length(freeCoefficientsIdx(mod(freeCoefficientsIdx, 2) == 1)) + length(freeCoefficientsIdx(mod(freeCoefficientsIdx, 2) == 0)));
    numel(1: freeCoefficientsIdx)
    for idx = [1: numel(freeCoefficientsIdx)]
        coeffIdx = freeCoefficientsIdx(idx);

        omegaCandidate = nthroot(denumerator(coeffIdx) / coefficients(coeffIdx), coeffIdx - 1);
        omegaCandidates(idx) = omegaCandidate;
         if mod(coeffIdx, 2) == 1
             omegaCandidates(idx) = -omegaCandidate;
         end
    end

    omegaCandidates = omegaCandidates(omegaCandidates > 0);
    omega = omegaCandidates(1);
    
    P = ((coefficients(coeffLength - 1) * omega ^ (coeffLength - 2)) - (denumerator(coeffLength - 1))) / numerator;

    if contains(type, "I")
        I = (coefficients(coeffLength) * omega ^ (coeffLength - 1)) / numerator;
    else
        I = 0;
    end

    if contains(type, "D")
        D = ((coefficients(coeffLength - 2) * omega ^ (coeffLength - 3)) - (denumerator(coeffLength - 2))) / numerator;
    else
        D = 0;
    end
end