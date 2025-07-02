function [Q, P, Ts] = PolesPlacement(sys, poles)
    Ts = sys.Ts;
    z = sym("z");

    num = cell2mat(sys.Numerator);
    denum = cell2mat(sys.Denominator);

    num = num / denum(1);
    denum = denum / denum(1);
    
    order = length(num(cumsum(num, 2) > 0));
    
    regulatorP = sym("p", [order, 1]);
    regulatorQ = sym("q", [(order + 1) 1]);
    
    regulatorNumEq = poly2sym(regulatorQ, z);
    regulatorDenumEq = (poly2sym(regulatorP, z) + z ^ (order));
    
    systemNumEq = poly2sym(num, z);
    systemDenumEq = poly2sym(denum, z);

    controlEquation = regulatorDenumEq * systemDenumEq + regulatorNumEq * systemNumEq;
    [charCoeffs, ~] = coeffs(controlEquation, z);

    if ~(length(poles) + 1 == length(charCoeffs))
        error("Incorrect amount of poles provided.");
    end
    
    desiredCoeffs = poly(poles);
    equations = sym("eqs", [length(charCoeffs) + 1, 1]);

    for i = [1: length(charCoeffs)]
        equations(i) = (charCoeffs(i) == desiredCoeffs(i));
    end
    equations(end) = (sum(regulatorP) == -1);
    
    results = solve(equations, [regulatorQ(:); regulatorP(:)]);
    results = struct2array(results);
    
    Q = double(reshape(results(1:numel(regulatorQ(:))), size(regulatorQ(:))))';
    P = reshape(results(numel(regulatorQ(:)) + 1: end), size(regulatorP(:)));
    P = double([1; P(:)])';
end