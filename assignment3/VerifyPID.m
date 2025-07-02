function [isStable, varargout] = VerifyPID(process, P, I, D)
    arguments
        process tf
        P double {mustBePositive}
        I double {mustBeNonnegative} = 0
        D double {mustBeNonnegative} = 0
    end
    [num, den] = tfdata(process, 'v');
    
    syms s
    proc = poly2sym(num, s)/poly2sym(den, s);
    reg = poly2sym([D, P, I], s)/s;
    
    CHR = 1 + proc*reg == 0;
    roots = double(vpasolve(CHR, s));
    positiveRoots = real(roots) >= 0;
    isStable = ~any(positiveRoots);

    if nargout == 2
        varargout{1} = roots;
    end
end