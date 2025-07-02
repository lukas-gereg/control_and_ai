function [isStable, varargout] = VerifyPID(process, P, I, D)
%   VerifyPID - Closed-loop system stability check
%
%   Function checks stability of continuous closed-loop system containing
%   process (in transfer function form) and continuous P(ID) regulator
%
%   Syntax
%       isStable = VerifyPID(process, P, I, D);
%       [isStable, roots] = VerifyPID(process, P, I, D);
%
%   Input Arguments
%       process - transfer function of process/plant
%         tf model object
%       P - Proportional gain of regulator
%         positive scalar
%       I - Interal gain of regulator        (optional)
%         non-negative scalar
%       D - Derivative gain of regulator     (optional)
%         non-negative scalar
%
%   Output arguments
%       isStable - logical 1 if closed-loop system is stable, otherwise 0
%       roots - roots of characteristic equation of closed-loop system
    

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