function [x, y] = rk(f, T, PP, h)
    x = [T(1) : h : T(2)];
    y(1, :) = PP;
    
    for i = 1 : (length(x) - 1)
        K1 = h .* f(x(i), y(i, :)); 
        K2 = h .* f(x(i) + h ./ 2, y(i, :) + K1' ./ 2);
        K3 = h .* f(x(i) + h ./ 2, y(i, :) + K2' ./ 2);
        K4 = h .*f(x(i) + h, y(i, :) + K3');
    
        y(i + 1, :) = y(i, :) + (K1' + 2 * (K2' + K3') + K4') ./ 6;
    end
end