function [P, I, D] = MOM(system, type)
    numerator = cell2mat(system.Numerator);
    denumerator = cell2mat(system.Denominator);

    numerator = flip(numerator / denumerator(end), 2);
    denumerator = flip(denumerator / denumerator(end), 2);
    
    a = zeros(1, 5);

    final_length = length(denumerator);
    
    if final_length > 6
        final_length = 6;
    end

    for i = [2: final_length]
        a(i - 1) = denumerator(i);
    end
    
    A = [a(1), -1, 0;
        a(3), -a(2), a(1);
        a(5), -a(4), a(3)];

    B = [1; (- (a(1) ^ 2) + 2 * a(2)); (a(2) ^ 2 - 2 * a (1) * a(3) + 2 * a(4))];
    B = (1 / (2 * numerator(1))) * B;
    
    to_remove = [];
    
    if ~contains(type, "D")
        to_remove(end + 1) = 3;
    end

    if ~contains(type, "I")
        to_remove(end + 1) = 1;
    end

    nonzero_rows_cols = ones(3, 1);
    nonzero_rows_cols(to_remove) = 0;
    nonzero_rows_cols = nonzero_rows_cols == 1;

    B = B(nonzero_rows_cols);
    A = A(nonzero_rows_cols, nonzero_rows_cols');
    
    sol = zeros(3, 1);
    sol(nonzero_rows_cols) = A \ B;
    
    P = sol(2);
    I = sol(1);
    D = sol(3);
end