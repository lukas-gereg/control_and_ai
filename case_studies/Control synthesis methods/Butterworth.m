function [P, I, D] = Butterworth(sys, type)
    lookUpTable = {[1, 1], [1, 1.4142, 1], [1, 2, 2, 1], [1, 2.6132, 3.4143, 2.6132, 1], [1, 3.2360, 5.2359, 5.2359, 3.2360, 1]};
    
    [P, I, D] = StandardShapes(sys, type, lookUpTable);
end