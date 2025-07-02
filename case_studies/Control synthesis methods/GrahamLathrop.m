function [P, I, D] = GrahamLathrop(sys, type)
    lookUpTable = {[1, 1], [1, 1.4, 1], [1, 1.75, 2.15, 1], [1, 2.1, 3.4, 2.7, 1], [1, 2.8, 5, 5.5, 3.4, 1]};
    
    [P, I, D] = StandardShapes(sys, type, lookUpTable);
end