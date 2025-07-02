function [na, nb, nc, nk] = tf2arxorders(G)
  [b, a] = tfdata(G, 'v');
  na = numel(a) - 1;
  
  firstNZ = find(b ~= 0, 1, 'first');
  
  if isempty(firstNZ)
    error('Numerator is all zero!');
  end

  nk = firstNZ - 1;
  nb = numel(b) - nk;
  
  D = round(G.InputDelay / G.Ts);
  nk = nk + D;
  
  nc = na;
end