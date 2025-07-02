function u = generateTestSignal(t, u_base, deviationRange, perturbAmp, perturbFreq)
    n = length(t);
    t = t(:);

    prbs = idinput(n, 'prbs', [0 0.1], [0 1]);
    changePoints = find(diff([1; prbs]) ~= 0);
    changePoints = [changePoints; n+1];

    u = zeros(n, 1);

    for i = 1:length(changePoints)-1
        idx = changePoints(i):(changePoints(i+1)-1);

        t_local = t(idx) - t(idx(1));
        amp = randn() * (deviationRange(2) - deviationRange(1)) + deviationRange(1);
        amp = max(min(amp, deviationRange(2) * 2), deviationRange(1) * 2); 
        
        base = u_base + amp;

        u(idx) = base + (rand() * (perturbAmp(2) - perturbAmp(1)) + perturbAmp(1)) * sin(2 * pi * perturbFreq * t_local);
    end
end