function y = prbsRandomAmplitude(t, base_amplitude, amplitude, frequency_change)
    n = length(t);

    prbs = idinput(n, 'prbs', frequency_change, [0 1]);

    changePoints = find(diff([1; prbs]) ~= 0);
    changePoints = [changePoints; n+1];

    y = zeros(n, 1);
    for i = 1:length(changePoints)-1
        idx = changePoints(i):(changePoints(i+1)-1);
        amp = randn() * (amplitude(2) - amplitude(1)) + amplitude(1);
        amp = max(min(amp, amplitude(2) * 2), amplitude(1) * 2);
        randAmp = amp + base_amplitude;
        y(idx) = randAmp;
    end
end