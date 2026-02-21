function [COEFFICIENTS, DOPPLER, RANGE, freq, t] = ...
    Range_Doppler_Process(y, sl, ts, PRI, C)

    y  = y(:);
    sl = sl(:);

    fs = 1 / ts;

    N_PRI = round(PRI / ts);

    N_sample = numel(y);
    N_pulse = floor(N_sample / N_PRI);

    y = y(1:N_pulse * N_PRI);
    N_sample = numel(y);

    Trec = N_pulse * PRI;
    deltaf = 1 / Trec;

    t = (0:N_sample-1).' * ts;

    freq = (-N_PRI/2:N_PRI/2-1).' * (fs / N_PRI);

    SL_PRI = sl(1:N_PRI);
    SL_PRI = SL_PRI(:);

    y_mat = reshape(y, N_PRI, N_pulse);

    Y = fft(y_mat, N_PRI, 1);
    SL_PRIf = fft(SL_PRI, N_PRI, 1);

    Ymf = Y .* conj(SL_PRIf);

    rng_prof = ifft(Ymf, N_PRI, 1);

    COEFFICIENTS = fftshift(fft(rng_prof, N_pulse, 2), 2);

    DOPPLER = (-floor(N_pulse/2):ceil(N_pulse/2)-1).' * deltaf;

    RANGE = (C / 2) * (0:N_PRI-1).' * ts;

    COEFFICIENTS = COEFFICIENTS / (N_sample);

end