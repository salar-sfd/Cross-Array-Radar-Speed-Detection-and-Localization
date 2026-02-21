function sl_PRI = Generate_Signal(ts, tau, PRI, type, start_f, stop_f)
    if nargin < 4 || isempty(type)
        type = 'rect';
    end

    N_PRI = round(PRI/ts);
    N_tau = round(tau/ts);

    switch lower(type)
        case 'rect'
            sl_PRI = [ones(1, N_tau), zeros(1, N_PRI - N_tau)];

        case 'chirp'
            t = (0:N_tau-1) * ts;
            beta  = start_f;
            alpha = (stop_f - start_f) / (2*tau);
            chirp_sig = exp(1j * 2*pi * (alpha*t.^2 + beta*t));

            sl_PRI = [chirp_sig, zeros(1, N_PRI - N_tau)];
    end

    sl_power = mean(abs(sl_PRI).^2);
    sl_PRI = sl_PRI / sqrt(sl_power);
end
