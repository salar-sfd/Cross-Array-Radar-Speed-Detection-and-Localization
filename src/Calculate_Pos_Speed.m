function cars_est = Calculate_Pos_Speed(cars, start_R, stop_R, fig)

    if nargin < 4 || isempty(fig)
        doPlot = false;
    else
        doPlot = true;
    end

    P = Cps_InitParams(cars, start_R, stop_R);
    Sig = Cps_BuildSignal(P);
    [Yr, Rx] = Cps_SimulateReceived(cars, P, Sig);
    [COEFF_MEMORY, COEFF_ALL, DOPPLER, RANGE] = Cps_RangeDoppler(Yr, Sig, P, start_R, stop_R);
    [idx_R, idx_D, peakVals] = Cps_FindPeaks(COEFF_ALL, P.signal_power.*1e-10);

    if doPlot
        Cps_PlotRD(fig, DOPPLER, RANGE, COEFF_ALL, idx_R, idx_D, peakVals);
    end

    cars_est = Cps_Estimate(cars, P, Rx, COEFF_MEMORY, DOPPLER, RANGE, idx_R, idx_D);

end

function P = Cps_InitParams(cars, start_R, stop_R)

    P.c = 3e8;
    P.fc = 2e9;
    P.lambda = P.c / P.fc;
    P.k = 2 * pi / P.lambda;

    P.fs = 100e6;
    P.ts = 1 / P.fs;

    P.PRI = 4e-4;
    P.PRF = 1 / P.PRI;

    P.tau = P.PRI;
    P.BW = 20e6;

    P.Trec = 0.1;
    P.delta_f = 1 / P.Trec;

    P.N_tau = round(P.tau / P.ts);
    P.N_pulse = round(P.Trec / P.PRI);

    P.signal_power = 1e-2;
    P.noise_power = (1.38e-23)*(323.15)*P.fs*(10^(5/10)); %K*T*B*F
    P.rcs = 10;
    P.system_loss = 10;
    P.G = 10;

    P.t = 0:P.ts:P.Trec-P.ts;

    P.h = cars.h;
    P.theta0 = 90 - atand((start_R+stop_R)/(2*P.h));
end

function Sig = Cps_BuildSignal(P)

    Sig.sl_PRI = Generate_Signal(P.ts, P.tau, P.PRI, 'chirp', -P.BW/2, P.BW/2);
    Sig.sl = repmat(Sig.sl_PRI, 1, P.N_pulse);

end

function [Yr, Rx] = Cps_SimulateReceived(cars, P, Sig)

    td = 2 * (cars.r) / P.c;
    vr = cars.v .* sind(cars.phi) .* cosd(cars.theta);
    fd = 2 * vr / P.lambda;
    
    % Transmitter
    Pt = P.signal_power;
    Tx = load("../conf/Tx_Array_Properties.mat");

    Yt = zeros(cars.N, length(P.t));
    for n = 1:cars.N
        at = Tx.w .* exp(-1j * P.k * Tx.d * sind(cars.theta(n)));
        Yt(n, :) = sum(at) .* Sig.sl;
    end

    % Receiver
    Pr = (Pt * (P.G)^2 * P.lambda^2 * P.rcs) ./ ((4*pi)^3 * (cars.r).^4 * P.system_loss);
    alpha = sqrt(Pr).*exp(1j*2*pi*rand(length(td), 1));
    Rx = load("../conf/Tx_Array_Properties.mat");

    M = length(Rx.d);
    Yr = zeros(M, length(P.t));

    for n = 1:cars.N
        ar = Rx.w .* exp(-1j * P.k * Rx.d * sind(cars.psi(n)));
        yr = (alpha(n) .* exp(1j * 2*pi * fd(n) * P.t) .* circshift(Yt(n, :), round(td(n) / P.ts), 2));
        Yr = Yr + ar * yr;
    end
    Nr = (randn(M, length(yr)) + 1j*randn(M, length(yr))).*sqrt(P.noise_power/2);
    Yr = Yr + Rx.w.*Nr;
end

function [COEFF_MEMORY, COEFF_ALL, DOPPLER, RANGE] = Cps_RangeDoppler(Yr, Sig, P, start_R, stop_R)

    RxN = size(Yr, 1);

    range_mask = round(start_R / (P.ts * P.c / 2)) : round(stop_R / (P.ts * P.c / 2));

    COEFF_MEMORY = [];
    DOPPLER = [];
    RANGE = [];

    for m = 1:RxN
        [COEFF, DOPPLER, RANGE] = Range_Doppler_Process(Yr(m, :), Sig.sl, P.ts, P.PRI, P.c);
        if m == 1
            range_mask = max(1, range_mask(1)) : min(numel(RANGE), range_mask(end));
            COEFF_MEMORY = zeros(numel(range_mask), numel(DOPPLER), RxN);
        end
        COEFF_MEMORY(:, :, m) = COEFF(range_mask, :);
    end

    RANGE = RANGE(range_mask);
    COEFF_ALL = mean(abs(COEFF_MEMORY).^2, 3);

end

function [idx_R, idx_D, peakVals] = Cps_FindPeaks(C, thr)

    nbR = 1;
    nbD = 1;
    Cpad = padarray(C, [nbR, nbD], -inf);

    isPeak = true(size(C));
    for dr = -nbR:nbR
        for dd = -nbD:nbD
            if dr == 0 && dd == 0, continue; end
            neigh = Cpad((1 + nbR + dr):(nbR + dr + size(C, 1)), (1 + nbD + dd):(nbD + dd + size(C, 2)));
            isPeak = isPeak & (C > neigh);
        end
    end

    [idx_R, idx_D] = find(isPeak & (C > thr));
    peakVals = C(sub2ind(size(C), idx_R, idx_D));

end

function Cps_PlotRD(fig, DOPPLER, RANGE, COEFF_ALL, idx_R, idx_D, peakVals)

    figure(fig);

    subplot(1, 2, 1)
    mesh(DOPPLER, RANGE, COEFF_ALL)
    ylabel('Range')
    xlabel('Doppler')
    hold on
    plot3(DOPPLER(idx_D), RANGE(idx_R), peakVals, 'r.', 'MarkerSize', 18)
    for kk = 1:numel(idx_R)
        text(DOPPLER(idx_D(kk)), RANGE(idx_R(kk)), peakVals(kk), sprintf('%.2e', peakVals(kk)), ...
            'Color', 'r', 'FontSize', 8, 'FontWeight', 'bold', 'VerticalAlignment', 'bottom', 'Clipping', 'on');
    end
    hold off

    subplot(1, 2, 2)
    imagesc(DOPPLER, RANGE, COEFF_ALL)
    axis xy
    ylabel('Range')
    xlabel('Doppler')
    hold on
    plot(DOPPLER(idx_D), RANGE(idx_R), 'r.', 'MarkerSize', 18)
    for kk = 1:numel(idx_R)
        text(DOPPLER(idx_D(kk)), RANGE(idx_R(kk)), sprintf('%.2e', peakVals(kk)), ...
            'Color', 'r', 'FontSize', 8, 'FontWeight', 'bold', 'VerticalAlignment', 'bottom', 'Clipping', 'on');
    end
    hold off

    sgtitle("Range-Doppler Matrix Magnitude")

end

function cars_est = Cps_Estimate(cars, P, Rx, COEFF_MEMORY, DOPPLER, RANGE, idx_R, idx_D)

    K = numel(idx_R);

    cars_est = struct();
    cars_est.N = K;
    cars_est.h = cars.h;
    cars_est.r = [];
    cars_est.x = [];
    cars_est.y = [];
    cars_est.v = [];

    if K == 0
        return
    end

    delta_psi = 0.1;
    psiGrid = -90:delta_psi:90;
    A = Rx.w .* exp(-1j * P.k * Rx.d * sind(psiGrid));

    theta_est = zeros(K, 1);
    psi_est = zeros(K, 1);

    for kk = 1:K
        yv = squeeze(COEFF_MEMORY(idx_R(kk), idx_D(kk), :));
        idx_A = MP(yv(:), A, 1);

        u = cars.h / RANGE(idx_R(kk));
        u = max(-1, min(1, u));
        theta_est(kk) = asind(u);

        psi_est(kk) = psiGrid(idx_A);
    end

    cars_est.r = RANGE(idx_R);
    cars_est.x = cars_est.r .* sind(psi_est) .* cosd(theta_est);
    cars_est.y = cars_est.r .* cosd(psi_est) .* cosd(theta_est);

    vr_kmh = 3.6 * DOPPLER(idx_D) * P.lambda / 2;
    cars_est.v = vr_kmh ./ max(1e-3, cosd(psi_est) .* cosd(theta_est));
    cars_est.psi = psi_est;
    cars_est.theta = theta_est;
end