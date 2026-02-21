function [Dtheta0] = Calculate_Directivity(A, fc, theta0_deg, doPlot)
    c = 3e8;
    lambda = c/fc;
    k = 2*pi/lambda;

    w = A.w(:);      % Nx1
    d = A.d(:);      % Nx1 (positions along array axis, meters)

    theta = (-90:0.1:90) * pi/180;   % radians
    theta0 = theta0_deg * pi/180;

    AF = zeros(size(theta));
    for i = 1:numel(theta)
        AF(i) = sum(w .* exp(-1j*k*d*sin(theta(i))));
    end

    U = abs(AF).^2;  % power pattern (up to a constant)

    Prad_2D = trapz(theta, U);
    D2D = (2*pi) * U / Prad_2D;
    
    if doPlot
        fig = figure;
        plot(theta, D2D);
        grid('on')
        xlabel('Angle (deg)');
        ylabel('Directivity');
    end

    [~, idx] = min(abs(theta - theta0));
    Dtheta0 = D2D(idx);
end