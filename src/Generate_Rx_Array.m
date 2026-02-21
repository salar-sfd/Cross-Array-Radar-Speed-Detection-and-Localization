function [d, w] = Generate_Rx_Array(fc, dmax, type)

    c = 3e8;
    lambda = c/fc;
    k = 2*pi/lambda;

    switch lower(type)
        case 'simp'
            if nargin < 5 || isempty(N)
                N = floor(dmax/(lambda/2));
            end
            d = linspace(0, dmax, N).';
            w = ones(1, length(d));
            w = w/norm(w);
        case 'opt'
            delta_d = lambda/2;
            d_0 = (0:delta_d:dmax).';
            M = floor(dmax/delta_d)+1;
            delta_theta = 0.1;
            theta = -90:delta_theta:90;
            A = exp(-1j*k*d_0*sind(theta));
            a_0 = exp(-1j*k*d_0*sind(0));
            r_0 = a_0'*A/M;
            

            d_grid = 0.01;
            d_p = (0:d_grid:dmax).';
            w_length = length(d_p);
            theta_mask = 4;
            
            A_p = exp(-1j*k*d_p*sind(theta));
            mask = (theta>theta_mask | theta<-theta_mask);
            
            cvx_begin
                variable w(w_length) nonnegative
                variable t nonnegative
                minimize( t )
                subject to
                    abs( w.' * A_p(:,mask) ) <= t;
                    sum(w) == 1;
                    w <= 1;
            cvx_end
            
            w_p = [0; w; 0];
            minPeakHeight   = 1e-3;
            minPeakDistance = d_grid;
            [~, locs_pad] = findpeaks(w_p, ...
                'MinPeakHeight', minPeakHeight, ...
                'MinPeakDistance', minPeakDistance);
            locs = locs_pad - 1;
            locs = locs(locs >= 1 & locs <= length(w_p));
            w_p = w(locs);
            d = d_p(locs);
            w = w_p/abs(sum(w_p));

            fig = figure;
            subplot(2, 1, 1);
            plot(theta, abs(r_0), 'LineWidth', 1);
            hold on
            plot(theta, abs(w.' * A_p(locs, :)), 'LineWidth', 1);
            xlabel("theta")
            ylabel("corr")
            subtitle("Designed Array Correlation with 0 Degree Steering Vector")
            legend(["Base Pattern", "Optimized Pattern"])
            grid on
            
            
            subplot(2, 1, 2);
            idx = round(d_0/d_grid) + 1;
            y_p = zeros(size(d_p));
            y_p(idx) = 1/M;
            stem(d_p, sqrt(y_p), 'LineWidth', 1, 'Marker', '+');
            hold on;
            stem(d, sqrt(w), 'LineWidth', 1, 'Marker', '+');
            xlabel("d")
            xlim([-10*d_grid, dmax+10*d_grid])
            ylabel("weight")
            subtitle(sprintf("Designed Array Weights (Sparse Pattern Active Elements = %d)", sum(w_p~=0)))
            legend(["Base Pattern Weights", "Optimized Pattern Weights"])
            grid on

            w = sqrt(w)/norm(sqrt(w));

        otherwise
            error("Unknown type: %s", type);
    end

    save('../conf/Rx_Array_Properties.mat','d','w');
end
