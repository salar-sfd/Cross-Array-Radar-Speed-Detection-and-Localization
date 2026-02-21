function cars = Highway_Sim(N, h, T, Trec, Tpause)

    if nargin < 1 || isempty(N), N = 30; end
    if nargin < 2 || isempty(h), h = 5; end
    if nargin < 3 || isempty(T), T = 5; end
    if nargin < 4 || isempty(Trec), Trec = 0.1; end
    if nargin < 5 || isempty(Tpause), Tpause = 0.02; end
    
    P = Init_Params(N, h, T, Trec, Tpause);
    S = Init_State(P);
    S = Init_Spawn_All(S, P);
    S = Update_Geometry(S, P);

    cars = Build_Cars_Struct(S, P);

    V = Init_Visuals(P, S);

    gif1 = '../gifs/simulation.gif';
    gif2 = '../gifs/range_doppler.gif';
    if exist(gif1, 'file'), delete(gif1); end
    if exist(gif2, 'file'), delete(gif2); end

    dt = P.Trec;
    steps = max(1, round(P.T / dt));
    
    for k = 1:steps
        
        if ~isgraphics(V.fig1) || ~isgraphics(V.ax)
            break
        end

        S = Step_Dynamics(S, P, dt);
        S = Replace_Out_Of_Bounds(S, P);
        S = Update_Geometry(S, P);

        cars = Build_Cars_Struct(S, P);

        cars_est = Calculate_Pos_Speed(cars, P.start_R, P.end_R, V.fig2);

        V = Update_Truth_Plots(V, S, P);
        disp(cars_est)
        V = Update_Est_Plots(V, cars_est);
        
        drawnow

        Append_Gif(V.ax, gif1, 1, k==1);
        Append_Gif(V.fig2, gif2, 1, k==1);

        pause(P.Tpause)
    end

end

function P = Init_Params(N, h, T, Trec, Tpause)

    P.N = N;
    P.h = h;
    P.T = T;
    P.Trec = Trec;
    P.Tpause = Tpause;

    P.W = 40;
    P.L = 50;
    P.Rmin = 5;
    P.laneW = P.W / 8;

    P.roadHalfW = 20;
    P.viewHalfW = 25;

    P.start_R = 25;
    P.end_R = 40;

    P.laneCenters = (-P.roadHalfW + P.laneW/2) : P.laneW : (P.roadHalfW - P.laneW/2);
    P.laneCenters = P.laneCenters(:);

    P.dirLane = [-ones(4, 1); ones(4, 1)];

    P.triesMax = 40000;

    P.carW = 1.8;
    P.carL = 4.2;

    P.colUp = [0.2, 0.8, 1.0];
    P.colDown = [1.0, 0.35, 0.35];

    P.dashLen = 6;
    P.gapLen = 5;

end

function S = Init_State(P)

    S.lane = zeros(P.N, 1);
    S.x = zeros(P.N, 1);
    S.y = zeros(P.N, 1);
    S.v = zeros(P.N, 1);
    S.a = zeros(P.N, 1);

    S.r = zeros(P.N, 1);
    S.theta = zeros(P.N, 1);
    S.phi = zeros(P.N, 1);
    S.psi = zeros(P.N, 1);

    S.alive = true(P.N, 1);

end

function S = Init_Spawn_All(S, P)

    for i = 1:P.N
        [S.lane(i), S.x(i), S.y(i), S.v(i), S.a(i)] = Spawn(i, P.laneCenters, P.laneW, P.dirLane, P.L, P.Rmin, P.h, S.x, S.y, S.alive, P.triesMax);
    end

end

function S = Update_Geometry(S, P)

    S.r = sqrt(S.x.^2 + S.y.^2 + P.h^2);
    S.theta = asind(P.h ./ S.r);
    S.phi = atan2(S.y, S.x) * 180/pi;
    S.psi = asind(cosd(S.theta) .* cosd(S.phi));

end

function cars = Build_Cars_Struct(S, P)

    cars = struct();
    cars.N = P.N;
    cars.h = P.h;
    cars.Rmin = P.Rmin;
    cars.lane = S.lane;
    cars.dir = sign(S.v);
    cars.x = S.x;
    cars.y = S.y;
    cars.theta = S.theta;
    cars.phi = S.phi;
    cars.psi = S.psi;
    cars.r = S.r;
    cars.v = S.v;
    cars.a = S.a;

end

function V = Init_Visuals(P, S)

    V.fig1 = figure;
    V.fig1.WindowState = 'maximized';
    V.ax = axes('Parent', V.fig1);
    hold(V.ax, 'on')
    axis(V.ax, [-P.viewHalfW, P.viewHalfW, 0, P.L])
    axis(V.ax, 'equal')
    V.ax.XTick = -P.viewHalfW:5:P.viewHalfW;
    V.ax.YTick = 0:5:P.L;
    V.ax.XLim = [-P.viewHalfW, P.viewHalfW];
    V.ax.YLim = [0, P.L];

    Draw_Road(V.ax, P);
    Draw_R_Contours(V.ax, P);
    Draw_Lanes(V.ax, P);

    V.carH = gobjects(P.N, 1);
    V.txtH = gobjects(P.N, 1);

    for i = 1:P.N
        fc = P.colUp;
        if S.v(i) < 0, fc = P.colDown; end

        V.carH(i) = rectangle(V.ax, 'Position', [S.x(i) - P.carW/2, S.y(i) - P.carL/2, P.carW, P.carL], ...
            'Curvature', 0.2, 'FaceColor', fc, 'EdgeColor', 'none');

        V.txtH(i) = text(V.ax, S.x(i), S.y(i), '', ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', ...
            'Color', [1, 1, 1], ...
            'FontSize', 7, ...
            'FontWeight', 'bold', ...
            'Clipping', 'on');
    end

    V.estH = plot(V.ax, NaN, NaN, 'k.', 'MarkerSize', 18);
    V.estTxtH = gobjects(0, 1);

    title(V.ax, sprintf('Highway (N=%d, h=%.2f m)', P.N, P.h))
    xlabel(V.ax, 'x (m)')
    ylabel(V.ax, 'y (m)')
    drawnow

    V.fig2 = figure;
    V.fig2.WindowState = 'maximized';
end

function Append_Gif(axH, gifFile, delay, isFirst)
    fr = getframe(axH);
    im = frame2im(fr);
    [imind, cm] = rgb2ind(im, 256);

    if isFirst
        imwrite(imind, cm, gifFile, 'gif', 'LoopCount', inf, 'DelayTime', delay);
    else
        imwrite(imind, cm, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', delay);
    end
end

function Draw_Road(ax, P)

    patch(ax, [-P.roadHalfW, P.roadHalfW, P.roadHalfW, -P.roadHalfW], ...
              [0, 0, P.L, P.L], ...
              [0.25, 0.25, 0.25], 'EdgeColor', 'none');

end

function Draw_R_Contours(ax, P)

    ang = linspace(0, 2*pi, 800);
    rho1 = sqrt(max(0, P.start_R^2 - P.h^2));
    rho2 = sqrt(max(0, P.end_R^2 - P.h^2));

    p1 = patch(ax, rho1*cos(ang), rho1*sin(ang), [1, 1, 1], 'FaceColor', 'none', 'EdgeColor', [1, 1, 1], 'LineWidth', 1.6, 'EdgeAlpha', 0.18);
    p2 = patch(ax, rho2*cos(ang), rho2*sin(ang), [1, 1, 1], 'FaceColor', 'none', 'EdgeColor', [1, 1, 1], 'LineWidth', 1.6, 'EdgeAlpha', 0.18);

    p1.Clipping = 'on';
    p2.Clipping = 'on';

end

function Draw_Lanes(ax, P)

    yStarts = 0:(P.dashLen + P.gapLen):P.L;

    for xb = (-P.roadHalfW + P.laneW):P.laneW:(P.roadHalfW - P.laneW)
        if abs(xb) < 1e-9
            line(ax, [xb, xb], [0, P.L], 'LineWidth', 2.5, 'Color', [1, 0.9, 0.2]);
        else
            for ys = yStarts
                ye = min(ys + P.dashLen, P.L);
                line(ax, [xb, xb], [ys, ye], 'LineWidth', 1.8, 'Color', [0.95, 0.95, 0.95]);
            end
        end
    end

    line(ax, [-P.roadHalfW, -P.roadHalfW], [0, P.L], 'LineWidth', 3.0, 'Color', [1, 1, 1]);
    line(ax, [ P.roadHalfW,  P.roadHalfW], [0, P.L], 'LineWidth', 3.0, 'Color', [1, 1, 1]);

end

function S = Step_Dynamics(S, P, dt)

    S.y = S.y + S.v * dt + 0.5 * S.a * dt^2;
    S.v = S.v + S.a * dt;

end

function S = Replace_Out_Of_Bounds(S, P)

    out = (S.y < 0) | (S.y > P.L);
    if ~any(out)
        return
    end

    idxOut = find(out);
    for ii = 1:numel(idxOut)
        i = idxOut(ii);
        S.alive(i) = false;
        [S.lane(i), S.x(i), S.y(i), S.v(i), S.a(i)] = Spawn(i, P.laneCenters, P.laneW, P.dirLane, P.L, P.Rmin, P.h, S.x, S.y, S.alive, P.triesMax);
        S.alive(i) = true;
    end

end

function V = Update_Truth_Plots(V, S, P)

    for i = 1:P.N

        if ~isgraphics(V.carH(i), 'rectangle')
            fc = P.colUp;
            if S.v(i) < 0, fc = P.colDown; end
            V.carH(i) = rectangle(V.ax, 'Position', [S.x(i) - P.carW/2, S.y(i) - P.carL/2, P.carW, P.carL], ...
                'Curvature', 0.2, 'FaceColor', fc, 'EdgeColor', 'none');
        else
            V.carH(i).Position = [S.x(i) - P.carW/2, S.y(i) - P.carL/2, P.carW, P.carL];
            if S.v(i) < 0
                V.carH(i).FaceColor = P.colDown;
            else
                V.carH(i).FaceColor = P.colUp;
            end
        end

        if ~isgraphics(V.txtH(i), 'text')
            V.txtH(i) = text(V.ax, S.x(i), S.y(i), '', ...
                'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'middle', ...
                'Color', [1, 1, 1], ...
                'FontSize', 7, ...
                'FontWeight', 'bold', ...
                'Clipping', 'on');
        end

        v_kmh = S.v(i) * 3.6;
        V.txtH(i).Position = [S.x(i), S.y(i), 0];
        V.txtH(i).String = sprintf('%.0f\nkm/h\n\n%.1f\nm', v_kmh, S.r(i));

    end

end

function V = Update_Est_Plots(V, cars_est)

    if ~isempty(V.estTxtH)
        delete(V.estTxtH(isgraphics(V.estTxtH)));
    end
    V.estTxtH = gobjects(0, 1);

    if isstruct(cars_est) && isfield(cars_est, 'x') && ~isempty(cars_est.x)

        set(V.estH, 'XData', cars_est.x, 'YData', cars_est.y);

        Kest = numel(cars_est.x);
        V.estTxtH = gobjects(Kest, 1);

        for kk = 1:Kest
            s = '';
            if isfield(cars_est, 'v') && numel(cars_est.v) >= kk
                s = sprintf('%.0f km/h', cars_est.v(kk));
            end
            V.estTxtH(kk) = text(V.ax, cars_est.x(kk) + 0.35, cars_est.y(kk) + 0.35, s, ...
                'Color', [0, 0, 0], 'FontSize', 7, 'FontWeight', 'bold', ...
                'BackgroundColor', [1, 1, 1], 'Margin', 1, ...
                'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', ...
                'Clipping', 'on');
        end

    else
        set(V.estH, 'XData', NaN, 'YData', NaN);
    end

end

function [lane_i, x_i, y_i, v_i, a_i] = Spawn(idx, laneCenters, laneW, dirLane, L, Rmin, h, xAll, yAll, aliveMask, triesMax)
    
    Wmin = -20 + 0.2;
    Wmax = 20 - 0.2;
    
    for t = 1:triesMax
        lane_i = randi(8);
        x_i = laneCenters(lane_i) + (rand - 0.5) * laneW * 0.6;
        x_i = min(max(x_i, Wmin), Wmax);
    
        dir = dirLane(lane_i);
    
        if dir > 0
            y_i = rand * (0.25 * L);
        else
            y_i = (0.75 * L) + rand * (0.25 * L);
        end
    
        if any(aliveMask)
            d = hypot(xAll(aliveMask) - x_i, yAll(aliveMask) - y_i);
            if any(d < Rmin), continue; end
        end
    
        vMag = (60 + (300 - 60) * rand) * (1000/3600);
        v_i = dir * vMag;
    
        a_i = -2 + 4 * rand;
        if dir < 0, a_i = -a_i; end
    
        return
    end
    
    error("Spawn failed for car %d. Try smaller N or smaller Rmin.", idx);

end