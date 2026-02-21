function Plot_Cross_Array(Tx_d, Tx_w, Rx_d, Rx_w, dmax)
    Tx_d = Tx_d(:);  Tx_w = Tx_w(:);
    Rx_d = Rx_d(:);  Rx_w = Rx_w(:);

    if numel(Tx_d) ~= numel(Tx_w), error('Tx_d and Tx_w must have same length'); end
    if numel(Rx_d) ~= numel(Rx_w), error('Rx_d and Rx_w must have same length'); end

    if min(Tx_d) >= 0 && max(Tx_d) <= dmax, Tx_d = Tx_d - dmax/2; end
    if min(Rx_d) >= 0 && max(Rx_d) <= dmax, Rx_d = Rx_d - dmax/2; end

    mag = abs([Tx_w; Rx_w]);
    mag = (mag - min(mag)) / (max(mag) - min(mag) + eps);
    bright = mag;

    fig = figure('Color','w'); hold on; axis equal; grid on;

    plot([-dmax/2 dmax/2],[0 0],'k-','LineWidth',2);
    plot([0 0],[-dmax/2 dmax/2],'k-','LineWidth',2);

    bt = bright(1:numel(Tx_w));
    br = bright(numel(Tx_w)+1:end);

    txRGB = [ones(numel(bt),1), 1-bt, 1-bt];
    rxRGB = [1-br, 1-br, ones(numel(br),1)];

    diam = 0.08;
    r = diam/2;

    for i = 1:numel(Tx_d)
        drawCircle(0, Tx_d(i), r, txRGB(i,:), [0 0 0], 1);
    end

    for i = 1:numel(Rx_d)
        drawCircle(Rx_d(i), 0, r, rxRGB(i,:), [0 0 0], 1);
    end

    xlabel('x (m)  [Rx]');
    ylabel('y (m)  [Tx]');
end

function drawCircle(x0, y0, r, faceColor, edgeColor, edgeWidth)
    t = linspace(0, 2*pi, 100);
    x = x0 + r*cos(t);
    y = y0 + r*sin(t);
    patch(x, y, faceColor, 'EdgeColor', edgeColor, 'LineWidth', edgeWidth);
end