function mpc_tcp()
    %% MATLAB NMPC Lane Following Controller (TCP transport)
    % Packet layout is unchanged from the UDP version:
    %   13 (State) + 360 (Lidar) + 320 (Lane Horizons) + 5 (CTE Horizon)
    %   = 698 doubles = 5584 bytes, little-endian
    %
    % TCP is a stream, so packets are framed by their fixed size:
    %   controller -> bridge: 16 bytes  (steer, speed)
    %   bridge -> controller: 5584 bytes (sensor frame)
    %
    % Run tools/bridge.py first, then start this file. The bridge listens on
    % TCP 4248 by default and serves the browser build on http://localhost:8080.

    % 1. INITIALIZATION & CLEANUP
    bridgeIP = "127.0.0.1";
    bridgePort = 4248;
    expectedDoubles = 698; % 13 State + 360 Lidar + 320 Lane (5x64) + 5 CTE
    expectedBytes = expectedDoubles * 8;

    t = tcpclient(bridgeIP, bridgePort, "Timeout", 30);
    t.ByteOrder = "little-endian";

    % Setup Visualization
    figure('Name', 'NMPC Lane Following Monitor (TCP)', 'NumberTitle', 'off', ...
        'Position', [100 100 1200 800]);

    % Subplot 1: Polar LiDAR
    subplot(2,2,[1 3]);
    theta = deg2rad(0:359);
    hPolar = polarplot(theta, zeros(1, 360), 'LineWidth', 1.5, 'Color', 'b');
    title('LiDAR 360° Scan');
    rlim([0 50]); grid on;

    % Subplot 2: Multi-Horizon Lane Intensity
    subplot(2,2,2);
    hold on; colors = jet(5); hLanes = gobjects(5,1);
    for i=1:5
        hLanes(i) = plot(linspace(-4, 4, 64), zeros(1, 64), 'LineWidth', 1.5, ...
            'Color', colors(i,:), 'DisplayName', sprintf('H=%dm', (i-1)*5));
    end
    title('Ground Lane Patterns (Multi-Horizon)');
    xlabel('Lateral Offset (m)'); ylabel('Intensity');
    ylim([-0.1 1.1]); legend('Location', 'best'); grid on;

    % Subplot 3: NMPC CTE Horizon
    subplot(2,2,4);
    cte_distances = [0 2.5 5 7.5 10];
    hCTE = plot(cte_distances, zeros(1, 5), 'r-o', 'LineWidth', 2, 'MarkerFaceColor', 'r');
    yline(0, 'k--', 'Center Line', 'LineWidth', 1.5);
    title('Look-ahead CTE & NMPC Tracking');
    xlabel('Look-ahead Distance (m)'); ylabel('CTE (m)');
    ylim([-3 3]); grid on;

    fprintf("====================================================\n");
    fprintf("MATLAB NMPC Lane Following Active (TCP)\n");
    fprintf("Bridge: %s:%d\n", bridgeIP, bridgePort);
    fprintf("====================================================\n\n");

    try
        last_steer = 0.0;
        speed_cmd = 15.0;

        while true
            % Send control to the bridge (and therefore to the browser).
            write(t, [last_steer, speed_cmd], "double");

            % Read sensor data.
            [has_data, state, laser, lane_horizons, cte_horizon] = ...
                read_tcp_sensors(t, expectedBytes, expectedDoubles);

            if has_data
                % --- SNAP TO CURRENT LANE ---
                current_lane_offset = round(cte_horizon(1) / 3.5) * 3.5;
                snapped_horizon = cte_horizon - current_lane_offset;

                % --- NMPC LANE FOLLOWING ---
                last_steer = lane_following_nmpc(state, snapped_horizon, speed_cmd, last_steer);

                % --- VISUALIZER UPDATES ---
                set(hPolar, 'YData', laser);
                for i=1:5
                    set(hLanes(i), 'YData', lane_horizons(i,:));
                end
                set(hCTE, 'YData', snapped_horizon);
                drawnow limitrate;
            end
        end
    catch ME
        fprintf("\n[STOPPED]: %s\n", ME.message);
        clear t;
    end
end

function [has_data, state, laser, lane_horizons, cte_horizon] = ...
        read_tcp_sensors(t, expectedBytes, expectedDoubles)

    has_data = false;
    state = []; laser = []; lane_horizons = []; cte_horizon = [];
    all_data = [];

    % Use the newest packet: drain every complete frame already buffered.
    while t.NumBytesAvailable >= expectedBytes
        all_data = read(t, expectedDoubles, "double");
        has_data = true;
    end

    % Nothing buffered: block for the next full frame (or until the timeout).
    if ~has_data
        all_data = read(t, expectedDoubles, "double");
        has_data = true;

        % A second frame may have arrived while we were blocked.
        while t.NumBytesAvailable >= expectedBytes
            all_data = read(t, expectedDoubles, "double");
        end
    end

    if has_data && ~any(isnan(all_data))
        state         = all_data(1:13);
        laser         = all_data(14:373);
        lane_horizons = reshape(all_data(374:693), [64, 5])'; % 5 rows, 64 cols
        cte_horizon   = all_data(694:698);
    else
        has_data = false;
    end
end

function opt_steer = lane_following_nmpc(state, cte_horizon, v, last_steer)
    % NMPC Formulation over 5 steps
    % Godot car physics: Unicycle model where 'steer' = yaw rate (omega) rad/s
    N = 5;
    horizon_step_m = 2.5;
    dt = horizon_step_m / v;
    lane_width = 3.5;
    vehicle_width = 2.0;
    wheel_margin = 0.10;
    max_center_error = (lane_width - vehicle_width) / 2.0 - wheel_margin;

    obj_fun = @(w) compute_nmpc_cost(w, cte_horizon, dt, v, last_steer, max_center_error);

    lb = -1.5 * ones(N, 1);
    ub =  1.5 * ones(N, 1);
    w0 = last_steer * ones(N, 1);

    options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp');
    w_opt = fmincon(obj_fun, w0, [], [], [], [], lb, ub, [], options);

    opt_steer = w_opt(1);
end

function cost = compute_nmpc_cost(w, cte_ref, dt, v, last_steer, max_center_error)
    N = length(w);
    cost = 0;

    y = 0;
    theta = 0;

    W_cte = 100.0;
    W_u = 5.0;
    W_du = 50.0;
    W_edge = 1200.0;

    prev_w = last_steer;

    for k = 1:N
        theta = theta + w(k) * dt;
        y = y + v * sin(theta) * dt;

        err = y - cte_ref(k);
        edge_excess = max(0.0, abs(err) - max_center_error);

        cost = cost + W_cte * (err^2) + W_u * (w(k)^2) ...
            + W_du * ((w(k) - prev_w)^2) + W_edge * (edge_excess^2);
        prev_w = w(k);
    end
end
