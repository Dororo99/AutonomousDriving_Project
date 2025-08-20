%{
planning path tester
    - input:
        - path.csv
%}
function main()
    path = csvread("hybrid_astar_path.csv");
    path = flipud(path)

    % 시작점과 목표점 설정 (행, 열 기준)
    start = [5,95,0]; % start
    goal = [95,5,0]; % goal
    plot(5, 95, 'bo');
    text(5,97, 'Goal');
    plot(95, 5, 'ro');
    text(80, 5, 'Start');

    % [path, x_path, y_path] = pre_path
    x_path = path(:,2);
    y_path = path(:,1);

    run_controller(x_path, y_path);
end

% Pure Pursuit Controller
function run_controller(x_path, y_path)
    %% Parameters
    wheel_base = 2.5;         % [m]
    dt = 0.1;                 % [s]
    total_time = 30;          % [s]
    steps = total_time / dt;
    N = length(x_path);

    %% Vehicle initialization
    x = x_path(1);
    y = y_path(1) - 1.0;   % 살짝 뒤쪽
    yaw = atan2(y_path(2) - y_path(1), x_path(2) - x_path(1));
    v = 0;
    target_speed = 30;
    car_length = 8;
    car_width = 4;
    
    % 실제 차 크기
    function h = draw_car(x, y, yaw, L, W, color)
    [x_corners, y_corners] = get_car_corners(x, y, yaw, L, W);
    h = fill(x_corners, y_corners, color, 'EdgeColor', 'k');
    end

    function [x_corners, y_corners] = get_car_corners(x, y, yaw, L, W)
    corner_x = [ L/2  L/2 -L/2 -L/2];
    corner_y = [ W/2 -W/2 -W/2  W/2];

    R = [cos(yaw), -sin(yaw); sin(yaw), cos(yaw)];
    rotated = R * [corner_x; corner_y];
    x_corners = rotated(1, :) + x;
    y_corners = rotated(2, :) + y;
    end

    %% Control gains
    Kp = 1.0; Ki = 0.1; Kd = 0.05;  % PID
    k_stanley = 1.0;
    L_d = 5.0;                      % Look-ahead for Pure Pursuit

    %% PID state
    integral = 0;
    prev_error = 0;

     %% Initial setting (배경 맵, 경로)
    figure;
    hold on;
    imagesc(csvread('./map_demo_1.csv'));  % map 다시 로딩
    colormap(gray); axis equal; axis ij;
    plot(x_path, y_path, 'b--', 'LineWidth', 2);  % 경로

    vehicle_plot = draw_car(x, y, yaw, car_length, car_width, 'r');  % 차량
    traj_plot = plot(x, y, 'r-', 'LineWidth', 4);  % 궤적

    history_x = x;  % 궤적 저장
    history_y = y;
    title('A*, PID, pure persuit & stanley Following Simulation');

    %% Simulation
    for step = 1:steps
        % Find nearest index
        dists = hypot(x - x_path, y - y_path); % hypot: 빗변길이 계산
        [~, idx] = min(dists);

        % Longitudinal PID
        error = target_speed - v;
        integral = integral + error * dt;
        derivative = (error - prev_error) / dt;
        prev_error = error;
        throttle = Kp * error + Ki * integral + Kd * derivative;
        throttle = max(min(throttle, 1), -1);

        % Pure Pursuit
        look_ahead_idx = idx;
        while look_ahead_idx < N
            dx_la = x_path(look_ahead_idx) - x;
            dy_la = y_path(look_ahead_idx) - y;
            dist = hypot(dx_la, dy_la);
            if dist >= L_d
                break;
            end
            look_ahead_idx = look_ahead_idx + 1;
        end
        look_ahead_idx = min(look_ahead_idx, N);
        target_x = x_path(look_ahead_idx);
        target_y = y_path(look_ahead_idx);
        alpha = atan2(target_y - y, target_x - x) - yaw;
        steer = atan2(2 * wheel_base * sin(alpha) / L_d, 1);

        % Limit steer (안전주행 : 조향각 변화를 최대 30도까지만)
        max_steer = deg2rad(45);
        steer = max(min(steer, max_steer), -max_steer);

        % Update vehicle state
        x = x + v * cos(yaw) * dt;
        y = y + v * sin(yaw) * dt;
        yaw = yaw + v / wheel_base * tan(steer) * dt;
        v = v + throttle * dt;
        
        % 궤적 저장
        history_x(end+1) = x;
        history_y(end+1) = y;

        % 차량 위치 및 궤적 갱신
        [x_corners, y_corners] = get_car_corners(x, y, yaw, car_length, car_width);
        set(vehicle_plot, 'XData', x_corners, 'YData', y_corners);
        set(traj_plot, 'XData', history_x, 'YData', history_y);
        pause(0.01);


    end
end
