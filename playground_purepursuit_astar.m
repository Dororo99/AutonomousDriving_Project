%{
function
    - Astar()
    - astar_search()
    - heuristic()
    - get_neighbors()
map
    - map_demo_1.csv
%}

function main()
    % 맵 불러오기
    map_csv = './map_demo_1.csv';
    % map_csv = './map_box_enclosure.csv'
    matrix = csvread(map_csv);
    matrix = expand_obstacles(matrix);  % 안전 여유를 고려한 맵으로 변환

    % 시작점과 목표점 설정 (행, 열 기준)
    % start = [5, 5];
    % goal = [95, 95];
    start = [5,95];
    goal = [95,5];
    plot(5, 95, 'bo');
    text(5,97, 'Goal');
    plot(95, 5, 'ro');
    text(80, 5, 'Start');

    % A* 실행 및 시각화
    [path, x_path, y_path] = Astar(matrix, start, goal);

    % x_path, y_path를 사용하는 컨트롤러 실행
    % path = astar_search(matrix, start, goal);
    run_controller(x_path, y_path);
end

function [path, x_path, y_path] = Astar(map, start, goal)
    path = astar_search(map, start, goal);

     % X, Y 경로 분리
    y_path = path(:,1);  % row -> Y
    x_path = path(:,2);  % col -> X

    % 뒤집기
    x_path = flip(x_path);
    y_path = flip(y_path);

    % 경로 시각화
    figure;
    imagesc(map);
    colormap(gray);
    axis equal tight;
    hold on;

    % 시작점, 목표점 표시
    plot(start(1),start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    text(start(1),start(2)+4, 'Start');
    plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    text(goal(1),goal(2)+4, 'Goal');

    % 경로 표시
    if ~isempty(path)
        for i = 1:size(path,1)
            plot(path(i,2), path(i,1), 'b.', 'MarkerSize', 15);
        end
    else
        disp('경로를 찾을 수 없습니다.');
    end

    hold off;
    title('A* Path Planning');
end

% Astar search
function path = astar_search(map, start, goal)
    [rows, cols] = size(map);
    visited = false(rows, cols);
    cost = Inf(rows, cols);
    parent = zeros(rows, cols, 2);

    cost(start(1), start(2)) = 0;
    h = heuristic(start, goal);
    open = [start, h];

    while ~isempty(open)
        [~, idx] = min(open(:, 3));
        current = open(idx, 1:2);
        open(idx,:) = [];

        if all(current == goal)
            break;
        end

        visited(current(1), current(2)) = true;
        neighbors = get_neighbors(current, rows, cols);

        for i = 1:size(neighbors, 1)
            r = neighbors(i,1);
            c = neighbors(i,2);

            if visited(r,c) || map(r,c) == 1
                continue;
            end

            % 기존의 path planning
            step = [r - current(1), c - current(2)];
            move_cost = norm(step);
            tentative_cost = cost(current(1),current(2)) + move_cost;
            
            % 최소 이동 경로를 발견한 경우
            if tentative_cost < cost(r,c)
                cost(r,c) = tentative_cost;
                parent(r,c,:) = current;
                f = tentative_cost + heuristic([r,c], goal);
                open = [open; r, c, f];
            end
        end
    end

    % 경로 복원
    if all(current == goal)
        path = goal;
        while ~all(path(1,:) == start)
            pr = parent(path(1,1), path(1,2), 1);
            pc = parent(path(1,1), path(1,2), 2);
            path = [ [pr, pc]; path ];
        end
    else
        path = [];
    end
end

function h = heuristic(pos, goal)
    % h_man = abs(pos(1) - goal(1)) + abs(pos(2) - goal(2));  % 맨해튼 거리
    h_eu = norm([pos(1) - goal(1) , pos(2) - goal(2)]);
    h = h_eu;
end

function neighbors = get_neighbors(pos, max_row, max_col)
    directions = [ -1 0; 1 0; 0 -1; 0 1; -1 1; -1 -1; 1 -1; 1 1 ];  % 상하좌우 & 대각선
    neighbors = [];

    for i = 1:8
        r = pos(1) + directions(i,1);
        c = pos(2) + directions(i,2);

        if r >= 1 && r <= max_row && c >= 1 && c <= max_col
            neighbors = [neighbors; r, c];
        end
    end
end

function map_expanded = expand_obstacles(map)
    [rows, cols] = size(map);
    map_expanded = map;

    for r = 1:rows
        for c = 1:cols
            if map(r,c) == 1  % 장애물이면
                for dr = -2:2
                    for dc = -2:2
                        nr = r + dr;
                        nc = c + dc;
                        if nr >= 1 && nr <= rows && nc >= 1 && nc <= cols
                            map_expanded(nr, nc) = 1;
                        end
                    end
                end
            end
        end
    end
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