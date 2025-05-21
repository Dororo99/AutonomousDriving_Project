%{
Hybrid A* Algorithm
--------------------
- Input
    - pose
    - sensor data
- Path
    which includes [x_posistion,y_posistion]
- Parameters
    - v: 1(const)
    - L: 5.0 (차량 축간 거리)
%}

function main()
    % 맵 불러오기
    map_csv = './map_demo_1.csv';
    matrix = csvread(map_csv);
    
    % 맵 반전 확인 (0이 자유 공간, 1이 장애물인지 확인)
    
    matrix_expanded = expand_obstacles(matrix);  % 확장된 맵 생성

    % 시작점과 목표점 설정 (맵 좌표계에 맞게 조정)
    start = [5, 95, 0];  % [행, 열, 방향(라디안)]
    goal = [95, 5, 0];
    
    % 디버깅을 위한 파라미터 출력
    disp(['맵 크기: ', num2str(size(matrix))]);
    disp(['시작점: ', num2str(start)]);
    disp(['목표점: ', num2str(goal)]);
    
    tic;

    % path
    path = hybrid_astar_search(matrix_expanded, start, goal);

    toc;

    % 원본 맵에 경로 시각화
    figure;
    imagesc(matrix);  % 원래 맵 사용
    colormap(gray);
    axis equal tight;
    hold on;

    % 시작점, 목표점 표시
    plot(start(2), start(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    text(start(2), start(1)+4, 'Start');
    plot(goal(2), goal(1), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    text(goal(2), goal(1)+4, 'Goal');

    % 경로 표시
    if ~isempty(path)
        for i = 1:size(path,1)
            plot(path(i,2), path(i,1), 'b.', 'MarkerSize', 10);
        end
        disp(['경로 길이: ', num2str(size(path,1))]);

        writematrix(path, 'hybrid_astar_path.csv');
        disp('경로가 hybrid_astar_path.csv 파일로 저장되었습니다.');
    else
        disp('경로를 찾을 수 없습니다.');
    end

    hold off;
    title('A* Path Planning with Safety Margin');
end

% Hybrid Astar search
function path = hybrid_astar_search(map, start, goal)
    [rows, cols] = size(map);
    visited = false(rows, cols, 72);  % 방향을 고려한 방문 배열 (5도 간격으로 36개 방향)
    cost = Inf(rows, cols, 72);       % 각 셀과 방향에 대한 비용
    parent = zeros(rows, cols, 72, 3); % 부모 노드 정보 (x, y, 방향 인덱스)
    
    % 시작 노드 초기화
    start_idx = discretize_angle(start(3));
    cost(round(start(1)), round(start(2)), start_idx) = 0;
    h = heuristic(start, goal);
    open = [start, h, 0];  % [x, y, theta, h, g]
    
    % 차량 모델 파라미터
    L = 5;  % 차량 축간 거리
    v = 1;  % 속도 (상수)
    dt = 0.5; % 시간 간격 (증가시켜 더 큰 스텝으로 이동)
    
    % 조향각 범위 설정 - 더 많은 각도 추가
    steer_angles = [-1, -0.8, -0.6, -0.5, -0.4,-0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1];  % 조향각 범위 (라디안)
    
    % 최대 반복 횟수 설정 (무한 루프 방지)
    max_iterations = 500000;  % 반복 횟수 증가
    iteration = 0;
    
    disp('Hybrid A* 탐색 시작...');

    while ~isempty(open) && iteration < max_iterations
        iteration = iteration + 1;
        if mod(iteration, 1000) == 0
            % disp(['반복 횟수: ', num2str(iteration), ', 오픈 리스트 크기: ', num2str(size(open,1))]);
            fprintf('반복 횟수: %d, 오픈 리스트 크기: %d\n', iteration, size(open,1));
        end
        
        [~, idx] = min(open(:, 4) + open(:, 5));  % f = g + h
        current = open(idx, 1:3);
        current_g = open(idx, 5);
        open(idx,:) = [];
        
        current_idx = discretize_angle(current(3));
        r_current = round(current(1));
        c_current = round(current(2));
        
        % 목표에 도달했는지 확인 (위치만 확인)
        if norm(current(1:2) - goal(1:2)) < 5  % 목표 도달 범위 더 확대
            disp('목표 도달!');
            break;
        end
        
        % 범위 체크
        if r_current < 1 || r_current > rows || c_current < 1 || c_current > cols
            continue;
        end
        
        % 이미 방문한 노드인지 확인 - 방문 체크 완화
        if visited(r_current, c_current, current_idx)
            continue;
        end
        
        % 방문 표시
        visited(r_current, c_current, current_idx) = true;
        
        % 후속 상태 생성 & 충돌 검사
        for steer = steer_angles
            % 차량 운동학 모델 적용 - 여러 단계로 나누어 충돌 검사
            valid_move = true;
            next_x = current(1);
            next_y = current(2);
            next_theta = current(3);
            
            % 작은 단계로 나누어 이동 (충돌 검사를 위해)
            steps = 3;
            % small_dt = dt / steps;
            small_dt = dt;
            
            for step = 1:steps
                next_x = next_x + v * cos(next_theta) * small_dt;
                next_y = next_y + v * sin(next_theta) * small_dt;
                next_theta = next_theta + v * tan(steer) / L * small_dt;
                
                % 각도 정규화 (-pi ~ pi)
                next_theta = mod(next_theta + pi, 2*pi) - pi;
                
                % 중간 위치가 맵 범위 내에 있고 장애물이 아닌지 확인
                r_check = round(next_x);
                c_check = round(next_y);
                
                if r_check < 1 || r_check > rows || c_check < 1 || c_check > cols || map(r_check, c_check) == 1
                    valid_move = false;
                    break;
                end
            end
            
            if ~valid_move
                continue;
            end
            
            % 최종 위치의 인덱스
            r = round(next_x);
            c = round(next_y);
            next_idx = discretize_angle(next_theta);
            
            % 이동 비용 계산
            move_cost = v * dt;  % 기본 이동 비용
            
            % 조향각에 따른 페널티 추가 (선회 비용)
            steer_cost = 0.1 * abs(steer);
            
            % 총 비용 계산
            tentative_g = current_g + move_cost + steer_cost;
            
            % 더 나은 경로를 발견한 경우
            if tentative_g < cost(r, c, next_idx)
                cost(r, c, next_idx) = tentative_g;
                % 부모 노드 정보 저장 (x, y, 방향 인덱스)
                parent(r, c, next_idx, 1) = current(1);
                parent(r, c, next_idx, 2) = current(2);
                parent(r, c, next_idx, 3) = current_idx;
                
                % 휴리스틱 계산
                h = heuristic([next_x, next_y, next_theta], goal);
                
                % 오픈 리스트에 추가
                open = [open; next_x, next_y, next_theta, h, tentative_g];
            end
        end
    end
    
    disp(['총 반복 횟수: ', num2str(iteration)]);

    % 경로 복원
    if ~isempty(open) || (iteration < max_iterations && norm(current(1:2) - goal(1:2)) < 10)
        disp('경로 복원 중...');
        path = current(1:2);
        curr_x = round(current(1));
        curr_y = round(current(2));
        curr_idx = discretize_angle(current(3));
        
        while ~(curr_x == round(start(1)) && curr_y == round(start(2)))
            p = squeeze(parent(curr_x, curr_y, curr_idx, :));
            if all(p == 0)  % 부모가 없는 경우 (경로 단절)
                disp('경로 복원 중 단절 발생!');
                break;
            end
            curr_x = round(p(1));
            curr_y = round(p(2));
            curr_idx = round(p(3));
            
            path = [[curr_x, curr_y]; path];
        end
    else
        path = [];
        if iteration >= max_iterations
            disp('최대 반복 횟수 초과!');
        end
    end
end

% 각도를 이산화하여 인덱스로 변환 (10도 간격으로 36개 방향)
function idx = discretize_angle(angle)
    % 각도를 0~2pi 범위로 정규화
    angle = mod(angle, 2*pi);
    % 10도 간격으로 인덱스 계산 (1~36)
    idx = floor(angle / (2*pi/36)) + 1;
    if idx > 36
        idx = 36;
    end
end

% norm + angle_difference에 penalty 적용
function h = heuristic(pos, goal)
    % 유클리드 거리
    h_eu = norm(pos(1:2) - goal(1:2));
    
    % 방향 차이에 대한 페널티 추가 (선택적)
    angle_diff = abs(pos(3) - goal(3));
    angle_diff = min(angle_diff, 2*pi - angle_diff);
    angle_penalty = 0.25 * angle_diff;
    
    h = h_eu + angle_penalty;
    % h = h_eu;
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

