function path = main(Pose, Sensor_data, map)
    % A* Algorithm
    % 1. 초기 설정
    [rows, cols] = size(map);
    max_path_size = 200; % 경로 배열의 최대 크기
    path = NaN(max_path_size, 2); % 최종 경로 출력 변수

    % start & goal
    start = [5, 5];   
    goal = [95, 95];  

    % A* 알고리즘에 필요한 변수 초기화
    visited = false(rows, cols);     % 방문한 노드를 기록하는 배열
    cost = Inf(rows, cols);          % 각 노드까지의 g(n) 비용
    parent = NaN(rows, cols, 2);   % 각 노드의 부모 노드를 기록하는 배열

    cost(start(1), start(2)) = 0;    % 시작점의 g(n) 비용은 0

    % Open list 초기화
    max_open_size = rows * cols;
    open_list = NaN(max_open_size, 3); % [row, col, f_cost]
    open_count = 0;

    h_cost = heuristic(start, goal); % 시작점에서 목표점까지의 휴리스틱 비용 계산 -> 이거 왜?
    open_count = open_count + 1;
    open_list(open_count, :) = [start, cost(start(1), start(2)) + h_cost];

    goal_found = false;

    % 2. 메인 루프 (Main Loop)
    while open_count > 0
        % Open 리스트에서 f_cost가 가장 작은 노드 찾기
        [~, idx] = min(open_list(1:open_count, 3));
        current_node = open_list(idx, 1:2);

        % 찾은 노드를 Open 리스트에서 제거
        open_list(idx, :) = open_list(open_count, :);
        open_count = open_count - 1;

        % 목표점에 도달했는지 확인
        if all(current_node == goal)
            goal_found = true;
            break;
        end

        % 방문 표시
        if visited(current_node(1), current_node(2))
            continue;
        end
        visited(current_node(1), current_node(2)) = true;

        % 이웃 노드 탐색
        neighbors = get_neighbors(current_node, rows, cols);

        for i = 1:size(neighbors, 1)
            neighbor = neighbors(i, :);
            r = neighbor(1);
            c = neighbor(2);

            % 장애물이거나 이미 방문했는지
            if map(r, c) ~= 0 || visited(r,c)
                continue;
            end

            % 현재 노드를 거쳐 이웃 노드로 가는 새로운 g_cost 계산
            move_cost = norm(current_node - neighbor); % 1 또는 sqrt(2)
            g_cost = cost(current_node(1), current_node(2)) + move_cost;

            % 더 나은 경로를 발견했을 때 정보 업데이트
            if g_cost < cost(r, c)
                cost(r, c) = g_cost;
                parent(r, c, :) = current_node;

                % f_cost 계산 및 Open 리스트에 추가
                f_cost = g_cost + heuristic(neighbor, goal);

                open_count = open_count + 1;
                open_list(open_count, :) = [neighbor, f_cost];
            end
        end
    end

    % 3. 경로 역추적 
    if goal_found
        % 경로를 역순으로 추적
        path_temp = goal;
        p_node = goal;

        while ~all(p_node == start)
            parent_node = squeeze(parent(p_node(1), p_node(2), :))';
            path_temp = [parent_node; path_temp];
            p_node = parent_node;
        end

        num_path_points = size(path_temp, 1);
        path(1:num_path_points, :) = path_temp;
        path = path(:, [2, 1])
    end
end

% 휴리스틱 함수 - 유클리드 거리
function h = heuristic(pos, goal)
    h = norm(pos - goal);
end

% 이웃 노드 탐색 함수 
function neighbors = get_neighbors(pos, max_row, max_col)
    neighbors = zeros(8, 2);
    neighbor_count = 0;

    % 8방향 탐색
    directions = [-1, 0; 1, 0; 0, -1; 0, 1; -1, 1; -1, -1; 1, -1; 1, 1];

    for i = 1:8
        r = pos(1) + directions(i, 1);
        c = pos(2) + directions(i, 2);

        % 맵 경계 내에 있는지 확인
        if r >= 1 && r <= max_row && c >= 1 && c <= max_col
            neighbor_count = neighbor_count + 1;
            neighbors(neighbor_count, :) = [r, c];
        end
    end
end
