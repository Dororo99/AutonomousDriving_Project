%{
function
    - Astar()
    - astar_search()
    - heuristic()
    - get_neighbors()
map
    - map_demo_1.csv
%}

function result = main()
    % 맵 불러오기
    map_csv = '../maps/map_demo_1.csv';
    matrix = csvread(map_csv);
    matrix_expanded = expand_obstacles(matrix);  % 확장된 맵 생성

    % 시작점과 목표점 설정
    start = [5,95];
    goal = [95,5];
    
    % path
    path = astar_search(matrix_expanded,start,goal);

    % 원본 맵에 경로 시각화
    figure;
    imagesc(matrix);  % 원래 맵 사용
    colormap(gray);
    axis equal tight;
    hold on;

    result = path;
    
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
    h_eu = norm([pos(1) - goal(1) , pos(2) - goal(2)]);
    h = h_eu
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

