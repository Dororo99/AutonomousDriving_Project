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
    Astar(matrix, start, goal);
end

function Astar(map, start, goal)
    path = astar_search(map, start, goal);

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
