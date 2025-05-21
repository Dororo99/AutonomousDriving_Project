%{
D* Algorithm
    - Add D* into A*
%}
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
    map = csvread(map_csv);
    [rows, cols] = size(map);

    % 시작점과 목표점 설정
    start = [5, 95];
    goal = [95, 5];

    % 초기 경로 계산 (D* Lite)
    [path, g, rhs, Q] = dstar_lite_search(map, start, goal);

    % 시각화
    visualize_path(map, path, start, goal);

    % 실시간 장애물 추가 (예시)
    new_obstacle = [70, 40];
    map(new_obstacle(1), new_obstacle(2)) = 1;
    neighbors = get_neighbors(new_obstacle, rows, cols);
    for i = 1:size(neighbors, 1)
        s = neighbors(i, :);
        rhs(s(1), s(2)) = min_successor_rhs(s, g, map);
        Q = update_vertex(Q, s, g, rhs, start);
    end

    % 경로 재계산
    [path, ~, ~, ~] = dstar_lite_search(map, start, goal, g, rhs, Q);
    visualize_path(map, path, start, goal);
end

function [path, g, rhs, Q] = dstar_lite_search(map, start, goal, g_in, rhs_in, Q_in)
    [rows, cols] = size(map);

    if nargin < 4
        g = Inf(rows, cols);
        rhs = Inf(rows, cols);
        rhs(goal(1), goal(2)) = 0;
        Q = [goal, heuristic(goal, start)];
    else
        g = g_in;
        rhs = rhs_in;
        Q = Q_in;
    end

    while ~isempty(Q)
        [~, idx] = min(Q(:,3));
        u = Q(idx, 1:2);
        Q(idx,:) = [];

        if g(u(1), u(2)) > rhs(u(1), u(2))
            g(u(1), u(2)) = rhs(u(1), u(2));
        else
            g(u(1), u(2)) = Inf;
            rhs(u(1), u(2)) = min_successor_rhs(u, g, map);
        end

        neighbors = get_neighbors(u, rows, cols);
        for i = 1:size(neighbors,1)
            s = neighbors(i,:);
            rhs(s(1), s(2)) = min(rhs(s(1), s(2)), g(u(1), u(2)) + norm(s - u));
            Q = update_vertex(Q, s, g, rhs, start);
        end
    end

    path = reconstruct_path(start, goal, g, map);
end

function Q = update_vertex(Q, u, g, rhs, start)
    Q(ismember(Q(:,1:2), u, 'rows'), :) = [];
    if g(u(1), u(2)) ~= rhs(u(1), u(2))
        key = min(g(u(1), u(2)), rhs(u(1), u(2))) + heuristic(u, start);
        Q = [Q; u, key];
    end
end

function val = min_successor_rhs(u, g, map)
    [rows, cols] = size(map);
    neighbors = get_neighbors(u, rows, cols);
    costs = [];
    for i = 1:size(neighbors,1)
        s = neighbors(i,:);
        if map(s(1), s(2)) == 1
            continue;
        end
        cost = g(s(1), s(2)) + norm(s - u);
        costs = [costs; cost];
    end
    val = isempty(costs) * Inf + ~isempty(costs) * min(costs);
end

function path = reconstruct_path(start, goal, g, map)
    current = start;
    path = current;
    [rows, cols] = size(map);

    while ~all(current == goal)
        neighbors = get_neighbors(current, rows, cols);
        best_cost = Inf;
        next_node = [];

        for i = 1:size(neighbors,1)
            s = neighbors(i,:);
            if map(s(1), s(2)) == 1
                continue;
            end
            c = norm(s - current) + g(s(1), s(2));
            if c < best_cost
                best_cost = c;
                next_node = s;
            end
        end

        if isempty(next_node)
            path = [];
            return;
        end

        path = [path; next_node];
        current = next_node;
    end
end

function h = heuristic(pos, goal)
    h = norm([pos(1) - goal(1), pos(2) - goal(2)]);  % Euclidean
end

function neighbors = get_neighbors(pos, max_row, max_col)
    directions = [ -1 0; 1 0; 0 -1; 0 1; -1 1; -1 -1; 1 -1; 1 1 ];
    neighbors = [];
    for i = 1:8
        r = pos(1) + directions(i,1);
        c = pos(2) + directions(i,2);
        if r >= 1 && r <= max_row && c >= 1 && c <= max_col
            neighbors = [neighbors; r, c];
        end
    end
end

function visualize_path(map, path, start, goal)
    figure;
    imagesc(map);
    colormap(gray);
    axis equal tight;
    hold on;
    plot(start(2), start(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(goal(2), goal(1), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    if ~isempty(path)
        for i = 1:size(path,1)
            plot(path(i,2), path(i,1), 'b.', 'MarkerSize', 15);
        end
    else
        disp('경로를 찾을 수 없습니다.');
    end
    title('D* Lite Path Planning');
    hold off;
end