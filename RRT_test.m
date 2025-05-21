%{
function
    - RRT()
    - rrt_search()
    - is_collision_free
mapHZ
    - map_demo_1.csv
%}

function main()
    % 맵 불러오기
    map_csv = './map_demo_1.csv';
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

    % RRT 알고리즘 실행
    RRT(matrix, start, goal);
end

function RRT(map, start, goal)
    path = rrt_search(map, start, goal);

    % visualize path
    figure;
    imagesc(map);
    colormap(gray);
    axis equal tight;
    hold on;

    % start, goal
    plot(start(1),start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    text(start(1),start(2)+4, 'Start');
    plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    text(goal(1),goal(2)+4, 'Goal');

    % plot path
    if ~isempty(path)
        for i = 1:size(path,1)
            plot(path(i,2), path(i,1), 'b.', 'MarkerSize', 15);
        end
    else
        disp('경로를 찾을 수 없습니다.');
    end

    hold off;
    title('RRT Path Planning');
end

function path = rrt_search(map, start, goal)
    max_iter = 10000;
    step_size = 5;
    goal_threshold = 5;

    [rows, cols] = size(map);
    
    tree = struct('parent',{},'position',{});
    tree(1).parent = 0;
    tree(1).position = start;

    for iter = 1:max_iter
        % random sampling
        rand_point =[randi(rows),randi(cols)];

        % find q_near
        min_dist = Inf;
        nearest_idx = 1;
        for i = 1:length(tree)
            dist = norm(tree(i).position - rand_point);
            if dist < min_dist
                min_dist = dist;
                nearest_idx = i;
            end
        end

        % add q_new into tree
        nearest = tree(nearest_idx).position;
        direction = rand_point - nearest;
        norm_dir = norm(direction);
        if norm_dir == 0
            continue;
        end
        direction = direction / norm_dir;
        new_point = round(nearest + step_size * direction);

        % check boundary
        if new_point(1) < 1 || new_point(1) > rows || new_point(2) < 1 || new_point(2) > cols
            continue;
        end

        % check collision
        if ~is_collision_free(map, nearest, new_point)
            continue;
        end

        % add new_point into tree
        tree(end+1).parent = nearest_idx;
        tree(end).position = new_point;

        % check if new_point near goal
        if norm(new_point - goal) < goal_threshold
            tree(end+1).parent = length(tree);
            tree(end).position = goal;
            break;
        end
    end

    % reconstruct path
    if norm(tree(end).position - goal) >= goal_threshold
        path = [];
        return;
    end

    % compose path from parent to child
    idx = length(tree);
    path = [];
    while idx ~= 0
        path = [tree(idx).position; path];
        idx = tree(idx).parent;
    end
end

function is_free = is_collision_free(map, p1, p2)
    x = linspace(p1(2),p2(2), 100);
    y = linspace(p1(1),p2(1), 100);

    for i = 1:length(x)
        xi = round(x(i));
        yi = round(y(i));

        if xi < 1 || xi > size(map,2) || yi < 1 || yi > size(map,1)
            is_free = false;
            return;
        end
        if map(yi, xi) == 1
            is_free = false;
            return;
        end
    end
    is_free = true;
end