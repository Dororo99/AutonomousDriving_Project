% 1. 시뮬레이션 '마지막' 시점의 기준 경로(path) 데이터 추출
final_path = path.Data(:,:,end);

% 2. x, y 궤적 데이터의 차원 축소
x_trajectory = squeeze(x_actual.Data);
y_trajectory = squeeze(y_actual.Data);

% 3. 마지막 기준 경로
plot(final_path(:,1), final_path(:,2), 'r--', 'LineWidth', 1.5);

hold on;

% 4. 실제 차량의 주행 궤적
plot(x_trajectory, y_trajectory, 'b-');

% 5. 장애물 추가
final_obs = obs(:,:,end);

% 각 장애물(행)에 대해 루프 실행
for i = 1:size(final_obs, 1)
    % i번째 장애물의 좌측상단(x1,y1), 우측하단(x2,y2) 좌표 추출
    x1 = final_obs(i, 1);
    y1 = final_obs(i, 2);
    x2 = final_obs(i, 3);
    y2 = final_obs(i, 4);

    % 사각형의 네 꼭짓점 좌표 벡터 생성
    % [x1, x2, x2, x1], [y1, y1, y2, y2] 순서로 연결
    verts_x = [x1, x2, x2, x1];
    verts_y = [y1, y1, y2, y2];

    % fill 함수로 속이 채워진 회색 사각형 그리기
    fill(verts_x, verts_y, [0.5 0.5 0.5], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
end


hold off; % 그래프 겹쳐 그리기 종료

% 6. 그래프
grid on;
axis equal;
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Path Tracking Performance (at Final Time Step)');
legend('Final Reference Path', 'Vehicle Trajectory');

