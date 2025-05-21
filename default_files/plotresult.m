%% Simulink 상에서 시뮬레이션이 끝나고, Simulation을 Plotting

    close all
    k = 1;
    w = 3;
    h = 1.5;
    scan_range = linspace(-pi/3, pi/3, 120);
    sampletime = 0.001;
    vizRate = rateControl(1/sampletime);
    % csvPath = '/Users/l/Club/사미용두/simulink files/map_demo_1.csv';
    csvPath = './map_demo_1.csv';
    matrix = csvread(csvPath);

    hold on
    imshow(~matrix, 'InitialMagnification', 'fit');
    plot(5, 95, 'bo');
    text(5,97, 'Start');
    plot(95, 5, 'ro');
    text(80, 5, 'Goal');
    G = plot(out.Total_Waypoints(:,1,1), 100-out.Total_Waypoints(:,2,1), 'g-');
    L = plot(out.Total_Waypoints(:,1,1), 100-out.Total_Waypoints(:,2,1), 'g-');
    C = plot(out.Pose(1,1), out.Pose(1,2));
    S = plot(out.Pose(1,1), out.Pose(1,2));


    for i = 2:size(out.Pose,1)        
        %% Visualization of Car
        plotvec = [out.Pose(i,1), 100-out.Pose(i,2), 0];
        plotrot = axang2quat([0, 0, 1, out.Pose(i,3)]);     
        plotTransforms(plotvec, plotrot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", 0.3, 'MeshColor', [0.5, 0, 0]);

        heading = out.Pose(i,3);
        rot_matrix = [cos(heading), -sin(heading); sin(heading), cos(heading)];
        q1 = rot_matrix * [ - w/2; + h/2] + [out.Pose(i,1); out.Pose(i,2)];
        q2 = rot_matrix * [ + w/2; + h/2] + [out.Pose(i,1); out.Pose(i,2)]; 
        q3 = rot_matrix * [ + w/2; - h/2] + [out.Pose(i,1); out.Pose(i,2)]; 
        q4 = rot_matrix * [ - w/2; - h/2] + [out.Pose(i,1); out.Pose(i,2)]; 
        x = [q1(1), q2(1), q3(1), q4(1), q1(1)];
        y = 100 - [q1(2), q2(2), q3(2), q4(2), q1(2)];
        delete(C);
        C = plot(x,y,'r-');

        % %% Visualization of Sensor
        Xn = zeros(120,2);
        Reference = [out.Pose(i,1) * ones(120,1), out.Pose(i,2) * ones(120,1)];
        for j = 1:size(out.sensor_data,2)
            Xn(j,:) = [out.Pose(i,1) + out.sensor_data(i,j) * cos(heading + scan_range(j)),...
                       out.Pose(i,2) + out.sensor_data(i,j) * sin(heading + scan_range(j))];
        end
        delete(S);
        S = plot([Reference(:,1), Xn(:,1)]', 100 - [Reference(:,2), Xn(:,2)]','b--',"LineWidth", 0.3);


        %% Visualization of Local Path
        if ~(isequal(out.Total_Waypoints(1,:,1), out.Total_Waypoints(1, :, i))) 
            plot(out.obs_list(:,1,k), 100-out.obs_list(:,2,k), 'bo', 'MarkerSize', 2);
            k = k + 1;
        end

        if ~(isequal(out.Total_Waypoints(1, 1, i), out.Total_Waypoints(1, 1, i-1)))
            delete(L); 
            L = plot(out.Total_Waypoints(:,1,i), 100-out.Total_Waypoints(:,2,i), 'r--', "LineWidth",0.1);
        end        

        waitfor(vizRate);
    end
    delete(L);
    delete(S);
    legend('Start', 'Goal', 'GlobalPath', 'Sensored Obstacle');
    title('Sensor Visualization');
    hold off
    
    

    % hold on
    % imshow(~matrix, 'InitialMagnification', 'fit');
    % plot(7, 93, 'bo');
    % text(7,95, 'Start');
    % plot(93, 7, 'ro');
    % text(80, 7, 'Goal');
    % q1 = rectangle('Position', [37,19,6,10],'FaceColor','b');
    % q2 = rectangle('Position', [52,10,6,10],'FaceColor','b');
    % q3 = rectangle('Position', [59,16,6,10],'FaceColor','b');
    % G2 = plot(out.Total_Waypoints(:,1,1), 100-out.Total_Waypoints(:,2,1), 'g-');
    % L2 = plot(out.Total_Waypoints(:,1,1), 100-out.Total_Waypoints(:,2,1), 'g-');
    % 
    % 
    %  for i = 2:size(out.Pose,1)
    %     plotvec = [out.Pose(i,1), 100-out.Pose(i,2), 0];
    %     plotrot = axang2quat([0, 0, 1, out.Pose(i,3)]);     
    %     plotTransforms(plotvec, plotrot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", 0.3, 'MeshColor', [0.5, 0, 0]);
    % 
    %     if ~(isequal(out.Total_Waypoints(1, 1, i), out.Total_Waypoints(1, 1, i-1)))
    %         delete(L2); 
    %         L2 = plot(out.Total_Waypoints(:,1,i), 100-out.Total_Waypoints(:,2,i), 'b--');
    %     end
    % 
    %     waitfor(vizRate);
    % end
    % delete(L2);
    % plot(out.Pose(:,1), 100-out.Pose(:,2), 'r-');
    % legend('Start', 'Goal', 'GlobalPath', 'LocalPath');
    % title('Real Map');
    % hold off