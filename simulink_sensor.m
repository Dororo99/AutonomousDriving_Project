function output = Sensor(Pose,obs)

%-----------------<Release Information >----------------------------
% version: alpha 1.0.0 (For distribution)
% Release Date: 2023.11.11
% Developer: Yu su Hyung
%-------------------------------------------------------------------

% <Block Description>
% The block was created to generate sensor data from the vehicle's current location and obstacle information. 
% Use a function called Inpolygon to find the distance between the obstruction and the vehicle.
% You can adjust the resolution and visibility of the sensor.

% Warning!!
% This algorithm calculates the sensor distance from the number of points within the obstacle area, 
% which can lead to errors if the sensor distance is longer than the thickness of the obstacle.
% 2023.11.6 It is resolved!!

% There may be additional changes depending on the difficulty or progress of the map.

resolution_deg = 120;
sensor_dist = 15;

output = ones(resolution_deg, 1) * sensor_dist;
scan_range = linspace(-60*pi/180, 60*pi/180, resolution_deg);

for output_idx = 1:resolution_deg
    
    scan_angle = scan_range(output_idx);
    d = laser_dist(Pose, sensor_dist, scan_angle, obs);
    output(output_idx) = d;
end

end




function d = dist(q1,q2)
d = sqrt( (q1(1) - q2(1))^2 + ( q1(2) - q2(2))^2 );
end


function [min_laser_dist_, check_block] = laser_dist(pose, sensor_dist, sensor_heading, obs)

% pose: 차량의 현재 위치(x,y,psi)
% sensor_dist: 센서의 가시거리
% sensor_heading: 센서의 방향(현재 차량 header 기준)


check_block = 0;
p0 = [pose(1), pose(2)];
heading = pose(3);

pf = [p0(1) + sensor_dist*cos(heading+sensor_heading), p0(2) + sensor_dist*sin(heading+sensor_heading)];

resolution_dis = 100;
xq = linspace(p0(1), pf(1), resolution_dis);
yq = linspace(p0(2), pf(2), resolution_dis);

min_laser_dist_ = sensor_dist; %default 값

for i = 1: size(obs,1)
    %p1은 왼쪽 상단 점, p2는 오른쪽 하단 점을 의미
    p1_x= obs(i,1);
    p1_y= obs(i,2);
    p2_x= obs(i,3);
    p2_y= obs(i,4);
    
    % obstacle 좌표 -> 시계방향
    xv = [p1_x, p1_x, p2_x, p2_x,p1_x];
    yv = [p1_y, p2_y, p2_y, p1_y,p1_y];
    
    [in,on] = inpolygon(xq,yq,xv,yv);

    for k = 1: size(in,2)
        if in(1, k) ~= 0
            % 장애물 영역 안에 들어가는 좌표 검사
            crossing_x = xq(k);
            crossing_y = yq(k);
            temp = dist([crossing_x, crossing_y], [p0(1), p0(2)]);
        else
            temp = sensor_dist;      
        end

        if temp < min_laser_dist_
            % 센서 최대거리 15m 및 센서 조향각 이내에 장애물이 존재하는 경우    
            min_laser_dist_ = temp;
        end
        
    end   

end

end
