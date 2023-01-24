% Reference: https://asl.ethz.ch/education/lectures/autonomous_mobile_robots/spring-2021.html
clc;

load sim_occmap;

% Make sure to have the simulation scene example_scene.ttt running in V-REP!

% simulation setup, will add the matlab paths
connection = simulation_setup();

% the robot we want to interact with
robotNb = 0;

% open the connection
connection = simulation_openConnection(connection, robotNb);

% start simulation if not already started
simulation_start(connection);

% now enable stepped simulation mode:
simulation_setStepped(connection,true);

% and step 100 times:

[bodyDiameter, wheelDiameter, interWheelDist] = robot_init(connection);

r = wheelDiameter / 2;
l = interWheelDist;
maxRange = 10.0;
% 

%%

%  while true
    simulation_triggerStep(connection);

    % get wheel encoders
    [encoder_left, encoder_right] = robot_getEncoders(connection);

    % get laser data
    [laserDataX, laserDataY] = robot_getLaserData(connection);
    
    angles = zeros(1, size(laserDataX, 2));
    ranges = zeros(1, size(laserDataX, 2));
    range_max = maxRange - 0.1;
    m_i = [];


    for j = 1:size(laserDataX, 2)
        angle = atan2(laserDataY(1, j), laserDataX(1, j));
        range = sqrt(laserDataX(1, j)*laserDataX(1, j) + laserDataY(1, j)*laserDataY(1, j));
    
        if (range >= range_max)
            range = inf;
            angle = inf;
        end
        
        

        angles(j) = angle;
        ranges(j) = range;
    end
    scan = lidarScan(ranges, angles);


%      figure(2);
%      plot(laserDataX(1, :), laserDataY(1, :))
%     plot(scan)

    % get laser data

    [x, y, theta] = robot_getPose(connection);
    fprintf("Robot x: %f , Robot y: %f\n" ,[x,y])

    robot_setWheelSpeeds(connection, 0, 0);
    figure(1);
    show(occMap);
    mcl = monteCarloLocalization;
    mcl.UseLidarScan = true;
    lf = likelihoodFieldSensorModel;
    lf.Map = occMap;
    lf.SensorLimits = [0,9.9];
    mcl.SensorModel = lf;
    odometryPose = robot_getPose(connection);

    %% your code here %%
    figure(2);
    Robot_Coordinate = [0 0];
    laserDataX_new = transpose(laserDataX);
    laserDataY_new = transpose(laserDataY);
    Obstacle_Coordinates = [laserDataX_new, laserDataY_new];
    Goal_Coordinate_1 = [1.395, 0.9];
    Sensor_Range = 0.3;
    Step_Size = 0.4*Sensor_Range;
    Obstacle= [6 6];
    Goal = [4 4];
    NPTS = 60;
    Step_Degree = 360/NPTS;
    Confirm_Message      = 'Solution Exists'   ;% Message Displayed if a Good Artificial Point Exists
    Error_Message        = 'No Solution Exists';% Message Displayed if no Good Artificial Point Exists
    Bacteria_x           = Robot_Coordinate(1) ;% Artificial Best Point x
    Bacteria_y           = Robot_Coordinate(2) ;% Artificial Best Point y
    Waypoints = [Robot_Coordinate(1), Robot_Coordinate(2)];


    hold on
    axis([0 12 0 12])
%--------------------------Target-----------------------------
    backx  = Goal_Coordinate_1(1) - 0.15;backy1 = Goal_Coordinate_1(2) + 0.15;
    frontx = Goal_Coordinate_1(1) + 0.15;fronty1 = Goal_Coordinate_1(2) + 0.15;
    middlex = Goal_Coordinate_1(1);middley1 = Goal_Coordinate_1(2)+ 0.15;
    middley2 = Goal_Coordinate_1(2)- 0.15;
    tri2 = [backx frontx middlex backx ;backy1 fronty1 middley2 backy1 ];
    plot(tri2(1,:), tri2(2,:));
    hold on
    plot(Goal_Coordinate_1(1),Goal_Coordinate_1(2),"xb")

    for l = 1:size(Obstacle_Coordinates,1)
        xc = Obstacle_Coordinates(l,1);
        yc = Obstacle_Coordinates(l,2);
        r = 0.01;
        x_map = r*sin(-pi:0.2*pi:pi) + xc;
        y_map = r*cos(-pi:0.2*pi:pi) + yc;
        c = [0.6 0 1];
        fill(x_map,y_map,c,"FaceAlpha",0.4);
    end
    DTG = 1;
    %---------------------------Robot_Init------------------
        backx  = Robot_Coordinate(1) - 0.05;backy1 = Robot_Coordinate(2) + 0.05;
        backy2 = Robot_Coordinate(2) - 0.05;frontx = Robot_Coordinate(1) + 0.05;
        fronty1 = Robot_Coordinate(2) + 0.05;fronty2 = Robot_Coordinate(2);
        tri1 = [backx backx frontx frontx backx;backy2 backy1 fronty1 backy2 backy2];
        plot(tri1(1,:), tri1(2,:));
        hold on
        plot(Robot_Coordinate(1),Robot_Coordinate(2),"xr")
    while(DTG>0.05)
        backx  = Robot_Coordinate(1) - 0.05;backy1 = Robot_Coordinate(2) + 0.05;
        backy2 = Robot_Coordinate(2) - 0.05;frontx = Robot_Coordinate(1) + 0.05;
        fronty1 = Robot_Coordinate(2) + 0.05;fronty2 = Robot_Coordinate(2);
        tri1 = [backx backx frontx frontx backx;backy2 backy1 fronty1 backy2 backy2];
        plot(tri1(1,:), tri1(2,:));
        hold on
        title("First APF Path")
        plot(Robot_Coordinate(1),Robot_Coordinate(2),"xr")


        %.........................Potential Calculations.......................
        J_ObstT = 0;
        for k = 1:size(Obstacle_Coordinates,1)
            J_ObstT = J_ObstT + Obstacle(1)*exp(-Obstacle(2)*((Robot_Coordinate(1)-Obstacle_Coordinates(k,1))^2+(Robot_Coordinate(2)-Obstacle_Coordinates(k,2))^2));
        end
        J_GoalT = -Goal(1)*exp(-Goal(2)*((Robot_Coordinate(1)-Goal_Coordinate_1(1))^2+(Robot_Coordinate(2)-Goal_Coordinate_1(2))^2));
        JT = J_ObstT + J_GoalT;

        %...........................Distance to Goal...........................
        DTG = sqrt((Robot_Coordinate(1)-Goal_Coordinate_1(1))^2+(Robot_Coordinate(2)-Goal_Coordinate_1(2))^2);

        %...........................Artificial Points..........................

        Theta = zeros(1,NPTS);
        Theta(1) = Step_Degree;
        for z = 1:NPTS-1
            Theta(z+1) = Theta(z) + Step_Degree;
        end

        Bacteria_x = zeros(1,NPTS);
        Bacteria_y = zeros(1,NPTS);
        for i=1:NPTS
            Bacteria_x(i) = Robot_Coordinate(1) + (Step_Size*cos(pi*Theta(i)/180));
            Bacteria_y(i) = Robot_Coordinate(2) + (Step_Size*sin(pi*Theta(i)/180));
        end
        plot(Bacteria_x,Bacteria_y,'.',Robot_Coordinate(1),Robot_Coordinate(2),".")
        pause(0.1);

        %........Calculating Cost Function and Distance of Artificial Point....

        [m,n] = size(Bacteria_x);
        J_ObstT_Bacteria = zeros(1,n);
        J_GoalT_Bacteria = zeros(1,n);
        JT_Bacteria = zeros(1,n);
        DTG_Bacteria = zeros(1,n);
        for i=1:n
            for j = 1:size(Obstacle_Coordinates,1)
                J_ObstT_Bacteria(i) = J_ObstT_Bacteria(i) + Obstacle(1)*exp(-Obstacle(2)*((Bacteria_x(i)-Obstacle_Coordinates(j,1))^2+(Bacteria_y(i)-Obstacle_Coordinates(j,2))^2));
            end
            J_GoalT_Bacteria(i) = -Goal(1)*exp(-Goal(2)*((Bacteria_x(i)-Goal_Coordinate_1(1))^2+(Bacteria_y(i)-Goal_Coordinate_1(2))^2));
            JT_Bacteria(i) = J_ObstT_Bacteria(i) + J_GoalT_Bacteria(i);
            DTG_Bacteria(i) = sqrt((Bacteria_x(i)-Goal_Coordinate_1(1))^2+(Bacteria_y(i)-Goal_Coordinate_1(2))^2);
        end
        J_GoalT_Bacteria;

        %....................Error in Cost Function & Distance.................

        err_J   = zeros(1,n);
        err_DTG = zeros(1,n);
        Fitness = zeros(1,n);
        for i=1:n
            err_J(i)   = JT_Bacteria(i)  - JT;
            err_DTG(i) = DTG_Bacteria(i) - DTG;
            Fitness(i) = -err_DTG(i);
        end
        Fitness;
        %..........................Best Point Selection........................
        Check = 0;
        for t=1:n
            [~,k]= max(Fitness);
            if err_J(k) < 0
                if err_DTG(k)<0
                    Check = Check + 1;
                    Robot_Coordinate(1) = Bacteria_x(k);
                    Robot_Coordinate(2) = Bacteria_y(k);
                    DTG = sqrt((Robot_Coordinate(1)-Goal_Coordinate_1(1))^2+(Robot_Coordinate(2)-Goal_Coordinate_1(2))^2);
                else
                    Fitness(k) = 0;
                end
            else
                Fitness(k) = 0;
            end
        end
        Check;
        if Check == 0
            disp(Error_Message)
            break;
        else
            disp(Confirm_Message);
        end
      
        Waypoints = [Waypoints;Robot_Coordinate(1), Robot_Coordinate(2)];

        %.................................Plot.................................
    
        %............................Create a Target...........................
        backx  = Goal_Coordinate_1(1) - 0.15;backy1 = Goal_Coordinate_1(2) + 0.15;
        frontx = Goal_Coordinate_1(1) + 0.15;fronty1 = Goal_Coordinate_1(2) + 0.15;
        middlex = Goal_Coordinate_1(1);middley1 = Goal_Coordinate_1(2)+ 0.15;
        middley2 = Goal_Coordinate_1(2)- 0.15;
        tri2 = [backx frontx middlex backx ;backy1 fronty1 middley2 backy1 ];
        hold on
        plot(tri2(1,:), tri2(2,:));

    end
    Ans = input("Would you like to move the robot according to path? (Y/N)","s");
    if Ans == "Y"

        controller = controllerPurePursuit("LookaheadDistance",0.5,"DesiredLinearVelocity",0.1);
        controller.Waypoints = Waypoints;
        r = wheelDiameter / 2;
        l = interWheelDist;
        c = 0;
        [x_init,y_init,theta] = robot_getPose(connection);

        while true
            [x, y, theta] = robot_getPose(connection);
            pose = [x - x_init, y - y_init, theta];
            Robot_Dist_to_Goal = sqrt((pose(1)-Goal_Coordinate_1(1))^2+(pose(2)-Goal_Coordinate_1(2))^2);
             if(Robot_Dist_to_Goal > 0.2)
                [v_target,w_target,Look_Ahead_Point] = controller([pose(1), pose(2), pose(3)]);
                [v_left,v_right] = Kinematic_Conv(v_target,w_target,r,l);
                robot_setWheelSpeeds(connection, v_left, v_right);
                pause(0.01);
                simulation_triggerStep(connection);
                figure(1);
                [laserDataX, laserDataY] = robot_getLaserData(connection);

                angles = zeros(1, size(laserDataX, 2));
                ranges = zeros(1, size(laserDataX, 2));
                range_max = maxRange - 0.1;

                for j = 1:size(laserDataX, 2)
                    angle = atan2(laserDataY(1, j), laserDataX(1, j));
                    range = sqrt(laserDataX(1, j)*laserDataX(1, j) + laserDataY(1, j)*laserDataY(1, j));

                    if (range >= range_max)
                        range = inf;
                        angle = inf;
                    end

                    angles(j) = angle;
                    ranges(j) = range;
                end
                scan = lidarScan(ranges, angles);
                [x, y, theta] = robot_getPose(connection);
                odometryPose = [x, y, theta];
                odometryPose = double(odometryPose);
                [isUpdated,estimatedPose,covariance] = mcl(odometryPose,scan);
                figure(1);
                hold on;
                plot(estimatedPose(1),estimatedPose(2),"rx");
                fprintf("Estimated x: %d, Estimated y: %d , Estimated T: %d \n ",estimatedPose);

                
             else
                 robot_setWheelSpeeds(connection, 0, 0);
                 [x, y, theta] = robot_getPose(connection);
                 if theta ~= 0
                     [v_left,v_right] = Kinematic_Conv(0,0.05,r,l);
                     robot_setWheelSpeeds(connection,v_left,v_right);
                     while abs(theta) > 0.01
                         simulation_triggerStep(connection);
                         [x, y, theta] = robot_getPose(connection);
                     end
                 end
                     break;
                 end

        end
        figure(2);
             simulation_triggerStep(connection);



    end


%--------------------Second Phase--------------------------

simulation_triggerStep(connection);

% get wheel encoders
[encoder_left, encoder_right] = robot_getEncoders(connection);

% get laser data
[laserDataX, laserDataY] = robot_getLaserData(connection);

figure(3);
plot(laserDataX(1, :), laserDataY(1, :))

clf;

[laserDataX, laserDataY] = robot_getLaserData(connection);
Robot_Coordinate = [0 0];
laserDataX_new = transpose(laserDataX);
laserDataY_new = transpose(laserDataY);
Obstacle_Coordinates = [laserDataX_new, laserDataY_new];
Goal_Coordinate_1 = [1.1750, -1.2];
Sensor_Range = 0.3;
Step_Size = 0.4*Sensor_Range;
Obstacle= [6 6];
Goal = [4 4];
NPTS = 60;
Step_Degree = 360/NPTS;
Confirm_Message      = 'Solution Exists'   ;% Message Displayed if a Good Artificial Point Exists
Error_Message        = 'No Solution Exists';% Message Displayed if no Good Artificial Point Exists
Bacteria_x           = Robot_Coordinate(1) ;% Artificial Best Point x
Bacteria_y           = Robot_Coordinate(2) ;% Artificial Best Point y
Waypoints_2 = [Robot_Coordinate(1), Robot_Coordinate(2)];

hold on
axis([0 12 0 12])
%--------------------------Target-----------------------------
backx  = Goal_Coordinate_1(1) - 0.15;backy1 = Goal_Coordinate_1(2) + 0.15;
frontx = Goal_Coordinate_1(1) + 0.15;fronty1 = Goal_Coordinate_1(2) + 0.15;
middlex = Goal_Coordinate_1(1);middley1 = Goal_Coordinate_1(2)+ 0.15;
middley2 = Goal_Coordinate_1(2)- 0.15;
tri2 = [backx frontx middlex backx ;backy1 fronty1 middley2 backy1 ];
plot(tri2(1,:), tri2(2,:));
hold on
plot(Goal_Coordinate_1(1),Goal_Coordinate_1(2),"xb")

for l = 1:size(Obstacle_Coordinates,1)
    xc = Obstacle_Coordinates(l,1);
    yc = Obstacle_Coordinates(l,2);
    r = 0.01;
    x_map = r*sin(-pi:0.2*pi:pi) + xc;
    y_map = r*cos(-pi:0.2*pi:pi) + yc;
    c = [0.6 0 1];
    fill(x_map,y_map,c,"FaceAlpha",0.4);
end
DTG = 1;
%---------------------------Robot_Init------------------
backx  = Robot_Coordinate(1) - 0.05;backy1 = Robot_Coordinate(2) + 0.05;
backy2 = Robot_Coordinate(2) - 0.05;frontx = Robot_Coordinate(1) + 0.05;
fronty1 = Robot_Coordinate(2) + 0.05;fronty2 = Robot_Coordinate(2);
tri1 = [backx backx frontx frontx backx;backy2 backy1 fronty1 backy2 backy2];
plot(tri1(1,:), tri1(2,:));
hold on
plot(Robot_Coordinate(1),Robot_Coordinate(2),"xr")
while(DTG>0.05)
    backx  = Robot_Coordinate(1) - 0.05;backy1 = Robot_Coordinate(2) + 0.05;
    backy2 = Robot_Coordinate(2) - 0.05;frontx = Robot_Coordinate(1) + 0.05;
    fronty1 = Robot_Coordinate(2) + 0.05;fronty2 = Robot_Coordinate(2);
    tri1 = [backx backx frontx frontx backx;backy2 backy1 fronty1 backy2 backy2];
    plot(tri1(1,:), tri1(2,:));
    hold on
    title("Second APF Path")
    plot(Robot_Coordinate(1),Robot_Coordinate(2),"xr")


    %.........................Potential Calculations.......................
    J_ObstT = 0;
    for k = 1:size(Obstacle_Coordinates,1)
        J_ObstT = J_ObstT + Obstacle(1)*exp(-Obstacle(2)*((Robot_Coordinate(1)-Obstacle_Coordinates(k,1))^2+(Robot_Coordinate(2)-Obstacle_Coordinates(k,2))^2));
    end
    J_GoalT = -Goal(1)*exp(-Goal(2)*((Robot_Coordinate(1)-Goal_Coordinate_1(1))^2+(Robot_Coordinate(2)-Goal_Coordinate_1(2))^2));
    JT = J_ObstT + J_GoalT;

    %...........................Distance to Goal...........................
    DTG = sqrt((Robot_Coordinate(1)-Goal_Coordinate_1(1))^2+(Robot_Coordinate(2)-Goal_Coordinate_1(2))^2);

    %...........................Artificial Points..........................

    Theta = zeros(1,NPTS);
    Theta(1) = Step_Degree;
    for z = 1:NPTS-1
        Theta(z+1) = Theta(z) + Step_Degree;
    end

    Bacteria_x = zeros(1,NPTS);
    Bacteria_y = zeros(1,NPTS);
    for i=1:NPTS
        Bacteria_x(i) = Robot_Coordinate(1) + (Step_Size*cos(pi*Theta(i)/180));
        Bacteria_y(i) = Robot_Coordinate(2) + (Step_Size*sin(pi*Theta(i)/180));
    end
    plot(Bacteria_x,Bacteria_y,'.',Robot_Coordinate(1),Robot_Coordinate(2),".")
    pause(0.1);

    %........Calculating Cost Function and Distance of Artificial Point....

    [m,n] = size(Bacteria_x);
    J_ObstT_Bacteria = zeros(1,n);
    J_GoalT_Bacteria = zeros(1,n);
    JT_Bacteria = zeros(1,n);
    DTG_Bacteria = zeros(1,n);
    for i=1:n
        for j = 1:size(Obstacle_Coordinates,1)
            J_ObstT_Bacteria(i) = J_ObstT_Bacteria(i) + Obstacle(1)*exp(-Obstacle(2)*((Bacteria_x(i)-Obstacle_Coordinates(j,1))^2+(Bacteria_y(i)-Obstacle_Coordinates(j,2))^2));
        end
        J_GoalT_Bacteria(i) = -Goal(1)*exp(-Goal(2)*((Bacteria_x(i)-Goal_Coordinate_1(1))^2+(Bacteria_y(i)-Goal_Coordinate_1(2))^2));
        JT_Bacteria(i) = J_ObstT_Bacteria(i) + J_GoalT_Bacteria(i);
        DTG_Bacteria(i) = sqrt((Bacteria_x(i)-Goal_Coordinate_1(1))^2+(Bacteria_y(i)-Goal_Coordinate_1(2))^2);
    end
    J_GoalT_Bacteria;

    %....................Error in Cost Function & Distance.................

    err_J   = zeros(1,n);
    err_DTG = zeros(1,n);
    Fitness = zeros(1,n);
    for i=1:n
        err_J(i)   = JT_Bacteria(i)  - JT;
        err_DTG(i) = DTG_Bacteria(i) - DTG;
        Fitness(i) = -err_DTG(i);
    end
    Fitness;
    %..........................Best Point Selection........................
    Check = 0;
    for t=1:n
        [~,k]= max(Fitness);
        if err_J(k) < 0
            if err_DTG(k)<0
                Check = Check + 1;
                Robot_Coordinate(1) = Bacteria_x(k);
                Robot_Coordinate(2) = Bacteria_y(k);
                DTG = sqrt((Robot_Coordinate(1)-Goal_Coordinate_1(1))^2+(Robot_Coordinate(2)-Goal_Coordinate_1(2))^2);
            else
                Fitness(k) = 0;
            end
        else
            Fitness(k) = 0;
        end
    end
    Check;
    if Check == 0
        disp(Error_Message)
        break;
    else
        disp(Confirm_Message);
    end

    Waypoints_2 = [Waypoints_2;Robot_Coordinate(1), Robot_Coordinate(2)];

    %.................................Plot.................................

    %............................Create a Target...........................
    backx  = Goal_Coordinate_1(1) - 0.15;backy1 = Goal_Coordinate_1(2) + 0.15;
    frontx = Goal_Coordinate_1(1) + 0.15;fronty1 = Goal_Coordinate_1(2) + 0.15;
    middlex = Goal_Coordinate_1(1);middley1 = Goal_Coordinate_1(2)+ 0.15;
    middley2 = Goal_Coordinate_1(2)- 0.15;
    tri2 = [backx frontx middlex backx ;backy1 fronty1 middley2 backy1 ];
    hold on
    plot(tri2(1,:), tri2(2,:));

end
Ans = input("Would you like to move the robot according to path? (Y/N)","s");
if Ans == "Y"

    controller_2 = controllerPurePursuit("LookaheadDistance",0.45,"DesiredLinearVelocity",0.1);
    controller_2.Waypoints = Waypoints_2;

    r = wheelDiameter / 2;
    l = interWheelDist;
    c = 0;
    [x_init,y_init,theta] = robot_getPose(connection);

    while true
        [x, y, theta] = robot_getPose(connection);
        pose = [x - x_init, y - y_init, theta];
        Robot_Dist_to_Goal_2 = sqrt((pose(1)-Goal_Coordinate_1(1))^2+(pose(2)-Goal_Coordinate_1(2))^2);
        if(Robot_Dist_to_Goal_2 > 0.2)
            [v_target,w_target,Look_Ahead_Point] = controller_2([pose(1), pose(2), pose(3)]);
            [v_left,v_right] = Kinematic_Conv(v_target,w_target,r,l);
            robot_setWheelSpeeds(connection, v_left, v_right);
            pause(0.01);
            simulation_triggerStep(connection);
            figure(1);
                [laserDataX, laserDataY] = robot_getLaserData(connection);

                angles = zeros(1, size(laserDataX, 2));
                ranges = zeros(1, size(laserDataX, 2));
                range_max = maxRange - 0.1;

                for j = 1:size(laserDataX, 2)
                    angle = atan2(laserDataY(1, j), laserDataX(1, j));
                    range = sqrt(laserDataX(1, j)*laserDataX(1, j) + laserDataY(1, j)*laserDataY(1, j));

                    if (range >= range_max)
                        range = inf;
                        angle = inf;
                    end

                    angles(j) = angle;
                    ranges(j) = range;
                end
                scan = lidarScan(ranges, angles);
                [x, y, theta] = robot_getPose(connection);
                odometryPose = [x, y, theta];
                odometryPose = double(odometryPose);
                [isUpdated,estimatedPose,covariance] = mcl(odometryPose,scan);
                figure(1);
                hold on;
                plot(estimatedPose(1),estimatedPose(2),"rx");
                fprintf("Estimated x: %d, Estimated y: %d , Estimated T: %d \n ",estimatedPose);
        else
            robot_setWheelSpeeds(connection, 0, 0);
            [x, y, theta] = robot_getPose(connection);
            if theta ~= 0
                [v_left,v_right] = Kinematic_Conv(0,0.05,r,l);
                robot_setWheelSpeeds(connection,v_left,v_right);
                while abs(theta) > 0.01
                    simulation_triggerStep(connection);
                    [x, y, theta] = robot_getPose(connection);
                end
            end

            break;
        end

    end
    simulation_triggerStep(connection);



end




% now disable stepped simulation mode:
simulation_setStepped(connection,false);

% stop the simulation
simulation_stop(connection);

% close the connection
simulation_closeConnection(connection);
