% Used for HKUST ELEC 5660

function run6formation()

addpath('./SI_DFM_2D_SYSU','./readonly','utils')
load 'SI_dynamic_fomation_manv_results.mat'

% Sensor parameters
fnoise = 1; % Standard deviation of gaussian noise for external disturbance (N)
ifov = 60; % Camera field of view

% Initialize simulation
global params;
params = quadModel_readonly(); % Quad model

yaw0 = -30 * pi / 180;
pitch0 = -30 * pi / 180;
roll0 = -30 * pi / 180;
Quat0 = R_to_quaternion(ypr_to_R([yaw0, pitch0, roll0])');
%% quadrotor true states initialize
x0 = [0, 0.5, -0.5, -0.5, 0, 0.5; ...
    1, 0.5, 0.5, -0.5, -1, -0.5; ...
    1, 1, 1, 1, 1, 1; ...
    0, 0, 0, 0, 0, 0; ...
    0, 0, 0, 0, 0, 0; ...
    0, 0, 0, 0, 0, 0; ...
    Quat0(1), Quat0(1), Quat0(1), Quat0(1), Quat0(1), Quat0(1); ...
    Quat0(2), Quat0(2), Quat0(2), Quat0(2), Quat0(2), Quat0(2); ...
    Quat0(3), Quat0(3), Quat0(3), Quat0(3), Quat0(3), Quat0(3); ...
    Quat0(4), Quat0(4), Quat0(4), Quat0(4), Quat0(4), Quat0(4); ...
    0, 0, 0, 0, 0, 0; ...
    0, 0, 0, 0, 0, 0; ...
    0, 0, 0, 0, 0, 0];

true_s = x0; % true state
F1 = params.mass * params.grav;
M1 = [0; 0; 0];

F2 = params.mass * params.grav;
M2 = [0; 0; 0];

F3 = params.mass * params.grav;
M3 = [0; 0; 0];

F4 = params.mass * params.grav;
M4 = [0; 0; 0];

F5 = params.mass * params.grav;
M5 = [0; 0; 0];

F6 = params.mass * params.grav;
M6 = [0; 0; 0];

% Time
tstep = 0.002; % Time step for solving equations of motion // FIXME: not 0.01
cstep = 0.01; % Period of calling student code
vstep = 0.05; % visualization interval
time     = 0; % current time
vis_time = 0; % Time of last visualization
time_tol = 25; % Maximum time that the quadrotor is allowed to fly

% Visualization
vis_init = false;


% h1
thprop1 = [];
thprop2 = [];
thprop3 = [];
thprop4 = [];
tharm1 = [];
tharm2 = [];
thfov1 = [];
thfov2 = [];
thfov3 = [];
thfov4 = [];
ehprop1 = [];
ehprop2 = [];
ehprop3 = [];
ehprop4 = [];
eharm1 = [];
eharm2 = [];
ehmap = [];
ehwindow = [];

% h2 h3
thpitch = [];
throll = [];

% Start Simulation run_trajectory_readonly
disp('Start Simulation ...');
while (1)

    % External disturbance
    Fd = randn(3, 1) * fnoise;

    % Run simulation for cstep
    timeint = time:tstep:time + cstep;
    [~, xsave] = ode45(@(t, s) quadEOM_readonly(t, s, F1, M1, Fd), timeint', true_s(:, 1));
    true_s(:, 1) = xsave(end, :)';

    [~, xsave] = ode45(@(t, s) quadEOM_readonly(t, s, F2, M2, Fd), timeint', true_s(:, 2));
    true_s(:, 2) = xsave(end, :)';

    [~, xsave] = ode45(@(t, s) quadEOM_readonly(t, s, F3, M3, Fd), timeint', true_s(:, 3));
    true_s(:, 3) = xsave(end, :)';

    [~, xsave] = ode45(@(t, s) quadEOM_readonly(t, s, F4, M4, Fd), timeint', true_s(:, 4));
    true_s(:, 4) = xsave(end, :)';

    [~, xsave] = ode45(@(t, s) quadEOM_readonly(t, s, F5, M5, Fd), timeint', true_s(:, 5));
    true_s(:, 5) = xsave(end, :)';

    [~, xsave] = ode45(@(t, s) quadEOM_readonly(t, s, F6, M6, Fd), timeint', true_s(:, 6));
    true_s(:, 6) = xsave(end, :)';

    time = time + cstep;

    i = ceil(time/0.1);


    s_des = [x(i, 1), x(i, 2), x(i, 3), x(i, 4), x(i, 5), x(i, 6); ...
        y(i, 1), y(i, 2), y(i, 3), y(i, 4), y(i, 5), y(i, 6); ...
        1, 1, 1, 1, 1, 1; ...
        0, 0, 0, 0, 0, 0; ...
        0, 0, 0, 0, 0, 0; ...
        0, 0, 0, 0, 0, 0; ...
        1, 1, 1, 1, 1, 1; ...
        0, 0, 0, 0, 0, 0; ...
        0, 0, 0, 0, 0, 0; ...
        0, 0, 0, 0, 0, 0; ...
        0, 0, 0, 0, 0, 0; ...
        0, 0, 0, 0, 0, 0; ...
        0, 0, 0, 0, 0, 0];


    [F1, M1] = controller1(time , true_s(:, 1), s_des(:, 1));

    [F2, M2] = controller2(time , true_s(:, 2), s_des(:, 2));

    [F3, M3] = controller3(time , true_s(:, 3), s_des(:, 3));

    [F4, M4] = controller4(time , true_s(:, 4), s_des(:, 4));

    [F5, M5] = controller5(time , true_s(:, 5), s_des(:, 5));

    [F6, M6] = controller6(time , true_s(:, 6), s_des(:, 6));

    if (i >= length(t)*4) || (time >= time_tol)
       save('./readonly/6formation_results.mat', 'time','thtraj1', 'ehtraj1',  'thtraj2', 'ehtraj2', 'thtraj3', 'ehtraj3', ...
            'thtraj4', 'ehtraj4','thtraj5', 'ehtraj5', 'thtraj6', 'ehtraj6', 'thvx1', 'ehvx1', 'thvx2', 'ehvx2','thvx3', 'ehvx3', ...
            'thvx4', 'ehvx4','thvx5', 'ehvx5','thvx6', 'ehvx6','thvy1', 'ehvy1','thvy2', 'ehvy2','thvy3', 'ehvy3','thvy4', 'ehvy4', ...
            'thvy5', 'ehvy5', 'thvy6', 'ehvy6');
       disp("simulation complete!");
        break;
    end
    disp("here");
    disp(i);
    disp("time");
    disp(length(t));
    %% Rlot Results
    if time - vis_time > vstep        
        ll = 0.175 / 3;
        rr = 0.1 / 3;
        ff = 0.3 / 3;
        nprop = 40 / 4;
        propangs = linspace(0, 2*pi, nprop);
        tR = QuatToRot(true_s(7:10, 1))';
        tpoint1 = tR * [ll; 0; 0];
        tpoint2 = tR * [0; ll; 0];
        tpoint3 = tR * [-ll; 0; 0];
        tpoint4 = tR * [0; -ll; 0];
        tproppts = rr * tR * [cos(propangs); sin(propangs); zeros(1, nprop)];
        
        %---
        twp1 = true_s(1:3, 1) + tpoint1;
        twp2 = true_s(1:3, 1) + tpoint2;
        twp3 = true_s(1:3, 1) + tpoint3;
        twp4 = true_s(1:3, 1) + tpoint4;
        tprop1 = tproppts + twp1 * ones(1, nprop);
        tprop2 = tproppts + twp2 * ones(1, nprop);
        tprop3 = tproppts + twp3 * ones(1, nprop);
        tprop4 = tproppts + twp4 * ones(1, nprop);
        tfov0 = true_s(1:3, 1);
        tfov1 = tR * [ff; ff * tan(ifov*pi/180/2); ff * tan(ifov*pi/180/2)] + true_s(1:3, 1);
        tfov2 = tR * [ff; ff * tan(ifov*pi/180/2); -ff * tan(ifov*pi/180/2)] + true_s(1:3, 1);
        tfov3 = tR * [ff; -ff * tan(ifov*pi/180/2); -ff * tan(ifov*pi/180/2)] + true_s(1:3, 1);
        tfov4 = tR * [ff; -ff * tan(ifov*pi/180/2); ff * tan(ifov*pi/180/2)] + true_s(1:3, 1);
        eR = QuatToRot(s_des(7:10, 1))';
        epoint1 = eR * [ll; 0; 0];
        epoint2 = eR * [0; ll; 0];
        epoint3 = eR * [-ll; 0; 0];
        epoint4 = eR * [0; -ll; 0];
        eproppts = rr * eR * [cos(propangs); sin(propangs); zeros(1, nprop)];
        
        
        ewp1 = s_des(1:3, 1) + epoint1;
        ewp2 = s_des(1:3, 1) + epoint2;
        ewp3 = s_des(1:3, 1) + epoint3;
        ewp4 = s_des(1:3, 1) + epoint4;
        eprop1 = eproppts + ewp1 * ones(1, nprop);
        eprop2 = eproppts + ewp2 * ones(1, nprop);
        eprop3 = eproppts + ewp3 * ones(1, nprop);
        eprop4 = eproppts + ewp4 * ones(1, nprop);
        emap = [0; 0; 0];
        ewindow = [0; 0; 0];

        if ~vis_init
            thtraj1.X =  true_s(1, 1);
            thtraj1.Y =  true_s(2, 1);
            thtraj1.Z =  true_s(3, 1);
        else
            thtraj1.X =  [thtraj1.X, true_s(1, 1)];
            thtraj1.Y =  [thtraj1.Y, true_s(2, 1)];
            thtraj1.Z =  [thtraj1.Z, true_s(3, 1)];
        end

       

        thprop11.X = tprop1(1, :);
        thprop11.Y = tprop1(2, :);
        thprop11.Z = tprop1(3, :);

        thprop12.X = tprop2(1, :);
        thprop12.Y = tprop2(2, :);
        thprop12.Z = tprop2(3, :);

        thprop13.X = tprop3(1, :);
        thprop13.Y = tprop3(2, :);
        thprop13.Z = tprop3(3, :);

        thprop14.X = tprop4(1, :);
        thprop14.Y = tprop4(2, :);
        thprop14.Z = tprop4(3, :);
        
        tharm11.X = [twp1(1), twp3(1)];
        tharm11.Y = [twp1(1), twp3(1)];
        tharm11.Z = [twp1(3), twp3(3)];
        
        tharm12.X = [twp2(1), twp4(1)];
        tharm12.Y = [twp2(1), twp4(1)];
        tharm12.Z = [twp2(3), twp4(3)];

        thfov11.X = [tfov0(1), tfov1(1), tfov2(1)];
        thfov11.Y = [tfov0(2), tfov1(2), tfov2(2)];
        thfov11.Z = [tfov0(3), tfov1(3), tfov2(3)];

        thfov12.X = [tfov0(1), tfov2(1), tfov3(1)];
        thfov12.Y = [tfov0(2), tfov2(2), tfov3(2)];
        thfov12.Z = [tfov0(3), tfov2(3), tfov3(3)];

        thfov13.X = [tfov0(1), tfov3(1), tfov4(1)];
        thfov13.Y = [tfov0(2), tfov3(2), tfov4(2)];
        thfov13.Z = [tfov0(3), tfov3(3), tfov4(3)];

        thfov14.X = [tfov0(1), tfov4(1), tfov1(1)];
        thfov14.Y = [tfov0(2), tfov4(2), tfov1(2)];
        thfov14.Z = [tfov0(3), tfov4(3), tfov1(3)];
       
        
        if ~vis_init
            ehtraj1.X = s_des(1,1);
            ehtraj1.Y = s_des(2,1);
            ehtraj1.Z = s_des(3,1);
        else
            ehtraj1.X = [ehtraj1.X, s_des(1,1)];
            ehtraj1.Y = [ehtraj1.Y, s_des(2,1)];
            ehtraj1.Z = [ehtraj1.Z, s_des(3,1)];
        end
        

        ehprop11.X = eprop1(1, :);
        ehprop11.Y = eprop1(2, :);
        ehprop11.Z = eprop1(3, :);
        
        ehprop12.X = eprop2(1, :);
        ehprop12.Y = eprop2(2, :);
        ehprop12.Z = eprop2(3, :);
        
        ehprop13.X = eprop3(1, :);
        ehprop13.Y = eprop3(2, :);
        ehprop13.Z = eprop3(3, :);
        
        ehprop14.X = eprop4(1, :);
        ehprop14.Y = eprop4(2, :);
        ehprop14.Z = eprop4(3, :);
        
        eharm11.X = [ewp1(1), ewp3(1)];
        eharm11.Y = [ewp1(2), ewp3(2)];
        eharm11.Z = [ewp1(3), ewp3(3)];
        
        eharm12.X = [ewp2(1), ewp4(1)];
        eharm12.Y = [ewp2(2), ewp4(2)];
        eharm12.Z = [ewp2(3), ewp4(3)];
        
        
        ehmap1.X = emap(1, :);
        ehmap1.Y = emap(2, :);
        ehmap1.Z = emap(3, :);
        ehwindow1.X = ewindow(1, :);
        ehwindow1.Y = ewindow(2, :);
        ehwindow1.Z = ewindow(3, :);
        disp("here1");
        %---


        twp1 = true_s(1:3, 2) + tpoint1;
        twp2 = true_s(1:3, 2) + tpoint2;
        twp3 = true_s(1:3, 2) + tpoint3;
        twp4 = true_s(1:3, 2) + tpoint4;
        tprop1 = tproppts + twp1 * ones(1, nprop);
        tprop2 = tproppts + twp2 * ones(1, nprop);
        tprop3 = tproppts + twp3 * ones(1, nprop);
        tprop4 = tproppts + twp4 * ones(1, nprop);
        tfov0 = true_s(1:3, 2);
        tfov1 = tR * [ff; ff * tan(ifov*pi/180/2); ff * tan(ifov*pi/180/2)] + true_s(1:3, 2);
        tfov2 = tR * [ff; ff * tan(ifov*pi/180/2); -ff * tan(ifov*pi/180/2)] + true_s(1:3, 2);
        tfov3 = tR * [ff; -ff * tan(ifov*pi/180/2); -ff * tan(ifov*pi/180/2)] + true_s(1:3, 2);
        tfov4 = tR * [ff; -ff * tan(ifov*pi/180/2); ff * tan(ifov*pi/180/2)] + true_s(1:3, 2);
        eR = QuatToRot(s_des(7:10, 2))';
        epoint1 = eR * [ll; 0; 0];
        epoint2 = eR * [0; ll; 0];
        epoint3 = eR * [-ll; 0; 0];
        epoint4 = eR * [0; -ll; 0];
        eproppts = rr * eR * [cos(propangs); sin(propangs); zeros(1, nprop)];


        ewp1 = s_des(1:3, 2) + epoint1;
        ewp2 = s_des(1:3, 2) + epoint2;
        ewp3 = s_des(1:3, 2) + epoint3;
        ewp4 = s_des(1:3, 2) + epoint4;

        eprop1 = eproppts + ewp1 * ones(1, nprop);
        eprop2 = eproppts + ewp2 * ones(1, nprop);
        eprop3 = eproppts + ewp3 * ones(1, nprop);
        eprop4 = eproppts + ewp4 * ones(1, nprop);
        emap = [0; 0; 0];
        ewindow = [0; 0; 0];
    
        if ~vis_init
            thtraj2.X =  true_s(1, 2);
            thtraj2.Y =  true_s(2, 2);
            thtraj2.Z =  true_s(3, 2);
        else
            thtraj2.X =  [thtraj2.X, true_s(1, 2)];
            thtraj2.Y =  [thtraj2.Y, true_s(2, 2)];
            thtraj2.Z =  [thtraj2.Z, true_s(3, 2)];
        end
        thprop21.X = tprop1(1, :);
        thprop21.Y = tprop1(2, :);
        thprop21.Z = tprop1(3, :);

        thprop22.X = tprop2(1, :);
        thprop22.Y = tprop2(2, :);
        thprop22.Z = tprop2(3, :);

        thprop23.X = tprop3(1, :);
        thprop23.Y = tprop3(2, :);
        thprop23.Z = tprop3(3, :);

        thprop24.X = tprop4(1, :);
        thprop24.Y = tprop4(2, :);
        thprop24.Z = tprop4(3, :);
        
        tharm21.X = [twp1(1), twp3(1)];
        tharm21.Y = [twp1(1), twp3(1)];
        tharm21.Z = [twp1(3), twp3(3)];
        
        tharm22.X = [twp2(1), twp4(1)];
        tharm22.Y = [twp2(1), twp4(1)];
        tharm22.Z = [twp2(3), twp4(3)];

        thfov21.X = [tfov0(1), tfov1(1), tfov2(1)];
        thfov21.Y = [tfov0(2), tfov1(2), tfov2(2)];
        thfov21.Z = [tfov0(3), tfov1(3), tfov2(3)];

        thfov22.X = [tfov0(1), tfov2(1), tfov3(1)];
        thfov22.Y = [tfov0(2), tfov2(2), tfov3(2)];
        thfov22.Z = [tfov0(3), tfov2(3), tfov3(3)];
        
        thfov23.X = [tfov0(1), tfov3(1), tfov4(1)];
        thfov23.Y = [tfov0(2), tfov3(2), tfov4(2)];
        thfov23.Z = [tfov0(3), tfov3(3), tfov4(3)];

        thfov24.X = [tfov0(1), tfov4(1), tfov1(1)];
        thfov24.Y = [tfov0(2), tfov4(2), tfov1(2)];
        thfov24.Z = [tfov0(3), tfov4(3), tfov1(3)];
       
        
        if ~vis_init
            ehtraj2.X = s_des(1,2);
            ehtraj2.Y = s_des(2,2);
            ehtraj2.Z = s_des(3,2);
        else
            ehtraj2.X = [ehtraj2.X, s_des(1,2)];
            ehtraj2.Y = [ehtraj2.Y, s_des(2,2)];
            ehtraj2.Z = [ehtraj2.Z, s_des(3,2)];
        end
        
        ehprop21.X = eprop1(1, :);
        ehprop21.Y = eprop1(2, :);
        ehprop21.Z = eprop1(3, :);

        ehprop22.X = eprop2(1, :);
        ehprop22.Y = eprop2(2, :);
        ehprop22.Z = eprop2(3, :);
        
        ehprop23.X = eprop3(1, :);
        ehprop23.Y = eprop3(2, :);
        ehprop23.Z = eprop3(3, :);
        
        ehprop24.X = eprop4(1, :);
        ehprop24.Y = eprop4(2, :);
        ehprop24.Z = eprop4(3, :);
        
        eharm21.X = [ewp1(1), ewp3(1)];
        eharm21.Y = [ewp1(2), ewp3(2)];
        eharm21.Z = [ewp1(3), ewp3(3)];
        
        eharm22.X = [ewp2(1), ewp4(1)];
        eharm22.Y = [ewp2(2), ewp4(2)];
        eharm22.Z = [ewp2(3), ewp4(3)];
        
        
        ehmap2.X = emap(1, :);
        ehmap2.Y = emap(2, :);
        ehmap2.Z = emap(3, :);
        ehwindow2.X = ewindow(1, :);
        ehwindow2.Y = ewindow(2, :);
        ehwindow2.Z = ewindow(3, :);
        %---

        twp1 = true_s(1:3, 3) + tpoint1;
        twp2 = true_s(1:3, 3) + tpoint2;
        twp3 = true_s(1:3, 3) + tpoint3;
        twp4 = true_s(1:3, 3) + tpoint4;
        tprop1 = tproppts + twp1 * ones(1, nprop);
        tprop2 = tproppts + twp2 * ones(1, nprop);
        tprop3 = tproppts + twp3 * ones(1, nprop);
        tprop4 = tproppts + twp4 * ones(1, nprop);
        tfov0 = true_s(1:3, 3);
        tfov1 = tR * [ff; ff * tan(ifov*pi/180/2); ff * tan(ifov*pi/180/2)] + true_s(1:3, 3);
        tfov2 = tR * [ff; ff * tan(ifov*pi/180/2); -ff * tan(ifov*pi/180/2)] + true_s(1:3, 3);
        tfov3 = tR * [ff; -ff * tan(ifov*pi/180/2); -ff * tan(ifov*pi/180/2)] + true_s(1:3, 3);
        tfov4 = tR * [ff; -ff * tan(ifov*pi/180/2); ff * tan(ifov*pi/180/2)] + true_s(1:3, 3);
        eR = QuatToRot(s_des(7:10, 3))';
        epoint1 = eR * [ll; 0; 0];
        epoint2 = eR * [0; ll; 0];
        epoint3 = eR * [-ll; 0; 0];
        epoint4 = eR * [0; -ll; 0];
        eproppts = rr * eR * [cos(propangs); sin(propangs); zeros(1, nprop)];


        ewp1 = s_des(1:3, 3) + epoint1;
        ewp2 = s_des(1:3, 3) + epoint2;
        ewp3 = s_des(1:3, 3) + epoint3;
        ewp4 = s_des(1:3, 3) + epoint4;

        eprop1 = eproppts + ewp1 * ones(1, nprop);
        eprop2 = eproppts + ewp2 * ones(1, nprop);
        eprop3 = eproppts + ewp3 * ones(1, nprop);
        eprop4 = eproppts + ewp4 * ones(1, nprop);
        emap = [0; 0; 0];
        ewindow = [0; 0; 0];
        
        if ~vis_init
            thtraj3.X =  true_s(1, 3);
            thtraj3.Y =  true_s(2, 3);
            thtraj3.Z =  true_s(3, 3);
        else
            thtraj3.X =  [thtraj3.X, true_s(1, 3)];
            thtraj3.Y =  [thtraj3.Y, true_s(2, 3)];
            thtraj3.Z =  [thtraj3.Z, true_s(3, 3)];
        end

       

        thprop31.X = tprop1(1, :);
        thprop31.Y = tprop1(2, :);
        thprop31.Z = tprop1(3, :);

        thprop32.X = tprop2(1, :);
        thprop32.Y = tprop2(2, :);
        thprop32.Z = tprop2(3, :);

        thprop33.X = tprop3(1, :);
        thprop33.Y = tprop3(2, :);
        thprop33.Z = tprop3(3, :);

        thprop34.X = tprop4(1, :);
        thprop34.Y = tprop4(2, :);
        thprop34.Z = tprop4(3, :);
        
        tharm31.X = [twp1(1), twp3(1)];
        tharm31.Y = [twp1(1), twp3(1)];
        tharm31.Z = [twp1(3), twp3(3)];
        
        tharm32.X = [twp2(1), twp4(1)];
        tharm32.Y = [twp2(1), twp4(1)];
        tharm32.Z = [twp2(3), twp4(3)];

        thfov31.X = [tfov0(1), tfov1(1), tfov2(1)];
        thfov31.Y = [tfov0(2), tfov1(2), tfov2(2)];
        thfov31.Z = [tfov0(3), tfov1(3), tfov2(3)];

        thfov32.X = [tfov0(1), tfov2(1), tfov3(1)];
        thfov32.Y = [tfov0(2), tfov2(2), tfov3(2)];
        thfov32.Z = [tfov0(3), tfov2(3), tfov3(3)];

        thfov33.X = [tfov0(1), tfov3(1), tfov4(1)];
        thfov33.Y = [tfov0(2), tfov3(2), tfov4(2)];
        thfov33.Z = [tfov0(3), tfov3(3), tfov4(3)];

        thfov34.X = [tfov0(1), tfov4(1), tfov1(1)];
        thfov34.Y = [tfov0(2), tfov4(2), tfov1(2)];
        thfov34.Z = [tfov0(3), tfov4(3), tfov1(3)];
       
        
        if ~vis_init
            ehtraj3.X = s_des(1,3);
            ehtraj3.Y = s_des(2,3);
            ehtraj3.Z = s_des(3,3);
        else
            ehtraj3.X = [ehtraj3.X, s_des(1,3)];
            ehtraj3.Y = [ehtraj3.Y, s_des(2,3)];
            ehtraj3.Z = [ehtraj3.Z, s_des(3,3)];
        end
        

        ehprop31.X = eprop1(1, :);
        ehprop31.Y = eprop1(2, :);
        ehprop31.Z = eprop1(3, :);

        ehprop32.X = eprop2(1, :);
        ehprop32.Y = eprop2(2, :);
        ehprop32.Z = eprop2(3, :);
        
        ehprop33.X = eprop3(1, :);
        ehprop33.Y = eprop3(2, :);
        ehprop33.Z = eprop3(3, :);
        
        ehprop34.X = eprop4(1, :);
        ehprop34.Y = eprop4(2, :);
        ehprop34.Z = eprop4(3, :);
        
        eharm31.X = [ewp1(1), ewp3(1)];
        eharm31.Y = [ewp1(2), ewp3(2)];
        eharm31.Z = [ewp1(3), ewp3(3)];
        
        eharm32.X = [ewp2(1), ewp4(1)];
        eharm32.Y = [ewp2(2), ewp4(2)];
        eharm32.Z = [ewp2(3), ewp4(3)];
        
        
        ehmap3.X = emap(1, :);
        ehmap3.Y = emap(2, :);
        ehmap3.Z = emap(3, :);
        ehwindow3.X = ewindow(1, :);
        ehwindow3.Y = ewindow(2, :);
        ehwindow3.Z = ewindow(3, :);
        %---


        twp1 = true_s(1:3, 4) + tpoint1;
        twp2 = true_s(1:3, 4) + tpoint2;
        twp3 = true_s(1:3, 4) + tpoint3;
        twp4 = true_s(1:3, 4) + tpoint4;
        tprop1 = tproppts + twp1 * ones(1, nprop);
        tprop2 = tproppts + twp2 * ones(1, nprop);
        tprop3 = tproppts + twp3 * ones(1, nprop);
        tprop4 = tproppts + twp4 * ones(1, nprop);
        tfov0 = true_s(1:3, 4);
        tfov1 = tR * [ff; ff * tan(ifov*pi/180/2); ff * tan(ifov*pi/180/2)] + true_s(1:3, 4);
        tfov2 = tR * [ff; ff * tan(ifov*pi/180/2); -ff * tan(ifov*pi/180/2)] + true_s(1:3, 4);
        tfov3 = tR * [ff; -ff * tan(ifov*pi/180/2); -ff * tan(ifov*pi/180/2)] + true_s(1:3, 4);
        tfov4 = tR * [ff; -ff * tan(ifov*pi/180/2); ff * tan(ifov*pi/180/2)] + true_s(1:3, 4);
        eR = QuatToRot(s_des(7:10, 4))';
        epoint1 = eR * [ll; 0; 0];
        epoint2 = eR * [0; ll; 0];
        epoint3 = eR * [-ll; 0; 0];
        epoint4 = eR * [0; -ll; 0];
        eproppts = rr * eR * [cos(propangs); sin(propangs); zeros(1, nprop)];


        ewp1 = s_des(1:3, 4) + epoint1;
        ewp2 = s_des(1:3, 4) + epoint2;
        ewp3 = s_des(1:3, 4) + epoint3;
        ewp4 = s_des(1:3, 4) + epoint4;

        eprop1 = eproppts + ewp1 * ones(1, nprop);
        eprop2 = eproppts + ewp2 * ones(1, nprop);
        eprop3 = eproppts + ewp3 * ones(1, nprop);
        eprop4 = eproppts + ewp4 * ones(1, nprop);
        emap = [0; 0; 0];
        ewindow = [0; 0; 0];

        if ~vis_init
            thtraj4.X =  true_s(1, 4);
            thtraj4.Y =  true_s(2, 4);
            thtraj4.Z =  true_s(3, 4);
        else
            thtraj4.X =  [thtraj4.X, true_s(1, 4)];
            thtraj4.Y =  [thtraj4.Y, true_s(2, 4)];
            thtraj4.Z =  [thtraj4.Z, true_s(3, 4)];
        end

       

        thprop41.X = tprop1(1, :);
        thprop41.Y = tprop1(2, :);
        thprop41.Z = tprop1(3, :);

        thprop42.X = tprop2(1, :);
        thprop42.Y = tprop2(2, :);
        thprop42.Z = tprop2(3, :);

        thprop43.X = tprop3(1, :);
        thprop43.Y = tprop3(2, :);
        thprop43.Z = tprop3(3, :);

        thprop44.X = tprop4(1, :);
        thprop44.Y = tprop4(2, :);
        thprop44.Z = tprop4(3, :);
        
        tharm41.X = [twp1(1), twp3(1)];
        tharm41.Y = [twp1(1), twp3(1)];
        tharm41.Z = [twp1(3), twp3(3)];
        
        tharm42.X = [twp2(1), twp4(1)];
        tharm42.Y = [twp2(1), twp4(1)];
        tharm42.Z = [twp2(3), twp4(3)];

        thfov41.X = [tfov0(1), tfov1(1), tfov2(1)];
        thfov41.Y = [tfov0(2), tfov1(2), tfov2(2)];
        thfov41.Z = [tfov0(3), tfov1(3), tfov2(3)];

        thfov42.X = [tfov0(1), tfov2(1), tfov3(1)];
        thfov42.Y = [tfov0(2), tfov2(2), tfov3(2)];
        thfov42.Z = [tfov0(3), tfov2(3), tfov3(3)];

        thfov43.X = [tfov0(1), tfov3(1), tfov4(1)];
        thfov43.Y = [tfov0(2), tfov3(2), tfov4(2)];
        thfov43.Z = [tfov0(3), tfov3(3), tfov4(3)];

        thfov44.X = [tfov0(1), tfov4(1), tfov1(1)];
        thfov44.Y = [tfov0(2), tfov4(2), tfov1(2)];
        thfov44.Z = [tfov0(3), tfov4(3), tfov1(3)];
       
        
        if ~vis_init
            ehtraj4.X = s_des(1,4);
            ehtraj4.Y = s_des(2,4);
            ehtraj4.Z = s_des(3,4);
        else
            ehtraj4.X = [ehtraj1.X, s_des(1,4)];
            ehtraj4.Y = [ehtraj1.Y, s_des(2,4)];
            ehtraj4.Z = [ehtraj1.Z, s_des(3,4)];
        end
        

        ehprop41.X = eprop1(1, :);
        ehprop41.Y = eprop1(2, :);
        ehprop41.Z = eprop1(3, :);
        
        ehprop42.X = eprop2(1, :);
        ehprop42.Y = eprop2(2, :);
        ehprop42.Z = eprop2(3, :);
        
        ehprop43.X = eprop3(1, :);
        ehprop43.Y = eprop3(2, :);
        ehprop43.Z = eprop3(3, :);
        
        ehprop44.X = eprop4(1, :);
        ehprop44.Y = eprop4(2, :);
        ehprop44.Z = eprop4(3, :);
        
        eharm41.X = [ewp1(1), ewp3(1)];
        eharm41.Y = [ewp1(2), ewp3(2)];
        eharm41.Z = [ewp1(3), ewp3(3)];
        
        eharm42.X = [ewp2(1), ewp4(1)];
        eharm42.Y = [ewp2(2), ewp4(2)];
        eharm42.Z = [ewp2(3), ewp4(3)];
        
        ehmap4.X = emap(1, :);
        ehmap4.Y = emap(2, :);
        ehmap4.Z = emap(3, :);
        ehwindow4.X = ewindow(1, :);
        ehwindow4.Y = ewindow(2, :);
        ehwindow4.Z = ewindow(3, :);
        
        %---
        twp1 = true_s(1:3, 5) + tpoint1;
        twp2 = true_s(1:3, 5) + tpoint2;
        twp3 = true_s(1:3, 5) + tpoint3;
        twp4 = true_s(1:3, 5) + tpoint4;
        tprop1 = tproppts + twp1 * ones(1, nprop);
        tprop2 = tproppts + twp2 * ones(1, nprop);
        tprop3 = tproppts + twp3 * ones(1, nprop);
        tprop4 = tproppts + twp4 * ones(1, nprop);
        tfov0 = true_s(1:3, 5);
        tfov1 = tR * [ff; ff * tan(ifov*pi/180/2); ff * tan(ifov*pi/180/2)] + true_s(1:3, 5);
        tfov2 = tR * [ff; ff * tan(ifov*pi/180/2); -ff * tan(ifov*pi/180/2)] + true_s(1:3, 5);
        tfov3 = tR * [ff; -ff * tan(ifov*pi/180/2); -ff * tan(ifov*pi/180/2)] + true_s(1:3, 5);
        tfov4 = tR * [ff; -ff * tan(ifov*pi/180/2); ff * tan(ifov*pi/180/2)] + true_s(1:3, 5);
        eR = QuatToRot(s_des(7:10, 5))';
        epoint1 = eR * [ll; 0; 0];
        epoint2 = eR * [0; ll; 0];
        epoint3 = eR * [-ll; 0; 0];
        epoint4 = eR * [0; -ll; 0];
        eproppts = rr * eR * [cos(propangs); sin(propangs); zeros(1, nprop)];


        ewp1 = s_des(1:3, 5) + epoint1;
        ewp2 = s_des(1:3, 5) + epoint2;
        ewp3 = s_des(1:3, 5) + epoint3;
        ewp4 = s_des(1:3, 5) + epoint4;

        eprop1 = eproppts + ewp1 * ones(1, nprop);
        eprop2 = eproppts + ewp2 * ones(1, nprop);
        eprop3 = eproppts + ewp3 * ones(1, nprop);
        eprop4 = eproppts + ewp4 * ones(1, nprop);
        emap = [0; 0; 0];
        ewindow = [0; 0; 0];

        if ~vis_init
            thtraj5.X =  true_s(1, 5);
            thtraj5.Y =  true_s(2, 5);
            thtraj5.Z =  true_s(3, 5);
        else
            thtraj5.X =  [thtraj5.X, true_s(1, 5)];
            thtraj5.Y =  [thtraj5.Y, true_s(2, 5)];
            thtraj5.Z =  [thtraj5.Z, true_s(3, 5)];
        end

        thprop51.X = tprop1(1, :);
        thprop51.Y = tprop1(2, :);
        thprop51.Z = tprop1(3, :);

        thprop52.X = tprop2(1, :);
        thprop52.Y = tprop2(2, :);
        thprop52.Z = tprop2(3, :);
        
        thprop53.X = tprop3(1, :);
        thprop53.Y = tprop3(2, :);
        thprop53.Z = tprop3(3, :);

        thprop54.X = tprop4(1, :);
        thprop54.Y = tprop4(2, :);
        thprop54.Z = tprop4(3, :);
        
        tharm51.X = [twp1(1), twp3(1)];
        tharm51.Y = [twp1(1), twp3(1)];
        tharm51.Z = [twp1(3), twp3(3)];
        
        tharm52.X = [twp2(1), twp4(1)];
        tharm52.Y = [twp2(1), twp4(1)];
        tharm52.Z = [twp2(3), twp4(3)];

        thfov51.X = [tfov0(1), tfov1(1), tfov2(1)];
        thfov51.Y = [tfov0(2), tfov1(2), tfov2(2)];
        thfov51.Z = [tfov0(3), tfov1(3), tfov2(3)];

        thfov52.X = [tfov0(1), tfov2(1), tfov3(1)];
        thfov52.Y = [tfov0(2), tfov2(2), tfov3(2)];
        thfov52.Z = [tfov0(3), tfov2(3), tfov3(3)];

        thfov53.X = [tfov0(1), tfov3(1), tfov4(1)];
        thfov53.Y = [tfov0(2), tfov3(2), tfov4(2)];
        thfov53.Z = [tfov0(3), tfov3(3), tfov4(3)];

        thfov54.X = [tfov0(1), tfov4(1), tfov1(1)];
        thfov54.Y = [tfov0(2), tfov4(2), tfov1(2)];
        thfov54.Z = [tfov0(3), tfov4(3), tfov1(3)];
       
        
        if ~vis_init
            ehtraj5.X = s_des(1,5);
            ehtraj5.Y = s_des(2,5);
            ehtraj5.Z = s_des(3,5);
        else
            ehtraj5.X = [ehtraj5.X, s_des(1,5)];
            ehtraj5.Y = [ehtraj5.Y, s_des(2,5)];
            ehtraj5.Z = [ehtraj5.Z, s_des(3,5)];
        end
        

        ehprop51.X = eprop1(1, :);
        ehprop51.Y = eprop1(2, :);
        ehprop51.Z = eprop1(3, :);
        
        ehprop52.X = eprop2(1, :);
        ehprop52.Y = eprop2(2, :);
        ehprop52.Z = eprop2(3, :);
        
        ehprop53.X = eprop3(1, :);
        ehprop53.Y = eprop3(2, :);
        ehprop53.Z = eprop3(3, :);
        
        ehprop54.X = eprop4(1, :);
        ehprop54.Y = eprop4(2, :);
        ehprop54.Z = eprop4(3, :);
        
        eharm51.X = [ewp1(1), ewp3(1)];
        eharm51.Y = [ewp1(2), ewp3(2)];
        eharm51.Z = [ewp1(3), ewp3(3)];
        
        eharm52.X = [ewp2(1), ewp4(1)];
        eharm52.Y = [ewp2(2), ewp4(2)];
        eharm52.Z = [ewp2(3), ewp4(3)];
        
        ehmap5.X = emap(1, :);
        ehmap5.Y = emap(2, :);
        ehmap5.Z = emap(3, :);
        ehwindow5.X = ewindow(1, :);
        ehwindow5.Y = ewindow(2, :);
        ehwindow5.Z = ewindow(3, :);
        
        %---

        twp1 = true_s(1:3, 6) + tpoint1;
        twp2 = true_s(1:3, 6) + tpoint2;
        twp3 = true_s(1:3, 6) + tpoint3;
        twp4 = true_s(1:3, 6) + tpoint4;
        tprop1 = tproppts + twp1 * ones(1, nprop);
        tprop2 = tproppts + twp2 * ones(1, nprop);
        tprop3 = tproppts + twp3 * ones(1, nprop);
        tprop4 = tproppts + twp4 * ones(1, nprop);
        tfov0 = true_s(1:3, 6);
        tfov1 = tR * [ff; ff * tan(ifov*pi/180/2); ff * tan(ifov*pi/180/2)] + true_s(1:3, 6);
        tfov2 = tR * [ff; ff * tan(ifov*pi/180/2); -ff * tan(ifov*pi/180/2)] + true_s(1:3, 6);
        tfov3 = tR * [ff; -ff * tan(ifov*pi/180/2); -ff * tan(ifov*pi/180/2)] + true_s(1:3, 6);
        tfov4 = tR * [ff; -ff * tan(ifov*pi/180/2); ff * tan(ifov*pi/180/2)] + true_s(1:3, 6);
        eR = QuatToRot(s_des(7:10, 6))';
        epoint1 = eR * [ll; 0; 0];
        epoint2 = eR * [0; ll; 0];
        epoint3 = eR * [-ll; 0; 0];
        epoint4 = eR * [0; -ll; 0];
        eproppts = rr * eR * [cos(propangs); sin(propangs); zeros(1, nprop)];


        ewp1 = s_des(1:3, 6) + epoint1;
        ewp2 = s_des(1:3, 6) + epoint2;
        ewp3 = s_des(1:3, 6) + epoint3;
        ewp4 = s_des(1:3, 6) + epoint4;

        eprop1 = eproppts + ewp1 * ones(1, nprop);
        eprop2 = eproppts + ewp2 * ones(1, nprop);
        eprop3 = eproppts + ewp3 * ones(1, nprop);
        eprop4 = eproppts + ewp4 * ones(1, nprop);
        emap = [0; 0; 0];
        ewindow = [0; 0; 0];

        if ~vis_init
            thtraj6.X =  true_s(1, 6);
            thtraj6.Y =  true_s(2, 6);
            thtraj6.Z =  true_s(3, 6);
        else
            thtraj6.X =  [thtraj6.X, true_s(1, 6)];
            thtraj6.Y =  [thtraj6.Y, true_s(2, 6)];
            thtraj6.Z =  [thtraj6.Z, true_s(3, 6)];
        end

        thprop61.X = tprop1(1, :);
        thprop61.Y = tprop1(2, :);
        thprop61.Z = tprop1(3, :);

        thprop62.X = tprop2(1, :);
        thprop62.Y = tprop2(2, :);
        thprop62.Z = tprop2(3, :);
        
        thprop63.X = tprop3(1, :);
        thprop63.Y = tprop3(2, :);
        thprop63.Z = tprop3(3, :);

        thprop64.X = tprop4(1, :);
        thprop64.Y = tprop4(2, :);
        thprop64.Z = tprop4(3, :);
        
        tharm61.X = [twp1(1), twp3(1)];
        tharm61.Y = [twp1(1), twp3(1)];
        tharm61.Z = [twp1(3), twp3(3)];
        
        tharm62.X = [twp2(1), twp4(1)];
        tharm62.Y = [twp2(1), twp4(1)];
        tharm62.Z = [twp2(3), twp4(3)];

        thfov61.X = [tfov0(1), tfov1(1), tfov2(1)];
        thfov61.Y = [tfov0(2), tfov1(2), tfov2(2)];
        thfov61.Z = [tfov0(3), tfov1(3), tfov2(3)];

        thfov62.X = [tfov0(1), tfov2(1), tfov3(1)];
        thfov62.Y = [tfov0(2), tfov2(2), tfov3(2)];
        thfov62.Z = [tfov0(3), tfov2(3), tfov3(3)];

        thfov63.X = [tfov0(1), tfov3(1), tfov4(1)];
        thfov63.Y = [tfov0(2), tfov3(2), tfov4(2)];
        thfov63.Z = [tfov0(3), tfov3(3), tfov4(3)];

        thfov64.X = [tfov0(1), tfov4(1), tfov1(1)];
        thfov64.Y = [tfov0(2), tfov4(2), tfov1(2)];
        thfov64.Z = [tfov0(3), tfov4(3), tfov1(3)];
       
        
        if ~vis_init
            ehtraj6.X = s_des(1,6);
            ehtraj6.Y = s_des(2,6);
            ehtraj6.Z = s_des(3,6);
        else
            ehtraj6.X = [ehtraj6.X, s_des(1,6)];
            ehtraj6.Y = [ehtraj6.Y, s_des(2,6)];
            ehtraj6.Z = [ehtraj6.Z, s_des(3,6)];
        end
        
        ehprop61.X = eprop1(1, :);
        ehprop61.Y = eprop1(2, :);
        ehprop61.Z = eprop1(3, :);
        
        ehprop62.X = eprop2(1, :);
        ehprop62.Y = eprop2(2, :);
        ehprop62.Z = eprop2(3, :);
        
        ehprop63.X = eprop3(1, :);
        ehprop63.Y = eprop3(2, :);
        ehprop63.Z = eprop3(3, :);
        
        ehprop64.X = eprop4(1, :);
        ehprop64.Y = eprop4(2, :);
        ehprop64.Z = eprop4(3, :);
        
        eharm61.X = [ewp1(1), ewp3(1)];
        eharm61.Y = [ewp1(2), ewp3(2)];
        eharm61.Z = [ewp1(3), ewp3(3)];
        
        eharm62.X = [ewp2(1), ewp4(1)];
        eharm62.Y = [ewp2(2), ewp4(2)];
        eharm62.Z = [ewp2(3), ewp4(3)];
        
        ehmap6.X = emap(1, :);
        ehmap6.Y = emap(2, :);
        ehmap6.Z = emap(3, :);
        ehwindow6.X = ewindow(1, :);
        ehwindow6.Y = ewindow(2, :);
        ehwindow6.Z = ewindow(3, :);
        %---

        %% Plot roll oriengation
        true_ypr = R_to_ypr(quaternion_to_R(true_s(7:10, 1))') * 180 / pi;
        true_ypr2 = R_to_ypr(quaternion_to_R(true_s(7:10, 2))') * 180 / pi;
        true_ypr3 = R_to_ypr(quaternion_to_R(true_s(7:10, 3))') * 180 / pi;
        true_ypr4 = R_to_ypr(quaternion_to_R(true_s(7:10, 4))') * 180 / pi;
        true_ypr5 = R_to_ypr(quaternion_to_R(true_s(7:10, 5))') * 180 / pi;
        true_ypr6 = R_to_ypr(quaternion_to_R(true_s(7:10, 6))') * 180 / pi;
         
        if ~vis_init
            throll.X = time;
            throll.Y = true_ypr(3);
            
            throll2.X = time;
            throll2.Y = true_ypr2(3);
            
            throll3.X = time;
            throll3.Y = true_ypr3(3);
           
            throll4.X = time;
            throll4.Y = true_ypr4(3);
            
            throll5.X = time;
            throll5.Y = true_ypr5(3);
            
            throll6.X = time;
            throll6.Y = true_ypr6(3);
        else
           throll.X = [throll.X, time];
           throll.Y = [throll.Y, true_ypr(3)];
           
           throll2.X = [throll.X, time];
           throll2.Y = [throll.Y, true_ypr2(3)];
           
           throll3.X = [throll.X, time];
           throll3.Y = [throll.Y, true_ypr3(3)];
           
           throll4.X = [throll.X, time];
           throll4.Y = [throll.Y, true_ypr4(3)];
           
           throll5.X = [throll.X, time];
           throll5.Y = [throll.Y, true_ypr5(3)];
           
           throll6.X = [throll.X, time];
           throll6.Y = [throll.Y, true_ypr6(3)];
        end
        %% Plot body frame velocity
        true_v1 = true_s(4:6, 1);
        des_v1 = s_des(4:6, 1);

        true_v2 = true_s(4:6, 2);
        des_v2 = s_des(4:6, 2);
        
        true_v3 = true_s(4:6, 3);
        des_v3 = s_des(4:6, 3);
        
        true_v4 = true_s(4:6, 4);
        des_v4 = s_des(4:6, 4);

        true_v5 = true_s(4:6, 5);
        des_v5 = s_des(4:6, 5);
        
        true_v6 = true_s(4:6, 6);
        des_v6 = s_des(4:6, 6);
        
        if ~vis_init 
            thvx1.X = time; thvx1.Y = true_v1(1);
            ehvx1.X = time; ehvx1.Y = des_v1(1);
            thvy1.X = time; thvy1.Y = true_v1(2);
            ehvy1.X = time; ehvy1.Y = des_v1(2);
            thvz1.X = time; thvz1.Y = true_v1(3);
            ehvz1.X = time; ehvz1.Y = des_v1(3);
            
            thvx2.X = time; thvx2.Y = true_v2(1);
            ehvx2.X = time; ehvx2.Y = des_v2(1);
            thvy2.X = time; thvy2.Y = true_v2(2);
            ehvy2.X = time; ehvy2.Y = des_v2(2);
            thvz2.X = time; thvz2.Y = true_v2(3);
            ehvz2.X = time; ehvz2.Y = des_v2(3);
            
            thvx3.X = time; thvx3.Y = true_v3(1);
            ehvx3.X = time; ehvx3.Y = des_v3(1);
            thvy3.X = time; thvy3.Y = true_v3(2);
            ehvy3.X = time; ehvy3.Y = des_v3(2);
            thvz3.X = time; thvz3.Y = true_v3(3);
            ehvz3.X = time; ehvz3.Y = des_v3(3);
            
            thvx4.X = time; thvx4.Y = true_v4(1);
            ehvx4.X = time; ehvx4.Y = des_v4(1);
            thvy4.X = time; thvy4.Y = true_v4(2);
            ehvy4.X = time; ehvy4.Y = des_v4(2);
            thvz4.X = time; thvz4.Y = true_v4(3);
            ehvz4.X = time; ehvz4.Y = des_v4(3);
            
            thvx5.X = time; thvx5.Y = true_v5(1);
            ehvx5.X = time; ehvx5.Y = des_v5(1);
            thvy5.X = time; thvy5.Y = true_v5(2);
            ehvy5.X = time; ehvy5.Y = des_v5(2);
            thvz5.X = time; thvz5.Y = true_v5(3);
            ehvz5.X = time; ehvz5.Y = des_v5(3);
            
            thvx6.X = time; thvx6.Y = true_v6(1);
            ehvx6.X = time; ehvx6.Y = des_v6(1);
            thvy6.X = time; thvy6.Y = true_v6(2);
            ehvy6.X = time; ehvy6.Y = des_v6(2);
            thvz6.X = time; thvz6.Y = true_v6(3);
            ehvz6.X = time; ehvz6.Y = des_v6(3);
        else
            thvx1.X = [thvx1.X, time]; thvx1.Y = [thvx1.Y, true_v1(1)];
            ehvx1.X = [ehvx1.X, time]; ehvx1.Y = [ehvx1.Y, des_v1(1)];
            thvy1.X = [thvy1.X, time]; thvy1.Y = [thvx1.Y, true_v1(2)];
            ehvy1.X = [ehvy1.X, time]; ehvy1.Y = [ehvx1.Y, des_v1(2)];
            thvz1.X = [thvz1.X, time]; thvz1.Y = [thvx1.Y, true_v1(3)];
            ehvz1.X = [ehvz1.X, time]; ehvz1.Y = [ehvx1.Y, des_v1(3)];
     
            thvx2.X = [thvx2.X, time]; thvx2.Y = [thvx2.Y, true_v2(1)];
            ehvx2.X = [ehvx2.X, time]; ehvx2.Y = [ehvx2.Y, des_v2(1)];
            thvy2.X = [thvy2.X, time]; thvy2.Y = [thvy2.Y, true_v2(2)];
            ehvy2.X = [ehvy2.X, time]; ehvy2.Y = [ehvy2.Y, des_v2(2)];
            thvz2.X = [thvz2.X, time]; thvz2.Y = [thvz2.Y, true_v2(3)];
            ehvz2.X = [ehvz2.X, time]; ehvz2.Y = [ehvz2.Y, des_v2(3)];
            
            thvx3.X = [thvx3.X, time]; thvx3.Y = [thvx3.Y, true_v3(1)];
            ehvx3.X = [ehvx3.X, time]; ehvx3.Y = [ehvx3.Y, des_v3(1)];
            thvy3.X = [thvy3.X, time]; thvy3.Y = [thvy3.Y, true_v3(2)];
            ehvy3.X = [ehvy3.X, time]; ehvy3.Y = [ehvy3.Y, des_v3(2)];
            thvz3.X = [thvz3.X, time]; thvz3.Y = [thvz3.Y, true_v3(3)];
            ehvz3.X = [ehvz3.X, time]; ehvz3.Y = [ehvz3.Y, des_v3(3)];
           
            thvx4.X = [thvx4.X, time]; thvx4.Y = [thvx4.Y, true_v4(1)];
            ehvx4.X = [ehvx4.X, time]; ehvx4.Y = [ehvx4.Y, des_v4(1)];
            thvy4.X = [thvy4.X, time]; thvy4.Y = [thvy4.Y, true_v4(2)];
            ehvy4.X = [ehvy4.X, time]; ehvy4.Y = [ehvy4.Y, des_v4(2)];
            thvz4.X = [thvz4.X, time]; thvz4.Y = [thvz4.Y, true_v4(3)];
            ehvz4.X = [ehvz4.X, time]; ehvz4.Y = [ehvz4.Y, des_v4(3)];
            
            thvx5.X = [thvx5.X, time]; thvx5.Y = [thvx5.Y, true_v5(1)];
            ehvx5.X = [ehvx5.X, time]; ehvx5.Y = [ehvx5.Y, des_v5(1)];
            thvy5.X = [thvy5.X, time]; thvy5.Y = [thvy5.Y, true_v5(2)];
            ehvy5.X = [ehvy5.X, time]; ehvy5.Y = [ehvy5.Y, des_v5(2)];
            thvz5.X = [thvz5.X, time]; thvz5.Y = [thvz5.Y, true_v5(3)];
            ehvz5.X = [ehvz5.X, time]; ehvz5.Y = [ehvz5.Y, des_v5(3)];
            
            thvx6.X = [thvx6.X, time]; thvx6.Y = [thvx6.Y, true_v6(1)];
            ehvx6.X = [ehvx6.X, time]; ehvx6.Y = [ehvx6.Y, des_v6(1)];
            thvy6.X = [thvy6.X, time]; thvy6.Y = [thvy6.Y, true_v6(2)];
            ehvy6.X = [ehvy6.X, time]; ehvy6.Y = [ehvy6.Y, des_v6(2)];
            thvz6.X = [thvz6.X, time]; thvz6.Y = [thvz6.Y, true_v6(3)];
            ehvz6.X = [ehvz6.X, time]; ehvz6.Y = [ehvz6.Y, des_v6(3)];
        end
            
          
        
        %% Plot world frame position

        true_p1 = true_s(1:3, 1);
        des_p1 = s_des(1:3, 1);
        
        true_p2 = true_s(1:3, 2);
        des_p2 = s_des(1:3, 2);
        
        true_p3 = true_s(1:3, 3);
        des_p3 = s_des(1:3, 3);
        
        true_p4 = true_s(1:3, 4);
        des_p4 = s_des(1:3, 4);
        
        true_p5 = true_s(1:3, 5);
        des_p5 = s_des(1:3, 5);
        
        true_p6 = true_s(1:3, 6);
        des_p6 = s_des(1:3, 6);
        
        if ~vis_init 
            thpx1.X = time; thpx1.Y = true_p1(1);
            ehpx1.X = time; ehpx1.Y = des_p1(1);
            thpy1.X = time; thpy1.Y = true_p1(2);
            ehpy1.X = time; ehpy1.Y = des_p1(2);
            thpz1.X = time; thpz1.Y = true_p1(3);
            ehpz1.X = time; ehpz1.Y = des_p1(3);
            
            thpx2.X = time; thpx2.Y = true_p2(1);
            ehpx2.X = time; ehpx2.Y = des_p2(1);
            thpy2.X = time; thpy2.Y = true_p2(2);
            ehpy2.X = time; ehpy2.Y = des_p2(2);
            thpz2.X = time; thpz2.Y = true_p2(3);
            ehpz2.X = time; ehpz2.Y = des_p2(3);
            
            thpx3.X = time; thpx3.Y = true_p3(1);
            ehpx3.X = time; ehpx3.Y = des_p3(1);
            thpy3.X = time; thpy3.Y = true_p3(2);
            ehpy3.X = time; ehpy3.Y = des_p3(2);
            thpz3.X = time; thpz3.Y = true_p3(3);
            ehpz3.X = time; ehpz3.Y = des_p3(3);
            
            thpx4.X = time; thpx4.Y = true_p4(1);
            ehpx4.X = time; ehpx4.Y = des_p4(1);
            thpy4.X = time; thpy4.Y = true_p4(2);
            ehpy4.X = time; ehpy4.Y = des_p4(2);
            thpz4.X = time; thpz4.Y = true_p4(3);
            ehpz4.X = time; ehpz4.Y = des_p4(3);
            
            thpx5.X = time; thpx5.Y = true_p5(1);
            ehpx5.X = time; ehpx5.Y = des_p5(1);
            thpy5.X = time; thpy5.Y = true_p5(2);
            ehpy5.X = time; ehpy5.Y = des_p5(2);
            thpz5.X = time; thpz5.Y = true_p5(3);
            ehpz5.X = time; ehpz5.Y = des_p5(3);
            
            thpx6.X = time; thpx6.Y = true_p6(1);
            ehpx6.X = time; ehpx6.Y = des_p6(1);
            thpy6.X = time; thpy6.Y = true_p6(2);
            ehpy6.X = time; ehpy6.Y = des_p6(2);
            thpz6.X = time; thpz6.Y = true_p6(3);
            ehpz6.X = time; ehpz6.Y = des_p6(3);
        else
            thpx1.X = [thpx1.X, time]; thpx1.Y = [thpx1.Y, true_p1(1)];
            ehpx1.X = [ehpx1.X, time]; ehpx1.Y = [ehpx1.Y, des_p1(1)];
            thpy1.X = [thpy1.X, time]; thpy1.Y = [thpx1.Y, true_p1(2)];
            ehpy1.X = [ehpy1.X, time]; ehpy1.Y = [ehpx1.Y, des_p1(2)];
            thpz1.X = [thpz1.X, time]; thpz1.Y = [thpx1.Y, true_p1(3)];
            ehpz1.X = [ehpz1.X, time]; ehpz1.Y = [ehpx1.Y, des_p1(3)];
     
            thpx2.X = [thpx2.X, time]; thpx2.Y = [thpx2.Y, true_p2(1)];
            ehpx2.X = [ehpx2.X, time]; ehpx2.Y = [ehpx2.Y, des_p2(1)];
            thpy2.X = [thpy2.X, time]; thpy2.Y = [thpy2.Y, true_p2(2)];
            ehpy2.X = [ehpy2.X, time]; ehpy2.Y = [ehpy2.Y, des_p2(2)];
            thpz2.X = [thpz2.X, time]; thpz2.Y = [thpz2.Y, true_p2(3)];
            ehpz2.X = [ehpz2.X, time]; ehpz2.Y = [ehpz2.Y, des_p2(3)];
            
            thpx3.X = [thpx3.X, time]; thpx3.Y = [thpx3.Y, true_p3(1)];
            ehpx3.X = [ehpx3.X, time]; ehpx3.Y = [ehpx3.Y, des_p3(1)];
            thpy3.X = [thpy3.X, time]; thpy3.Y = [thpy3.Y, true_p3(2)];
            ehpy3.X = [ehpy3.X, time]; ehpy3.Y = [ehpy3.Y, des_p3(2)];
            thpz3.X = [thpz3.X, time]; thpz3.Y = [thpz3.Y, true_p3(3)];
            ehpz3.X = [ehpz3.X, time]; ehpz3.Y = [ehpz3.Y, des_p3(3)];
           
            thpx4.X = [thpx4.X, time]; thpx4.Y = [thpx4.Y, true_p4(1)];
            ehpx4.X = [ehpx4.X, time]; ehpx4.Y = [ehpx4.Y, des_p4(1)];
            thpy4.X = [thpy4.X, time]; thpy4.Y = [thpy4.Y, true_p4(2)];
            ehpy4.X = [ehpy4.X, time]; ehpy4.Y = [ehpy4.Y, des_p4(2)];
            thpz4.X = [thpz4.X, time]; thpz4.Y = [thpz4.Y, true_p4(3)];
            ehpz4.X = [ehpz4.X, time]; ehpz4.Y = [ehpz4.Y, des_p4(3)];
            
            thpx5.X = [thpx5.X, time]; thpx5.Y = [thpx5.Y, true_p5(1)];
            ehpx5.X = [ehpx5.X, time]; ehpx5.Y = [ehpx5.Y, des_p5(1)];
            thpy5.X = [thpy5.X, time]; thpy5.Y = [thpy5.Y, true_p5(2)];
            ehpy5.X = [ehpy5.X, time]; ehpy5.Y = [ehpy5.Y, des_p5(2)];
            thpz5.X = [thpz5.X, time]; thpz5.Y = [thpz5.Y, true_p5(3)];
            ehpz5.X = [ehpz5.X, time]; ehpz5.Y = [ehpz5.Y, des_p5(3)];
            
            thpx6.X = [thpx6.X, time]; thpx6.Y = [thpx6.Y, true_p6(1)];
            ehpx6.X = [ehpx6.X, time]; ehpx6.Y = [ehpx6.Y, des_p6(1)];
            thpy6.X = [thpy6.X, time]; thpy6.Y = [thpy6.Y, true_p6(2)];
            ehpy6.X = [ehpy6.X, time]; ehpy6.Y = [ehpy6.Y, des_p6(2)];
            thpz6.X = [thpz6.X, time]; thpz6.Y = [thpz6.Y, true_p6(3)];
            ehpz6.X = [ehpz6.X, time]; ehpz6.Y = [ehpz6.Y, des_p6(3)];
        end
        
        %% Render
        drawnow;
        vis_time = time;
        vis_init = true;
    end
end
