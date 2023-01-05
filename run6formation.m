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
            'thvy5', 'ehvy5', 'thvy6', 'ehvy6', 'x0');
       disp("simulation complete!");
        break;
    end

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
        
        if ~vis_init
            ehtraj1.X = s_des(1,1);
            ehtraj1.Y = s_des(2,1);
            ehtraj1.Z = s_des(3,1);
        else
            ehtraj1.X = [ehtraj1.X, s_des(1,1)];
            ehtraj1.Y = [ehtraj1.Y, s_des(2,1)];
            ehtraj1.Z = [ehtraj1.Z, s_des(3,1)];
        end
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
        if ~vis_init
            ehtraj2.X = s_des(1,2);
            ehtraj2.Y = s_des(2,2);
            ehtraj2.Z = s_des(3,2);
        else
            ehtraj2.X = [ehtraj2.X, s_des(1,2)];
            ehtraj2.Y = [ehtraj2.Y, s_des(2,2)];
            ehtraj2.Z = [ehtraj2.Z, s_des(3,2)];
        end
        
        
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
        if ~vis_init
            ehtraj3.X = s_des(1,3);
            ehtraj3.Y = s_des(2,3);
            ehtraj3.Z = s_des(3,3);
        else
            ehtraj3.X = [ehtraj3.X, s_des(1,3)];
            ehtraj3.Y = [ehtraj3.Y, s_des(2,3)];
            ehtraj3.Z = [ehtraj3.Z, s_des(3,3)];
        end
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

       

        if ~vis_init
            ehtraj4.X = s_des(1,4);
            ehtraj4.Y = s_des(2,4);
            ehtraj4.Z = s_des(3,4);
        else
            ehtraj4.X = [ehtraj4.X, s_des(1,4)];
            ehtraj4.Y = [ehtraj4.Y, s_des(2,4)];
            ehtraj4.Z = [ehtraj4.Z, s_des(3,4)];
        end
        

        
        
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

        if ~vis_init
            ehtraj5.X = s_des(1,5);
            ehtraj5.Y = s_des(2,5);
            ehtraj5.Z = s_des(3,5);
        else
            ehtraj5.X = [ehtraj5.X, s_des(1,5)];
            ehtraj5.Y = [ehtraj5.Y, s_des(2,5)];
            ehtraj5.Z = [ehtraj5.Z, s_des(3,5)];
        end
        

        
        
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

        if ~vis_init
            ehtraj6.X = s_des(1,6);
            ehtraj6.Y = s_des(2,6);
            ehtraj6.Z = s_des(3,6);
        else
            ehtraj6.X = [ehtraj6.X, s_des(1,6)];
            ehtraj6.Y = [ehtraj6.Y, s_des(2,6)];
            ehtraj6.Z = [ehtraj6.Z, s_des(3,6)];
        end
        
        
        %---
        
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
            thvy1.X = [thvy1.X, time]; thvy1.Y = [thvy1.Y, true_v1(2)];
            ehvy1.X = [ehvy1.X, time]; ehvy1.Y = [ehvy1.Y, des_v1(2)];
            thvz1.X = [thvz1.X, time]; thvz1.Y = [thvz1.Y, true_v1(3)];
            ehvz1.X = [ehvz1.X, time]; ehvz1.Y = [ehvz1.Y, des_v1(3)];
     
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
       
        %% Render
        %drawnow;
        vis_time = time;
        vis_init = true;
    end
end
