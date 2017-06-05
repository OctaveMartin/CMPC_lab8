clc
clear all
close all
warning off


%% INITIALIZATION
% include needed folders/files
addpath([pwd '/extras']);

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
% desktop;
% keyboard;

% do we run a WEBOTS or MATLAB simulation? 
if exist('wb_robot_step', 'file') 
    IS_WEBOTS_SIMULATION=true; 
else
    IS_WEBOTS_SIMULATION=false; 
end

% simulation parameters
CONTROLLER_TIME_STEP = 20;      % in ms
dt=CONTROLLER_TIME_STEP/1000;

% get and enable devices:
if IS_WEBOTS_SIMULATION; 
    robot=initializeSRLink(CONTROLLER_TIME_STEP); 
end;

% initialize time
k=1;
t=0;

% initialize q
q=zeros(14,1);

% initialize parameters <- FREE TO CHANGE
f=1;
f_limb=1;
R=0.37;
R_limb=0.3;
phi_total=2*pi;
limb_spine_offset=3*pi/2;

%% GRID SEARCH INITIALIZATION
% Simulation length - only for Matlab simulation and GRID SEARCH
Trun = 30;   

is_swimming = false;

% the flag indicating whether we use grid search or not
GRID_SEARCH=true;
if GRID_SEARCH && IS_WEBOTS_SIMULATION
    % specify name of the .mat file which will contain grid search results
    % every time simulation starts with a new parameters, variables in the
    % workspace are lost. Therefore, storing them into .mat and saving is
    % the only way to preserve data between grid search iterations
    filename='gridSearch1';
    
    % define search parameters as vectors of equaly spaced values. MODIFY AS NEEDED
    limb_spine_offset_vector=3*pi/2;
    R_vector=0.37;
    % In every iteration, this function returs a pair of search parameters MODIFY AS NEEDED
    [limb_spine_offset, R, GS]=initializeGridSearch(filename, limb_spine_offset_vector, R_vector);
    
end

%-------------------------------------------------------------------------%
%                                 Figure 1                                %
%-------------------------------------------------------------------------%
%
Theta = [];
Q = [];
F_Limb = [];
F_Spine = [];
R_Limb = [];
R_Spine = [];

%}

%% MAIN CONTROLLER LOOP
while true  
    % check if we run WEBOTS or MATLAB simulation
    if IS_WEBOTS_SIMULATION
        if wb_robot_step(CONTROLLER_TIME_STEP)== -1
            break;
        end
        % read robot's GPS
        gps(1:3, k)=wb_gps_get_values(robot.gps);
        % read motor angles and torques from Webots
        for jj=1:14
            motor_torque(jj, k) = wb_motor_get_torque_feedback(robot.motor(jj));
            motor_position(jj, k) = wb_position_sensor_get_value(robot.positionSensor(jj));
        end
    else
        if t(k)>Trun
            break
        end
    end
    

    % RUN THE CONTROLLER
    % IMPLEMENT saturation function (UNCOMMENT for question 11.C)
    [f, f_limb, R, R_limb, is_swimming]=limbCPGsaturation(f, f_limb, R, R_limb, gps(1, k));                                   % <- IMPLEMENT INSIDE
    
    % IMPLEMENT double chain of oscillators inside this function:
    [theta, r]=runCPGNetwork(f, f_limb, phi_total, R, R_limb, limb_spine_offset, dt);                                           % <- IMPLEMENT INSIDE
    
    % IMPLEMENT relation to get spine joint angle reference from the CPG output:
    qs=spineController(theta(1:20), r(1:20));                                                                                   % <- IMPLEMENT INSIDE

    % limb angles correspond to limb phase 
    if is_swimming
        n = floor(theta(21:24)./(2*pi));
        qlimb = -pi/2*ones(4,1) - n*2*pi;
    else
        qlimb = -theta(21:24);
    end

    % collect all joint angles for the robot
    q=[qs;qlimb];
    
    
%-------------------------------------------------------------------------%
%                                 Figure 1                                %
%-------------------------------------------------------------------------%
%
Theta = [Theta theta];
Q = [Q q];
F_Limb = [F_Limb f_limb];
F_Spine = [F_Spine f];
R_Limb = [R_Limb R_limb];
R_Spine = [R_Spine R];


%}    
    % interface to the robot
    if IS_WEBOTS_SIMULATION 
        % send angles to the Webots simulator
        sendCommands2Robot(robot, q); 
        if GRID_SEARCH && t(k)>Trun
            % When maximum simulation time is achieved, save search results
            % IMPLEMENT your search metrics (speed, energy etc.) inside this function. Modify inputs if needed
            if saveGridSearch(filename, GS, gps, F_Spine, F_Limb, R_Spine, R_Limb, t, Q);                                 % <- IMPLEMENT INSIDE
                % if grid search is done, pause simulation and stop the control loop 
                disp('GRID SEARCH FINISHED');
                break;
            end
        end
    else
        % visualize robot's kinematics in matlab
        p=SR_forwardKinematics(q);
        end
    q_log(1:14,k)=q;
    % track time
    t(k+1,1)=t(k,1)+dt;
    k=k+1;
end

%-------------------------------------------------------------------------%
%                                 Figure 1                                %
%-------------------------------------------------------------------------%
%{
figure();
subplot(2,1,1);
plot(cos(Theta(1:10,1:600))'+ repmat((1:10),600,1),'LineWidth',2);
ylim([-0.5 11.5])
ylabel('Phase');
subplot(2,1,2);
plot(diff(Theta(1:10,1:600))','LineWidth',2);
xlabel('Time');
ylabel('Phase differences');
%}

%% save logged variables after simulation is finished (RELOAD button must be pressed)
if IS_WEBOTS_SIMULATION
    t(end)=[];    %remove last element from time vector to match length of other stored variables
    dlmwrite('./logs/time_log.txt', t, '\t');
    dlmwrite('./logs/gps_log.txt', gps', '\t');
    dlmwrite('./logs/angles_reference_log.txt', q_log', '\t');
    dlmwrite('./logs/angles_feedback_log.txt', motor_position', '\t');
    dlmwrite('./logs/torque_log.txt', motor_torque', '\t');
end
disp('DONE');
