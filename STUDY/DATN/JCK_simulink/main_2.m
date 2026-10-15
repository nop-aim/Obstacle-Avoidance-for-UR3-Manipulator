clc; clear; close all;
%% Section 1: Robot and World Initialization
folder = fileparts(which('test.m')); 
if isempty(folder), folder = pwd; end
addpath(genpath(fullfile(folder,'enviroment')));
initialPosition = deg2rad([-179.40, -42.14, -132.36, -92.97, 89.72, 0]');

% ================= ATTRACTOR FUNCTION CONFIGURATION =================
% attractor_func = @(t) [0.517; -0.1115; 0.32];
% attractor_vel_func = @(t) [0; 0]; 
A = 0.1;        % biên độ dao động theo Z (m)
w = 2*pi/5;     % chu kỳ 5 giây
attractor_func     = @(t) [0.417; -0.1115; 0.32 + A*sin(w*t)];
attractor_vel_func = @(t) [0;  A*w*cos(w*t)];
% =====================================================================
myWorld = World(initialPosition);
robot = myWorld.robot.robot;  

JP  = 2; 
DOJ = 2;
% ================= OBSTACLE CONFIGURATION =================
% vel_func = @(x,t) [0; 0; 0];
% myWorld.addEllipsoid([0.025, 0.025, 0.025],[0.317; -0.1115; 0.32], 1, vel_func);
vel_func = @(x,t) [0.0; 0; A*w*sin(w*t)];
% myWorld.addEllipsoid([0.025, 0.025, 0.025],[0.417; -0.1115; 0.35], 1, vel_func); % ee
myWorld.addEllipsoid([0.025, 0.025, 0.025],[0.217; -0.1115; 0.25], 1, vel_func); % 3link
%% Section 2: IK + Joint Limits & Synchronized Time Structure
theta = initialPosition;
[theta_min, theta_max] = getJointLimits(robot);
lambda = 1;
%% ===== CLOSED LOOP =====
f = figure(1);
Basename = 'full_1obs_movetwo_2';
videoFolder = fullfile(folder, 'video_2');
if ~exist(videoFolder, 'dir'), mkdir(videoFolder); end
videoFile = fullfile(videoFolder, sprintf('%s.mp4', Basename));
v = VideoWriter(videoFile, 'MPEG-4');
v.FrameRate = 30;
open(v);
hold on; grid on; view(45,45);
delta_t = 25e-3;
i = 0;

t_now = 0;
attractor = attractor_func(t_now);
hAttr = plot3(attractor(1), attractor(2), attractor(3),'go', 'LineWidth',3,'MarkerSize',15);
% ===== Initialize state q =====
q = - (1/lambda) * log(max(eps, (theta_max - theta_min)./(max(eps, theta - theta_min)) - 1));
[theta, ~] = jointMapping(q, theta_min, theta_max, lambda);
position = myWorld.robot.computeForwardKinematics(theta);

% Khởi tạo các biến lưu trữ vận tốc khớp thực tế phục vụ tính Torque
theta_dot_prev = zeros(6,1); 
theta_prev     = theta;

%% ===== INITIALIZE LOG DATA STRUCTURE =====
logData.time             = [];
logData.position_tcp     = [];
logData.initialPosition  = [];
logData.attractor        = [];
logData.error_norm       = [];
logData.q                = [];
logData.theta            = [];
nJoints = 6;
logData.joint_positions  = cell(nJoints, 1);
for j = JP:nJoints, logData.joint_positions{j} = []; end
logData.obstacle_position  = [];
logData.dist_to_obs        = cell(nJoints, 1);
for j = DOJ:6, logData.dist_to_obs{j} = []; end
logData.obstacle2_position = [];
logData.dist_to_obs2       = cell(nJoints, 1);
for j = DOJ:6, logData.dist_to_obs2{j} = []; end
logData.R1 = []; logData.R2 = [];
logData.R1_2 = []; logData.R2_2 = [];
logData.control_input = [];  

% --- LOG TORQUE CHO 6 KHỚP ---
logData.torque = [];  

highLC = @(t, x, theta) highLevelController(t, delta_t, x, attractor_func, attractor_vel_func, theta, theta_min, theta_max, myWorld);
global stopFlag;
stopFlag = false;
start = tic;

try
while i*delta_t <= 40
    t_now = i * delta_t;
    attractor = attractor_func(t_now);
   
    e_now = [position(1); position(3)] - [attractor(1); attractor(3)];
    
    %% --- TÍNH TOÁN TORQUE CHÍNH XÁC ---
    if i == 0
        theta_dot_now  = zeros(6,1);
        theta_ddot_now = zeros(6,1);
    else
        theta_dot_now  = (theta - theta_prev) / delta_t;
        theta_ddot_now = (theta_dot_now - theta_dot_prev) / delta_t;
    end
    
    % Tính toán Inverse Dynamics từ toolbox hệ thống
    torque_now = inverseDynamics(robot, theta, theta_dot_now, theta_ddot_now);
    
    %% --- LOG CURRENT STEP PARAMETERS ---
    logData.time(end+1)            = t_now;
    logData.position_tcp(:, i+1)  = position;
    logData.attractor(:, i+1)     = attractor;
    logData.initialPosition(:, i+1)= myWorld.endEffectorPosition;
    logData.error_norm(end+1)     = norm(e_now);
    logData.q(:, i+1)             = q;
    logData.theta(:, i+1)         = theta;
    logData.theta_min = theta_min;
    logData.theta_max = theta_max;
    
    % Ghi log torque khớp
    logData.torque(:, i+1)         = torque_now; 
    
    [p_now, ~] = JointKinematics(theta);
    for j = JP:nJoints
        pos_j = (j == 6) * position + (j ~= 6) * p_now(:,j);
        logData.joint_positions{j}(:, i+1) = pos_j(:);
    end
    
    %% --- Auto-adaptive obstacle log ---
    nObs = length(myWorld.listOfObstacles);
    if nObs >= 1
        obs1 = myWorld.listOfObstacles(1);
        xobs1_2d = [obs1.position(1); obs1.position(3)];
        R1 = obs1.ellipseAxes(1);
        
        logData.obstacle_position(:, i+1) = obs1.position(:);
        logData.R1(end+1) = R1;
        logData.R2(end+1) = 1.5 * R1; 
        
        for j = DOJ:6
            xi = (j == 6)*[position(1); position(3)] + (j ~= 6)*[p_now(1,j); p_now(3,j)];
            logData.dist_to_obs{j}(end+1) = norm(xi - xobs1_2d);
        end
    end
    
    if nObs >= 2
        obs2 = myWorld.listOfObstacles(2);
        xobs2_2d = [obs2.position(1); obs2.position(3)];
        R1_2 = obs2.ellipseAxes(1);
        
        logData.obstacle2_position(:, i+1) = obs2.position(:);
        logData.R1_2(end+1) = R1_2;
        logData.R2_2(end+1) = 1.5 * R1_2;
        for j = DOJ:6
            xi = (j == 6)*[position(1); position(3)] + (j ~= 6)*[p_now(1,j); p_now(3,j)];
            logData.dist_to_obs2{j}(end+1) = norm(xi - xobs2_2d);
        end
    end
    
    %% --- LOG CONTROL INPUT (qdot_virtual_d) ---
    qdot_log = highLevelController(t_now, delta_t, position, attractor_func, attractor_vel_func, theta, theta_min, theta_max, myWorld);
    logData.control_input(:, i+1) = qdot_log;
    
    % Cập nhật biến bộ nhớ trạng thái trước
    theta_prev = theta;
    theta_dot_prev = theta_dot_now;
    
    %% --- RUN ODE SOLVER ---
    [~, y_ode] = ode15s(@(t,q) forwardStateDynamic_q(t, q, robot, highLC, myWorld, theta_min, theta_max), ...
        [t_now, t_now + delta_t], q);
    
    q = y_ode(end,:)';
    [theta, ~] = jointMapping(q, theta_min, theta_max, lambda);
    position = myWorld.robot.computeForwardKinematics(theta);
    
    % --- Real-time visualization & Write frame to Video ---
    if isvalid(f)
        show(robot, theta, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on', 'FastUpdate', true);
        hold on;
        plot3(position(1), position(2), position(3), '.k', 'MarkerSize', 10);
        set(hAttr, ...
            'XData', attractor(1), ...
            'YData', attractor(2), ...
            'ZData', attractor(3));
        title(['Iteration: ', num2str(i), ' | Dist: ', num2str(norm([position(1); position(3)] - [attractor(1); attractor(3)]))]);
        drawnow; 
        frame = getframe(gcf);
        writeVideo(v, frame);
    end
    
    for k = 1:length(myWorld.listOfObstacles)
        myWorld.listOfObstacles(k).moveShape(t_now);
    end
    i = i + 1;
end
disp("Finished Simulation On the Target")
disp("Time: " + i*delta_t + " [s]");

catch ME
    disp("Chương trình dừng do lỗi thực thi thuật toán:");
    disp(ME.message);
end

%% ===== CLEANUP BLOCK - ALWAYS RUNS ON EXIT =====
if exist('v','var') && isvalid(v)
    try
        close(v);
        disp(['Video saved to: ', videoFile]);
    catch
        disp('Không có frame video nào được ghi nhận hoặc không đóng được file video.');
    end
end

logFolder = fullfile(folder, 'logs_2');
if ~exist(logFolder, 'dir'), mkdir(logFolder); end
logFileName = fullfile(logFolder, sprintf('%s.mat', Basename));

% Thực hiện kiểm tra an toàn dữ liệu log trước khi lưu xuống file .mat
if exist('logData','var') && isfield(logData, 'time') && ~isempty(logData.time)
    save(logFileName, 'logData');
    fprintf('Log data successfully saved to: %s\n', logFileName);
else
    disp('Cảnh báo: Dữ liệu logData trống, không thể lưu file .mat!');
end

%% ==================== HELPER FUNCTIONS ====================
function qdot_virtual_d = highLevelController(t, ~, x, attractor_func, attractor_vel_func, theta, theta_min, theta_max, world)
    lambda = 1; gamma = 45; eta = 1.5; mu = 18; ai = 12; beta = 0.95; sigma = 0.1;
    % lambda = 1; gamma = 45; eta = 1.5; mu = 18; ai = 10; beta = 0.95;
    % sigma = 0.1; %TH1
    % lambda = 1; gamma  = 80 ; eta    = 1.5; mu     = 18; ai     = 30; beta   = 0.85; sigma = 1;
    Ki = diag([1 1 1 1 1 1]); K = 1.2*eye(6);
    
    persistent lastPrintTime
    if isempty(lastPrintTime), lastPrintTime = tic; end
    
    q_virtual = - (1/lambda) * log(max(eps, (theta_max - theta_min)./(max(eps, theta - theta_min)) - 1));
    attractor = attractor_func(t);
    xdot_d = attractor_vel_func(t); 
    e = [x(1); x(3)] - [attractor(1); attractor(3)];
    
    J  = geometricJacobian(world.robot.robot, theta, 'tool0');
    Jv = J([4 6],:);   
    
    A_task     = gamma * e' * Jv;
    A_obs_total = zeros(1,6);
    B = -gamma * e' * xdot_d; 
    debugStr = sprintf('\n[EE] ||e||=%.3f ', norm(e));
    
    if ~isempty(world.listOfObstacles)
        [p, Jp] = JointKinematics(theta);
        for obs_idx = 1:length(world.listOfObstacles)
            obs = world.listOfObstacles(obs_idx);
            x_obs    = obs.position;
            xobs_2d  = [x_obs(1); x_obs(3)];
            x_dot_obs= [obs.velocity(1); obs.velocity(3)];
            R1 = obs.ellipseAxes(1);
            R2 = 1.5 * R1;
            alpha = exp(-(R2^2 - R1^2)) / beta;
            
            for idx = 2:6
                if idx == 6
                    xi  = [x(1); x(3)]; Jvi = Jv;
                else
                    xi_full = p(:,idx); xi = [xi_full(1); xi_full(3)]; Jvi = Jp([1 3],:,idx);
                end
                dist_vec = xi - xobs_2d;
                d_iobs   = dist_vec' * dist_vec;
                dist     = sqrt(d_iobs);
                d_i      = exp(-(d_iobs - R1^2)/sigma);
                expTerm  = exp(-mu*(d_i - alpha));
                aii      = ai * mu * expTerm / (1 + expTerm)^2;
                
                A_obs_i = (aii * d_i * dist_vec' * Jvi) * Ki;
                status = '';
                if dist < R1, status = ['OBS' num2str(obs_idx) '_COLLISION'];
                elseif dist < R2, status = ['OBS' num2str(obs_idx) '_NEAR']; end
                
                debugStr = [debugStr, sprintf('\n Obs%d-L%d | dist=%.3f | d=%.3f | aii=%.3f %s', ...
                    obs_idx, idx, dist, d_i, aii, status)];
                A_obs_total = A_obs_total + A_obs_i;
                B = B + aii * d_i * dist_vec' * x_dot_obs;
            end
        end
    end
    
    A = A_task - A_obs_total;
    if norm(A_obs_total) > 1e-3, debugStr = [debugStr, sprintf('\n >>> AVOIDANCE ACTIVE <<<')]; end
    if (A*A') < 1e-6, Aplus = zeros(6,1); else, Aplus = A' / (A*A' + 1e-4); end
    
    rhs = -eta * A' - Aplus * B;
    [~, Jg] = jointMapping(q_virtual, theta_min, theta_max, lambda);
    qdot_virtual_d = K * pinv(Jg) * rhs;
    qdot_virtual_d(1) = 0; qdot_virtual_d(5) = 0; qdot_virtual_d(6) = 0;
  
    debugStr = [debugStr, sprintf('\n[CTRL] A_t=%.2f | A_o=%.2f | A=%.2f | qdot=%.2f\n', ...
        norm(A_task), norm(A_obs_total), norm(A), norm(qdot_virtual_d))];
    if toc(lastPrintTime) > 0.5
        fprintf('%s', debugStr);
        lastPrintTime = tic;
    end
end

function dqdt = forwardStateDynamic_q(t, q, robot, highLC, world, theta_min, theta_max)
    lambda = 1;
    [theta, ~] = jointMapping(q, theta_min, theta_max, lambda);
    x = world.robot.computeForwardKinematics(theta);
    dqdt = highLC(t, x, theta); 
    dqdt(1)=0; dqdt(5)=0; dqdt(6)=0;
    if norm(dqdt) > 5, dqdt = dqdt / norm(dqdt) * 5; end
end

function [theta, Jg] = jointMapping(q_v, t_min, t_max, lambda)
    theta = (t_max - t_min) ./ (1 + exp(-lambda*q_v)) + t_min;
    dg = lambda*(t_max-t_min).* exp(-lambda*q_v) ./ (1+exp(-lambda*q_v)).^2;
    dg(dg < 1e-9) = 1e-9; 
    Jg = diag(dg);
end

function [theta_min, theta_max] = getJointLimits(robot)
    n = robot.NumBodies;
    theta_min = []; theta_max = [];
    for idx = 1:n
        joint = robot.Bodies{idx}.Joint;
        if strcmp(joint.Type,'revolute')
            limits = joint.PositionLimits;
            theta_min = [theta_min; limits(1)];
            theta_max = [theta_max; limits(2)];
        end
    end
end