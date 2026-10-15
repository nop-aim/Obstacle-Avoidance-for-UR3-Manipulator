clc; clear; close all;

%% Section 1: Khởi tạo Robot và Thế giới
folder = fileparts(which('test.m')); 
addpath(genpath(fullfile(folder,'enviroment')));
initialPosition = [0.4569; 0.1943; 0.0666];
attractor = [-0.217;  0.1943; 0.562];

myWorld = World(initialPosition);
robot = myWorld.robot.robot;  

    % lambda = 1;
    % gamma  = 80;
    % eta    = 1;
    % mu     = 15;
    % ai     = 180;
    % beta   = 0.9;

% myWorld.addEllipsoid([0.05, 0.05, 0.05], [-0.36,-0.196, 0.450]); 
myWorld.addEllipsoid([0.1, 0.1, 0.1], [0.1217;  0.1943; 0.1462]); 
% % Define a constant velocity in Z axis
% vel_func = @(x,t) [-0.3; 0; 0.2];
% 
% % Define the obstacle velocity as a DS
% vel_func_2 = @(x,t) (A_obs * x + b_obs);
% 
% % Define a time-varying velocity vector
% vel_func_3 = @(x,t) [2*cos(50*t); 0; 0 ];
% Create a sphere with 10cm radius at the origin, a rho value of 0.5, ...and a velocity function
%myWorld.addEllipsoid([0.1, 0.1, 0.1],[0.57, 0.1943, 0.2], 1, vel_func);

% attractor_func = @(t) [ ...
%     -0.217 + 0.15*sin(0.3*t);
%      0.1943;
%      0.562 + 0.1*cos(0.3*t)];
% attractor_func = @(t) [ ...
%      0.147 + 0.05*t;
%      0.1943;
%      0.562 - 0.05*t];

%% Section 2: IK + Joint limit
theta = myWorld.robot.computeInverseKinematics(initialPosition, robot.homeConfiguration, ...
                false)
[theta_min, theta_max] = getJointLimits(robot);

dt = 0.01;
lambda = 1;

% % ===== q init =====
% q_sim= - (1/lambda) * log(max(eps, (theta_max - theta_min)./(max(eps, theta - theta_min)) - 1));
% [theta_sim, ~] = jointMapping(q_sim, theta_min, theta_max, lambda);
% 
% q_dot_sim = zeros(6,1);
% 
% %% ===== OPEN LOOP =====
% for k = 1:3000
%     x = myWorld.robot.computeForwardKinematics(theta_sim);
%     t = k * dt;
% 
%     attractor = attractor_func(t);
% 
%     qdot_virtual_d = highLevelController(dt, x, attractor, theta_sim, ...
%                                             theta_min, theta_max, myWorld);
% 
%     q_dot_sim = qdot_virtual_d;
%     q_sim = q_sim + q_dot_sim * dt;
% 
%     [theta_sim, ~] = jointMapping(q_sim, theta_min, theta_max, lambda);
%     traj(:,k) = myWorld.robot.computeForwardKinematics(theta_sim);
% end

%% ===== CLOSED LOOP =====
f=figure(1);
% tạo folder ghi video 
videoFolder = fullfile(folder, 'video');
if ~exist(videoFolder, 'dir')
    mkdir(videoFolder);
end

baseName = 'robot_motion';
i_video = 1;

while true
    videoFile = fullfile(videoFolder, sprintf('%s_%d.mp4', baseName, i_video));
    if ~exist(videoFile, 'file')
        break;
    end
    i_video = i_video + 1;
end

v = VideoWriter(videoFile, 'MPEG-4');
v.FrameRate = 30;
open(v);
hold on; grid on;view(45,45);
%==================== chuyển động attractor=======================
% t_now = 0;
% attractor = attractor_func(t_now);
%===============================================================
hAttr = plot3(attractor(1), attractor(2), attractor(3),'go', 'LineWidth',3,'MarkerSize',15);
%plot3(traj(1,:), traj(2,:), traj(3,:), 'r--', 'LineWidth', 1);

delta_t = 0.01;
i = 0;

% ===== init q =====
q = - (1/lambda) * log(max(eps, (theta_max - theta_min)./(max(eps, theta - theta_min)) - 1));

[theta, ~] = jointMapping(q, theta_min, theta_max, lambda)
position = myWorld.robot.computeForwardKinematics(theta);
traj_tcp = position; 
nJoints = 6;

traj_Joint= cell(nJoints,1);

[p_init, ~] = JointKinematics(theta);

for j = 1:nJoints

    if j == 6
        pos_j = position;
    else
        pos_j = p_init(:,j);
    end

    traj_Joint{j} = pos_j(:);
end
if ~isempty(myWorld.listOfObstacles)
    obs_traj = myWorld.listOfObstacles(1).position(:); % Lưu tọa độ vật cản
end
q_history = q;
highLC = @(x,theta) highLevelController(delta_t, x, attractor, theta, theta_min, theta_max, myWorld);
global stopFlag;
stopFlag = false;
start = tic;
try
while vecnorm(position - attractor) > 0.01 && ~stopFlag
    %bật nếu chạy chuyển động 
    % t_now = i * delta_t;
    % attractor = attractor_func(t_now);
    % highLC = @(x,theta) highLevelController(delta_t, x, attractor, theta, theta_min, theta_max, myWorld);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [t, y_ode] = ode15s (@(t,q) forwardStateDynamic_q(t, q, robot, highLC, myWorld, theta_min, theta_max), ...
        [i * delta_t, (i+1) * delta_t], q);
    
    q = y_ode(end,:)';
    
    [theta, ~] = jointMapping(q, theta_min, theta_max, lambda);
    position = myWorld.robot.computeForwardKinematics(theta);
    traj_tcp(:, end+1) = position; 
    [p_now, ~] = JointKinematics(theta);
    for j = 1:nJoints
    
        if j == 6
            pos_j = position;
        else
            pos_j = p_now(:,j);
        end
    
        traj_Joint{j}(:, end+1) = pos_j(:);
    end
    if isvalid(f)
        show(robot, theta, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on', 'FastUpdate', true);
        hold on;
        plot3(position(1), position(2), position(3), '.k', 'MarkerSize', 10);
        set(hAttr,...
        'XData', attractor(1),...
        'YData', attractor(2),...
        'ZData', attractor(3));
        title(['Iteration: ', num2str(i), ' | Dist: ', num2str(vecnorm(position - attractor))]);
        drawnow; 
        frame = getframe(gcf);
        writeVideo(v, frame);
    end
     % Move obstacles if desired
    for k =1:length(myWorld.listOfObstacles)
        myWorld.listOfObstacles(k).moveShape(i*delta_t)
    end
    if ~isempty(myWorld.listOfObstacles)
        obs_traj(:, end+1) = myWorld.listOfObstacles(1).position(:);
    end
    q_history(:, end+1) = q;
    i = i + 1;
end
disp("Finished Simulation On the Target")

%Top + Left + obstacle 2D
plot_case_views_links( ...
    traj_tcp,...
    traj_Joint,...
    attractor,...
    initialPosition,...
    myWorld);
time_elapsed = toc(start);
disp("Average computation time: " + time_elapsed / i + " [s] | Total time: " + time_elapsed + " [s]")
% ===== CLOSE VIDEO =====
catch ME
    disp("Stopped by user or error!");
    disp(ME.message);
end

% ===== LUÔN LUÔN CHẠY =====
if exist('v','var')
    try
        close(v);
        disp(['Video saved to: ', videoFile]);
    catch
        disp("Error closing video file!");
    end
end

function qdot_virtual_d = highLevelController(~, x, attractor, theta, theta_min, theta_max, world)

    %% ===== PARAMETERS =====
    lambda      = 1;
    gain_attr   = 80;      % attraction gain
    eta         = 1;      % control gain
    gain_rep    = 8;     % obstacle repulsion gain
    ratio_safe  = 2.5;     % safety zone = ratio_safe * R  →  γ_safe = ratio_safe^2

    Ki = diag([1 1 1 1 1 1]);
    K  = eye(6);

    %% ===== DEBUG TIMER =====
    persistent lastPrintTime
    if isempty(lastPrintTime), lastPrintTime = tic; end

    %% ===== VIRTUAL JOINT MAPPING =====
    q_virtual = -(1/lambda) * log(max(eps, ...
        (theta_max - theta_min) ./ (max(eps, theta - theta_min)) - 1));

    %% ===== TASK ERROR (X-Z plane) =====
    e      = [x(1); x(3)] - [attractor(1); attractor(3)];
    xdot_d = [0; 0];

    %% ===== EE JACOBIAN — rows X and Z =====
    J  = geometricJacobian(world.robot.robot, theta, 'tool0');
    Jv = J([4 6], :);   % [2×6]

    %% ===== ATTRACTION =====
    A_task      = gain_attr * e' * Jv;    % [1×6]
    A_obs_total = zeros(1, 6);
    B           = -gain_attr * e' * xdot_d;

    debugStr = sprintf('\n[EE] ||e||=%.4f', norm(e));

    %% ===== OBSTACLE AVOIDANCE — Gamma Function =====
    if ~isempty(world.listOfObstacles)

        [p, Jp] = JointKinematics(theta);   % joint positions + Jacobians

        for obs_idx = 1:length(world.listOfObstacles)

            obs   = world.listOfObstacles(obs_idx);
            c_obs = obs.position;
            c_2d  = [c_obs(1); c_obs(3)];       % obstacle center in X-Z
            vobs  = [0; 0];                       % obstacle velocity (static)

            % --- Ellipse semi-axes in X-Z ---
            Rx = obs.ellipseAxes(1);
            Rz = obs.ellipseAxes(3);             % use axis(1) if isotropic

            gamma_safe = ratio_safe^2;           % γ active zone boundary

            for i = 2:6

                %% --- Link point & Jacobian in X-Z ---
                if i == 6
                    xi  = [x(1); x(3)];
                    Jvi = Jv;
                else
                    xi_full = p(:, i);
                    xi      = [xi_full(1); xi_full(3)];
                    Jvi     = Jp([1 3], :, i);
                end

                %% ============================================
                %  GAMMA FUNCTION  (ellipsoidal, 2D X-Z)
                %
                %       ( xi(1)-cx )^2   ( xi(2)-cz )^2
                %  γ =  -------------- + --------------
                %            Rx²               Rz²
                %
                %  γ = 1  →  on surface
                %  γ > 1  →  outside
                %% ============================================
                dv      = xi - c_2d;                          % [2×1]
                gamma_i = (dv(1)/Rx)^2 + (dv(2)/Rz)^2;
                %gamma_i = max(gamma_i, 1 + 1e-6);            % clamp: never inside

                %% ============================================
                %  GRADIENT OF GAMMA  (analytical, exact)
                %
                %  ∂γ/∂xi = [ 2(xi(1)-cx)/Rx² ]
                %            [ 2(xi(2)-cz)/Rz² ]
                %% ============================================
                % grad_gamma = [(dv(1)/Rx^2); (dv(2)/Rz^2)] / gamma_i;              % [2×1]
                grad_gamma = [(dv(1)/Rx^2); (dv(2)/Rz^2)];  
                %% ============================================
                %  REPULSIVE WEIGHT
                %
                %  w(γ) = 1/(γ-1) - 1/(γ_safe-1)
                %       = 0          khi  γ ≥ γ_safe  (ngoài vùng an toàn)
                %       → +∞         khi  γ → 1       (chạm bề mặt)
                %% ============================================
                if gamma_i >= gamma_safe
                    continue;       % ngoài vùng ảnh hưởng
                end

                % w_i = 1/(gamma_i - 1) - 1/(gamma_safe - 1);
                % w_i = max(w_i, 0);
                sigma = 0.5; 
                w_i = exp(-(gamma_i - 1)^2 / (2 * sigma^2));
                aii = gain_rep * w_i;

                %% ============================================
                %  OBSTACLE TERM
                %
                %  A_obs_i = aii · (∂γ/∂xi)ᵀ · Jvi · Ki
                %  B      += aii · (∂γ/∂xi)ᵀ · v_obs
                %% ============================================
                A_obs_i     = aii * (grad_gamma' * Jvi) * Ki;
                A_obs_total = A_obs_total + A_obs_i;
                B           = B + aii * (grad_gamma' * vobs);

                %% --- Debug status ---
                dist   = norm(dv);
                status = '';
                if gamma_i < 1.05,  status = '!! COLLISION !!';
                elseif gamma_i < 1.5, status = 'NEAR'; end

                debugStr = [debugStr, sprintf( ...
                    '\n  L%d|obs%d  γ=%.3f  dist=%.3f  w=%.3f  aii=%.2f  %s', ...
                    i, obs_idx, gamma_i, dist, w_i, aii, status)];
            end  % end links
        end  % end obstacles
    end

    %% ===== CONTROL LAW =====
    A = A_task - A_obs_total;

    if norm(A_obs_total) > 1e-3
        debugStr = [debugStr, sprintf('\n >>> AVOIDANCE ACTIVE | ||A_obs||=%.3f <<<', ...
            norm(A_obs_total))];
    end

    %% ===== PSEUDO-INVERSE =====
    denom = A * A' + 1e-4;
    Aplus = A' / denom;

    %% ===== OUTPUT =====
    rhs = -eta * A' - Aplus * B;

    [~, Jg] = jointMapping(q_virtual, theta_min, theta_max, lambda);
    qdot_virtual_d = K * pinv(Jg) * rhs;

    %% ===== LOCK PLANAR (joints 1, 5, 6) =====
    qdot_virtual_d(1) = 0;
    qdot_virtual_d(5) = 0;
    qdot_virtual_d(6) = 0;

    %% ===== DEBUG PRINT =====
    debugStr = [debugStr, sprintf( ...
        '\n[CTRL] A_t=%.2f  A_o=%.2f  A=%.2f  |qdot|=%.2f\n', ...
        norm(A_task), norm(A_obs_total), norm(A), norm(qdot_virtual_d))];

    if toc(lastPrintTime) > 0.5
        fprintf('%s', debugStr);
        lastPrintTime = tic;
    end
end
%% ===== ODE =====
function dqdt = forwardStateDynamic_q(t, q, robot, highLC, world, theta_min, theta_max)

    lambda = 1;

    [theta, ~] = jointMapping(q, theta_min, theta_max, lambda);

    x = world.robot.computeForwardKinematics(theta);

    dqdt = highLC(x, theta);
    dqdt(1)=0;
    dqdt(5)=0;
    dqdt(6)=0;
    if norm(dqdt) > 5
        dqdt = dqdt / norm(dqdt) * 5;
    end
end
%% ===== Mapping =====
function [theta, Jg] = jointMapping(q_v, t_min, t_max, lambda)
    theta = (t_max - t_min) ./ (1 + exp(-lambda*q_v)) + t_min;

    dg = lambda*(t_max-t_min).* exp(-lambda*q_v) ./ (1+exp(-lambda*q_v)).^2;
    dg(dg < 1e-9) = 1e-9; 
    Jg = diag(dg);
end

%% ===== Joint limits =====
function [theta_min, theta_max] = getJointLimits(robot)
    n = robot.NumBodies;
    theta_min = []; theta_max = [];
    for i = 1:n
        joint = robot.Bodies{i}.Joint;
        if strcmp(joint.Type,'revolute')
            limits = joint.PositionLimits;
            theta_min = [theta_min; limits(1)];
            theta_max = [theta_max; limits(2)];
        end
    end
end