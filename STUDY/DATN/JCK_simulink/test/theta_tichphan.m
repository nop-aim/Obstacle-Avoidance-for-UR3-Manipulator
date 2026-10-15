clc; clear; close all;

%% Section 1: Khởi tạo Robot và Thế giới
folder = fileparts(which('test.m')); 
addpath(genpath(fullfile(folder,'enviroment')));
initialPosition = [-0.4569; -0.196; 0.069];
myWorld = World(initialPosition);
robot = myWorld.robot.robot;  
    % lambda = 1;
    % gamma  = 80;
    % eta    = 1;
    % mu     = 15;
    % ai     = 180;
    % beta   = 0.9;

myWorld.addEllipsoid([0.1, 0.1, 0.1], [-0.35, -0.196, 0.35]); 
%myWorld.addEllipsoid([0.1, 0.1, 0.1], [0.3, 0.1943, 0.3]); 
attractor = [-0.217; -0.196; 0.562];


%% Section 2: IK + Joint limit
theta = myWorld.robot.computeInverseKinematics(initialPosition, robot.homeConfiguration, ...
                false);

[theta_min, theta_max] = getJointLimits(robot);

dt = 0.01;
lambda = 1;

% ===== q init =====
q_sim= - (1/lambda) * log(max(eps, (theta_max - theta_min)./(max(eps, theta - theta_min)) - 1));
[theta_sim, ~] = jointMapping(q_sim, theta_min, theta_max, lambda);

q_dot_sim = zeros(6,1);

%% ===== OPEN LOOP =====
for k = 1:3000
    x = myWorld.robot.computeForwardKinematics(theta_sim);
    
    qdot_virtual_d = highLevelController(dt, x, attractor, theta_sim, ...
                                            theta_min, theta_max, myWorld);
    
    q_dot_sim = qdot_virtual_d;
    q_sim = q_sim + q_dot_sim * dt;
    
    [theta_sim, ~] = jointMapping(q_sim, theta_min, theta_max, lambda);
    traj(:,k) = myWorld.robot.computeForwardKinematics(theta_sim);
end

%% ===== CLOSED LOOP =====
f=figure(1);
hold on; grid on; view(0,0);

plot3(attractor(1), attractor(2), attractor(3), 'go', 'LineWidth', 3, 'MarkerSize', 15);
plot3(traj(1,:), traj(2,:), traj(3,:), 'r--', 'LineWidth', 1);

delta_t = 0.01;
i = 0;

% ===== init q =====
q = - (1/lambda) * log(max(eps, (theta_max - theta_min)./(max(eps, theta - theta_min)) - 1));

[theta, ~] = jointMapping(q, theta_min, theta_max, lambda);
position = myWorld.robot.computeForwardKinematics(theta);

highLC = @(x,theta) highLevelController(delta_t, x, attractor, theta, theta_min, theta_max, myWorld);
global stopFlag;
stopFlag = false;
while vecnorm(position - attractor) > 0.01 && ~stopFlag 
    
    [t, y_ode] = ode15s (@(t,q) forwardStateDynamic_q(t, q, robot, highLC, myWorld, theta_min, theta_max), ...
        [i * delta_t, (i+1) * delta_t], q);
    
    q = y_ode(end,:)';
    
    [theta, ~] = jointMapping(q, theta_min, theta_max, lambda);
    position = myWorld.robot.computeForwardKinematics(theta);
    
    if isvalid(f)
        show(robot, theta, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on', 'FastUpdate', true);
        hold on;
        plot3(position(1), position(2), position(3), '.k', 'MarkerSize', 10);
        plot3(attractor(1), attractor(2), attractor(3), 'go', 'MarkerSize', 10, 'LineWidth', 2); 
        
        title(['Iteration: ', num2str(i), ' | Dist: ', num2str(vecnorm(position - attractor))]);
        drawnow; 
    end
    
    i = i + 1;
end

function theta_dot = highLevelController(~, x, attractor, theta, theta_min, theta_max, world)

    %% ===== PARAM =====
    lambda = 1;
    gamma  = 50;
    eta    = 10;
    mu     = 8;
    ai     = 180;
    beta   = 0.9;

    Ki = diag([0 1 1 1 0 5]);
    K = 1*eye(6);

    %% ===== DEBUG TIMER =====
    persistent lastPrintTime
    if isempty(lastPrintTime)
        lastPrintTime = tic;
    end

    %% ===== MAPPING =====
    q_virtual = - (1/lambda) * log(max(eps,...
        (theta_max - theta_min)./(max(eps, theta - theta_min)) - 1));

    %% ===== ERROR (2D X-Z) =====
    e = [x(1); x(3)] - [attractor(1); attractor(3)];
    xdot_d = [0;0];

    %% ===== JACOBIAN EE (FIX DUY NHẤT) =====
    J = geometricJacobian(world.robot.robot, theta, 'tool0');
    Jv = J([4 6],:);   

    %% ===== INIT =====
    A_task = gamma * e' * Jv;
    A_obs_total = zeros(1,6);
    B = -gamma * e' * xdot_d;

    % %% ===== DEBUG STRING =====
    % debugStr = sprintf('\n[EE] ||e||=%.3f ', norm(e));

    %% ===== OBSTACLE =====
    if ~isempty(world.listOfObstacles)

        obs = world.listOfObstacles(1);
        x_obs = obs.position;
        xobs_2d = [x_obs(1); x_obs(3)];
        x_dot_obs = [0;0];

        R1 = obs.ellipseAxes(1);
        R2 = 2* R1;

        alpha = exp(-(R2^2 - R1^2)) / beta;

        [p, Jp] = linkKinematics(theta);
        
        for i = 6

            if i == 6
                xi  = [x(1); x(3)];
                Jvi = Jv;
            else
                xi_full = p(:,i);
                xi  = [xi_full(1); xi_full(3)];
                Jvi = Jp([1 3],:,i);
            end

            %% ===== DISTANCE =====
            dist_vec = xi - xobs_2d;
            d_iobs   = dist_vec' * dist_vec;
            %dist=sqrt(d_iobs);
            %% ===== EXP DIST =====
            sigma = 0.05;
            d_i = exp(-(d_iobs - R1^2)/sigma);

            %% ===== SIGMOID =====
            expTerm = exp(-mu*(d_i - alpha));
            aii = ai * mu * expTerm / (1 + expTerm)^2;

            %% ===== OBSTACLE =====
            A_obs_i = (aii * d_i * dist_vec' * Jvi) * Ki;
            % %% ===== ADD DEBUG LINK =====
            % status = '';
            % if dist < R1
            %     status = 'COLLISION';
            % elseif dist < 1.5*R1
            %     status = 'NEAR';
            % end
            % 
            % debugStr = [debugStr, sprintf('\n L%d | dist=%.3f | d=%.3f | aii=%.3f %s', ...
            %     i, dist, d_i, aii, status)];
            A_obs_total = A_obs_total + A_obs_i;
            B = B + aii * d_i * dist_vec' * x_dot_obs;
        end
    end

    %% ===== FINAL =====
    A = A_task - A_obs_total;
    % if norm(A_obs_total) > 1e-3
    % debugStr = [debugStr, sprintf('\n >>> AVOIDANCE ACTIVE <<<')];
    % end
    % 
    %% ===== PSEUDO-INVERSE =====
    if (A*A') < 1e-6
        Aplus = zeros(6,1);
    else
        Aplus = A' / (A*A' + 1e-4);
    end

    %% ===== CONTROL =====
    rhs = -eta * A' - Aplus * B;

    %% ===== MAP BACK =====
    [~, Jg] = jointMapping(q_virtual, theta_min, theta_max, lambda);
    qdot_virtual_d = K * pinv(Jg) * rhs;

    %% ===== GIỮ 2D =====
    qdot_virtual_d(1) = 0;
    qdot_virtual_d(5) = 0;
    qdot_virtual_d(6) = 0;
  
    %% ===== LIMIT =====
    
    %qdot_virtual_d = max(min(qdot_virtual_d, 5), -5);
    theta_dot = Jg*qdot_virtual_d;
    % %% ===== DEBUG =====
    % debugStr = [debugStr, sprintf('\n[CTRL] A_t=%.2f | A_o=%.2f | A=%.2f | qdot=%.2f\n', ...
    % norm(A_task), norm(A_obs_total), norm(A), norm(qdot_virtual_d))];
    
    if toc(lastPrintTime) > 0.5
        disp('qdot_virtual_d = '); disp(qdot_virtual_d);
        disp('theta_dot = '); disp(theta_dot);
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
    if norm(dqdt) > 1
        dqdt = dqdt / norm(dqdt) * 1;
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