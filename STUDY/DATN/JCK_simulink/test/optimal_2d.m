clc; clear; close all;
%% Section 1: Khởi tạo Robot và Thế giới
folder = fileparts(which('test.m')); 
addpath(genpath(fullfile(folder,'enviroment')));
initialPosition = [0.4569; 0.1943; 0.0666];
% attractor = [-0.217;  0.1943; 0.562];
attractor_func = @(t) [ ...
    -0.217 + 0.1*t;
     0.1943;
     0.562 - 0.15*t];
myWorld = World(initialPosition);
robot = myWorld.robot.robot;  

% ================= CẤU HÌNH VẬT CẢN TẠI ĐÂY =================
% Bạn có thể comment hoặc uncomment dòng bên dưới để test 1 hoặc 2 vật cản
% myWorld.addEllipsoid([0.1, 0.1, 0.1], [0.26, 0.1943, 0.250]); % Vật cản 1
% myWorld.addEllipsoid([0.08, 0.08, 0.08], [-0.05, 0.1943, 0.350]); % Hãy bỏ dấu % nếu muốn test 2 vật cản
vel_func = @(x,t) [-0.2; 0; 0.25];
myWorld.addEllipsoid([0.1, 0.1, 0.1],[0.2, 0.1943, 0.2], 1, vel_func);


%% Section 2: IK + Joint limit & Tạo Cấu trúc Đồng bộ Thời gian
theta = myWorld.robot.computeInverseKinematics(initialPosition, robot.homeConfiguration, false);
[theta_min, theta_max] = getJointLimits(robot);
dt = 0.01;
lambda = 1;
% === SINH CHUỖI MỐC THỜI GIAN CHUNG (Đồng bộ Video và Log) ===
timestampStr = datestr(now, 'yyyymmdd_HHMMSS');

%% ===== CLOSED LOOP =====
f = figure(1);
% --- Cấu hình xuất Video theo Timestamp ---
videoFolder = fullfile(folder, 'video');
if ~exist(videoFolder, 'dir')
    mkdir(videoFolder);
end
videoFile = fullfile(videoFolder, sprintf('robot_motion_%s.mp4', timestampStr));
v = VideoWriter(videoFile, 'MPEG-4');
v.FrameRate = 30;
open(v);
hold on; grid on; view(45,45);
delta_t = 0.005;
i = 0;
t_now = 0;
attractor = attractor_func(t_now);
hAttr = plot3(attractor(1), attractor(2), attractor(3),'go', 'LineWidth',3,'MarkerSize',15);
% ===== Khởi tạo trạng thái q =====
q = - (1/lambda) * log(max(eps, (theta_max - theta_min)./(max(eps, theta - theta_min)) - 1));
[theta, ~] = jointMapping(q, theta_min, theta_max, lambda);
position = myWorld.robot.computeForwardKinematics(theta);

%% ===== KHỞI TẠO CẤU TRÚC LOG DỮ LIỆU =====
logData.time = [];
logData.position_tcp = [];
logData.attractor = [];
logData.error_norm = [];
logData.q = [];
logData.theta = [];
nJoints = 6;
logData.joint_positions = cell(nJoints, 1);
[p_init, ~] = JointKinematics(theta);
for j = 1:nJoints
    logData.joint_positions{j} = [];
end

% --- Đã sửa: Khởi tạo linh hoạt cấu trúc cho cả 2 vật cản ---
logData.obstacle_position = [];
logData.dist_to_obs = cell(nJoints, 1);
for j = 2:6
    logData.dist_to_obs{j} = [];
end

logData.obstacle2_position = [];
logData.dist_to_obs2 = cell(nJoints, 1);
for j = 2:6
    logData.dist_to_obs2{j} = [];
end

logData.R1 = [];
logData.R2 = [];

% Thiết lập bộ điều khiển cấp cao
highLC = @(x,theta) highLevelController(delta_t, x, attractor, theta, theta_min, theta_max, myWorld);
global stopFlag;
stopFlag = false;
start = tic;

try
while vecnorm(position - attractor) > 0.01 && ~stopFlag 
    t_now = i * delta_t;
    attractor = attractor_func(t_now);
    % --- Tính toán sai số bám 2D (X-Z) để ghi log ---
    e_now = [position(1); position(3)] - [attractor(1); attractor(3)];
    
    %% --- GHI LOG CÁC THÔNG SỐ BƯỚC HIỆN TẠI ---
    logData.time(end+1)          = t_now;
    logData.position_tcp(:, i+1) = position;
    logData.attractor(:, i+1)    = attractor;
    logData.error_norm(end+1)    = norm(e_now);
    logData.q(:, i+1)            = q;
    logData.theta(:, i+1)        = theta;
    
    [p_now, ~] = JointKinematics(theta);
    for j = 1:nJoints
        pos_j = (j == 6) * position + (j ~= 6) * p_now(:,j);
        logData.joint_positions{j}(:, i+1) = pos_j(:);
    end
    
    %% --- Đã nâng cấp: Ghi log tự động thích ứng số lượng vật cản ---
    nObs = length(myWorld.listOfObstacles);
    if nObs >= 1
        obs1 = myWorld.listOfObstacles(1);
        xobs1_2d = [obs1.position(1); obs1.position(3)];
        R1 = obs1.ellipseAxes(1);
        
        logData.obstacle_position(:, i+1) = obs1.position(:);
        logData.R1(end+1) = R1;
        logData.R2(end+1) = 3 * R1; % Vùng ảnh hưởng tránh va chạm
        
        for j = 2:6
            xi = (j == 6)*[position(1); position(3)] + (j ~= 6)*[p_now(1,j); p_now(3,j)];
            logData.dist_to_obs{j}(end+1) = norm(xi - xobs1_2d);
        end
    end
    
    if nObs >= 2
        obs2 = myWorld.listOfObstacles(2);
        xobs2_2d = [obs2.position(1); obs2.position(3)];
        
        logData.obstacle2_position(:, i+1) = obs2.position(:);
        
        for j = 2:6
            xi = (j == 6)*[position(1); position(3)] + (j ~= 6)*[p_now(1,j); p_now(3,j)];
            logData.dist_to_obs2{j}(end+1) = norm(xi - xobs2_2d);
        end
    end
    
    highLC = @(x,theta) highLevelController(delta_t, x, attractor, theta, theta_min, theta_max, myWorld);
    %% --- CHẠY ODE SOLVER ---
    [t, y_ode] = ode15s (@(t,q) forwardStateDynamic_q(t, q, robot, highLC, myWorld, theta_min, theta_max), ...
        [t_now, t_now + delta_t], q);
    
    q = y_ode(end,:)';
    [theta, ~] = jointMapping(q, theta_min, theta_max, lambda);
    position = myWorld.robot.computeForwardKinematics(theta);
    
    % --- Hiển thị đồ họa thời gian thực & Ghi khung hình vào Video ---
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
    
    % Cập nhật trạng thái động của các vật cản trong World
    for k = 1:length(myWorld.listOfObstacles)
        myWorld.listOfObstacles(k).moveShape(t_now);
    end
    i = i + 1;
end
disp("Finished Simulation On the Target")
time_elapsed = toc(start);
disp("Average computation time: " + time_elapsed / i + " [s] | Total time: " + time_elapsed + " [s]")
catch ME
    disp("Stopped by user or error!");
    disp(ME.message);
end

% ===== KHỒI LỆNH ĐẢM BẢO LUÔN LUÔN CHẠY KHI KẾT THÚC =====
if exist('v','var')
    try
        close(v);
        disp(['Video saved to: ', videoFile]);
    catch
        disp("Error closing video file!");
    end
end

logFolder = fullfile(folder, 'logs');
if ~exist(logFolder, 'dir'), mkdir(logFolder); end
logFileName = fullfile(logFolder, sprintf('simulation_log_%s.mat', timestampStr));
save(logFileName, 'logData');
fprintf('Dữ liệu Log đã được lưu an toàn tại: %s\n', logFileName);

%% ==================== CÁC HÀM PHỤ TRỢ (GIỮ NGUYÊN) ====================
function qdot_virtual_d = highLevelController(~, x, attractor, theta, theta_min, theta_max, world)
    lambda = 1; gamma  = 80; eta    = 10; mu     = 18; ai     = 190; beta   = 0.95; sigma = 0.1;
    Ki = diag([1 1 1 1 1 1]); K = 1*eye(6);
    
    persistent lastPrintTime
    if isempty(lastPrintTime), lastPrintTime = tic; end
    
    q_virtual = - (1/lambda) * log(max(eps, (theta_max - theta_min)./(max(eps, theta - theta_min)) - 1));
    e = [x(1); x(3)] - [attractor(1); attractor(3)];
    xdot_d = [0;0];
    
    J = geometricJacobian(world.robot.robot, theta, 'tool0');
    Jv = J([4 6],:);   
    
    A_task = gamma * e' * Jv;
    A_obs_total = zeros(1,6);
    B = -gamma * e' * xdot_d;
    debugStr = sprintf('\n[EE] ||e||=%.3f ', norm(e));
    
    % Vòng lặp tính toán thuật toán tránh cho tất cả vật cản có trong danh sách
    if ~isempty(world.listOfObstacles)
        [p, Jp] = JointKinematics(theta);
        
        for obs_idx = 1:length(world.listOfObstacles)
            obs = world.listOfObstacles(obs_idx);
            x_obs = obs.position;
            xobs_2d = [x_obs(1); x_obs(3)];
            x_dot_obs = [0;0];
            R1 = obs.ellipseAxes(1);
            R2 = 3* R1;
            alpha = exp(-(R2^2 - R1^2)) / beta;
            
            for idx = 2:6
                if idx == 6
                    xi  = [x(1); x(3)]; Jvi = Jv;
                else
                    xi_full = p(:,idx); xi  = [xi_full(1); xi_full(3)]; Jvi = Jp([1 3],:,idx);
                end
                dist_vec = xi - xobs_2d;
                d_iobs   = dist_vec' * dist_vec;
                dist = sqrt(d_iobs);
                d_i = exp(-(d_iobs - R1^2)/sigma);
                expTerm = exp(-mu*(d_i - alpha));
                aii = ai * mu * expTerm / (1 + expTerm)^2;
                
                A_obs_i = (aii * d_i * dist_vec' * Jvi) * Ki;
                status = '';
                if dist < R1, status = ['OBS' num2str(obs_idx) '_COLLISION']; ...
                elseif dist < 1.5*R1, status = ['OBS' num2str(obs_idx) '_NEAR']; end
                
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

function dqdt = forwardStateDynamic_q(~, q, robot, highLC, world, theta_min, theta_max)
    lambda = 1;
    [theta, ~] = jointMapping(q, theta_min, theta_max, lambda);
    x = world.robot.computeForwardKinematics(theta);
    dqdt = highLC(x, theta);
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