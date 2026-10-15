%% SECTION 1: KHỞI TẠO MÔI TRƯỜNG VÀ VẬT CẢN
clc; clear; close all;
folder = fileparts(which('test_task.m'));
if ~isempty(folder), addpath(genpath(fullfile(folder, 'library'))); end

% Cấu hình vị trí
initialPosition = [0.15; 0.6; 0.5];
attractor = [0.5; -0.6; 0.45];

myWorld = Environment(initialPosition);
% Thêm vật cản (Ellipsoid)
myWorld.addEllipsoid([0.15, 0.15, 0.15], [0.3, 0.4, 0.5]);
myWorld.addEllipsoid([0.15, 0.15, 0.15], [0.38, -0.0, 0.45]);

% Khởi tạo bộ điều khiển và thông số mô phỏng
ctrl = LimitedJointVelocityController(500, myWorld.robot.robot);
disturbance = CartesianForceDisturbance(5e3, 2);
delta_t = 25e-3; 

%% SECTION 2: CHẠY MÔ PHỎNG ROBOT THỰC TẾ (LIVE RUN)
q = myWorld.robot.computeInverseKinematics(initialPosition, myWorld.robot.robot.homeConfiguration, ...
                  false, diag([1, -1, -1]));
q_dot = zeros(length(q), 1);

% Định nghĩa handle DS (Truyền [] cho nominalDS)
dsModulated = @(x) modulatedDS(x, attractor, myWorld, [], 1.0); 

% Thiết lập giao diện
ax = show(myWorld.robot.robot, q, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on');
hold on;
plot3(attractor(1), attractor(2), attractor(3), 'go', 'LineWidth', 3, 'MarkerSize', 20);
title('Đang chạy mô phỏng Robot thực tế (Rho = 1.0)');

% Tắt các chế độ đồ họa để tránh lỗi WindowKeyPressFcn
hFig = myWorld.figure;
rotate3d(hFig, 'off'); zoom(hFig, 'off'); pan(hFig, 'off');
set(hFig, 'WindowKeyPressFcn', {@KeyboardCb, disturbance});

global pauseFlag stopFlag
pauseFlag = false; stopFlag = false;

tcp_traj_main = [];
position = initialPosition;
i = 0;

while vecnorm(position - attractor) > 0.01 && ~stopFlag
    while pauseFlag, drawnow; end
    
    [~, y] = ode15s(@(t,y) forwardStateDynamic(t, y, myWorld.robot.robot, dsModulated, disturbance.values, ctrl), ...
        [i * delta_t, (i+1) * delta_t], [q; q_dot]);

    q = y(end,1:6)';
    q_dot = y(end,7:12)';
    position = myWorld.robot.computeForwardKinematics(q);
    tcp_traj_main(:,end+1) = position;
    
    show(myWorld.robot.robot, q, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on');
    plot3(position(1), position(2), position(3), '.k', 'MarkerSize', 10);
    drawnow;
    i = i + 1;
end

%% SECTION 3: SO SÁNH 4 QUỸ ĐẠO TCP THỰC TẾ (4 RHO)
fprintf('Bắt đầu tính toán so sánh 4 quỹ đạo thực tế...\n');
rho_test = [0.1, 1.0, 5.0, 10.0];
colors = {'b', 'r', 'm', 'y'};

figure; hold on; grid on; axis equal;
title('So sánh quỹ đạo DS với các giá trị \rho khác nhau');
view(65,65); xlabel('X'); ylabel('Y'); zlabel('Z');

% Vẽ vật cản (Mờ)
for k = 1:length(myWorld.listOfObstacles)
    obs = myWorld.listOfObstacles(k);
    if isa(obs,'VerticalEllipsoid')
        a = obs.ellipseAxes(1); b = obs.ellipseAxes(2); c0 = obs.ellipseAxes(3); c = obs.position;
        [u,v] = meshgrid(linspace(0,2*pi,30), linspace(0,pi,30));
        surf(c(1)+a*cos(u).*sin(v), c(2)+b*sin(u).*sin(v), c(3)+c0*cos(v), ...
             'FaceColor',[0.5 0.5 0.5], 'FaceAlpha',0.1, 'EdgeColor','none');
    end
end

h_plots = [];
for j = 1:length(rho_test)
    q_sim = myWorld.robot.computeInverseKinematics(initialPosition, myWorld.robot.robot.homeConfiguration, ...
                                                 false, diag([1, -1, -1]));
    q_dot_sim = zeros(6, 1);
    pos_sim = initialPosition;
    ds_test = @(x) modulatedDS(x, attractor, myWorld, [], rho_test(j));
    
    path_data = [];
    for step = 1:1500
        if vecnorm(pos_sim - attractor) < 0.02, break; end
        [~, y_out] = ode15s(@(t,y) forwardStateDynamic(t, y, myWorld.robot.robot, ds_test, [0;0;0], ctrl), ...
                            [0, delta_t], [q_sim; q_dot_sim]);
        q_sim = y_out(end,1:6)';
        q_dot_sim = y_out(end,7:12)';
        pos_sim = myWorld.robot.computeForwardKinematics(q_sim);
        path_data(:, end+1) = pos_sim;
    end
    h_plots(j) = plot3(path_data(1,:), path_data(2,:), path_data(3,:), colors{j}, 'LineWidth', 2.5);
end

scatter3(initialPosition(1), initialPosition(2), initialPosition(3), 80, 'k', 'filled');
scatter3(attractor(1), attractor(2), attractor(3), 120, 'g', 'filled');
legend(h_plots, {'\rho = 0.1', '\rho = 1.0', '\rho = 5.0', '\rho = 10.0'}, 'Location', 'northeastoutside');

% --- HÀM BỔ TRỢ (FUNCTIONS) ---

function xdot = modulatedDS(x, attractor, world, nominalDS, rho)
    % Sửa lỗi kiểm tra nominalDS rỗng
    if isempty(nominalDS)
        xdot_nominal = -4*eye(3)*(x-attractor);
    else
        xdot_nominal = nominalDS(x);
    end

    M_tot = eye(3);
    kObst = length(world.listOfObstacles);
    gamma = inf(1, kObst);
    gradient_gamma = zeros(3, kObst);
    eps_d = 1e-9;

    for k = 1:kObst
        gamma(k) = world.listOfObstacles(k).gamma(x(1), x(2), x(3));
        gradient_gamma(:,k) = world.listOfObstacles(k).gradientGamma(x(1), x(2), x(3));
    end
    
    gminus1 = max(gamma - 1, eps_d);
    alpha = 1;

    for k = 1:kObst   
        if kObst > 1
            gk = gminus1(k);
            gi = gminus1;
            frac = gi ./ (gk + gi);
            frac(k) = 1; 
            weight = prod(frac)^alpha;
        else
            weight = 1;
        end

        % Tính ma trận điều biến D
        D = diag([ 1 - 1*weight/(gamma(k))^(1/rho), 1 + 1*weight/(gamma(k))^(1/rho), 1 + 1*weight/(gamma(k))^(1/rho)]);

        normal = gradient_gamma(:,k); 
        n_norm = norm(normal);
        if n_norm < 1e-6, continue; end

        normal = normal / n_norm;
        P = eye(3) - normal*normal.';

        tangent1 = P * [1;0;0];
        if norm(tangent1) < 1e-6, tangent1 = P * [0;1;0]; end
        tangent1 = tangent1 / norm(tangent1);
        tangent2 = cross(normal, tangent1);
        
        E = [normal, tangent1, tangent2 / norm(tangent2)];
        M_tot = (E * D * E') * M_tot;
    end    
    xdot = M_tot * xdot_nominal;
end

function dydt = forwardStateDynamic(~, y, robot, ds_control, force, ctrl)
    q = y(1:6); q_dot = y(7:12);
    current_position = tform2trvec(robot.getTransform(q, 'tool0'))';
    desired_velocity = ds_control(current_position);   
    full_jac = robot.geometricJacobian(q, 'tool0');
    controlAcceleration = ctrl.computeCommand(desired_velocity, q_dot, full_jac);
    torque_disturbance = full_jac(4:end,:)' * force;
    dydt = [q_dot; controlAcceleration + torque_disturbance];
end

function KeyboardCb(~, event, force_disturbance)
    global pauseFlag stopFlag
    switch event.Key
        case 'space', stopFlag = true; disp("Stopped simulation");
        case 'return', pauseFlag = false; disp("Continue");
        case 'p', pauseFlag = true; disp("Paused");
    end
    force_disturbance.handleKeyEvent(event.Key);
end