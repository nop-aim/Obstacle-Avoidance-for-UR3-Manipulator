%% Section 1: Create world and define obstacles
clc; clear; close all;
folder = fileparts(which('main_gofa.m'));
addpath(genpath(fullfile(folder, 'library')));

initialPosition = [0.3; 0.6; 0.5];
%initialPosition = [0.2; 0.4; 0.3];
myWorld = Environment(initialPosition);
stiffness=1000;
%3 VẬT CẢN 
%initialPosition=  [0.25; 0.6; 0.5];
%myWorld.addCylinder(0.15, [0.42, 0.5, 0.5]);      %radius, pos,rho, vel
%myWorld.addEllipsoid([0.15, 0.15, 0.15], [0.5, -0.21, 0.4]);  %ok
%myWorld.addEllipsoid([0.15, 0.15, 0.15], [0.45, 0.1, 0.5]);  %k chạm 
% 2 VẬT CẢN ĐỔI RHO 
%myWorld.addEllipsoid([0.15, 0.15, 0.15], [0.45, 0.4, 0.5]);
myWorld.addEllipsoid([0.15, 0.15, 0.15], [0.43, 0.0, 0.45]);
%1 vật cản đổi RHO
%myWorld.addCylinder(0.07, [0.3, 0.1, 0.5]);      %radius, pos,rho, vel
% đổi K
%myWorld.addEllipsoid([0.1, 0.1, 0.1], [0.35, 0.15, 0.45]);  %ok
% 1 vật cản 
%myWorld.addEllipsoid([0.07, 0.07, 0.07], [0.3, 0.1, 0.5]);  %ok
% % ==== TASK 3.3: USE gammaDistance (PHYSICAL DISTANCE) ====
%isoDist = [-0.10 0 0.10 0.30];        % m
%myWorld.showGammaIsosurfaces(isoDist, 1);     % 1 = dung gammaDistance

% Section 2: Simulate initial conditions
% Initial state: joint velocity and position
%Dòng code này có đủ các đối số: targetPosition, initialGuess, useFastSolver (false), và ma trận trọng số {W} (diag([1, -1, -1])).
%Việc này đảm bảo robot bắt đầu ở một cấu hình khớp hợp lệ và được kiểm soát ngay từ đầu.
q = myWorld.robot.computeInverseKinematics(initialPosition, myWorld.robot.robot.homeConfiguration, ...
                 false,diag([1, -1, -1]));
q_dot = zeros(length(q), 1);
%attractor = [0.5; -0.7; 0.5];
attractor = [0.5; -0.6; 0.45] ;
dsModulated = @(x) modulatedDS(x, attractor, myWorld);

% Display initial robot pose
ax = show(myWorld.robot.robot, q, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on');

% Plot target point and DS open-loop path
hold on;
plot3(attractor(1), attractor(2), attractor(3), 'go', 'LineWidth', 3, 'MarkerSize', 20);

opt_sim = []; 
opt_sim.plot  = 0;
opt_sim.dt = 3e-3;
opt_sim.i_max = 3000;
echo off;
[x_sim, ~] = Simulation(initialPosition , [], dsModulated, opt_sim);
plot3(x_sim(1,:), x_sim(2,:), x_sim(3,:), 'r', 'LineWidth', 2);

% Plot initial velocity and position
desired_vel = dsModulated(initialPosition);
plot3(initialPosition(1), initialPosition(2), initialPosition(3), '.k', 'MarkerSize', 10)
quiver3(initialPosition(1), initialPosition(2), initialPosition(3), ...
        desired_vel(1), desired_vel(2), desired_vel(3), 0.1, 'b'); 


% Plot DS open-loop path starting at random points
% axisLimit = 0.9*myWorld.axisLimit;
% for iTraj = 1:30
%     randomPosition = rand(3, 1);
%     randomPosition(1) = randomPosition(1)*(axisLimit(2)-axisLimit(1)) + axisLimit(1);
%     randomPosition(2) = randomPosition(2)*(axisLimit(4)-axisLimit(3)) + axisLimit(3);
%     randomPosition(3) = randomPosition(3)*(axisLimit(6)-axisLimit(5)) + axisLimit(5);
%     scatter3(randomPosition(1), randomPosition(2), randomPosition(3))
%     [~, x_sim, ~] = evalc('Simulation(randomPosition , [], dsModulated, opt_sim)');
% 
%     plot3(x_sim(1,:), x_sim(2,:), x_sim(3,:), 'Color', [1,0.6,0.2,0.6]);
% end

drawnow;

%legend('','','','','','','','','', ...
%       'Target','Open-loop trajectory', 'Current position', 'Desired velocity', 'DS trajectories', 'AutoUpdate','off')

%% Section 3: Simulate closed-loop trajectory

% Setup controller and disturbance
ctrl = LimitedJointVelocityController(stiffness, myWorld.robot.robot);
disturbance = CartesianForceDisturbance(5e3, 2);

% Simulation setup and parameters
% text(axisLimit(2), axisLimit(4), 0.5 * axisLimit(6),{'\bf Perturbations: ', '\rm A/D keys : X axis ', 'Q/E keys : Y axis ', 'W/S keys : Z axis ', ...
%                                                     '', '\bfStop simulation : \rm Space bar'}, 'FontSize', 12);
rotate3d off
zoom off
pan off
set(myWorld.figure, ...
    'WindowKeyPressFcn', {@KeyboardCb, disturbance}, ...
    'KeyPressFcn',       {@KeyboardCb, disturbance});
fig = myWorld.figure;
rotate3d(fig,'off');
zoom(fig,'off');
pan(fig,'off');

delta_t = 25e-3;
sim_time = 0;
i = 0;
position = myWorld.robot.computeForwardKinematics(q);
global pauseFlag stopFlag
pauseFlag = false;
stopFlag  = false;
%tao time 
% time_text = text( ...
%     myWorld.axisLimit(2)*0.5, ...
%     myWorld.axisLimit(4)*0.5, ...
%     myWorld.axisLimit(6)*0.5, ...
%     'Time: 0.00 s', ...
%     'FontSize', 12, ...
%     'FontWeight','bold', ...
%     'Color','k', ...
%     'BackgroundColor','w');
pause_times = [1 2 3];
pause_idx = 1;
tcp_traj = [];
start = tic;
while vecnorm(position - attractor) > 0.01 && ~stopFlag
    while pauseFlag
        drawnow;
    end
    % Display disturbance
    if norm(disturbance.values) > 0 && disturbance.iteration < disturbance.nb_active_iterations
        if disturbance.iteration == 0
            if exist('hline', 'var') && exist('hhead', 'var')
                delete(hline);
                delete(hhead);
            end
            [hline, hhead] = arrow3d(position', position' + 0.5 * disturbance.values'/norm(disturbance.values), ...
                15, 'cylinder', [0.2, 0.2], [20, 10], [1, 0 ,0; 1, 0, 0]);
            drawnow;
        end
        disturbance.iteration = disturbance.iteration + 1;
    else
        disturbance.values = zeros(3,1);
        disturbance.iteration = inf;
    end

    % Integrate forward given current state and control
    [t, y] = ode15s(@(t,y) forwardStateDynamic(t, y, myWorld.robot.robot, dsModulated, disturbance.values, ctrl), ...
        [i * delta_t, (i+1) * delta_t], [q; q_dot]);

    q = y(end,1:6)';
    q_dot = y(end,7:12)';
    tcp_traj(:,end+1) = position;
    %show(myWorld.robot.robot, q, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on', 'FastUpdate', 1);
    show(myWorld.robot.robot, q, 'PreservePlot', false, 'Frames', 'off', 'Visuals', 'on');
    plot3(position(1), position(2), position(3), '.k', 'MarkerSize', 10)
    drawnow;

    if ~pauseFlag && ~stopFlag
        sim_time = sim_time + delta_t;
        t_now = sim_time;
        %set(time_text, 'String', sprintf('Time: %.2f s', t_now));
    end
    position = myWorld.robot.computeForwardKinematics(q);
    myWorld.endEffectorPosition = position;

    % Plot desired velocity from DS
    if mod(i, 4)==0
        desired_vel = dsModulated(position);
        quiver3(position(1), position(2), position(3), desired_vel(1), desired_vel(2), desired_vel(3), 0.2, 'Color', [0.2,0,1,0.9]);   
    end

    % Move obstacles if desired
    for k =1:length(myWorld.listOfObstacles)
        myWorld.listOfObstacles(k).moveShape(i*delta_t)
    end

    i = i + 1;
end

myWorld.robot.robot.getTransform(q, 'tool0')
time_elapsed = toc(start);
disp("Average computation time: " + time_elapsed / i + " [s] | Total time: " + time_elapsed + " [s]")
%% ===== 3D VIEW (NO ROBOT) =====
% figure; hold on; grid on; axis equal
% xlabel('x'); ylabel('y'); zlabel('z');
% title('3D Trajectories and Obstacles')
% view(65,65)
% 
% %% ===== Obstacles (transparent) =====
% for k = 1:length(myWorld.listOfObstacles)
%     obs = myWorld.listOfObstacles(k);
% 
%     if isa(obs,'VerticalCylinder')
%         % ----- Cylinder -----
%         r = obs.radius;
%         c = obs.position;
% 
%         [th,z] = meshgrid(linspace(0,2*pi,50), ...
%                           linspace(c(3)-0.4, c(3)+0.4, 30));
% 
%         X = c(1) + r*cos(th);
%         Y = c(2) + r*sin(th);
%         Z = z;
% 
%         Cyl=surf(X,Y,Z, ...
%             'FaceColor',[0.2 0.2 0.2], ...
%             'FaceAlpha',0.25, ...
%             'EdgeColor','none');
% 
%     elseif isa(obs,'VerticalEllipsoid')
%         % ----- Ellipsoid -----
%         a = obs.ellipseAxes(1);
%         b = obs.ellipseAxes(2);
%         c0 = obs.ellipseAxes(3);
%         c = obs.position;
% 
%         [u,v] = meshgrid(linspace(0,2*pi,40), ...
%                          linspace(0,pi,40));
% 
%         X = c(1) + a*cos(u).*sin(v);
%         Y = c(2) + b*sin(u).*sin(v);
%         Z = c(3) + c0*cos(v);
% 
%         Elip=surf(X,Y,Z, ...
%             'FaceColor',[0 0.6 0], ...
%             'FaceAlpha',0.3, ...
%             'EdgeColor','none');
%     end
% end
% %% ===== Straight line (Start -> Attractor) =====
% line=plot3([initialPosition(1), attractor(1)], ...
%       [initialPosition(2), attractor(2)], ...
%       [initialPosition(3), attractor(3)], ...
%       'k--','LineWidth',1.5);
% 
% %% ===== DS open-loop (RED) =====
% DS=plot3(x_sim(1,:), x_sim(2,:), x_sim(3,:), ...
%       'r','LineWidth',3);
% 
% %% ===== TCP real trajectory =====
% TCP=plot3(tcp_traj(1,:), tcp_traj(2,:), tcp_traj(3,:), ...
%       '.','Color',[0 0 0],'MarkerSize',10);
% %% ===== Start & Attractor =====
% initial = scatter3(initialPosition(1),initialPosition(2),initialPosition(3), ...
%           80,'k','filled');
% atc = scatter3(attractor(1),attractor(2),attractor(3), ...
%           120,'g','LineWidth',2);
% %% ===== Xử lý Legend thông minh =====
% 
% % Khởi tạo danh sách các đối tượng muốn hiện trong legend
% handles = [];
% labels = {};
% 
% % 1. Kiểm tra vật cản
% if ~isempty(myWorld.listOfObstacles)
%     % Nếu có ít nhất 1 vật cản, ta ưu tiên thêm Cyl hoặc Elip vào legend
%     % Lưu ý: Kiểm tra biến tồn tại để tránh lỗi khi chỉ có 1 trong 2 loại vật cản
%     if exist('Cyl', 'var'), handles = [handles, Cyl]; labels{end+1} = 'Cylinder Obstacle'; end
%     if exist('Elip', 'var'), handles = [handles, Elip]; labels{end+1} = 'Ellipsoid Obstacle'; end
% end
% 
% % 2. Luôn thêm các đường và điểm đích (vì chúng luôn có)
% handles = [handles, line, DS, TCP, initial, atc];
% labels = [labels, {'Straight line', 'DS open-loop', 'TCP real', 'Start', 'Attractor'}];
% 
% % 3. Gọi lệnh legend duy nhất một lần
% % 'atc' là handle của scatter3 màu xanh lá, nên nó sẽ hiện đúng màu xanh lá trong legend
% lgd=legend(handles, labels, 'Location', 'northeastoutside', 'TextColor', 'blue');
% lgd.Units = 'normalized';
% lgd.Position = [0.8 0.8 0.3 0.3]; 
% lgd.FontSize = 13;        % tăng thêm nếu muốn (16, 18...)
% lgd.LineWidth = 1;        % độ dày viền legend
% 
% %lgd.FontWeight = 'bold';
% lgd.ItemTokenSize = [25 15];   % làm ký hiệu trong legend to hơn
% 
% camlight headlight
% lighting gouraud
%% ============ SO SÁNH ==========================
%% ===== EVALUATION METRICS =====
tcp = tcp_traj;
N = size(tcp,2);

%% 1. Final error
final_error = norm(tcp(:,end) - attractor);

%% 2. Convergence time
dist_to_goal = vecnorm(tcp - attractor,2,1);
eps_conv = 0.03;
idx_conv = find(dist_to_goal < eps_conv, 1, 'first');
if isempty(idx_conv)
    T_conv = NaN;
else
    T_conv = idx_conv * delta_t;
end

%% 3. Path length
L_tcp = sum(vecnorm(diff(tcp,1,2),2,1));
L_ds  = sum(vecnorm(diff(x_sim,1,2),2,1));

%% 4. Min distance to obstacles (gamma-1)
gamma_min = inf;
for i = 1:N
    for k = 1:length(myWorld.listOfObstacles)
        g = myWorld.listOfObstacles(k).gamma( ...
            tcp(1,i), tcp(2,i), tcp(3,i));
        gamma_min = min(gamma_min, g-1);
    end
end

%% 5. Velocity smoothness
vel = zeros(3,N);
for i = 1:N
    vel(:,i) = dsModulated(tcp(:,i));
end
J_v = sum(vecnorm(diff(vel,1,2),2,1));

%% 6. Straight-line deviation
L_straight = norm(initialPosition - attractor);
J_dev = L_tcp / L_straight;

%% Display
fprintf('\n===== PERFORMANCE METRICS =====\n')
fprintf('Độ chính xác hội tụ    : %.4f m\n', final_error)
fprintf('Thời gian hội tụ       : %.2f s\n', T_conv)
fprintf('Độ dài quỹ đạo(TPC)    : %.2f m\n', L_tcp)
fprintf('Độ dài quỹ đạo(DS open): %.2f m\n', L_ds)
fprintf('Min obstacle distance  : %.4f m\n', gamma_min)
fprintf('Độ mượt                : %.2f\n', J_v)
fprintf('Hiệu quả Jdev          : %.2f\n', J_dev)
%% =================== ĐỒ THỊ SO SÁNH ======================
% %Khoảng cách đến goal theo time
% figure; grid on; hold on
% t_tcp = (0:N-1)*delta_t;
% plot(t_tcp, dist_to_goal,'LineWidth',2)
% yline(0.01,'r--','Threshold')
% xlabel('Time [s]')
% ylabel('||x - x^*|| [m]')
% title('Khoảng cách đến goal theo time')
% % so sánh độ dài quỹ đạo
% figure
% bar([L_ds, L_tcp])
% set(gca,'XTickLabel',{'DS open-loop','TCP real'})
% ylabel('Path length [m]')
% title('Trajectory Length Comparison')
% grid on
% %Khoảng cách tới vật cản 
% gamma_traj = zeros(1,N);
% for i = 1:N
%     gmin = inf;
%     for k = 1:length(myWorld.listOfObstacles)
%         g = myWorld.listOfObstacles(k).gamma( ...
%             tcp(1,i), tcp(2,i), tcp(3,i));
%         gmin = min(gmin, g-1);
%     end
%     gamma_traj(i) = gmin;
% end
% 
% figure; grid on; hold on
% plot(t_tcp, gamma_traj,'LineWidth',2)
% yline(0,'r--','Collision boundary')
% xlabel('Time [s]')
% ylabel('gamma(x)-1')
% title('Obstacle Clearance')
% % Độ mượt vận tốc
% figure; grid on
% plot(vecnorm(vel,2,1),'LineWidth',2)
% xlabel('Iteration')
% ylabel('||v||')
% title('Velocity magnitude along trajectory')

%% ================ Simulation function===============
function dydt = forwardStateDynamic(~, y, robot, ds_control, force, ctrl)

    % Current state
    q = y(1:6);
    q_dot = y(7:12);

    % Desired state
    current_position = tform2trvec(robot.getTransform(q, 'tool0'))';
    desired_velocity = ds_control(current_position);   

    % Compute corresponding control torque (PD controller)
    full_jac = robot.geometricJacobian(q, 'tool0');
    controlAcceleration = ctrl.computeCommand(desired_velocity, q_dot, full_jac);

    % Compute disturbance
    torque_disturbance = full_jac(4:end,:)' * force;

    % Compute state derivative from current state and control
    dydt = [q_dot; ...
            controlAcceleration + torque_disturbance];
end


function KeyboardCb(~, event, force_disturbance)
    global pauseFlag stopFlag

    switch event.Key
        case 'space'   % dừng hẳn
            stopFlag = true;
            disp("Stopped simulation")

        case 'return'  % ENTER = tiếp tục
            pauseFlag = false;
            disp("Continue")

        case 'p'       % P = pause
            pauseFlag = true;
            disp("Paused")
    end

    force_disturbance.handleKeyEvent(event.Key);
end


function xdot = modulatedDS(x, attractor, world, nominalDS)

    if nargin < 4
        xdot_nominal = -4*eye(3)*(x-attractor);
    else
        xdot_nominal = nominalDS(x);
    end

    % Initialize modulation matrix and average obstacle velocity
    M_tot = eye(3);
    %meanObstacleVelocity = zeros(3, 1);

    %% ------ Write your code below ------ %%
    %  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv %

    % Create the array of gamma values from the list world.listOfObstacles
    % for convenience
    kObst = length(world.listOfObstacles);
    gamma = inf(1, kObst);
    gradient_gamma = zeros(3, kObst);
    eps_d = 1e-9;                   % tranh chia cho 0
    %gminus1 = max(gamma - 1, eps_d);
    rho=1;

    for k = 1:kObst
        % Get the gamma value of obstacle k at point x
        gamma(k) =  world.listOfObstacles(k).gamma(x(1), x(2), x(3));
        %gamma(k) =  world.listOfObstacles(k).gammaDistance(x(1), x(2), x(3));
        % Get the gradient of obstacle k at point x
        gradient_gamma(:,k)=world.listOfObstacles(k).gradientGamma(x(1), x(2), x(3));
    end
     % phải đưa xuốn sau for để cập nhật trọng số với nhiều vật cản 
    gminus1 = max(gamma - 1, eps_d);% vector: gamma^i(x) - 1 cho moi i
    % Go over each obstacle, compute the modulation matrix and apply it to
    % the initial ds velocity xdot_modulated
    alpha=1;
    for k = 1:kObst   
        % Task 2: Compute weight function 
        if kObst > 1
            gk   = gminus1(k);
            gi   = gminus1;
            frac = gi ./ (gk + gi);
            frac(k) = 1; 
            weight = prod(frac)^alpha;
        else
            weight = 1;
        end
        %rho là p trong lí thuyết là reactivity
        % Compute modulation matrix 
        D = diag([ 1 - 1*weight/(gamma(k))^(1/rho), 1 + 1*weight/(gamma(k))^(1/rho), 1 + 1*weight/(gamma(k))^(1/rho)]);
        %lambda_n = 1 - weight * exp(-gamma(k)/rho);
        %lambda_t = 1 + weight * exp(-gamma(k)/rho);
        %D = diag([lambda_n, lambda_t, lambda_t]);
        % Compute normal and tangent on ellipse

        %normal = gradient_gamma(:,k) / norm(gradient_gamma(:,k));
        normal = gradient_gamma(:,k); 
        n_norm = norm(normal);

        if n_norm < 1e-6
            continue   % bỏ obstacle này tại điểm x
        end

        normal = normal / (norm(normal));              
        normal = normal(:);
        P = eye(3) - normal*normal.';

        tangent1 = P * [1;0;0];
        if norm(tangent1) < 1e-6
            tangent1 = P * [0;1;0];
        end
        tangent1 = tangent1 / norm(tangent1);
        tangent2 = cross(normal, tangent1);
        tangent2 = tangent2 / norm(tangent2);

        E = [normal, tangent1, tangent2];

        Mk = E * D * E';
        M_tot = Mk * M_tot;
    end    

    xdot = M_tot * xdot_nominal;

end