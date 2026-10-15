clc; clear; close all;
%% ================= CONFIGURATION =================
% Đường dẫn tới file .mat sinh ra từ file mô phỏng của bạn
filename = 'logs\3link_1_obs_and_target_vel.mat'; 
step_render = 2;       % Bước nhảy tăng tốc độ ghi video (1: vẽ đủ khung hình, 2: nhảy cách 1 khung)
colors_main = lines(7); 
enable_snapshot_logging = true;       % Đặt thành true để BẬT, false để TẮT tính năng lưu theo thời gian
snapshot_interval = 0.25;
%% ===== STYLE GIỐNG FILE REE =====
FONT_NAME = 'Helvetica';

figSize   = [100 100 700 520];   % giống REE

fontAxis  = 13;                  % số trên trục
fontLabel = 13;                  % chữ tên trục
fontTitle = 15;                  % tiêu đề
fontLeg   = 13;                  % legend

lineW     = 2.3;                 % line chính
lineW_sub = 1.6;                 % line phụ
gridAlpha = 0.25;

%% ================= AUTOMATICALLY CREATE VIDEO & IMAGE DIRECTORIES =================
if ~exist(filename, 'file')
    error('Log data file not found. Please run the simulation file first or check the path: %s', filename);
end
[folder, baseLogName, ~] = fileparts(filename);
% DEFINE COMMON SAVE DIRECTORY: logs/video
videoFolder = fullfile(folder, 'video'); 
if ~exist(videoFolder, 'dir'), mkdir(videoFolder); end
% Create a separate subdirectory to hold .fig files for this specific .mat file
figFolder = fullfile(videoFolder, [baseLogName, '_figs']);
if ~exist(figFolder, 'dir'), mkdir(figFolder); end
% Output path for .mp4 video file
video_path = fullfile(videoFolder, sprintf('combined_%s.mp4', baseLogName));

%% ================= READ AND EXTRACT DATA =================
loaded = load(filename);
logData = loaded.logData;
t = logData.time;
N_steps = length(t);
dt = 0.01; % Sampling period corresponding to delta_t in the simulation

% --- Auto-detect plot mode (Case 1: Only Joint 6 vs Case 2: Joint 3 to Joint 6) ---
onlyJoint6 = true;
if isfield(logData, 'dist_to_obs') && length(logData.dist_to_obs) >= 3 && ~isempty(logData.dist_to_obs{3})
    onlyJoint6 = false;
elseif ~isfield(logData, 'dist_to_obs') || isempty(logData.dist_to_obs)
    % Nếu không có dữ liệu chướng ngại vật, mặc định vẽ từ Joint 3 đến Joint 6
    onlyJoint6 = false;
end

% 0. Initial position of the robot
sx = logData.initialPosition(1,:);
sy = logData.initialPosition(2,:);
sz = logData.initialPosition(3,:);
% 1. TCP Position of the Robot
x = logData.position_tcp(1, :);
y = logData.position_tcp(2, :);
z = logData.position_tcp(3, :);

% 2. Auto-computed TCP Velocity
vx = [0, diff(x) / dt];
vy = [0, diff(y) / dt];
vz = [0, diff(z) / dt];
v_norm = sqrt(vx.^2 + vy.^2 + vz.^2);

% 3. Target (Attractor) Position
tx = logData.attractor(1, :);
ty = logData.attractor(2, :);
tz = logData.attractor(3, :);

% 4. TCP Tracking Error
tcp_error = logData.error_norm;
err_x = x - tx; 
err_y = y - ty; 
err_z = z - tz;

% 5. Joint Positions (Joint Trajectories)
x_3 = []; y_3 = []; z_3 = [];
x_4 = []; y_4 = []; z_4 = [];
x_5 = []; y_5 = []; z_5 = [];
if ~onlyJoint6 && isfield(logData, 'joint_positions') && length(logData.joint_positions) >= 5
    x_3 = logData.joint_positions{3}(1,:); y_3 = logData.joint_positions{3}(2,:); z_3 = logData.joint_positions{3}(3,:);
    x_4 = logData.joint_positions{4}(1,:); y_4 = logData.joint_positions{4}(2,:); z_4 = logData.joint_positions{4}(3,:);
    x_5 = logData.joint_positions{5}(1,:); y_5 = logData.joint_positions{5}(2,:); z_5 = logData.joint_positions{5}(3,:);
end
x_6 = logData.joint_positions{6}(1,:); y_6 = logData.joint_positions{6}(2,:); z_6 = logData.joint_positions{6}(3,:);

% --- Extract joint angle data and physical limits ---
theta_data = logData.theta;
theta_min = logData.theta_min;
theta_max = logData.theta_max;

% 6. Obstacle & Safety Distance Data (AUTO-ADAPTIVE: 1 OR 2 OBSTACLES)
hasObstacle = isfield(logData, 'obstacle_position') && ~isempty(logData.obstacle_position);
hasObstacle2 = isfield(logData, 'obstacle2_position') && ~isempty(logData.obstacle2_position);

% --- Kiểm tra xem vật cản là chuyển động hay đứng yên ---
isMoving = false;
if hasObstacle
    pos_diff = max(logData.obstacle_position, [], 2) - min(logData.obstacle_position, [], 2);
    if any(pos_diff > 1e-4) % Dịch chuyển lớn hơn 0.1 mm
        isMoving = true;
    end
end

% --- Xác định các bước cần chụp và lưu hình dưới dạng .fig ---
loop_steps = 1:step_render:N_steps;
n_loop = length(loop_steps);
if hasObstacle && isMoving
    % Chọn 4 chỉ số trải đều từ các bước lặp thực tế
    idx1 = 1;
    idx2 = round(n_loop / 3);
    idx3 = round(2 * n_loop / 3);
    idx4 = n_loop;
    
    idx2 = max(1, min(idx2, n_loop));
    idx3 = max(1, min(idx3, n_loop));
    
    snap_steps = unique([loop_steps(idx1), loop_steps(idx2), loop_steps(idx3), loop_steps(idx4)]);
else
    % Chỉ chụp ở bước cuối cùng
    snap_steps = [loop_steps(end)];
end

if hasObstacle
    % Obstacle 1
    ox1 = logData.obstacle_position(1, :);
    oy1 = logData.obstacle_position(2, :);
    oz1 = logData.obstacle_position(3, :);
    
    R1_val = logData.R1;
    R2_val = logData.R2;
    
    dist_3 = []; dist_4 = []; dist_5 = [];
    if ~onlyJoint6
        dist_3 = logData.dist_to_obs{3};
        dist_4 = logData.dist_to_obs{4};
        dist_5 = logData.dist_to_obs{5};
    end
    dist_6 = logData.dist_to_obs{6};
    
    % Obstacle 2
    if hasObstacle2
        ox2 = logData.obstacle2_position(1, :);
        oy2 = logData.obstacle2_position(2, :);
        oz2 = logData.obstacle2_position(3, :);
        
        dist2_3 = []; dist2_4 = []; dist2_5 = [];
        if ~onlyJoint6
            dist2_3 = logData.dist_to_obs2{3};
            dist2_4 = logData.dist_to_obs2{4};
            dist2_5 = logData.dist_to_obs2{5};
        end
        dist2_6 = logData.dist_to_obs2{6};
    end
end
theta_vec = linspace(0, 2*pi, 100);
[u_mesh, v_mesh] = meshgrid(linspace(0, 2*pi, 30), linspace(0, pi, 30));

%% ================= INITIALIZE AND POSITION 9 PLOT WINDOWS =================
% % FIG 1: Top View
% fig1 = figure('Color','w','Name','Top View (X,Y)','Visible','on','NumberTitle','off'); hold on; grid on; box on; axis equal;
% h_top_initialPosition = plot(NaN,NaN,'k.', 'MarkerSize', 10, 'LineWidth', 1.5);
% h_top_tcp = plot(NaN, NaN, 'b-', 'LineWidth', 2);
% h_top_tcp_pt = plot(NaN, NaN, 'ro', 'MarkerFaceColor', 'r');
% h_top_target = plot(NaN, NaN, 'g*', 'MarkerSize', 10, 'LineWidth', 1.5);
% h_top_obs1 = fill(NaN, NaN, [0.7 0.7 0.7], 'FaceAlpha', 0.4, 'EdgeColor', [0.3 0.3 0.3]);
% h_top_obs2 = fill(NaN, NaN, [0.9 0.4 0.4], 'FaceAlpha', 0.4, 'EdgeColor', [0.6 0.2 0.2]); 
% xlabel('x (m)', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_LABEL); 
% ylabel('y (m)', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_LABEL); 
% title('Top View (X,Y)', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_TITLE, 'FontWeight', 'bold');
% set(gca, 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_LABEL - 1);
% axis([-0.6 0.6 -0.1 0.5]);

% % FIG 2: Left View
% fig2 = figure('Color','w','Name','Left View (Y,Z)','Visible','on','NumberTitle','off'); hold on; grid on; box on; axis equal;
% h_left_initialPosition = plot(NaN,NaN,'k.', 'MarkerSize', 10, 'LineWidth', 1.5);
% h_left_tcp = plot(NaN, NaN, 'b-', 'LineWidth', 2);
% h_left_tcp_pt = plot(NaN, NaN, 'ro', 'MarkerFaceColor', 'r');
% h_left_target = plot(NaN, NaN, 'g*', 'MarkerSize', 10, 'LineWidth', 1.5);
% h_left_obs1 = fill(NaN, NaN, [0.7 0.7 0.7], 'FaceAlpha', 0.4, 'EdgeColor', [0.3 0.3 0.3]);
% h_left_obs2 = fill(NaN, NaN, [0.9 0.4 0.4], 'FaceAlpha', 0.4, 'EdgeColor', [0.6 0.2 0.2]);
% xlabel('y (m)', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_LABEL); 
% ylabel('z (m)', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_LABEL); 
% title('Left View (Y,Z)', 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_TITLE, 'FontWeight', 'bold');
% set(gca, 'FontName', FONT_NAME, 'FontSize', FONT_SIZE_LABEL - 1);
% axis([-0.1 0.5 -0.1 0.7]);

% FIG 3: Front View
fig3 = figure('Color','w','Name','Front View (X,Z)','Visible','on','NumberTitle','off'); hold on; grid on; box on; axis equal;
h_front_initialPosition = plot(NaN,NaN,'k.', 'MarkerSize', 10, 'LineWidth', lineW_sub);
h_front_tcp = plot(NaN, NaN, 'b-', 'LineWidth', lineW);
h_front_tcp_pt = plot(NaN, NaN, 'ro', 'MarkerFaceColor', 'r');
h_front_target = plot(NaN, NaN, 'g*', 'MarkerSize', 10, 'LineWidth', lineW_sub);
h_front_obs1 = fill(NaN, NaN, [0.7 0.7 0.7], 'FaceAlpha', 0.4, 'EdgeColor', [0.3 0.3 0.3]);
h_front_obs2 = fill(NaN, NaN, [0.9 0.4 0.4], 'FaceAlpha', 0.4, 'EdgeColor', [0.6 0.2 0.2]);
xlabel('\bf X (m)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold'); 
ylabel('\bf Z (m)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold'); 
title('\bf Front View (X,Z)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontTitle, 'FontWeight', 'bold');
if hasObstacle2
    lgd3 = legend([h_front_initialPosition,h_front_tcp, h_front_target, h_front_obs1, h_front_obs2], ...
        {'Start','TCP','Target', 'Obstacle 1', 'Obstacle 2'}, 'Location', 'best');
elseif hasObstacle
    lgd3 = legend([h_front_initialPosition,h_front_tcp, h_front_target,h_front_obs1], ...
        {'Start','TCP','Target', 'Obstacle 1'}, 'Location', 'best');
else
    lgd3 = legend([h_front_initialPosition,h_front_tcp, h_front_target], ...
        {'Start','TCP','Target'}, 'Location', 'best');
end
set(lgd3, 'FontName', FONT_NAME, 'FontSize', fontLeg);
set(gca, 'FontName', FONT_NAME, 'FontSize', fontAxis, 'FontWeight', 'bold', 'LineWidth', 1.2, 'GridAlpha', gridAlpha);
% axis([-0.6 0.6 -0.1 0.7]);

% FIG 4: 3D View Dynamic
fig4 = figure('Color','w','Name','3D View','Visible','on','NumberTitle','off'); 
hold on; grid on; box on; axis equal; view(3);
view(7.5, 2);
h_3d_initialPosition = plot(NaN,NaN,'k.', 'MarkerSize', 10, 'LineWidth', 1.5);
h_3d_tcp = plot3(NaN, NaN, NaN, 'k-', 'LineWidth', 2);
h_3d_tcp_pt = plot3(NaN, NaN, NaN, 'ro', 'MarkerFaceColor', 'r');
h_3d_target = plot3(NaN, NaN, NaN, 'g*', 'MarkerSize', 10, 'LineWidth', 1.5);
h_3d_obs1 = surf(nan(30), nan(30), nan(30), 'FaceAlpha', 0.4, 'EdgeColor', 'none', 'FaceColor', [0.8 0.8 0.8],'FaceLighting', 'gouraud');
h_3d_obs2 = surf(nan(30), nan(30), nan(30), 'FaceAlpha', 0.4, 'EdgeColor', 'none', 'FaceColor', [0.9 0.4 0.4]);
xlabel('\bf X (m)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold'); 
ylabel('\bf Y (m)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold'); 
zlabel('\bf Z (m)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold'); 
h_title4 = title('\bf t = 0.00 s', 'Interpreter','tex', 'FontName',FONT_NAME,'FontSize',fontTitle,'FontWeight','bold');
% if hasObstacle2
%     lgd4 = legend([h_3d_initialPosition,h_3d_tcp, h_3d_target, h_3d_obs1, h_3d_obs2], ...
%         {'Start','TCP','Target', 'Obstacle 1', 'Obstacle 2'}, 'Location', 'best');
% elseif hasObstacle
%     lgd4 = legend([h_3d_initialPosition,h_3d_tcp, h_3d_target, h_3d_obs1], ...
%         {'Start','TCP','Target', 'Obstacle 1'}, 'Location', 'best');
% else
%     lgd4 = legend([h_3d_initialPosition,h_3d_tcp, h_3d_target], ...
%         {'Start','TCP','Target'}, 'Location', 'best');
% end
axis([0.1, 0.9, -0.27, 0.34, 0.15, 0.48]);
% set(lgd4, 'FontName', FONT_NAME, 'FontSize', fontLeg);
set(gca, 'FontName', FONT_NAME, 'FontSize', fontAxis, 'FontWeight', 'bold', 'LineWidth', 1.2, 'GridAlpha', gridAlpha);

% FIG 5: Velocity Profile
fig5 = figure('Color','w','Name','Velocity Profile','Visible','on','NumberTitle','off');
subplot(2,1,1); hold on; grid on; box on;
h_v_norm = plot(NaN, NaN, 'LineWidth', lineW, 'Color', colors_main(1,:));
ylabel('\bf |V| (m/s)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold'); 
title('\bf TCP Resultant Velocity', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontTitle, 'FontWeight', 'bold');
set(gca, 'FontName', FONT_NAME, 'FontSize', fontAxis, 'FontWeight', 'bold', 'LineWidth', 1.2, 'GridAlpha', gridAlpha); xlim([0, t(end)]);
subplot(2,1,2); hold on; grid on; box on;
h_vx = plot(NaN, NaN, 'r', 'LineWidth', lineW_sub);
h_vy = plot(NaN, NaN, 'g', 'LineWidth', lineW_sub);
h_vz = plot(NaN, NaN, 'b', 'LineWidth', lineW_sub);
xlabel('\bf Time (s)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold'); 
ylabel('\bf Velocity Components (m/s)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold');
lgd5 = legend('V_x','V_y','V_z', 'Location', 'best'); 
set(lgd5, 'FontName', FONT_NAME, 'FontSize', fontLeg);
set(gca, 'FontName', FONT_NAME, 'FontSize', fontAxis, 'FontWeight', 'bold', 'LineWidth', 1.2, 'GridAlpha', gridAlpha); xlim([0, t(end)]);

% FIG 6: Joint Trajectories 
fig6 = figure('Color','w','Name','Joint Trajectories','Visible','on','NumberTitle','off'); hold on; grid on; box on; axis equal; view(3);
h_initialPosition = plot(NaN,NaN,'k.', 'MarkerSize', 10, 'LineWidth', lineW_sub);

if onlyJoint6
    h_j6 = plot3(NaN,NaN,NaN,'k','LineWidth',lineW);
else
    h_j3 = plot3(NaN,NaN,NaN,'b','LineWidth',lineW_sub);
    h_j4 = plot3(NaN,NaN,NaN,'c','LineWidth',lineW_sub);
    h_j5 = plot3(NaN,NaN,NaN,'m','LineWidth',lineW_sub);
    h_j6 = plot3(NaN,NaN,NaN,'k','LineWidth',lineW);
end

h_j6_obs1 = surf(nan(30), nan(30), nan(30), 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceColor', [0.6 0.6 0.6]);
h_j6_obs2 = surf(nan(30), nan(30), nan(30), 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceColor', [0.9 0.4 0.4]);
xlabel('\bf X (m)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold'); 
ylabel('\bf Y (m)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold'); 
zlabel('\bf Z (m)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold'); 
title('\bf Joint Trajectories', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontTitle, 'FontWeight', 'bold');
set(gca, 'FontName', FONT_NAME, 'FontSize', fontAxis, 'FontWeight', 'bold', 'LineWidth', 1.2, 'GridAlpha', gridAlpha);

% Fix dynamic legend for FIG 6 (avoid empty handles when no obstacles)
legend_handles = [];
legend_labels = {};
if onlyJoint6
    legend_handles = [h_j6];
    legend_labels = {'Joint 6 (TCP)'};
else
    legend_handles = [h_j3, h_j4, h_j5, h_j6];
    legend_labels = {'Joint 3','Joint 4','Joint 5','Joint 6 (TCP)'};
end

if hasObstacle
    legend_handles = [legend_handles, h_j6_obs1];
    legend_labels = [legend_labels, {'Obs1'}];
end
if hasObstacle2
    legend_handles = [legend_handles, h_j6_obs2];
    legend_labels = [legend_labels, {'Obs2'}];
end

lgd6 = legend(legend_handles, legend_labels, 'Location', 'best');
set(lgd6, 'FontName', FONT_NAME, 'FontSize', fontLeg);
% axis([-0.6 0.6 -0.1 0.5 -0.1 0.7]);

% FIG 7: Tracking Error
fig7 = figure('Color','w','Name','Tracking Error','Visible','on','NumberTitle','off'); hold on; grid on; box on;
h_err_norm = plot(NaN, NaN, 'k-', 'LineWidth', lineW);
h_err_x = plot(NaN, NaN, 'r--', 'LineWidth', lineW_sub);
h_err_y = plot(NaN, NaN, 'g--', 'LineWidth', lineW_sub);
h_err_z = plot(NaN, NaN, 'b--', 'LineWidth', lineW_sub);
xlabel('\bf Time (s)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold'); 
ylabel('\bf Tracking Error (m)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold'); 
title('\bf Target Tracking Error', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontTitle, 'FontWeight', 'bold');
lgd7 = legend('||e|| Resultant', 'X-axis Error', 'Y-axis Error', 'Z-axis Error', 'Location', 'best');
set(lgd7, 'FontName', FONT_NAME, 'FontSize', fontLeg);
set(gca, 'FontName', FONT_NAME, 'FontSize', fontAxis, 'FontWeight', 'bold', 'LineWidth', 1.2, 'GridAlpha', gridAlpha);
xlim([0, t(end)]);

% FIG 8: Joint Obstacle Distance
fig8 = figure('Color','w','Name','Joint Obstacle Distance','Visible','on','NumberTitle','off'); hold on; grid on; box on;
if hasObstacle
    if onlyJoint6
        h_d6 = plot(NaN, NaN, '-', 'Color', colors_main(6,:), 'LineWidth', lineW);
        if hasObstacle2
            h_d6_obs2 = plot(NaN, NaN, '-.', 'Color', colors_main(6,:), 'LineWidth', lineW_sub);
        end
    else
        h_d3 = plot(NaN, NaN, '-', 'Color', colors_main(3,:), 'LineWidth', lineW);
        h_d4 = plot(NaN, NaN, '-', 'Color', colors_main(4,:), 'LineWidth', lineW);
        h_d5 = plot(NaN, NaN, '-', 'Color', colors_main(5,:), 'LineWidth', lineW);
        h_d6 = plot(NaN, NaN, '-', 'Color', colors_main(6,:), 'LineWidth', lineW);
        if hasObstacle2
            h_d3_obs2 = plot(NaN, NaN, '-.', 'Color', colors_main(3,:), 'LineWidth', lineW_sub);
            h_d4_obs2 = plot(NaN, NaN, '-.', 'Color', colors_main(4,:), 'LineWidth', lineW_sub);
            h_d5_obs2 = plot(NaN, NaN, '-.', 'Color', colors_main(5,:), 'LineWidth', lineW_sub);
            h_d6_obs2 = plot(NaN, NaN, '-.', 'Color', colors_main(6,:), 'LineWidth', lineW_sub);
        end
    end
    
    h_R1 = plot(NaN, NaN, 'r--', 'LineWidth', lineW);
    h_R2 = plot(NaN, NaN, 'm:', 'LineWidth', lineW);
    xlabel('\bf Time (s)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold'); 
    ylabel('\bf Distance (m)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold'); 
    set(gca, 'FontName', FONT_NAME, 'FontSize', fontAxis, 'FontWeight', 'bold', 'LineWidth', 1.2, 'GridAlpha', gridAlpha);
    
    if hasObstacle2
        title('\bf Joint Obstacle Avoidance - 2 Obstacles', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontTitle, 'FontWeight', 'bold');
        if onlyJoint6
            lgd8 = legend([h_d6, h_d6_obs2, h_R1, h_R2], ...
                {'Joint 6 (Obs1)','Joint 6 (Obs2)','R_1 Danger','R_2 Influence'}, ...
                'Location', 'best', 'NumColumns', 2);
            ylim([0, max([max(dist_6), max(dist2_6), max(R2_val)]) * 1.2]);
        else
            lgd8 = legend([h_d3, h_d6, h_d3_obs2, h_d6_obs2, h_R1, h_R2], ...
                {'Joint 3 (Obs1)','Joint 6 (Obs1)','Joint 3 (Obs2)','Joint 6 (Obs2)','R_1 Danger','R_2 Influence'}, ...
                'Location', 'best', 'NumColumns', 2);
            ylim([0, max([max(dist_3), max(dist2_3), max(R2_val)]) * 1.2]);
        end
    else
        title('\bf Joint Obstacle Distance', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontTitle, 'FontWeight', 'bold');
        if onlyJoint6
            lgd8 = legend([h_d6, h_R1, h_R2], ...
                {'Joint 6','R_1 Danger Boundary','R_2 Influence Boundary'}, 'Location', 'best');
            ylim([0, max([max(dist_6), max(R2_val)]) * 1.2]);
        else
            lgd8 = legend([h_d3, h_d4, h_d5, h_d6, h_R1, h_R2], ...
                {'Joint 3','Joint 4','Joint 5','Joint 6','R_1 Danger Boundary','R_2 Influence Boundary'}, 'Location', 'best');
            ylim([0, max([max(dist_3), max(R2_val)]) * 1.2]);
        end
    end
    set(lgd8, 'FontName', FONT_NAME, 'FontSize', fontLeg);
    xlim([0, t(end)]);
else
    title('\bf No obstacle data available', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontTitle, 'FontWeight', 'bold');
end

% ===== FIG 9: NEW CHART - JOINT ANGLES & SAFE BOUNDARY LIMITS =====
fig9 = figure('Color','w','Name','Joint Angle Limits','Visible','on','NumberTitle','off');
h_joint_lines = zeros(6, 1);
for j_idx = 1:6
    subplot(3,2,j_idx); hold on; grid on; box on;
    h_joint_lines(j_idx) = plot(NaN, NaN, 'b-', 'LineWidth', lineW);
    plot([0, t(end)], [theta_max(j_idx), theta_max(j_idx)], 'r--', 'LineWidth', lineW_sub);
    plot([0, t(end)], [theta_min(j_idx), theta_min(j_idx)], 'r--', 'LineWidth', lineW_sub);
    
    title(sprintf('\\bf Joint %d', j_idx), 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold');
    xlim([0, t(end)]);
    ylim([theta_min(j_idx) - 0.5, theta_max(j_idx) + 0.5]);
    set(gca, 'FontName', FONT_NAME, 'FontSize', fontAxis - 2, 'FontWeight', 'bold', 'LineWidth', 1.2, 'GridAlpha', gridAlpha);
    if mod(j_idx, 2) == 1, ylabel('\bf \theta (rad)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold'); end
    if j_idx >= 5, xlabel('\bf Time (s)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold'); end
end

sgtitle('\bf Joint Angle Variation', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontTitle, 'FontWeight', 'bold');

% --- POSITION WINDOWS ON SCREEN (OVERLAPPING/CASCADED WITH FIXED REE SIZE) ---
% figs = [fig1, fig2, fig3, fig4, fig5, fig6, fig7, fig8, fig9];
figs = [fig3,fig4, fig5, fig6, fig7, fig8, fig9];

for k = 1:length(figs)
    % Xếp chồng lên nhau (cascaded) với kích thước chuẩn 700x520 giống REE
    posX = figSize(1) + (k-1) * 25;
    posY = figSize(2) + (k-1) * 25;
    set(figs(k), 'Position', [posX, posY, figSize(3), figSize(4)]);
end

%% ================= INITIALIZE NEW VIDEO WRITER =================
vout = VideoWriter(video_path, 'MPEG-4');
vout.FrameRate = 30;
open(vout);

%% ================= DYNAMIC PLOT GENERATION LOOP (3X3 GRID) =================
disp('Processing data, drawing real-time plots and recording 3x3 grid video...');
last_saved_time = -inf;
for i = 1:step_render:N_steps
    % % --- 1. Update Top View ---
    % set(h_top_initialPosition,'XData', sx(1:i), 'YData', sy(1:i));
    % set(h_top_tcp, 'XData', x(1:i), 'YData', y(1:i));
    % set(h_top_tcp_pt, 'XData', x(i), 'YData', y(i));
    % set(h_top_target, 'XData', tx(i), 'YData', ty(i));
    % if hasObstacle
    %     r_current = R1_val(i); 
    %     set(h_top_obs1, 'XData', ox1(i) + r_current*cos(theta_vec), 'YData', oy1(i) + r_current*sin(theta_vec));
    %     if hasObstacle2
    %         set(h_top_obs2, 'XData', ox2(i) + r_current*cos(theta_vec), 'YData', oy2(i) + r_current*sin(theta_vec));
    %     end
    % end
    % 
    % % --- 2. Update Left View ---
    % set(h_left_initialPosition,'XData', sy(1:i), 'YData', sz(1:i));
    % set(h_left_tcp, 'XData', y(1:i), 'YData', z(1:i));
    % set(h_left_tcp_pt, 'XData', y(i), 'YData', z(i));
    % set(h_left_target, 'XData', ty(i), 'YData', tz(i));
    % if hasObstacle
    %     set(h_left_obs1, 'XData', oy1(i) + r_current*cos(theta_vec), 'YData', oz1(i) + r_current*sin(theta_vec));
    %     if hasObstacle2
    %         set(h_left_obs2, 'XData', oy2(i) + r_current*cos(theta_vec), 'YData', oz2(i) + r_current*sin(theta_vec));
    %     end
    % end
    
    % --- 3. Update Front View ---
    set(h_front_initialPosition,'XData', sx(1:i), 'YData', sz(1:i));
    set(h_front_tcp, 'XData', x(1:i), 'YData', z(1:i));
    set(h_front_tcp_pt, 'XData', x(i), 'YData', z(i));
    set(h_front_target, 'XData', tx(i), 'YData', tz(i));
    if hasObstacle
        r_current = R1_val(i); 
        set(h_front_obs1, 'XData', ox1(i) + r_current*cos(theta_vec), 'YData', oz1(i) + r_current*sin(theta_vec));
        if hasObstacle2 
            set(h_front_obs2, 'XData', ox2(i) + r_current*cos(theta_vec), 'YData', oz2(i) + r_current*sin(theta_vec));
        end
    end
    
    % --- 4. Update 3D View ---
    set(h_3d_initialPosition,'XData', sx(1:i), 'YData', sy(1:i), 'ZData', sz(1:i));
    set(h_3d_tcp, 'XData', x(1:i), 'YData', y(1:i), 'ZData', z(1:i));
    set(h_3d_tcp_pt, 'XData', x(i), 'YData', y(i), 'ZData', z(i));
    set(h_3d_target, 'XData', tx(i), 'YData', ty(i), 'ZData', tz(i));
    set(h_title4,'String', sprintf('\\bf t = %.2f s', t(i)));
    if hasObstacle
        set(h_3d_obs1, 'XData', ox1(i) + r_current*cos(u_mesh).*sin(v_mesh), ...
                       'YData', oy1(i) + r_current*sin(u_mesh).*sin(v_mesh), ...
                       'ZData', oz1(i) + r_current*cos(v_mesh));
        if hasObstacle2
            set(h_3d_obs2, 'XData', ox2(i) + r_current*cos(u_mesh).*sin(v_mesh), ...
                           'YData', oy2(i) + r_current*sin(u_mesh).*sin(v_mesh), ...
                           'ZData', oz2(i) + r_current*cos(v_mesh));
        end
    end
    
    % --- 5. Update TCP Velocity Plot ---
    set(h_v_norm, 'XData', t(1:i), 'YData', v_norm(1:i));
    set(h_vx, 'XData', t(1:i), 'YData', vx(1:i));
    set(h_vy, 'XData', t(1:i), 'YData', vy(1:i));
    set(h_vz, 'XData', t(1:i), 'YData', vz(1:i));
    
    % --- 6. Update Joint Trajectory Plot ---
    set(h_initialPosition,'XData', sx(1:i), 'YData', sy(1:i), 'ZData', sz(1:i));
    if onlyJoint6
        set(h_j6, 'XData', x_6(1:i), 'YData', y_6(1:i), 'ZData', z_6(1:i));
    else
        set(h_j3, 'XData', x_3(1:i), 'YData', y_3(1:i), 'ZData', z_3(1:i));
        set(h_j4, 'XData', x_4(1:i), 'YData', y_4(1:i), 'ZData', z_4(1:i));
        set(h_j5, 'XData', x_5(1:i), 'YData', y_5(1:i), 'ZData', z_5(1:i));
        set(h_j6, 'XData', x_6(1:i), 'YData', y_6(1:i), 'ZData', z_6(1:i));
    end
    if hasObstacle
        set(h_j6_obs1, 'XData', ox1(i) + r_current*cos(u_mesh).*sin(v_mesh), ...
                       'YData', oy1(i) + r_current*sin(u_mesh).*sin(v_mesh), ...
                       'ZData', oz1(i) + r_current*cos(v_mesh));
        if hasObstacle2
            set(h_j6_obs2, 'XData', ox2(i) + r_current*cos(u_mesh).*sin(v_mesh), ...
                           'YData', oy2(i) + r_current*sin(u_mesh).*sin(v_mesh), ...
                           'ZData', oz2(i) + r_current*cos(v_mesh));
        end
    end
    
    % --- 7. Update Tracking Error Plot ---
    set(h_err_norm, 'XData', t(1:i), 'YData', tcp_error(1:i));
    set(h_err_x, 'XData', t(1:i), 'YData', err_x(1:i));
    set(h_err_y, 'XData', t(1:i), 'YData', err_y(1:i));
    set(h_err_z, 'XData', t(1:i), 'YData', err_z(1:i));
    
    % --- 8. Update Obstacle Distance Plot ---
    if hasObstacle
        if onlyJoint6
            set(h_d6, 'XData', t(1:i), 'YData', dist_6(1:i));
            if hasObstacle2
                set(h_d6_obs2, 'XData', t(1:i), 'YData', dist2_6(1:i));
            end
        else
            set(h_d3, 'XData', t(1:i), 'YData', dist_3(1:i));
            set(h_d4, 'XData', t(1:i), 'YData', dist_4(1:i));
            set(h_d5, 'XData', t(1:i), 'YData', dist_5(1:i));
            set(h_d6, 'XData', t(1:i), 'YData', dist_6(1:i));
            if hasObstacle2
                set(h_d3_obs2, 'XData', t(1:i), 'YData', dist2_3(1:i));
                set(h_d4_obs2, 'XData', t(1:i), 'YData', dist2_4(1:i));
                set(h_d5_obs2, 'XData', t(1:i), 'YData', dist2_5(1:i));
                set(h_d6_obs2, 'XData', t(1:i), 'YData', dist2_6(1:i));
            end
        end
        set(h_R1, 'XData', t(1:i), 'YData', R1_val(1:i));
        set(h_R2, 'XData', t(1:i), 'YData', R2_val(1:i));
    end
    
    % --- 9. Update Joint Angle vs Limits Plot ---
    for j_idx = 1:6
        set(h_joint_lines(j_idx), 'XData', t(1:i), 'YData', theta_data(j_idx, 1:i));
    end

    
    % FORCE SCREEN TO UPDATE PLOT IMMEDIATELY
    drawnow;
    %LƯU FIG THEO MỐC THỜI GIAN
    if enable_snapshot_logging
        t_now = t(i); % Lấy mốc thời gian thực tế từ cột thời gian của file log
        
        % Nếu là bước đầu tiên (t=0) hoặc khoảng cách từ lần lưu trước đã đủ 0.5s
        % Sử dụng (snapshot_interval - 1e-4) để bù trừ sai số dấu phẩy động của ma trận
        if i == 1 || (t_now - last_saved_time) >= (snapshot_interval - 1e-4)
            
            % Tạo thư mục riêng để lưu các file fig theo mốc 0.5s
            log_time_folder = fullfile(folder, 'saved_figs_log_time_0.25s');
            if ~exist(log_time_folder, 'dir'), mkdir(log_time_folder); end
            figFolder_time = fullfile(log_time_folder, [baseLogName, '_figs']);
            if ~exist(figFolder_time, 'dir'), mkdir(figFolder_time); end
            t_str = sprintf('%.2f', t_now);
            
            % 1. Lưu Figure 3 (Front View) tại mốc thời gian này
            fig4_name = fullfile(figFolder_time, sprintf('%s_3DView_t_%s.fig', baseLogName, t_str));
            savefig(fig4, fig4_name);
            
            % % 2. Lưu Figure 6 (Joint Trajectories) tại mốc thời gian này
            % fig6_name = fullfile(log_time_folder, sprintf('%s_JointTrajectories_t_%s.fig', baseLogName, t_str));
            % savefig(fig6, fig6_name);
            
            fprintf('--> [Log Time Logged] Đã lưu fig3 & fig6 tại mốc t = %s [s]\n', t_str);
            
            % Cập nhật lại mốc thời gian vừa lưu thành công
            last_saved_time = t_now;
        end
    end
    %% ===== CAPTURE FRAME AND ASSEMBLE 3X3 GRID VIDEO =====
    % f1 = getframe(fig1); 
    % f2 = getframe(fig2); 
    f3 = getframe(fig3);  
    f4 = getframe(fig4);  
    f5 = getframe(fig5);  f6 = getframe(fig6);  
    f7 = getframe(fig7);  f8 = getframe(fig8);  f9 = getframe(fig9); 
    % 
    % img1 = imresize(f1.cdata, [480 640]);  
    % img2 = imresize(f2.cdata, [480 640]);  
    img3 = imresize(f3.cdata, [480 640]); if size(img3,3)==1, img3 = repmat(img3, [1 1 3]); end
    img4 = imresize(f4.cdata, [480 640]); if size(img4,3)==1, img4 = repmat(img4, [1 1 3]); end
    img5 = imresize(f5.cdata, [480 640]); if size(img5,3)==1, img5 = repmat(img5, [1 1 3]); end
    img6 = imresize(f6.cdata, [480 640]); if size(img6,3)==1, img6 = repmat(img6, [1 1 3]); end
    img7 = imresize(f7.cdata, [480 640]); if size(img7,3)==1, img7 = repmat(img7, [1 1 3]); end
    img8 = imresize(f8.cdata, [480 640]); if size(img8,3)==1, img8 = repmat(img8, [1 1 3]); end
    img9 = imresize(f9.cdata, [480 640]); if size(img9,3)==1, img9 = repmat(img9, [1 1 3]); end
    img_blank = ones(480, 640, 3, 'uint8') * 255;
    % Assemble combined 3x3 grid matrix
    % combined_matrix = [img1, img2, img3;
    %                    img4, img5, img6;
    %                    img7, img8, img9];
    combined_matrix = [img3,img4, img5;
                       img6,img7, img8;
                       img9,img_blank,img_blank];
                   
    writeVideo(vout, combined_matrix);
end
close(vout);
disp(['-->  Combined grid video exported successfully to: ', video_path]);

%% ================= AUTOMATICALLY SAVE PLOTS AS .FIG FILES =================
% disp('Saving all plots as original .fig files...');
% % savefig(fig1, fullfile(figFolder, [baseLogName, '_1_Top_View.fig']));
% % savefig(fig2, fullfile(figFolder, [baseLogName, '_2_Left_View.fig']));
% savefig(fig3, fullfile(figFolder, [baseLogName, '_3_Front_View.fig']));
% % savefig(fig4, fullfile(figFolder, [baseLogName, '_4_3D_View.fig']));
% savefig(fig5, fullfile(figFolder, [baseLogName, '_5_Velocity_Profiles.fig']));
% savefig(fig6, fullfile(figFolder, [baseLogName, '_6_Joint_Trajectories.fig']));
% savefig(fig7, fullfile(figFolder, [baseLogName, '_7_Tracking_Errors.fig']));
% savefig(fig8, fullfile(figFolder, [baseLogName, '_8_Joint_Obstacle_Distances.fig']));
% savefig(fig9, fullfile(figFolder, [baseLogName, '_9_Joint_Limits.fig'])); 
% disp(['All .fig files saved to separate directory: ', figFolder]);
disp('Saving all plots as MATLAB figure files...');

savefig(fig3, fullfile(figFolder, [baseLogName, '_Front_View.fig']));
savefig(fig4, fullfile(figFolder, [baseLogName, '_4_3D_View.fig']));
savefig(fig5, fullfile(figFolder, [baseLogName, '_Velocity_Profiles.fig']));
savefig(fig6, fullfile(figFolder, [baseLogName, '_Joint_Trajectories.fig']));
savefig(fig7, fullfile(figFolder, [baseLogName, '_Tracking_Errors.fig']));
savefig(fig8, fullfile(figFolder, [baseLogName, '_Joint_Obstacle_Distances.fig']));
savefig(fig9, fullfile(figFolder, [baseLogName, '_Joint_Limits.fig']));

disp(['All FIG files saved to: ', figFolder]);
disp('=== POST-PROCESSING AND VISUALIZATION COMPLETE ===');