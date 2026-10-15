clc; clear; close all;
%% ================= CONFIGURATION =================
simu = true;               % Set true to plot simulation (.mat), false to plot real robot (.csv)
step_render = 5;            % Frame skip for video
snapshot_interval = 0.5;    % (s) interval between snapshots
enable_snapshot   = false;  % Set true to save snapshots of Figure 6
delta_t = 25e-3;
colors_main = lines(7);
%% ================= PATH SETUP =================
if simu
    filename = 'logs_2\full_1obs_movetwo_2.mat'; 
else
    filename = 'logs_2jck\target_obs_move_3.csv'; 
end

[folder, baseName, ext] = fileparts(filename);
videoFolder = fullfile(folder, 'video');
if ~exist(videoFolder, 'dir'), mkdir(videoFolder); end
figFolder = fullfile(videoFolder, [baseName '_figs2']);
if ~exist(figFolder, 'dir'), mkdir(figFolder); end

fprintf('>>> Loading log: %s (simu=%d)\n', filename, simu);

%% ===== STYLE (giống paper + REE) =====
FONT_NAME = 'Helvetica';
fontAxis  = 12;
fontLabel = 13;
fontTitle = 14;
fontLeg   = 11;
lineW     = 2.5;
lineW_sub = 2.0;
lineW_ref = 2.5;
gridAlpha = 0.30;
figSize   = [100 100 700 520];

% Màu sắc
C_JCK = [0.08 0.35 0.73];   % xanh đặc — JCK
C_SP  = [0.72 0.22 0.12];   % đỏ gạch — Setpoint
C_R1  = [0.8  0.0  0.8 ];   % magenta  — R1 danger
C_R2  = [0.0  0.0  0.0 ];   % đen đứt  — R2 influence
C_ACT = [0.46 0.46 0.46];   % xám đậm  — Actual speed / response
C_TARGET = [0.47, 0.67, 0.19];   % Green (Màu xanh lá học thuật)
C_TORQUE = [
    0.00 0.45 0.74;  % Joint 1: Xanh dương
    0.85 0.33 0.10;  % Joint 2: Cam gạch
    0.93 0.69 0.13;  % Joint 3: Vàng nghệ
    0.49 0.18 0.56;  % Joint 4: Tím
    0.46 0.67 0.18;  % Joint 5: Xanh lá mạ
    0.30 0.75 0.93   % Joint 6: Xanh da trời
];
clr = lines(7);              % palette cho joints
clr3 = clr(1,:);  clr4 = clr(2,:);
clr5 = clr(3,:);  clr6 = clr(5,:);
clr2 = clr(4,:);

%% ================= READ LOG DATA =================
if simu
    % Load from MAT
    loaded  = load(filename);
    logData = loaded.logData;
    t       = logData.time;
    N       = length(t);
    
    x_tcp = logData.position_tcp(1,:);
    y_tcp = logData.position_tcp(2,:);
    z_tcp = logData.position_tcp(3,:);
    % 2. Auto-computed TCP Velocity
    vx = [0, diff(x_tcp) / delta_t];
    vy = [0, diff(y_tcp) / delta_t];
    vz = [0, diff(z_tcp) / delta_t];
    v_norm = sqrt(vx.^2  +vy.^2 + vz.^2);
    x_sp = logData.attractor(1,:);
    z_sp = logData.attractor(3,:);
    
    err_x = x_tcp - x_sp;
    err_z = z_tcp - z_sp;
    
    theta_data = logData.theta;
    theta_min  = logData.theta_min;
    theta_max  = logData.theta_max;
    
    ctrl_cmd = logData.control_input;  % 6 x N
    ctrl_act = [];
    
    hasObs = isfield(logData,'obstacle_position') && ~isempty(logData.obstacle_position);
    if hasObs
        ox1 = logData.obstacle_position(1,:);
        oz1 = logData.obstacle_position(3,:);
        R1v = logData.R1;
        R2v = logData.R2;
        dist6 = logData.dist_to_obs{6};
        
        onlyJ6 = true;
        if isfield(logData,'dist_to_obs') && length(logData.dist_to_obs) >= 3 && ~isempty(logData.dist_to_obs{3})
            onlyJ6 = false;
            dist2 = logData.dist_to_obs{2};
            dist3 = logData.dist_to_obs{3};
            dist4 = logData.dist_to_obs{4};
            dist5 = logData.dist_to_obs{5};
        end
    end
else
    % Load from CSV
    opts = detectImportOptions(filename);
    opts.VariableNamingRule = 'preserve';
    data = readtable(filename, opts);
    
    t = data.timestamp - data.timestamp(1);
    N = length(t);
    
    x_tcp = data.actual_TCP_pose_0;
    z_tcp = data.actual_TCP_pose_2;
    
    x_sp = data.target_x;
    z_sp = data.target_z;
    
    err_x = data.err_x;
    err_z = data.err_z;
    
    theta_data = [data.q1, data.q2, data.q3, data.q4, data.q5, data.q6]'; % 6 x N
    theta_min  = -1.5 * pi * ones(6, 1);
    theta_max  =  1.5 * pi * ones(6, 1);
    
    ctrl_cmd = [data.q_dot_cmd1, data.q_dot_cmd2, data.q_dot_cmd3, data.q_dot_cmd4, data.q_dot_cmd5, data.q_dot_cmd6]';
    ctrl_act = [data.q_dot_act1, data.q_dot_act2, data.q_dot_act3, data.q_dot_act4, data.q_dot_act5, data.q_dot_act6]';
    torque_act = [data.tau_act1, data.tau_act2, data.tau_act3, data.tau_act4, data.tau_act5, data.tau_act6]';

    hasObs = ismember('obs1_x', data.Properties.VariableNames) && any(data.obs1_x ~= 0);
    if hasObs
        ox1 = data.obs1_x;
        oz1 = data.obs1_z;
        R1v = data.R1;
        R2v = data.R2;
        dist6 = data.dist_joint6;
        
        onlyJ6 = true;
        if ismember('dist_joint3', data.Properties.VariableNames) && any(data.dist_joint3 ~= 0)
            onlyJ6 = false;
            dist3 = data.dist_joint3;
            dist4 = data.dist_joint4;
            dist5 = data.dist_joint5;
        end
    end
end
theta_vec = linspace(0,2*pi,100);

%% ===================================================================
%  FIGURE 1 — X và Z POSITION TRACKING  (Setpoint vs JCK)
%% ===================================================================
fig1 = figure('Color','w','Name','Position Tracking','NumberTitle','off');

subplot(2,1,1); hold on; grid on; box on;
h1_sp  = plot(t, x_sp,  ':', 'Color', C_SP,  'LineWidth', lineW_ref);
h1_obs = plot(t,ox1,'k--',  'LineWidth', lineW_ref);
h1_jck = plot(t, x_tcp, '-', 'Color', C_JCK, 'LineWidth', lineW);
ylabel('\bf X (m)','Interpreter','tex','FontName',FONT_NAME,'FontSize',fontLabel,'FontWeight','bold');
title('\bf Position Tracking','Interpreter','tex','FontName',FONT_NAME,'FontSize',fontTitle,'FontWeight','bold');
lgd1a = legend([h1_jck,h1_obs, h1_sp],{'JCK','Obs','SP'},'Location','best');
set(lgd1a,'FontName',FONT_NAME,'FontSize',fontLeg);
xlim([0 t(end)]);
set(gca,'FontName',FONT_NAME,'FontSize',fontAxis,'FontWeight','bold','LineWidth',1.2,'GridAlpha',gridAlpha,'GridLineStyle','--');

subplot(2,1,2); hold on; grid on; box on;
h2_sp  = plot(t, z_sp,  ':', 'Color', C_SP,  'LineWidth', lineW_ref);
h2_obs = plot(t,oz1,'k--',  'LineWidth', lineW_ref);
h2_jck = plot(t, z_tcp, '-', 'Color', C_JCK, 'LineWidth', lineW);
xlabel('\bf Time (s)','Interpreter','tex','FontName',FONT_NAME,'FontSize',fontLabel,'FontWeight','bold');
ylabel('\bf Z (m)','Interpreter','tex','FontName',FONT_NAME,'FontSize',fontLabel,'FontWeight','bold');
lgd1b = legend([h2_jck,h2_obs, h2_sp,],{'JCK','Obs','SP'},'Location','best');
set(lgd1b,'FontName',FONT_NAME,'FontSize',fontLeg);
xlim([0 t(end)]);
set(gca,'FontName',FONT_NAME,'FontSize',fontAxis,'FontWeight','bold','LineWidth',1.2,'GridAlpha',gridAlpha,'GridLineStyle','--');

%% ===================================================================
%  FIGURE 2 — X và Z TRACKING ERRORS
%% ===================================================================
fig2 = figure('Color','w','Name','Velocity Profile','Visible','on','NumberTitle','off');
subplot(2,1,1); hold on; grid on; box on;
h_v_norm = plot(t,v_norm, 'LineWidth', lineW, 'Color', colors_main(1,:));
ylabel('\bf |V| (m/s)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold'); 
title('\bf TCP Resultant Velocity', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontTitle, 'FontWeight', 'bold');
set(gca, 'FontName', FONT_NAME, 'FontSize', fontAxis, 'FontWeight', 'bold', 'LineWidth', 1.2, 'GridAlpha', gridAlpha); xlim([0, t(end)]);
subplot(2,1,2); hold on; grid on; box on;
h_vx = plot(t, vx, 'r', 'LineWidth', lineW_sub);
h_vy = plot(t, vy, 'g', 'LineWidth', lineW_sub);
h_vz = plot(t, vz, 'b', 'LineWidth', lineW_sub);
xlabel('\bf Time (s)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold'); 
ylabel('\bf Velocity Components (m/s)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold');
lgd5 = legend('V_x','V_y','V_z', 'Location', 'best'); 
set(lgd5, 'FontName', FONT_NAME, 'FontSize', fontLeg);
set(gca, 'FontName', FONT_NAME, 'FontSize', fontAxis, 'FontWeight', 'bold', 'LineWidth', 1.2, 'GridAlpha', gridAlpha); xlim([0, t(end)]);

%% ===================================================================
%  FIGURE 3 — JOINT-OBSTACLE DISTANCES  (R1 / R2 reference lines)
%% ===================================================================
fig3 = figure('Color','w','Name','Joint Obstacle Distances','NumberTitle','off');

if hasObs
    if onlyJ6
        subplot(1,1,1); hold on; grid on; box on;
        hd6 = plot(t, dist6, '-', 'Color', clr6, 'LineWidth', lineW);
        hR1 = plot(t, R1v,   '-', 'Color', C_R1, 'LineWidth', lineW_ref);
        hR2 = plot(t, R2v,   '--','Color', C_R2, 'LineWidth', lineW_ref);
        xlabel('\bf Time (s)','Interpreter','tex','FontName',FONT_NAME,'FontSize',fontLabel,'FontWeight','bold');
        ylabel('\bf Distance (m)','Interpreter','tex','FontName',FONT_NAME,'FontSize',fontLabel,'FontWeight','bold');
        title('\bf Obstacle Distance','Interpreter','tex','FontName',FONT_NAME,'FontSize',fontTitle,'FontWeight','bold');
        lgd3 = legend([hd6,hR1,hR2],{'Joint 6','R_1 Danger','R_2 Influence'},'Location','best');
        set(lgd3,'FontName',FONT_NAME,'FontSize',fontLeg);
        xlim([0 t(end)]); ylim([0 max([max(dist6) max(R2v)])*1.15]);
        set(gca,'FontName',FONT_NAME,'FontSize',fontAxis,'FontWeight','bold','LineWidth',1.2,'GridAlpha',gridAlpha,'GridLineStyle','--');
    else
        subplot(2,1,1); hold on; grid on; box on;
        hd2=  plot(t, dist2, '-', 'Color', clr2, 'LineWidth', lineW);
        hd3 = plot(t, dist3, '-', 'Color', clr3, 'LineWidth', lineW);
        hd4 = plot(t, dist4, '-', 'Color', clr4, 'LineWidth', lineW);
        hR1a= plot(t, R1v,   '-', 'Color', C_R1, 'LineWidth', lineW_ref);
        hR2a= plot(t, R2v,  '--', 'Color', C_R2, 'LineWidth', lineW_ref);
        ylabel('\bf Distance (m)','Interpreter','tex','FontName',FONT_NAME,'FontSize',fontLabel,'FontWeight','bold');
        title('\bf Obstacle Distance','Interpreter','tex','FontName',FONT_NAME,'FontSize',fontTitle,'FontWeight','bold');
        lgd3a = legend([hd2,hd3,hd4,hR1a,hR2a],{'Joint 2','Joint 3','Joint 4','R_1','R_2'},'Location','best','NumColumns',2);
        set(lgd3a,'FontName',FONT_NAME,'FontSize',fontLeg);
        xlim([0 t(end)]); ylim([0 max([max(dist3) max(R2v)])*1.15]);
        set(gca,'FontName',FONT_NAME,'FontSize',fontAxis,'FontWeight','bold','LineWidth',1.2,'GridAlpha',gridAlpha,'GridLineStyle','--');

        subplot(2,1,2); hold on; grid on; box on;
        hd5 = plot(t, dist5, '-', 'Color', clr5, 'LineWidth', lineW);
        hd6 = plot(t, dist6, '-', 'Color', clr6, 'LineWidth', lineW);
        hR1b= plot(t, R1v,   '-', 'Color', C_R1, 'LineWidth', lineW_ref);
        hR2b= plot(t, R2v,  '--', 'Color', C_R2, 'LineWidth', lineW_ref);
        xlabel('\bf Time (s)','Interpreter','tex','FontName',FONT_NAME,'FontSize',fontLabel,'FontWeight','bold');
        ylabel('\bf Distance (m)','Interpreter','tex','FontName',FONT_NAME,'FontSize',fontLabel,'FontWeight','bold');
        lgd3b = legend([hd5,hd6,hR1b,hR2b],{'Joint 5','Joint 6','R_1','R_2'},'Location','best','NumColumns',2);
        set(lgd3b,'FontName',FONT_NAME,'FontSize',fontLeg);
        xlim([0 t(end)]); ylim([0 max([max(dist5) max(R2v)])*1.15]);
        set(gca,'FontName',FONT_NAME,'FontSize',fontAxis,'FontWeight','bold','LineWidth',1.2,'GridAlpha',gridAlpha,'GridLineStyle','--');
    end
else
    axis off;
    text(0.5,0.5,'No obstacle data','Units','normalized','HorizontalAlignment','center','FontSize',14);
    title('\bf No Obstacle Data','FontName',FONT_NAME,'FontSize',fontTitle);
end

%% ===================================================================
%  FIGURE 4 — CONTROL INPUT  (Cmd vs Act Joint Velocity)
%% ===================================================================
fig4 = figure('Color','w','Name','Control Input','NumberTitle','off');

% Auto-detect active commanded joints
active_ctrl = [];
for jj = 1:6
    if max(abs(ctrl_cmd(jj,:))) > 1e-3
        active_ctrl(end+1) = jj; %#ok<AGROW>
    end
end
if isempty(active_ctrl), active_ctrl = 2:4; end
nC = length(active_ctrl);

for k = 1:nC
    jj = active_ctrl(k);
    subplot(nC, 1, k); hold on; grid on; box on;
    h_cmd = plot(t, ctrl_cmd(jj,:), '-', 'Color', C_JCK, 'LineWidth', lineW);
    if ~simu && ~isempty(ctrl_act)
        h_act = plot(t, ctrl_act(jj,:), ':', 'Color', C_ACT, 'LineWidth', lineW - 0.2);
    end
    ylabel(sprintf('\\bf Joint %d (rad/s)', jj),'Interpreter','tex','FontName',FONT_NAME,'FontSize',fontLabel-1,'FontWeight','bold');
    xlim([0 t(end)]);
    set(gca,'FontName',FONT_NAME,'FontSize',fontAxis,'FontWeight','bold','LineWidth',1.2,'GridAlpha',gridAlpha,'GridLineStyle','--');
    if k == 1
        title('\bf Joint Velocity Commands','Interpreter','tex','FontName',FONT_NAME,'FontSize',fontTitle,'FontWeight','bold');
        if ~simu && ~isempty(ctrl_act)
            lgdVel = legend([h_cmd, h_act], {'Cmd','Act'}, 'Location', 'northeast');
            set(lgdVel, 'FontName', FONT_NAME, 'FontSize', fontLeg - 1);
        end
    end
    if k == nC
        xlabel('\bf Time (s)','Interpreter','tex','FontName',FONT_NAME,'FontSize',fontLabel,'FontWeight','bold');
    end
end

%% ===================================================================
%  FIGURE 5 — JOINT ANGLES vs LIMITS  (MAX / MIN)
%% ===================================================================
active_joints = 1:6; % Cố định 6 joint luôn
nAct = length(active_joints);

fig5 = figure('Color','w','Name','Joint Angles','NumberTitle','off');
% Thay đổi kích thước figure cho phù hợp với cấu trúc lưới 3x2 (thường rộng hơn)
set(fig5,'Position',[figSize(1) figSize(2) max(800, figSize(3)) max(600, figSize(4))]);

for k = 1:nAct
    jj = active_joints(k);
    
    % Thay đổi cấu trúc subplot thành 3 dòng, 2 cột
    subplot(3, 2, k); hold on; grid on; box on;
    
    % Vẽ dữ liệu góc khớp và các đường giới hạn Max/Min
    h_jck_j = plot(t, theta_data(jj,:), '-', 'Color', C_JCK, 'LineWidth', lineW);
    h_max_j = plot([0 t(end)], [theta_max(jj) theta_max(jj)], '--', 'Color', [0.85 0 0.85], 'LineWidth', lineW_ref);
    h_min_j = plot([0 t(end)], [theta_min(jj) theta_min(jj)], '--', 'Color', [0 0 0],       'LineWidth', lineW_ref);
    
    % Tiêu đề riêng cho từng subplot (Joint 1, Joint 2,...)
    title(sprintf('\\bf Joint %d', jj), 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold');
    
    % Giới hạn trục
    xlim([0 t(end)]);
    ylim([theta_min(jj)-0.25, theta_max(jj)+0.25]);
    
    % Tạo Legend gọn gàng ở góc đông bắc cho từng ô
    lgdJ = legend([h_jck_j, h_max_j, h_min_j], {'Angle','Max','Min'}, 'Location','northeast','NumColumns',3);
    set(lgdJ,'FontName',FONT_NAME,'FontSize',fontLeg-2); % Giảm size legend một chút cho vừa ô lưới
    
    % Cấu hình trục tọa độ
    set(gca,'FontName',FONT_NAME,'FontSize',fontAxis-1,'FontWeight','bold','LineWidth',1.2,'GridAlpha',gridAlpha,'GridLineStyle','--');
    
    % Chỉ hiện ylabel ở cột bên trái (các ô lẻ: 1, 3, 5)
    if mod(k, 2) == 1
        ylabel('\bf \theta (rad)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold');
    end
    
    % Chỉ hiện xlabel ở hàng cuối cùng (các ô: 5, 6)
    if k >= 5
        xlabel('\bf Time (s)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold');
    end
end

% Tiêu đề tổng lớn ở trên cùng của figure
sgtitle('\bf Joint Angle Variation', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontTitle, 'FontWeight', 'bold');

%% ===================================================================
%  FIGURE 6 — FRONT VIEW X-Z  (dynamic — animated + static final)
%% ===================================================================
fig6 = figure('Color','w','Name','XZ View','NumberTitle','off');
hold on; grid on; box on; axis equal;

h_start   = plot(x_tcp(1),   z_tcp(1),   'ko','MarkerFaceColor','k','MarkerSize',8,'DisplayName','Start');
% h_end     = plot(x_tcp(end), z_tcp(end), 'r^','MarkerFaceColor','r','MarkerSize',8,'DisplayName','End');
h_sp_traj = plot(NaN,NaN,'g*','MarkerFaceColor','none','MarkerSize',8,'LineWidth',2.0,'DisplayName','Target');
h_jck_xz  = plot(NaN, NaN,   '-', 'Color', C_JCK, 'LineWidth', lineW,     'DisplayName','Trajectory JCK');
if hasObs
    h_obs_xz = fill(NaN, NaN, [0.75 0.75 0.75],'FaceAlpha',0.45,'EdgeColor',[0.3 0.3 0.3],'DisplayName','Obstacle');
end
xlabel('\bf X (m)','Interpreter','tex','FontName',FONT_NAME,'FontSize',fontLabel,'FontWeight','bold');
ylabel('\bf Z (m)','Interpreter','tex','FontName',FONT_NAME,'FontSize',fontLabel,'FontWeight','bold');
h_title6 = title('\bf XZ View  |  t = 0.00 s','Interpreter','tex','FontName',FONT_NAME,'FontSize',fontTitle,'FontWeight','bold');
if hasObs
    lgd6 = legend([h_jck_xz, h_sp_traj, h_obs_xz, h_start],'Location','best');
else
    lgd6 = legend([h_jck_xz, h_sp_traj, h_start],'Location','best');
end
set(lgd6,'FontName',FONT_NAME,'FontSize',fontLeg);
set(gca,'FontName',FONT_NAME,'FontSize',fontAxis,'FontWeight','bold','LineWidth',1.2,'GridAlpha',gridAlpha,'GridLineStyle','--');

%% ===================================================================
%  FIGURE 7 — Supplementary: Calculation Time & Control Cycle Period (Real Only)
%% ===================================================================
if ~simu
    fig7 = figure('Color','w','Name','Loop Timing','NumberTitle','off');
    set(fig7,'Position',[figSize(1) figSize(2) figSize(3) 520]);

    % Calculation time
    subplot(2,1,1); hold on; grid on; box on;
    t_calc_ms = data.t_calc;
    plot(t, t_calc_ms, '-', 'Color', [0.85 0.33 0.1], 'LineWidth', 1.3);
    y_mean = mean(t_calc_ms);
    plot([0 t(end)], [y_mean y_mean], '--', 'Color', [0 0 0], 'LineWidth', 1.2);
    ylabel('\bf Calc Time (ms)','Interpreter','tex','FontName',FONT_NAME,'FontSize',fontLabel,'FontWeight','bold');
    title('\bf Controller Calculation Speed','Interpreter','tex','FontName',FONT_NAME,'FontSize',fontTitle,'FontWeight','bold');
    text(t(end)*0.65, max(t_calc_ms)*0.85, sprintf('Mean = %.2f ms\nMax = %.2f ms', y_mean, max(t_calc_ms)), ...
        'FontName', FONT_NAME, 'FontSize', fontLeg, 'FontWeight', 'bold', 'EdgeColor', 'k', 'BackgroundColor', 'w');
    xlim([0 t(end)]);
    set(gca,'FontName',FONT_NAME,'FontSize',fontAxis,'FontWeight','bold','LineWidth',1.2,'GridAlpha',gridAlpha,'GridLineStyle','--');


    subplot(2,1,2); hold on; grid on; box on;
    loop_ms_data = data.loop_ms; 
    plot(t, loop_ms_data, '-', 'Color', [0.46 0.67 0.18], 'LineWidth', 1.3);
    plot([t(1) t(end)], [20.0 20.0], 'r--', 'LineWidth', 1.2); 
    ylabel('\bf Loop Time (ms)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold');
    xlabel('\bf Time (s)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold');
    title('\bf Total Python Loop Execution Time (loop\_ms)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontTitle-1, 'FontWeight', 'bold');
    xlim([t(1) t(end)]);
    ylim([0 max([max(loop_ms_data)*1.2, 35.0])]);
    legend({'Actual loop\_ms', 'UR Limit (20ms)'}, 'Location', 'best');
    set(gca, 'FontName', FONT_NAME, 'FontSize', fontAxis, 'FontWeight', 'bold', 'LineWidth', 1.2, 'GridAlpha', gridAlpha, 'GridLineStyle', '--');
end

%% ===================================================================
%  FIGURE 8 — Supplementary: Estimated Velocities (Real Only)
%% ===================================================================
if ~simu
    fig8 = figure('Color','w','Name','Estimated Velocities','NumberTitle','off');
    set(fig8,'Position',[figSize(1) figSize(2) figSize(3) 520]);

    % Target
    subplot(2,1,1); hold on; grid on; box on;
    plot(t, data.target_vx, '-', 'Color', [0.08 0.35 0.73], 'LineWidth', lineW, 'DisplayName', 'V_X');
    plot(t, data.target_vz, '--', 'Color', [0.72 0.22 0.12], 'LineWidth', lineW, 'DisplayName', 'V_Z');
    ylabel('\bf Velocity (m/s)','Interpreter','tex','FontName',FONT_NAME,'FontSize',fontLabel,'FontWeight','bold');
    title('\bf Target Estimated Velocity','Interpreter','tex','FontName',FONT_NAME,'FontSize',fontTitle,'FontWeight','bold');
    legend('show', 'Location', 'best');
    xlim([0 t(end)]);
    set(gca,'FontName',FONT_NAME,'FontSize',fontAxis,'FontWeight','bold','LineWidth',1.2,'GridAlpha',gridAlpha,'GridLineStyle','--');

    % Obstacle 1
    subplot(2,1,2); hold on; grid on; box on;
    if hasObs
        plot(t, data.obs1_vx, '-', 'Color', [0.08 0.35 0.73], 'LineWidth', lineW, 'DisplayName', 'V_X');
        plot(t, data.obs1_vz, '--', 'Color', [0.72 0.22 0.12], 'LineWidth', lineW, 'DisplayName', 'V_Z');
        ylabel('\bf Velocity (m/s)','Interpreter','tex','FontName',FONT_NAME,'FontSize',fontLabel,'FontWeight','bold');
        title('\bf Obstacle Estimated Velocity','Interpreter','tex','FontName',FONT_NAME,'FontSize',fontTitle,'FontWeight','bold');
        legend('show', 'Location', 'best');
    else
        text(0.5, 0.5, 'No obstacle velocity data', 'Units', 'normalized', 'HorizontalAlignment', 'center', 'FontSize', 12);
    end
    xlabel('\bf Time (s)','Interpreter','tex','FontName',FONT_NAME,'FontSize',fontLabel,'FontWeight','bold');
    xlim([0 t(end)]);
    set(gca,'FontName',FONT_NAME,'FontSize',fontAxis,'FontWeight','bold','LineWidth',1.2,'GridAlpha',gridAlpha,'GridLineStyle','--');
end
%% ===================================================================
%  FIGURE 9 — Actual Joint Torques (Real Only)
%% ===================================================================
if ~simu
    fig9 = figure('Color','w','Name','Actual Joint Torques','NumberTitle','off');
    set(fig9, 'Position', [figSize(1) figSize(2) figSize(3) 680]); % Tăng chiều cao để lưới 3x2 không đè chữ
    
    for i = 1:6
        subplot(3, 2, i); hold on; grid on; box on;
        
        % Vẽ dữ liệu torque thực tế
        plot(t, torque_act(i, :), 'Color', C_TORQUE(i, :), 'LineWidth', lineW_sub);
        
        % Cấu hình hiển thị chuẩn kỹ thuật giống các hình trên
        ylabel(sprintf('\\bf Joint %d (Nm)', i), 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel-1, 'FontWeight', 'bold');
        xlim([0 t(end)]);
        set(gca, 'FontName', FONT_NAME, 'FontSize', fontAxis, 'FontWeight', 'bold', 'LineWidth', 1.2, 'GridAlpha', gridAlpha, 'GridLineStyle', '--');
        
        % Tách tiêu đề phụ theo nhóm khớp để cấu trúc rõ ràng trong luận văn
        if i <= 3
            title(sprintf('\\bf Base/Body Joint %d', i), 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLeg, 'Color', [0.2 0.2 0.2]);
        else
            title(sprintf('\\bf Wrist Joint %d', i), 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLeg, 'Color', [0.2 0.2 0.2]);
        end
        
        % Chỉ bật nhãn trục X cho 2 đồ thị dưới cùng
        if i == 5 || i == 6
            xlabel('\bf Time (s)', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontLabel, 'FontWeight', 'bold');
        end
    end
    
    % Tiêu đề tổng trên cùng của Figure
    sgtitle('\bf Real-time Joint Torques Profile', 'Interpreter', 'tex', 'FontName', FONT_NAME, 'FontSize', fontTitle, 'FontWeight', 'bold');
end

%% ===== CỬA SỔ CHỒNG LÊN (CASCADE) =====
if simu
    figs = [fig1, fig2, fig3, fig4, fig5, fig6];
else
    figs = [fig1, fig2, fig3, fig4, fig5, fig6, fig7, fig8, fig9];
end
% for k = 1:length(figs)
%     set(figs(k),'Position',[figSize(1)+(k-1)*28, figSize(2)+(k-1)*28, figSize(3), figSize(4)]);
% end

%% ===================================================================
%  VIDEO WRITER (ghép 6 hình thành lưới 3x2)
%% ===================================================================
video_path = fullfile(videoFolder, sprintf('metrics_%s.mp4', baseName));
vout = VideoWriter(video_path,'MPEG-4');
vout.FrameRate = 30;
open(vout);
disp('>>> Đang ghi video 6 đồ thị...');

% Capture static figures (Fig 1 to 5) ONCE before the loop to speed up rendering
rsz = [480 640];
imR = cell(1,6);
disp('Capturing static figures...');
for kk = 1:5
    try
        f = getframe(figs(kk));
        im = f.cdata;
    catch
        im = print(figs(kk), '-RGBImage', '-r100');
    end
    im = imresize(im, rsz);
    if size(im,3) < 3, im = repmat(im,[1 1 3]); end
    imR{kk} = im;
end

last_save_t = -inf;

for i = 1:step_render:N
    cur_t = t(i);

    %--- Chỉ cập nhật Fig 6 (Front View) — 5 hình còn lại là tĩnh ---
    set(h_sp_traj, 'XData', x_sp(i), 'YData', z_sp(i));
    set(h_jck_xz,  'XData', x_tcp(1:i), 'YData', z_tcp(1:i));
    set(h_title6,  'String', sprintf('\\bf XZ View  |  t = %.2f s', cur_t));
    if hasObs
        r_now = R1v(i);
        set(h_obs_xz,'XData', ox1(i)+r_now*cos(theta_vec), 'YData', oz1(i)+r_now*sin(theta_vec));
    end
    drawnow;

    % Lưu snapshot theo mốc thời gian
    if enable_snapshot && (i == 1 || (cur_t - last_save_t) >= (snapshot_interval - 1e-4))
        snap_dir = fullfile(folder, sprintf('snapshots_%s', baseName));
        if ~exist(snap_dir,'dir'), mkdir(snap_dir); end
        t_str = sprintf('%.2fs', cur_t);
        savefig(fig6, fullfile(snap_dir, sprintf('FrontView_t%s.fig', t_str)));
        last_save_t = cur_t;
    end

    % Chụp frame của Fig 6 (Front View) duy nhất trong loop
    try
        f6c = getframe(fig6);
        im6 = f6c.cdata;
    catch
        im6 = print(fig6, '-RGBImage', '-r100');
    end
    im6 = imresize(im6, rsz);
    if size(im6,3) < 3, im6 = repmat(im6,[1 1 3]); end
    imR{6} = im6;

    combined = [imR{1}, imR{2}, imR{3};
                imR{4}, imR{5}, imR{6}];
    writeVideo(vout, combined);
end

close(vout);
fprintf('--> Video saved: %s\n', video_path);

%% ===================================================================
%  LƯU TẤT CẢ FIG
%% ===================================================================
disp('Saving .fig files...');
savefig(fig1, fullfile(figFolder, [baseName +'_Position_Tracking.fig']));
savefig(fig2, fullfile(figFolder, [baseName +'_Velocity_Profile.fig']));
savefig(fig3, fullfile(figFolder, [baseName +'_Obstacle_Distances.fig']));
savefig(fig4, fullfile(figFolder, [baseName +'_Control_Input.fig']));
savefig(fig5, fullfile(figFolder, [baseName +'_Joint_Angles.fig']));
savefig(fig6, fullfile(figFolder, [baseName +'_XZ_View.fig']));
if ~simu
    savefig(fig7, fullfile(figFolder, [baseName '_Loop_Timing.fig']));
    savefig(fig8, fullfile(figFolder, [baseName '_Estimated_Velocities.fig']));
    savefig(fig9, fullfile(figFolder, [baseName '_Actual_Joint_Torques.fig']));
end
fprintf('--> All .fig files saved to: %s\n', figFolder);
disp('=== DONE ===');
