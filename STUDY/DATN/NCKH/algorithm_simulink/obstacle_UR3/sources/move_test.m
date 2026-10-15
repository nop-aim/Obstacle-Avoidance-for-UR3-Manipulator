% move_test.m
clc; clear; close all;
folder = fileparts(which('move_test.m')); 
addpath(genpath(fullfile(folder, 'library')));

initialPosition = [0.2; 0.1; 0.1];
myWorld = EnvironmentUR3(initialPosition);

ur3 = myWorld.robot;  % chính là UR3Wrapper()

% %% === 2) Tạo quỹ đạo Descartes (đường tròn nhỏ) ===
% N = 100;
% center = [0.45; 0.00; 0.10];   % tâm quỹ đạo tròn
% R = 0.05;                      % bán kính 
% theta = linspace(0, 2*pi, N);
% 
% cartTraj = [ center(1) + R*cos(theta) ; ...
%              center(2) + R*sin(theta) ; ...
%              repmat(center(3), 1, N) ];          % 3xN
% 
% % Tốc độ Descartes (xấp xỉ) để vẽ quiver3:
% dCart = [ diff(cartTraj,1,2), zeros(3,1) ];      % 3xN, đơn vị ~ m/step
% cartSpeed = dCart;                                % chỉ để minh hoạ mũi tên
% 
% %% === 3) Giải IK để ra quỹ đạo khớp (6xN) ===
% q0 = zeros(6,1);           % khởi đoán (home)
% showBar = false;
% % Không ràng buộc orientation để dễ hội tụ:
% qTraj = ur3.computeInverseKinematics(cartTraj, q0, showBar);   % 6xN
% 
% %% === 4) Animate robot chạy theo quỹ đạo ===
% cartTarget = cartTraj(:,end);   % mục tiêu để vẽ dấu đỏ
% ur3.animateTrajectory(qTraj, cartTraj, cartSpeed, cartTarget, 'UR3 – Circle Demo');


%% === 2) Quỹ đạo thẳng đi qua 2 điểm trong Descartes ===
N = 100;                            % số mẫu trên đường
P1 = [0.45; -0.10; 0.10];           % Điểm đầu (x;y;z) [m]
P2 = [0.55;  0.05; 0.20];           % Điểm cuối (x;y;z) [m]

s = linspace(0,1,N);                % tham số nội suy
cartTraj = P1 .* (1 - s) + P2 .* s; % 3xN: nội suy tuyến tính

% Vận tốc Descartes xấp xỉ để vẽ quiver3:
dCart = [diff(cartTraj,1,2), zeros(3,1)];
cartSpeed = dCart;

%% === 3') Giải IK và animate ===
q0 = zeros(6,1);
showBar = true;
qTraj = ur3.computeInverseKinematics(cartTraj, q0, showBar);

cartTarget = P2;
ur3.animateTrajectory(qTraj, cartTraj, cartSpeed, cartTarget, 'UR3 – Line P1-P2');

 
%% Programmed by Le Nam Chau
%% Date: 2025, Feb.27th
close all
figure(1); hold on; grid on;

% Time axis (sample index)
t = 1:size(cartTraj,2);

% Desired TCP (x-axis)
x_des = cartTraj(1,:);

% Actual TCP (x-axis from FK)
x_act = zeros(1,length(t));
for k = 1:length(t)
    x_act(k) = ur3.forwardKinematics(qTraj(:,k));
end
x_act = x_act(1,:);   % lấy trục x

% Plot
plot(t, x_act, 'k', 'LineWidth', 1);      % Actual
plot(t, x_des, 'r', 'LineWidth', 1);      % Desired

xlabel('Time (samples)');
ylabel('TCP position x (m)');
legend('TCP actual','TCP desired');
title('TCP Tracking in x-direction');


