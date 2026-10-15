clc; clear; close all;

%% ===== 1. LOAD WORLD & ROBOT =====
folder = fileparts(which('test_pos.m'));
addpath(genpath(fullfile(folder,'enviroment')));

initialPosition = deg2rad([-179.40, -42.14, -132.36, -92.97, 89.72, 0]');
% initialPosition = [0.4; 0; 0.8];
myWorld = World(initialPosition);

robot = myWorld.robot.robot;
%% ===== 2. TEST JOINT CONFIG =====

q = deg2rad([-179.40, -42.14, -132.36, -92.97, 89.72, 0]');


%% ===== 3. FK =====

eeName = robot.BodyNames{end};

T_ee = getTransform(robot, q, eeName);

x = T_ee(1,4);
y = T_ee(2,4);
z = T_ee(3,4);

fprintf('\n=== FK RESULT ===\n');
fprintf('X: %.4f (m)\n', x);
fprintf('Y: %.4f (m)\n', y);
fprintf('Z: %.4f (m)\n', z);

disp('T06 = ');
disp(T_ee);

%% ===== 4. PLOT =====

figure('Color','w');

show(robot, q, ...
    'Frames','on', ...
    'Visuals','on');

hold on;
grid on;
axis equal;

view(135,25);

plot3(x,y,z,...
    'ro',...
    'MarkerSize',12,...
    'LineWidth',3,...
    'MarkerFaceColor','r');

text(x,y,z+0.05,...
    sprintf('EE [%.3f %.3f %.3f]',x,y,z),...
    'FontSize',10,...
    'FontWeight','bold');

title('Robot Test Pose');
xlabel('X');
ylabel('Y');
zlabel('Z');