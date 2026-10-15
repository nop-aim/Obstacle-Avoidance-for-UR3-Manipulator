clc;
clear;
close all;

% =========================
% Test JointKinematics
% =========================

% Góc joint (rad)
q = deg2rad([0 0 0 0 0 0]');

% Gọi hàm
[p, Jp] = JointKinematics(q);

% =========================
% In vị trí từng link
% =========================

fprintf('===== POSITION OF EACH LINK =====\n');

for i = 1:6
    fprintf('Link %d:\n', i);
    fprintf('x = %.4f m\n', p(1,i));
    fprintf('y = %.4f m\n', p(2,i));
    fprintf('z = %.4f m\n\n', p(3,i));
end

% =========================
% Hiển thị Jacobian
% =========================

for i = 1:6
    fprintf('===== Jacobian of Link %d =====\n', i);
    disp(Jp(:,:,i));
end

% =========================
% Plot robot points
% =========================

figure;
hold on;
grid on;
axis equal;

xlabel('X');
ylabel('Y');
zlabel('Z');
title('Robot Link Positions');

% Base
plot3(0,0,0,'ko','MarkerSize',10,'LineWidth',2);

% Plot links
plot3([0 p(1,:)], ...
      [0 p(2,:)], ...
      [0 p(3,:)], ...
      '-o', ...
      'LineWidth',2);

% Label từng link
for i = 1:6
    text(p(1,i), p(2,i), p(3,i), sprintf('L%d',i), ...
        'FontSize',12, ...
        'FontWeight','bold');
end

view(3);