clc; clear; close all;

%% =====================================================
% IEEE FIGURE STYLE (Conference Standard)
%% =====================================================
set(groot,...
    'defaultAxesFontName','Times New Roman',...
    'defaultTextFontName','Times New Roman',...
    'defaultAxesFontSize',9,...
    'defaultTextFontSize',9,...
    'defaultLineLineWidth',0.8);

%% =====================================================
% LOAD ROBOT FROM URDF
%% =====================================================
robot = importrobot("gofa.urdf");
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

eeName = robot.BodyNames{end};

%% =====================================================
% GET JOINT LIMITS FROM URDF
%% =====================================================
nJ = numel(robot.homeConfiguration);
qMin = zeros(1,nJ);
qMax = zeros(1,nJ);

for i = 1:nJ
    qMin(i) = robot.Bodies{i}.Joint.PositionLimits(1);
    qMax(i) = robot.Bodies{i}.Joint.PositionLimits(2);
end

%% =====================================================
% MONTE CARLO SAMPLING (Research Standard)
%% =====================================================
nSamples = 60000;   % suitable for IEEE figure

pts = zeros(nSamples,3);

for k = 1:nSamples
    q = qMin + (qMax-qMin).*rand(1,nJ);
    T = getTransform(robot,q,eeName);
    pts(k,:) = T(1:3,4)';
end

%% =====================================================
% WORKSPACE ENVELOPE (Alpha Shape)
%% =====================================================
alphaRadius = 0.05;   % tuned for GOFA scale
shp = alphaShape(pts,alphaRadius);

[F,P] = boundaryFacets(shp);
[Fr,Pr] = reducepatch(F,P,0.2);   % mesh reduction for clarity

%% =====================================================
% IEEE FIGURE PLOT
%% =====================================================
fig = figure('Color','w','Units','centimeters',...
             'Position',[5 5 8 6]);   % 1-column IEEE size
hold on;

trisurf(Fr,...
    Pr(:,1),Pr(:,2),Pr(:,3),...
    'FaceColor','none',...
    'EdgeColor',[0 0 0],...
    'EdgeAlpha',0.25);

axis equal;
grid on;
box on;

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('GOFA Position Workspace');

view(135,30);
set(gca,'LineWidth',0.8);

%% =====================================================
% EXPORT FOR IEEE PAPER
%% =====================================================
print(fig,'GOFA_Workspace','-depsc','-r600');   % EPS 600 dpi
clc; clear; close all;

%% IEEE Figure Style
set(groot,...
    'defaultAxesFontName','Times New Roman',...
    'defaultTextFontName','Times New Roman',...
    'defaultAxesFontSize',10,...
    'defaultTextFontSize',10,...
    'defaultLineLineWidth',0.5);

%% Load UR3
robot = importrobot('gofa.urdf');
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

ee = 'tool0';          % Correct end-effector
qHome = homeConfiguration(robot);

%% Get joint limits
qlim = zeros(6,2);
count = 1;
for i = 1:robot.NumBodies
    joint = robot.Bodies{i}.Joint;
    if strcmp(joint.Type,'revolute')
        qlim(count,:) = joint.PositionLimits;
        count = count + 1;
    end
end

%% Sampling (only first 3 joints)
q1 = linspace(qlim(1,1), qlim(1,2), 50);
q2 = linspace(qlim(2,1), qlim(2,2), 50);
q3 = linspace(qlim(3,1), qlim(3,2), 40);

pts = zeros(length(q1)*length(q2)*length(q3),3);
idx = 1;

for i = 1:length(q1)
    for j = 1:length(q2)
        for k = 1:length(q3)

            q = zeros(1,6);
            q(1)=q1(i);
            q(2)=q2(j);
            q(3)=q3(k);
            q(4:6)=0;

            T = getTransform(robot,q,ee);
            pts(idx,:) = T(1:3,4)';
            idx = idx + 1;
        end
    end
end

%% Alpha Shape Workspace
shp = alphaShape(pts,0.05);
[F,P] = boundaryFacets(shp);
[Fr,Pr] = reducepatch(F,P,0.2);

%% Plot
figure('Color','w'); hold on;

show(robot,qHome,'Frames','off','Visuals','on');
camlight headlight; lighting gouraud;

trisurf(Fr,...
    Pr(:,1),Pr(:,2),Pr(:,3),...
    'FaceColor','none',...
    'EdgeColor',[0 0 0],...
    'EdgeAlpha',0.15,...
    'LineWidth',0.3);

axis equal
grid on
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
view(135,30)

title('GOFA Position Workspace')
