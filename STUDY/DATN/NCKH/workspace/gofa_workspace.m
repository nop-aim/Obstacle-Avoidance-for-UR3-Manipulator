clc; clear; close all;

%% ============================
% IEEE FIGURE STYLE
%% ============================
set(groot,...
    'defaultAxesFontName','Times New Roman',...
    'defaultTextFontName','Times New Roman',...
    'defaultAxesFontSize',10,...
    'defaultTextFontSize',10,...
    'defaultLineLineWidth',0.5);

%% ============================
% LOAD ROBOT
%% ============================
robot = importrobot("gofa.urdf");
robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

qHome = homeConfiguration(robot);
ee = robot.BodyNames{end};

%% ============================
% GRID-BASED SAMPLING
%% ============================
q1 = linspace(-pi, pi, 60);
q2 = linspace(-pi, pi, 60);
q3 = linspace(-3.93, 1.48, 80);

pts = zeros(length(q1)*length(q2)*length(q3),3);
idx = 1;

for i = 1:length(q1)
    for j = 1:length(q2)
        for k = 1:length(q3)

            q = zeros(1,6);
            q(1)=q1(i); q(2)=q2(j); q(3)=q3(k);
            q(4:6)=0;

            T = getTransform(robot,q,ee);
            pts(idx,:) = T(1:3,4)';
            idx = idx + 1;
        end
    end
end

%% ============================
% WORKSPACE ENVELOPE
%% ============================
shp = alphaShape(pts,0.06);
[F,P] = boundaryFacets(shp);
[Fr,Pr] = reducepatch(F,P,0.15);

%% ============================
% PLOT (IEEE)
%% ============================
figure('Color','w'); hold on;

show(robot,qHome,'Frames','off','Visuals','on');
camlight headlight
lighting gouraud

trisurf(Fr,...
    Pr(:,1),Pr(:,2),Pr(:,3),...
    'FaceColor','none',...
    'EdgeColor',[0 0 0],...
    'EdgeAlpha',0.15,...
    'LineWidth',0.3);

axis equal;
grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
view(135,30);

title('ABB GoFa Position Workspace');
