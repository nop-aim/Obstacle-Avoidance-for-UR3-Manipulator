clc;
clear;
close all;

%% =====================================================
% 1. IMPORT ROBOTS FROM URDF
%% =====================================================

robotUR3  = importrobot("ur3.urdf");
robotGoFa = importrobot("gofa.urdf");

robotUR3.DataFormat  = 'row';
robotGoFa.DataFormat = 'row';

robotUR3.Gravity  = [0 0 -9.81];
robotGoFa.Gravity = [0 0 -9.81];

%% =====================================================
% 2. HOME CONFIGURATION
%% =====================================================

qHome_UR3  = homeConfiguration(robotUR3);
qHome_GoFa = homeConfiguration(robotGoFa);

%% =====================================================
% 3. JOINT LIMITS EXTRACTION
%% =====================================================

qLim_UR3  = getJointLimits(robotUR3);
qLim_GoFa = getJointLimits(robotGoFa);

%% =====================================================
% 4. WORKSPACE SAMPLING PARAMETERS
%% =====================================================

nUR3  = 40000;
nGoFa = 60000;

eeUR3  = robotUR3.BodyNames{end};
eeGoFa = robotGoFa.BodyNames{end};

%% =====================================================
% 5. COMPUTE WORKSPACE POINT CLOUD
%% =====================================================

disp("Sampling UR3 workspace...");
ptsUR3 = computeWorkspace(robotUR3, qLim_UR3, eeUR3, nUR3);

disp("Sampling GoFa workspace...");
ptsGoFa = computeWorkspace(robotGoFa, qLim_GoFa, eeGoFa, nGoFa);

%% =====================================================
% 6. CREATE ALPHASHAPE (OUTER ENVELOPE ONLY)
%% =====================================================

alphaUR3  = 0.035;
alphaGoFa = 0.05;

shpUR3  = alphaShape(ptsUR3,  alphaUR3);
shpGoFa = alphaShape(ptsGoFa, alphaGoFa);

fprintf("UR3 workspace volume  = %.3f m^3\n", volume(shpUR3));
fprintf("GoFa workspace volume = %.3f m^3\n", volume(shpGoFa));

%% =====================================================
% 7. PLOT UR3 + WORKSPACE (FINAL FIXED)
%% =====================================================

figure('Color','w');
hold on;

% ---- ROBOT (draw first, make it dominant)
show(robotUR3, qHome_UR3, ...
    'Frames','off', ...
    'Visuals','on');

camlight headlight
material dull
lighting gouraud

% ---- WORKSPACE SHELL (LIGHT WIREFRAME)
[F1,P1] = boundaryFacets(shpUR3);
[Fr1, Pr1] = reducepatch(F1, P1, 0.10);

trisurf(Fr1, ...
    Pr1(:,1), Pr1(:,2), Pr1(:,3), ...
    'FaceColor','none', ...
    'EdgeColor',[0.75 0.2 0.2], ...
    'LineWidth',0.3, ...
    'EdgeAlpha',0.10);

axis equal;
axis manual;
daspect([1 1 1])
pbaspect([1 1 1])
grid on;

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title("UR3 Robot with Workspace Outer Envelope");

xlim([-0.6 0.6])
ylim([-0.6 0.6])
zlim([-0.2 0.8])

view(135,30);

%% =====================================================
% 8. PLOT GOFA + WORKSPACE (FINAL FIXED)
%% =====================================================

figure('Color','w');
hold on;

% ---- ROBOT
show(robotGoFa, qHome_GoFa, ...
    'Frames','off', ...
    'Visuals','on');

camlight headlight
material dull
lighting gouraud

% ---- WORKSPACE SHELL (LIGHT WIREFRAME)
[F2,P2] = boundaryFacets(shpGoFa);
[Fr2, Pr2] = reducepatch(F2, P2, 0.10);

trisurf(Fr2, ...
    Pr2(:,1), Pr2(:,2), Pr2(:,3), ...
    'FaceColor','none', ...
    'EdgeColor',[0.2 0.3 0.8], ...
    'LineWidth',0.3, ...
    'EdgeAlpha',0.10);

axis equal;
axis manual;
daspect([1 1 1])
pbaspect([1 1 1])
grid on;

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title("ABB GoFa Robot with Workspace Outer Envelope");

xlim([-1.4 1.4])
ylim([-1.4 1.4])
zlim([-0.5 1.4])

view(135,30);

disp("DONE ✔ FULL FINAL VERSION");
%% =====================================================
% 9. WORKSPACE METRICS
%% =====================================================

volUR3  = volume(shpUR3);
volGoFa = volume(shpGoFa);

fprintf("\n===== WORKSPACE METRICS =====\n");
fprintf("UR3 Reachable Workspace Volume  : %.3f m^3\n", volUR3);
fprintf("GoFa Reachable Workspace Volume : %.3f m^3\n", volGoFa);

fprintf("GoFa workspace is %.1f times larger than UR3\n", ...
        volGoFa / volUR3);


%% =====================================================
%                LOCAL FUNCTIONS
%% =====================================================

function qLim = getJointLimits(robot)
    joints = robot.homeConfiguration;
    n = numel(joints);
    qLim = zeros(n,2);

    for i = 1:n
        lim = robot.Bodies{i+1}.Joint.PositionLimits;
        if isempty(lim) || any(isnan(lim))
            qLim(i,:) = [-pi pi];
        else
            qLim(i,:) = lim;
        end
    end
end

function pts = computeWorkspace(robot, qLim, eeName, nSamples)
    nJoint = size(qLim,1);
    pts = zeros(nSamples,3);

    for i = 1:nSamples
        q = qLim(:,1)' + rand(1,nJoint).*(qLim(:,2)-qLim(:,1))';
        T = getTransform(robot, q, eeName);
        pts(i,:) = T(1:3,4)';
    end
end
