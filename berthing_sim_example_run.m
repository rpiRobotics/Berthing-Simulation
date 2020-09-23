% Baxter Transport Simulation
% Use defined path/trajectory, execute trajectory using motor controller

clear variables; close all; clc


% Simulation Parameters
T =10; % Sim Time
ts_control = 0.002; %0.0025;%0.02;%0.0025;

% Add models and subfunction paths:

% Load Baxter Model
[robot_const, ~] = defineBaxterFlexJoint();
%robot_const.mprops.bm = robot_const.mprops.bm/20;
qlimit = [robot_const(1).limit.lower_joint_limit,...
    robot_const(1).limit.upper_joint_limit];

% Torque saturation
taulimit = Inf*[1;1;1;1;1;1;1];

% Controller parameters and contact model
kp = 5*[1;1;1;1;1;1;1];
ki = 0.001*ones(7,1);
kd = 0.00005*[1;1;1;1;1;1;1];

% Contact parameters
kBP = 10000;
kt_contact = 12000;
bt_contact = 200;
kr_contact = 14000;
br_contact = 500;

% Load Mass-Inertia Properties of Grasped Object:
RTC = eye(3);
m_sat = 2200; % 
c_sat = [0;0;0];
r = 0.7;
h = 4.3;
PGsat = [-r;0;h/2]; % Vector from gripper to object center; % Case 1
PTC = PGsat;
I_sat = [1/12*m_sat*(3*r^2+h^2),0,0;0,1/12*m_sat*(3*r^2+h^2),0;0,0,1/2*m_sat*r^2];

% Set initial conditions for joints:
q0 = [-0.0572;0.8617;0;-1.2870;0.0002;0.4292;0.0570];

qdot0 = zeros(7,1);
qm0 = robot_const.mprops.N.*q0;
qmdot0 = zeros(7,1);
[RBT0,PBT0] = fwdkin(robot_const(1).kin, q0);
P0C = [1;0;2.5] + PBT0 + RBT0*PTC;
R0C = RBT0;
Pd = PBT0;

% Simulate

tic
%sim('notorque_sim.slx')
sim('threepost_sim.slx')
%sim('fourpost_sim.sxl')
toc

qm_sim = squeeze(qm);
qmdot_sim = squeeze(qmdot);
qmddot_sim = squeeze(qmddot);
qd_sim = squeeze(qd);
qdotd_sim = squeeze(qdotd);
q_sim = q';
qdot_sim = qdot';
qddot_sim = qddot';
tout_sim = t_dt;
tout = t_dt;
qd = squeeze(qd);
q = q';
qd = qd';
tau = tau';
qe = abs(qd-q);
qmd = squeeze(qmd);
qmd = qmd';
qm = qm';
qme = abs(qmd-qm);
qmdot = qmdot';
qdot = qdot';

% Plot results
for k =1:7
    if k~=7
        subplot(3,3,k)
    else
        subplot(3,3,8)
    end
    plot(tout,qd(k,:),'b','LineWidth',2)
    hold on
    plot(tout,q(k,:),'r--','LineWidth',2)
    xlabel('Time (s)')
    if (k==1)||(k==4)||(k==7)
        ylabel('Pos (rad)')
    end
    if k==7
        legend('Desired','Actual')
    end
    title(['q',num2str(k)])
end


figure(2)
for k =1:7
    if k~=7
        subplot(3,3,k)
    else
        subplot(3,3,8)
    end
    plot(tout,tau(k,:),'b','LineWidth',2)
    xlabel('Time (s)')
    if (k==1)||(k==4)||(k==7)
        ylabel('Torque (Nm)')
    end
%     if k==7
%         legend('Desired','Actual')
%     end
    title(['q',num2str(k)])
end


figure(4)
for k =1:7
    if k~=7
        subplot(3,3,k)
    else
        subplot(3,3,8)
    end
    plot(tout,qmd(k,:),'b','LineWidth',2)
    hold on
    plot(tout,qm(k,:),'r--','LineWidth',2)
    xlabel('Time (s)')
    if (k==1)||(k==4)||(k==7)
        ylabel('Pos (rad)')
    end
    if k==7
        legend('Desired','Actual')
    end
    title(['qm',num2str(k)])
end

ee_forces_names = ["x-dir","y-dir","z-dir"];

figure(6)
for k=1:6
    subplot(2,3,k)
    if k<4
        plot(tout,FEE(:,k),'b','LineWidth',2)
        ylabel('Force (N)')
        title(ee_forces_names(k))
    else
        plot(tout,tauEE(:,k-3),'b','LineWidth',2)
        ylabel('Torque (Nm)')
        title(ee_forces_names(k-3))
    end
    xlabel('Time (s)')
end

figure(7)
subplot(1,3,1)
plot(tout,ds1,'b','LineWidth',2)
xlabel('Time (s)')
ylabel('Sep Dist (m)')
subplot(1,3,2)
plot(tout,fn1,'b','LineWidth',2)
xlabel('Time (s)')
ylabel('Normal Force (N)')
title('BP 1')
subplot(1,3,3)
plot(tout,ffric1,'b','LineWidth',2)
xlabel('Time (s)')
ylabel('Friction Force (N)')

figure(8)
subplot(1,3,1)
plot(tout,ds2,'b','LineWidth',2)
xlabel('Time (s)')
ylabel('Sep Dist (m)')
subplot(1,3,2)
plot(tout,fn2,'b','LineWidth',2)
xlabel('Time (s)')
ylabel('Normal Force (N)')
title('BP 2')
subplot(1,3,3)
plot(tout,ffric2,'b','LineWidth',2)
xlabel('Time (s)')
ylabel('Friction Force (N)')

figure(9)
subplot(1,3,1)
plot(tout,ds3,'b','LineWidth',2)
xlabel('Time (s)')
ylabel('Sep Dist (m)')
subplot(1,3,2)
plot(tout,fn3,'b','LineWidth',2)
xlabel('Time (s)')
ylabel('Normal Force (N)')
title('BP 3')
subplot(1,3,3)
plot(tout,ffric3,'b','LineWidth',2)
xlabel('Time (s)')
ylabel('Friction Force (N)')
