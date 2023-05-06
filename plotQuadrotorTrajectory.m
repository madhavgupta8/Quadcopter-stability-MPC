% This script plots the closed-loop responses of the nonlinear MPC
% controller used in the quadrotor path following example.

% Copyright 2019 The MathWorks, Inc.

% Plot the closed-loop response.
global Duration;
time = 0:Ts:Duration;
yreftot = QuadrotorReferenceTrajectory(time)';

% Plot the states.
fontsize = 14;
figure('Name','States','NumberTitle','off')

subplot(2,3,1)
hold on
plot(time,xHistory(:,1),'linewidth',2)
plot(time,yreftot(:,1),'--','linewidth',2)
grid on
xlabel('time','FontSize',fontsize)
ylabel('x','FontSize',fontsize)
legend('actual','reference','Location','southeast','FontSize',fontsize)
title('Quadrotor x position','FontSize',fontsize)

subplot(2,3,2)
hold on
plot(time,xHistory(:,2),'linewidth',2)
plot(time,yreftot(:,2),'--','linewidth',2)
grid on
xlabel('time','FontSize',fontsize)
ylabel('y','FontSize',fontsize)
legend('actual','reference','Location','southeast','FontSize',fontsize)
title('Quadrotor y position','FontSize',fontsize)

subplot(2,3,3)
hold on
plot(time,xHistory(:,3),'linewidth',2)
plot(time,yreftot(:,3),'--','linewidth',2)
grid on
xlabel('time','FontSize',fontsize)
ylabel('z','FontSize',fontsize)
legend('actual','reference','Location','northeast','FontSize',fontsize)
title('Quadrotor z position','FontSize',fontsize)

subplot(2,3,4)
hold on
plot(time,xHistory(:,4)/pi*180,'linewidth',2)
plot(time,yreftot(:,4)/pi*180,'--','linewidth',2)
grid on
xlabel('time','FontSize',fontsize)
ylabel('\phi','FontSize',fontsize)
legend('actual','reference','Location','southeast','FontSize',fontsize)
title('Quadrotor phi angle (deg)','FontSize',fontsize)

subplot(2,3,5)
hold on
plot(time,xHistory(:,5)/pi*180,'linewidth',2)
plot(time,yreftot(:,5)/pi*180,'--','linewidth',2)
grid on
xlabel('time','FontSize',fontsize)
ylabel('\theta','FontSize',fontsize)
legend('actual','reference','Location','southeast','FontSize',fontsize)
title('Quadrotor theta angle (deg)','FontSize',fontsize)

subplot(2,3,6)
hold on
plot(time,xHistory(:,6)/pi*180,'linewidth',2)
plot(time,yreftot(:,6)/pi*180,'--','linewidth',2)
grid on
xlabel('time','FontSize',fontsize)
ylabel('\psi','FontSize',fontsize)
legend('actual','reference','Location','southeast','FontSize',fontsize)
title('Quadrotor psi angle (deg)','FontSize',fontsize)

%% Plot the manipulated variables.
figure('Name','Control Inputs','NumberTitle','off')

subplot(2,2,1)
hold on
stairs(time,uHistory(:,1),'linewidth',2)
% ylim([-0.5,12.5])
plot(time,nloptions.MVTarget(2)*ones(1,length(time)),'--','linewidth',2)
grid on
xlabel('time','FontSize',fontsize)
legend('actual','reference','FontSize',fontsize)
title('Input 1','FontSize',fontsize)

subplot(2,2,2)
hold on
stairs(time,uHistory(:,2),'linewidth',2)
% ylim([-0.5,12.5])
plot(time,nloptions.MVTarget(2)*ones(1,length(time)),'--','linewidth',2)
grid on
xlabel('time','FontSize',fontsize)
title('Input 2','FontSize',fontsize)
legend('actual','reference','FontSize',fontsize)

subplot(2,2,3)
hold on
stairs(time,uHistory(:,3),'linewidth',2)
% ylim([-0.5,12.5])
plot(time,nloptions.MVTarget(2)*ones(1,length(time)),'--','linewidth',2)
grid on
xlabel('time','FontSize',fontsize)
title('Input 3','FontSize',fontsize)
legend('actual','reference','FontSize',fontsize)

subplot(2,2,4)
hold on
stairs(time,uHistory(:,4),'linewidth',2)
% ylim([-0.5,12.5])
plot(time,nloptions.MVTarget(2)*ones(1,length(time)),'--','linewidth',2)
grid on
xlabel('time','FontSize',fontsize)
title('Input 4','FontSize',fontsize)
legend('actual','reference','FontSize',fontsize)


