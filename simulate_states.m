load ('states.mat')
load ('control.mat')
% load('extra.mat')


figure
plot(state_trajectory(:,13), 'b', 'Linewidth', 1.5);
xlim([1 50])
% ylim([0.5 1])
ylabel('x_{bar} (m)','FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
plot(state_trajectory(:,14), 'b', 'Linewidth', 1.5);
xlim([1 50])
% ylim([0.5 1])
ylabel('y_{bar} (m)','FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
plot(state_trajectory(:,15), 'b', 'Linewidth', 1.5);
xlim([1 50])
% ylim([0.5 1])
ylabel('z_{bar} (m)','FontSize',14)
xlabel('Discrete Time','FontSize',14)


figure
plot(state_trajectory(:,16),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('$\dot{x}_{bar}$ (m/s)', 'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
plot(state_trajectory(:,17),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('$\dot{y}_{bar}$ (m/s)', 'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
plot(state_trajectory(:,18),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('$\dot{z}_{bar}$ (m/s)', 'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)


figure
% subplot(2,1,1)
plot(rad2deg(state_trajectory(:,19)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('\phi_{bar} (º)','FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
% subplot(2,1,1)
plot(rad2deg(state_trajectory(:,20)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('\theta_{bar} (º)','FontSize',14)
xlabel('Discrete Time','FontSize',14)


figure
% subplot(2,1,1)
plot(rad2deg(state_trajectory(:,21)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('\psi_{bar} (º)','FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
plot(rad2deg(state_trajectory(:,22)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('$wx_{bar}$ $(^\circ / s)$', 'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
plot(rad2deg(state_trajectory(:,23)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('$wy_{bar}$ $(^\circ / s)$', 'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
plot(rad2deg(state_trajectory(:,24)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel('$wz_{bar}$ $(^\circ / s)$', 'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)

% Drone 1

figure
plot(rad2deg(state_trajectory(:,1)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel({'\phi_1 (º)'}, 'FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
plot(rad2deg(state_trajectory(:,2)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel({'\theta_1 (º)'}, 'FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
plot(rad2deg(state_trajectory(:,3)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel({'\psi_1 (º)'}, 'FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
plot(rad2deg(state_trajectory(:,4)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel({'wx_1'}, 'FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
plot(rad2deg(state_trajectory(:,5)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel({'wy_1'}, 'FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
plot(rad2deg(state_trajectory(:,6)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel({'wz_1'}, 'FontSize',14)
xlabel('Discrete Time','FontSize',14)

% Drone 2

figure
plot(rad2deg(state_trajectory(:,7)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel({'\phi_2 (º)'}, 'FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
plot(rad2deg(state_trajectory(:,8)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel({'\theta_2 (º)'}, 'FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
plot(rad2deg(state_trajectory(:,9)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel({'\psi_2 (º)'}, 'FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
plot(rad2deg(state_trajectory(:,10)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel({'wx_2'}, 'FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
plot(rad2deg(state_trajectory(:,11)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel({'wy_2'}, 'FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
plot(rad2deg(state_trajectory(:,14)),'b', 'Linewidth', 1.5);
xlim([1 50])
ylabel({'wz_2'}, 'FontSize',14)
xlabel('Discrete Time','FontSize',14)

% drone 1
figure
stairs(control_variables(:,1),'-k', 'Linewidth', 1);
xlim([1 50])
ylabel({'$T_{1}$ (N)'},'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
stairs(control_variables(:,2),'-k', 'Linewidth', 1);
xlim([1 50])
ylabel({'$\tau_{x1}$ (Nm)'},'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
stairs(control_variables(:,3),'-k', 'Linewidth', 1);
xlim([1 50])
ylabel({'$\tau_{y1}$ (Nm)'},'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
stairs(control_variables(:,4),'-k', 'Linewidth', 1);
xlim([1 50])
ylabel({'$\tau_{z1}$ (Nm)'},'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)

%drone 2
figure
stairs(control_variables(:,5),'-k', 'Linewidth', 1);
xlim([1 50])
ylabel({'$T_{2}$ (N)'},'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
stairs(control_variables(:,6),'-k', 'Linewidth', 1);
xlim([1 50])
ylabel({'$\tau_{x2}$ (Nm)'},'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
stairs(control_variables(:,7),'-k', 'Linewidth', 1);
xlim([1 50])
ylabel({'$\tau_{y2}$ (Nm)'},'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)

figure
stairs(control_variables(:,8),'-k', 'Linewidth', 1);
xlim([1 50])
ylabel({'$\tau_{z2}$ (Nm)'},'Interpreter','latex','FontSize',14)
xlabel('Discrete Time','FontSize',14)