t = openLoopNozzleTest(:,1);
pump1 = openLoopNozzleTest(:,2);
valve1 = openLoopNozzleTest(:,3);
flow1 = openLoopNozzleTest(:,4);
pump2 = openLoopNozzleTest(:,5);
valve2 = openLoopNozzleTest(:,6);
flow2 = openLoopNozzleTest(:,7);
pump3 = openLoopNozzleTest(:,8);
valve3 = openLoopNozzleTest(:,9);
flow3 = openLoopNozzleTest(:,10);
pump4 = openLoopNozzleTest(:,11);
valve4 = openLoopNozzleTest(:,12);
flow4 = openLoopNozzleTest(:,13);

subplot(2,2,1);
yyaxis right 
plot(t,flow1,'r-','LineWidth',1.5);
ylabel('\textbf{Flow Rate (L/h)}', 'Interpreter','latex');
ylim([0 300]);
hold on
grid on;

yyaxis left 
ylabel('\textbf{Pump Power (\%)}', 'Interpreter', 'latex');
plot(t,pump1,'k');
xlabel('\textbf{Time (s)}', 'Interpreter','latex');
title('Nozzle Unit 1 Open Loop Response');
hold off;
legend('$u_1$','$q_1$', 'Location', 'northwest', 'FontSize', 14, 'Interpreter', 'latex');

subplot(2,2,2);
yyaxis right 
plot(t,flow2,'b-','LineWidth',1.5);
ylabel('\textbf{Flow Rate (L/h)}', 'Interpreter','latex');
ylim([0 300]);
hold on
grid on;

yyaxis left 
ylabel('\textbf{Pump Power (\%)}', 'Interpreter', 'latex');
plot(t,pump2,'k');
xlabel('\textbf{Time (s)}', 'Interpreter','latex');
title('Nozzle Unit 2 Open Loop Response');
hold off;
legend('$u_2$','$q_2$', 'Location', 'northwest', 'FontSize', 14, 'Interpreter', 'latex');

subplot(2,2,3);
yyaxis right 
plot(t,flow3,'g-','LineWidth',1.5);
ylabel('\textbf{Flow Rate (L/h)}', 'Interpreter','latex');
ylim([0 300]);
hold on
grid on;

yyaxis left 
ylabel('\textbf{Pump Power (\%)}', 'Interpreter', 'latex');
plot(t,pump3,'k');
xlabel('\textbf{Time (s)}', 'Interpreter','latex');
title('Nozzle Unit 3 Open Loop Response');
hold off;
legend('$u_3$','$q_3$', 'Location', 'northwest', 'FontSize', 14, 'Interpreter', 'latex');

subplot(2,2,4);
yyaxis right 
plot(t,flow4,'m-','LineWidth',1.5);
ylabel('\textbf{Flow Rate (L/h)}', 'Interpreter','latex');
ylim([0 300]);
hold on
grid on;

yyaxis left 
ylabel('\textbf{Pump Power (\%)}', 'Interpreter', 'latex');
plot(t,pump4,'k');
xlabel('\textbf{Time (s)}', 'Interpreter','latex');
title('Nozzle Unit 4 Open Loop Response');
hold off;
legend('$u_4$','$q_4$', 'Location', 'northwest', 'FontSize', 14, 'Interpreter', 'latex');

% subplot(2,1,1);
% yyaxis right
% plot(t,flow1, 'r-', 'LineWidth',1.5);
% hold on;
% plot(t,flow2, 'b-', 'LineWidth',1.5);
% ylabel('Flow Rate (L/h)');
% grid on;
% 
% yyaxis left
% plot(t,pump2, 'k');
% ylabel('Pump Power (%)');
% xlabel('Time(s)');
% legend('Left Pump Pwr', 'Nozzle1 Flow', 'Nozzle2 Flow', 'Location', 'northwest');
% 
% subplot(2,1,2);
% yyaxis right
% plot(t,flow3, 'r-', 'LineWidth',1.5);
% hold on;
% plot(t,flow4, 'b-', 'LineWidth',1.5);
% ylabel('Flow Rate (L/h)');
% grid on;
% 
% yyaxis left
% plot(t,pump4, 'k');
% ylabel('Pump Power (%)');
% xlabel('Time(s)');
% legend('Right Pump Pwr', 'Nozzle3 Flow', 'Nozzle4 Flow', 'Location', 'northwest');

% set(gcf, 'Units', 'pixels', 'Position', [1 1 1920 1200]);
% print(gcf,'openLoopTest_1-to-1.png','-dpng', '-r0');