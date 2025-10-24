t = closedLoopTest(:,1);
pump1 = closedLoopTest(:,2);
dfr1 = closedLoopTest(:,3);
flow1 = closedLoopTest(:,4);
valve1 = closedLoopTest(:,5);
pump2 = closedLoopTest(:,6);
dfr2 = closedLoopTest(:,7);
flow2 = closedLoopTest(:,8);
valve2 = closedLoopTest(:,9);
pump3 = closedLoopTest(:,10);
dfr3 = closedLoopTest(:,11);
flow3 = closedLoopTest(:,12);
valve3 = closedLoopTest(:,13);
pump4 = closedLoopTest(:,14);
dfr4 = closedLoopTest(:,15);
flow4 = closedLoopTest(:,16);
valve4 = closedLoopTest(:,17);

font = 21;

figure(1);
subplot(2,1,1);
plot(t,dfr1,'r--',t,flow1,'r','LineWidth',1.5);
hold on
plot(t,dfr2,'b--',t,flow2,'b','LineWidth',1.5);
plot(t,dfr3,'g--',t,flow3,'g','LineWidth',1.5);
plot(t,dfr4,'k--', t,flow4,'k','LineWidth',1.5);

legend('$\bar{q}_1$', '$q_1$', '$\bar{q}_2$', '$q_2$', '$\bar{q}_3$', '$q_3$', '$\bar{q}_4$', '$q_4$', ...
    'Interpreter', 'latex', 'Location', 'northwest', 'FontSize', font);
grid on;
ylabel('\textbf{Flow Rate (L/h)}', 'Interpreter','latex', 'FontSize', font);
xlabel('\textbf{Time (s)}', 'Interpreter','latex', 'FontSize', font);
% title('Flow Rate');
hold off;

subplot(2,1,2);
plot(t,pump1,'r','LineWidth',1.5);
hold on
plot(t,pump2,'b','LineWidth',1.5);
plot(t,pump3,'g','LineWidth',1.5);
plot(t,pump4,'k','LineWidth',1.5);
legend('$u_1$', '$u_2$', '$u_3$', '$u_4$', 'Interpreter', 'latex', 'Location', 'northwest', 'FontSize', font);
grid on;
ylabel('\textbf{Pump Power(\%)}', 'Interpreter','latex', 'FontSize', font);
xlabel('\textbf{Time (s)}', 'Interpreter','latex', 'FontSize', font);
% title('Pump Power');
% sgtitle('Closed Loop Time History, Pump: Kp=0.07, Ki=0.03');

% subplot(3,1,3);
% plot(t,valve1,'r','LineWidth',1.5);
% hold on
% plot(t,valve2,'b','LineWidth',1.5);
% plot(t,valve3,'g','LineWidth',1.5);
% plot(t,valve4,'k','LineWidth',1.5);
% legend('$V_1$', '$V_2$', '$V_3$', '$V_4$', 'Interpreter', 'latex', 'Location', 'northwest', 'FontSize', 14);
% grid on;
% ylabel('\textbf{Valve Position (\% Open)}', 'Interpreter','latex');
% xlabel('\textbf{Time (s)}', 'Interpreter','latex');
