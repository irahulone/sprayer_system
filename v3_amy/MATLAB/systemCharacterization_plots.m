%% 
% Extract data
pump1_pwr_30 = pump_valve_data(:,2); 
valve1_pos_30 = pump_valve_data(:,3); 
flow1_30 = pump_valve_data(:,4); 
pump2_pwr_30 = pump_valve_data(:,5); 
valve2_pos_30 = pump_valve_data(:,6);  
flow2_30 = pump_valve_data(:,7);  

pump3_pwr_30 = pump_valve_data(:,8); 
valve3_pos_30 = pump_valve_data(:,9); 
flow3_30 = pump_valve_data(:,10); 
pump4_pwr_30 = pump_valve_data(:,11); 
valve4_pos_30 = pump_valve_data(:,12); 
flow4_30 = pump_valve_data(:,13); 

pump1_pwr_40 = pump_valve_data(:,2); 
valve1_pos_40 = pump_valve_data(:,3); 
flow1_40 = pump_valve_data(:,4); 
pump2_pwr_40 = pump_valve_data(:,5); 
valve2_pos_40 = pump_valve_data(:,6);  
flow2_40 = pump_valve_data(:,7);  

pump3_pwr_40 = pump_valve_data(:,8); 
valve3_pos_40 = pump_valve_data(:,9); 
flow3_40 = pump_valve_data(:,10); 
pump4_pwr_40 = pump_valve_data(:,11); 
valve4_pos_40 = pump_valve_data(:,12); 
flow4_40 = pump_valve_data(:,13); 

pump1_pwr_50 = pump_valve_data(:,2); 
valve1_pos_50 = pump_valve_data(:,3); 
flow1_50 = pump_valve_data(:,4); 
pump2_pwr_50 = pump_valve_data(:,5); 
valve2_pos_50 = pump_valve_data(:,6);  
flow2_50 = pump_valve_data(:,7);  

pump3_pwr_50 = pump_valve_data(:,8); 
valve3_pos_50 = pump_valve_data(:,9); 
flow3_50 = pump_valve_data(:,10); 
pump4_pwr_50 = pump_valve_data(:,11); 
valve4_pos_50 = pump_valve_data(:,12); 
flow4_50 = pump_valve_data(:,13); 


% Create grid for interpolation
[Xq30_pump1, Yq30_pump1] = meshgrid(linspace(min(valve1_pos_30), max(valve1_pos_30), 30), linspace(min(valve2_pos_30), max(valve2_pos_30), 30));
[Xq30_pump2, Yq30_pump2] = meshgrid(linspace(min(valve1_pos_30), max(valve1_pos_30), 30), linspace(min(valve2_pos_30), max(valve2_pos_30), 30));

% Interpolate flow rates onto grid
Zq_flow1_30 = griddata(valve1_pos_30, valve2_pos_30, flow1_30, Xq30_pump1, Yq30_pump1, 'cubic');
Zq_flow2_30 = griddata(valve1_pos_30, valve2_pos_30, flow2_30, Xq30_pump1, Yq30_pump1, 'cubic');
Zq_flow3_30 = griddata(valve3_pos_30, valve4_pos_30, flow3_30, Xq30_pump2, Yq30_pump2, 'cubic');
Zq_flow4_30 = griddata(valve3_pos_30, valve4_pos_30, flow4_30, Xq30_pump2, Yq30_pump2, 'cubic');

% Plot for Nozzle 1
figure(1);
sgtitle('Data From 7/14/25');
subplot(3,2,1);
surf(Xq30_pump1, Yq30_pump1, Zq_flow1_30);
xlabel('Valve 1 Position (%)');
ylabel('Valve 2 Position (%)');
zlabel('Flow Rate Nozzle 1 (L/h)');
title('3D Flow Rate Surface for Nozzle 1 @ Pump 1 Pwr: 30%');
colorbar;
shading interp; % Smooths color transitions

% Plot for Nozzle 2
subplot(3,2,2);
surf(Xq30_pump1, Yq30_pump1, Zq_flow2_30);
xlabel('Valve 1 Position (%)');
ylabel('Valve 2 Position (%)');
zlabel('Flow Rate Nozzle 2 (L/h)');
title('3D Flow Rate Surface for Nozzle 2 @ Pump 1 Pwr: 30%');
colorbar;
shading interp; % Smooths color transitions

% Plot for Nozzle 3
figure(2);
sgtitle('Data From 7/14/25');
subplot(3,2,1);
surf(Xq30_pump2, Yq30_pump2, Zq_flow3_30);
xlabel('Valve 3 Position (%)');
ylabel('Valve 4 Position (%)');
zlabel('Flow Rate Nozzle 3 (L/h)');
title('3D Flow Rate Surface for Nozzle 3 @ Pump 2 Pwr: 30%');
colorbar;
shading interp; % Smooths color transitions

% Plot for Nozzle 4
subplot(3,2,2);
surf(Xq30_pump2, Yq30_pump2, Zq_flow4_30);
xlabel('Valve 3 Position (%)');
ylabel('Valve 4 Position (%)');
zlabel('Flow Rate Nozzle 4 (L/h)');
title('3D Flow Rate Surface for Nozzle 4 @ Pump 2 Pwr: 30%');
colorbar;
shading interp; % Smooths color transitions


% Create grid for interpolation
[Xq40_pump1, Yq40_pump1] = meshgrid(linspace(min(valve1_pos_40), max(valve1_pos_40), 30), linspace(min(valve2_pos_40), max(valve2_pos_40), 30));
[Xq40_pump2, Yq40_pump2] = meshgrid(linspace(min(valve3_pos_40), max(valve3_pos_40), 30), linspace(min(valve4_pos_40), max(valve4_pos_40), 30));

% Interpolate flow rates onto grid
Zq_flow1_40 = griddata(valve1_pos_40, valve2_pos_40, flow1_40, Xq40_pump1, Yq40_pump1, 'cubic');
Zq_flow2_40 = griddata(valve1_pos_40, valve2_pos_40, flow2_40, Xq40_pump1, Yq40_pump1, 'cubic');
Zq_flow3_40 = griddata(valve3_pos_40, valve4_pos_40, flow3_40, Xq40_pump2, Yq40_pump2, 'cubic');
Zq_flow4_40 = griddata(valve3_pos_40, valve4_pos_40, flow4_40, Xq40_pump2, Yq40_pump2, 'cubic');


% Plot for Nozzle 1
figure(1);
subplot(3,2,3);
surf(Xq40_pump1, Yq40_pump1, Zq_flow1_40);
xlabel('Valve 1 Position (%)');
ylabel('Valve 2 Position (%)');
zlabel('Flow Rate Nozzle 1 (L/h)');
title('3D Flow Rate Surface for Nozzle 1 @ Pump 1 Pwr: 40%');
colorbar;
shading interp; % Smooths color transitions

% Plot for Nozzle 2
subplot(3,2,4);
surf(Xq40_pump1, Yq40_pump1, Zq_flow2_40);
xlabel('Valve 1 Position (%)');
ylabel('Valve 2 Position (%)');
zlabel('Flow Rate Nozzle 2 (L/h)');
title('3D Flow Rate Surface for Nozzle 2 @ Pump 1 Pwr: 40%');
colorbar;
shading interp; % Smooths color transitions

% Plot for Nozzle 3
figure(2);
subplot(3,2,3);
surf(Xq40_pump2, Yq40_pump2, Zq_flow3_40);
xlabel('Valve 3 Position (%)');
ylabel('Valve 4 Position (%)');
zlabel('Flow Rate Nozzle 3 (L/h)');
title('3D Flow Rate Surface for Nozzle 3 @ Pump 2 Pwr: 40%');
colorbar;
shading interp; % Smooths color transitions

% Plot for Nozzle 4
subplot(3,2,4);
surf(Xq40_pump2, Yq40_pump2, Zq_flow4_40);
xlabel('Valve 3 Position (%)');
ylabel('Valve 4 Position (%)');
zlabel('Flow Rate Nozzle 4 (L/h)');
title('3D Flow Rate Surface for Nozzle 4 @ Pump 2 Pwr: 40%');
colorbar;
shading interp; % Smooths color transitions


% Create grid for interpolation
[Xq50_pump1, Yq50_pump1] = meshgrid(linspace(min(valve1_pos_50), max(valve1_pos_50), 30), linspace(min(valve2_pos_50), max(valve2_pos_50), 30));
[Xq50_pump2, Yq50_pump2] = meshgrid(linspace(min(valve3_pos_50), max(valve3_pos_50), 30), linspace(min(valve4_pos_50), max(valve4_pos_50), 30));

% Interpolate flow rates onto grid
Zq_flow1_50 = griddata(valve1_pos_50, valve2_pos_50, flow1_50, Xq50_pump1, Yq50_pump1, 'cubic');
Zq_flow2_50 = griddata(valve1_pos_50, valve2_pos_50, flow2_50, Xq50_pump1, Yq50_pump1, 'cubic');
Zq_flow3_50 = griddata(valve3_pos_50, valve4_pos_50, flow3_50, Xq50_pump2, Yq50_pump2, 'cubic');
Zq_flow4_50 = griddata(valve3_pos_50, valve4_pos_50, flow4_50, Xq50_pump2, Yq50_pump2, 'cubic');

% Plot for Nozzle 1
figure(1);
subplot(3,2,5);
surf(Xq50_pump1, Yq50_pump1, Zq_flow1_50);
xlabel('Valve 1 Position (%)');
ylabel('Valve 2 Position (%)');
zlabel('Flow Rate Nozzle 1 (L/h)');
title('3D Flow Rate Surface for Nozzle 1 @ Pump 1 Pwr: 50%');
colorbar;
shading interp; % Smooths color transitions

% Plot for Nozzle 2
subplot(3,2,6);
surf(Xq50_pump1, Yq50_pump1, Zq_flow2_50);
xlabel('Valve 1 Position (%)');
ylabel('Valve 2 Position (%)');
zlabel('Flow Rate Nozzle 2 (L/h)');
title('3D Flow Rate Surface for Nozzle 2 @ Pump 1 Pwr: 50%');
colorbar;
shading interp; % Smooths color transitions

% % Plot for Nozzle 3
figure(2);
subplot(3,2,5);
surf(Xq50_pump2, Yq50_pump2, Zq_flow3_50);
xlabel('Valve 3 Position (%)');
ylabel('Valve 4 Position (%)');
zlabel('Flow Rate Nozzle 3 (L/h)');
title('3D Flow Rate Surface for Nozzle 3 @ Pump 2 Pwr: 50%');
colorbar;
shading interp; % Smooths color transitions

% Plot for Nozzle 4
subplot(3,2,6);
surf(Xq50_pump2, Yq50_pump2, Zq_flow4_50);
xlabel('Valve 3 Position (%)');
ylabel('Valve 4 Position (%)');
zlabel('Flow Rate Nozzle 4 (L/h)');
title('3D Flow Rate Surface for Nozzle 4 @ Pump 2 Pwr: 50%');
colorbar;
shading interp; % Smooths color transitions

% --- Combine Data from Different Pump Powers ---
all_valve1_pos = [valve1_pos_30; valve1_pos_40; valve1_pos_50];
all_valve2_pos = [valve2_pos_30; valve2_pos_40; valve2_pos_50];
all_valve3_pos = [valve3_pos_30; valve3_pos_40; valve3_pos_50];
all_valve4_pos = [valve4_pos_30; valve4_pos_40; valve4_pos_50];
all_pump1_pwr = [pump1_pwr_30; pump1_pwr_40; pump1_pwr_50]; % Pump 1 Power
all_pump2_pwr = [pump2_pwr_30; pump2_pwr_40; pump2_pwr_50]; % Pump 2 Power
all_flow1 = [flow1_30; flow1_40; flow1_50]; % Nozzle 1 Flow Rate
all_flow2 = [flow2_30; flow2_40; flow2_50]; % Nozzle 2 Flow Rate
all_flow3 = [flow3_30; flow3_40; flow3_50]; % Nozzle 3 Flow Rate
all_flow4 = [flow4_30; flow4_40; flow4_50]; % Nozzle 4 Flow Rate

% --- Create 3D Interpolation Function ---
F_flow1 = scatteredInterpolant(all_valve1_pos, all_valve2_pos, all_pump1_pwr, all_flow1, 'linear', 'none');
F_flow2 = scatteredInterpolant(all_valve1_pos, all_valve2_pos, all_pump1_pwr, all_flow2, 'linear', 'none');
F_flow3 = scatteredInterpolant(all_valve3_pos, all_valve4_pos, all_pump2_pwr, all_flow3, 'linear', 'none');
F_flow4 = scatteredInterpolant(all_valve3_pos, all_valve4_pos, all_pump2_pwr, all_flow4, 'linear', 'none');

% --- Define Fine Grid for Full 3D Interpolation ---
[V1q, V2q, P1q] = meshgrid(linspace(min(all_valve1_pos), max(all_valve1_pos), 30), ...
                            linspace(min(all_valve2_pos), max(all_valve2_pos), 30), ...
                            linspace(30, 50, 30)); % Pump Power Interpolated from 30% to 50%
[V3q, V4q, P2q] = meshgrid(linspace(min(all_valve3_pos), max(all_valve3_pos), 30), ...
                            linspace(min(all_valve4_pos), max(all_valve4_pos), 30), ...
                            linspace(30, 50, 30)); % Pump Power Interpolated from 30% to 50%

% --- Evaluate Interpolated Flow Rates ---
Flow1q = F_flow1(V1q, V2q, P1q);
Flow2q = F_flow2(V1q, V2q, P1q);
Flow3q = F_flow3(V3q, V4q, P2q);
Flow4q = F_flow4(V3q, V4q, P2q);

% % --- Create 3D Mesh Plots ---
% figure(3);
% sgtitle('Data From 4/2/25');
% 
% % --- Subplot 1: Nozzle 3 Flow Rate ---
% subplot(2,1,1);
% hold on;
% surf(V3q(:,:,1), V4q(:,:,1), Flow3q(:,:,1), P2q(:,:,1), 'EdgeColor', 'none'); % Pump Power 30%
% surf(V3q(:,:,15), V4q(:,:,15), Flow3q(:,:,15), P2q(:,:,15), 'EdgeColor', 'none'); % Pump Power 40% (Interpolated)
% surf(V3q(:,:,end), V4q(:,:,end), Flow3q(:,:,end), P2q(:,:,end), 'EdgeColor', 'none'); % Pump Power 50%
% xlabel('Valve 3 Position (%)');
% ylabel('Valve 4 Position (%)');
% zlabel('Flow Rate Nozzle 3 (L/h)');
% title('Nozzle 3 Flow Rate ');
% colorbar;
% caxis([30 50]); % Color gradient corresponds to pump power (30% - 50%)
% colormap(jet);
% view(3);
% grid on;
% 
% % --- Subplot 2: Nozzle 4 Flow Rate ---
% subplot(2,1,2);
% hold on;
% surf(V3q(:,:,1), V4q(:,:,1), Flow4q(:,:,1), P2q(:,:,1), 'EdgeColor', 'none'); % Pump Power 30%
% surf(V3q(:,:,15), V4q(:,:,15), Flow4q(:,:,15), P2q(:,:,15), 'EdgeColor', 'none'); % Pump Power 40% (Interpolated)
% surf(V3q(:,:,end), V4q(:,:,end), Flow4q(:,:,end), P2q(:,:,end), 'EdgeColor', 'none'); % Pump Power 50%
% xlabel('Valve 3 Position (%)');
% ylabel('Valve 4 Position (%)');
% zlabel('Flow Rate Nozzle 4 (L/h)');
% title('Nozzle 4 Flow Rate');
% colorbar;
% caxis([30 50]); % Color gradient corresponds to pump power (30% - 50%)
% colormap(jet);
% view(3);
% grid on;
