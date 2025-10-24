clear
clc

% ---- Serial Setup ----
port = "COM3"; % Adjust to your Arduino port
baudrate = 9600;
s = serialport(port, baudrate);
configureTerminator(s, "LF");
flush(s);

% ---- Logging Setup ----
fs = 2; % Hz
data = []; % growable array
startT = tic;
i = 1;

sens_protocol = {'0 0 121 0'};  % test each nozzle at 121, 154, 169 to test sensor accuracy; run each test 5x 
cl_protocol = {'130 130 130 130', '100 100 160 160', '90 110 150 180', '170 140 130 100'};
% cl_protocol = {'130 130 130 130', '100 100 160 160', '90 120 150 180', '170 130 140 100'};
% cl_protocol = {'130 130 130 130', '130 120 130 140', '130 110 130 150', '130 100 130 160', '130 90 130 170', '130 80 130 180'}; % banked

% protocol = sens_protocol;
% pauseBtwnCommands = 90; % second s between each command
protocol = cl_protocol;
pauseBtwnCommands = 30; % seconds between each command

% % ---- Command Box GUI ----
% fig = figure('Name', 'Set Desired Flows', 'NumberTitle', 'off', 'Position', [500, 500, 300, 100]);
% commandBox = uicontrol(fig, 'Style', 'edit', 'Position', [10, 40, 280, 30], 'String', '');    
% uicontrol(fig, 'Style', 'text', 'Position', [10, 70, 280, 20], 'String', 'Enter [dfr1 dfr2 dfr3 dfr4] and press Enter');
% set(commandBox, 'Callback', @(src, event) setappdata(fig, 'command', get(src, 'String')));

disp("Logging nozzle flow rates...");
% disp("Use GUI to send flow rate commands like: 0 0 100 100");
disp("Running protocol...");

pause(15); % wait 15s before sending first command
% ---- Logging Loop ----
for p = 1:length(protocol)
    % Send current protocol command
    cmd = protocol{p};
    writeline(s, cmd);
    disp(['Sent: ', cmd]);

    tStart = tic;
    while toc(tStart) < pauseBtwnCommands
        % --- Read Arduino Line ---
        if s.NumBytesAvailable > 0
            line = readline(s);
            vals = sscanf(line, '%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f'); % 16 values: pump, dfr, flow, valve...
            if length(vals) == 16
                t = toc(startT);
                data(i,:) = [t, vals(:)'];
                i = i+1;
            else
                disp("Malformed line: " + line);
            end
        end
        pause(1/fs);
    end
end
writeline(s, '0 0 0 0'); % turn off pumps and valves before saving data 
disp(['Turning Pumps Off']);
pause(15);
            
% while isvalid(fig)
%     % --- Handle GUI command input ---
%     if isappdata(fig, 'command')
%         userInput = getappdata(fig, 'command');
%         rmappdata(fig, 'command');
%         if strcmpi(userInput, 'exit')
%             break;
%         elseif ~isempty(userInput)
%             writeline(s, userInput);
%             disp(['Sent: ', userInput]);
%         end
%     end
% 
%     % --- Read Arduino line ---
%     if s.NumBytesAvailable > 0
%         line = readline(s);
%         vals = sscanf(line, '%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f'); % 16 values: pump, dfr, flow, valve...
%         if length(vals) == 16
%             t = toc(startT);
%             data(i, :) = [t, vals(:)'];
%             i = i + 1;
%         else
%             disp("Malformed line: " + line);
%         end
%     end
% 
%     pause(1/fs);
% end

% ---- Close Serial and GUI ----
% if isvalid(fig), close(fig); end
clear s;

% ---- Save CSV ----
timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
% filename = sprintf("closedLoopTest_%s_banked.csv", timestamp);
filename = sprintf("closedLoopTest_%s_1-to-1.csv", timestamp);
% filename = sprintf("closedLoopTest_sensorValidation_%s.csv", timestamp);

headers = {'Time_s', 'Pump1_Pwr', 'Dfr1', 'Flow1_Lph', 'Valve1_Pos', ...
    'Pump2_Pwr', 'Dfr2', 'Flow2_Lph', 'Valve2_Pos', ...
    'Pump3_Pwr', 'Dfr3', 'Flow3_Lph', 'Valve3_Pos', ...
    'Pump4_Pwr', 'Dfr4', 'Flow4_Lph', 'Valve4_Pos'};
fid = fopen(filename, 'w');
fprintf(fid, '%s,', headers{1:end-1});
fprintf(fid, '%s\n', headers{end});
fclose(fid);

writematrix(data, filename, 'WriteMode', 'append');

disp(['Saved flow + setpoint log to ', filename]);
