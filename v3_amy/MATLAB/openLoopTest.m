clear
clc

% ---- Serial Setup ----
port = "COM3"; % adjust as needed
baudrate = 9600;
serialObj = serialport(port, baudrate);

configureTerminator(serialObj, "LF");
flush(serialObj);

pause(2); % allow Arduino to reset after connection
writeline(serialObj, "START"); % tell Arduino to begin
disp('Sent START to Arduino, beginning data collection...');

% ---- Logging Setup ----
maxSamples = 5000;   % safeguard
data = zeros(maxSamples, 12);
timestamps = zeros(maxSamples, 1);

timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
disp('Collecting data only (no commands sent)...');

% ---- Stop Button GUI ----
stopFig = figure('Name', 'Data Logger', 'NumberTitle', 'off', ...
    'Position', [500 500 200 100], 'MenuBar', 'none', 'ToolBar', 'none');
uicontrol(stopFig, 'Style', 'pushbutton', 'String', 'STOP LOGGING', ...
    'FontSize', 12, 'Position', [40 30 120 40], ...
    'Callback', @(src, event) setappdata(stopFig, 'stopNow', true));
setappdata(stopFig, 'stopNow', false);

% ---- Data Collection Loop ----
i = 1;
startTime = tic;

while i <= maxSamples && ~getappdata(stopFig, 'stopNow')
    try
        if serialObj.NumBytesAvailable > 0
            line = readline(serialObj);
            values = str2double(strsplit(strtrim(line), ","));
            if length(values) == 12
                data(i,:) = values;
                timestamps(i) = toc(startTime);
                i = i + 1;
            end
        end
    catch
        disp('Read error, skipping...');
    end
    pause(0.01); % throttle loop slightly
end

% ---- Cleanup ----
clear serialObj
if isvalid(stopFig), close(stopFig); end

% Trim unused rows
data = data(1:i-1,:);
timestamps = timestamps(1:i-1);

% ---- Save to CSV ----
csv_filename = sprintf('openLoopNozzleTest_%s.csv', timestamp);
headers = {'Time (s)', 'Pump1','Valve1','Flow1',...
           'Pump2','Valve2','Flow2',...
           'Pump3','Valve3','Flow3',...
           'Pump4','Valve4','Flow4'};

fid = fopen(csv_filename,'w');
fprintf(fid, '%s,', headers{1:end-1});
fprintf(fid, '%s\n', headers{end});
fclose(fid);

writematrix([timestamps data], csv_filename, 'WriteMode','append');
disp(['Data saved to ', csv_filename]);
