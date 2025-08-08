% MATLAB script to process ALL CSV files in the current folder
% and plot v0_spd - v1_spd and v0_dist - v1_dist for each file.
% The legend will show only the trailing part of the filename
% (e.g., "_r2_1.csv" -> "r2_1", "_r2A.csv" -> "r2A").

% Get list of all CSV files in the folder
files = dir('*.csv');

% Prepare figure
figure;

% --- Subplot 1: Speed Difference ---
subplot(2,1,1); hold on;
title('v0\_spd - v1\_spd Over Time');
ylabel('Speed Difference (m/s)');
grid on;

% --- Subplot 2: Distance Difference ---
subplot(2,1,2); hold on;
title('v0\_dist - v1\_dist Over Time');
xlabel('Simulation Time (sec)');
ylabel('Distance Difference (m)');
grid on;

% Loop through each file
for i = 1:length(files)
    % Read CSV file
    data = readtable(files(i).name);
    
    % Extract required columns
    time = data.SimTime_sec_;
    spd_diff = data.v0_spd_m_s_ - data.v1_spd_m_s_;
    dist_diff = data.v0_dist_m_ - data.v1_dist_m_;
    
    % Extract label from filename (remove .csv and everything before last underscore)
    [~, name, ~] = fileparts(files(i).name);
    underscoreIdx = strfind(name, '_');
    if ~isempty(underscoreIdx)
        label = name(underscoreIdx(end)+1:end); % everything after last underscore
    else
        label = name; % if no underscore, just use full name
    end
    
    % Plot on respective subplots
    subplot(2,1,1);
    plot(time, spd_diff, 'LineWidth', 1.2, 'DisplayName', label);
    
    subplot(2,1,2);
    plot(time, dist_diff, 'LineWidth', 1.2, 'DisplayName', label);
end

% Add legends after all plots are added
subplot(2,1,1); legend('show', 'Interpreter', 'none');
subplot(2,1,2); legend('show', 'Interpreter', 'none');

sgtitle('Comparison of v0-v1 Speed and Distance Differences Across All CSV Files');
