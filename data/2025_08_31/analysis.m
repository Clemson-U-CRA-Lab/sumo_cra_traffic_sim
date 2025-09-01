clc;
close all;
clear all;
% Directory containing the CSV files
dataDir = 'd:\git_repos\sumo_cra_traffic_sim\data\2025_08_31\';


pred = readtable("r2_sum_log_2025_08_31-02_11_09_PM.csv");
prev = readtable("r4_sum_log_2025_08_31-02_20_27_PM.csv");

% 20 sec
pred_2_20 = readtable("r7_sumo_log_2025_08_31-02_35_41_PM.csv");
prev_2_20 = readtable("r5_sum_log_2025_08_31-02_24_09_PM.csv");

pred_1_20 = readtable("r8_sumo_log_2025_08_31-02_38_42_PM.csv");
prev_1_20 = readtable("r10_sumo_log_2025_08_31-02_45_34_PM.csv");

pred_0p5_20 = readtable("r9_sumo_log_2025_08_31-02_41_53_PM.csv");
prev_0p5_20 = readtable("r11_sumo_log_2025_08_31-02_48_47_PM.csv");

% 10 sec
pred_2_10 = readtable("r16_sumo_log_2025_08_31-03_24_43_PM.csv");
prev_2_10 = readtable("r15_sumo_log_2025_08_31-03_04_24_PM.csv");

pred_1_10 = readtable("r19a_sumo_log_2025_08_31-03_50_36_PM.csv");
prev_1_10 = readtable("r21_sumo_log_2025_08_31-03_59_23_PM.csv");

pred_0p5_10 = readtable("r20_sumo_log_2025_08_31-03_56_17_PM.csv");
prev_0p5_10 = readtable("r23_sumo_log_2025_08_31-04_05_24_PM.csv");

pred_0p2_10 = readtable("r25_sumo_log_2025_08_31-04_25_19_PM.csv");
prev_0p2_10 = readtable("r24_sumo_log_2025_08_31-04_21_15_PM.csv");

pred_0p1_10 = readtable("r27_sumo_log_2025_08_31-04_30_25_PM.csv");
prev_0p1_10 = readtable("r26_sumo_log_2025_08_31-04_28_01_PM.csv");


% for i = 1:length(pred_no_attack)
%     files = dir(fullfile(dataDir, [pred_no_attack{i} '_*.csv']));
% end

%%
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

for i = 1:length(files)
    data = readtable(files(i).name);

    time = data.SimTime_sec_;
    spd_diff = data.v0_spd_m_s_ - data.v1_spd_m_s_;
    dist_diff = data.v0_dist_m_ - data.v1_dist_m_;

    % RMS calculations
    rms_spd = sqrt(mean(spd_diff.^2));
    rms_dist = sqrt(mean(dist_diff.^2));

    % Extract label from filename (remove .csv and everything before last underscore)
    [~, name, ~] = fileparts(files(i).name);
    underscoreIdx = strfind(name, '_');
    if ~isempty(underscoreIdx)
        label = name(underscoreIdx(end)+1:end);
    else
        label = name;
    end

    % Plot with RMS in legend
    subplot(2,1,1);
    plot(time, spd_diff, 'LineWidth', 1.2, 'DisplayName', sprintf('%s | RMS=%.2f', label, rms_spd));

    subplot(2,1,2);
    plot(time, dist_diff, 'LineWidth', 1.2, 'DisplayName', sprintf('%s | RMS=%.2f', label, rms_dist));
end

subplot(2,1,1); legend('show', 'Interpreter', 'none');
subplot(2,1,2); legend('show', 'Interpreter', 'none');
sgtitle('Comparison of v0-v1 Speed and Distance Differences (RMS Shown)');