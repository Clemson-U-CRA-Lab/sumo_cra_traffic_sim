clc;
close all;
clear all;

dataDir = 'd:\git_repos\sumo_cra_traffic_sim\data\2025_08_31\';

% Define your labeled files and groupings
fileStruct = struct( ...
    'label', {'pred', 'prev', ...
                'pred_2_22', 'prev_2_22', ...
              'pred_2_19', 'prev_2_19', ...
              'pred_2_20', 'prev_2_20', ...
              'pred_1_20', 'prev_1_20', ...
              'pred_0p5_20', 'prev_0p5_20', ...
              'pred_2_10', 'prev_2_10', ...
              'pred_1_10', 'prev_1_10', ...
              'pred_0p5_10', 'prev_0p5_10', ...
              'pred_0p2_10', 'prev_0p2_10', ...
              'pred_0p1_10', 'prev_0p1_10'}, ...
    'file', {'r2_sum_log_2025_08_31-02_11_09_PM.csv', 'r4_sum_log_2025_08_31-02_20_27_PM.csv', ...
            'r28_sumo_log_2025_08_31-04_34_59_PM.csv', 'r29_sumo_log_2025_08_31-04_37_50_PM.csv', ...
             'r30_sumo_log_2025_08_31-04_42_17_PM.csv', 'r31_sumo_log_2025_08_31-04_45_10_PM.csv', ...
             'r7_sumo_log_2025_08_31-02_35_41_PM.csv', 'r5_sum_log_2025_08_31-02_24_09_PM.csv', ...
             'r8_sumo_log_2025_08_31-02_38_42_PM.csv', 'r10_sumo_log_2025_08_31-02_45_34_PM.csv', ...
             'r9_sumo_log_2025_08_31-02_41_53_PM.csv', 'r11_sumo_log_2025_08_31-02_48_47_PM.csv', ...
             'r16_sumo_log_2025_08_31-03_24_43_PM.csv', 'r15_sumo_log_2025_08_31-03_04_24_PM.csv', ...
             'r19a_sumo_log_2025_08_31-03_50_36_PM.csv', 'r21_sumo_log_2025_08_31-03_59_23_PM.csv', ...
             'r20_sumo_log_2025_08_31-03_56_17_PM.csv', 'r23_sumo_log_2025_08_31-04_05_24_PM.csv', ...
             'r25_sumo_log_2025_08_31-04_25_19_PM.csv', 'r24_sumo_log_2025_08_31-04_21_15_PM.csv', ...
             'r27_sumo_log_2025_08_31-04_30_25_PM.csv', 'r26_sumo_log_2025_08_31-04_28_01_PM.csv'} ...
);

% Grouping for plotting (cell arrays of labels for each group)
groups = { ...
    {'pred', 'prev'}, ...
    {'pred_2_22', 'prev_2_22'}, ...
    {'pred_2_19', 'prev_2_19'}, ...
    {'pred_2_20', 'prev_2_20'}, ...
    {'pred_1_20', 'prev_1_20'}, ...
    {'pred_0p5_20', 'prev_0p5_20'}, ...
    {'pred_2_10', 'prev_2_10'}, ...
    {'pred_1_10', 'prev_1_10'}, ...
    {'pred_0p5_10', 'prev_0p5_10'}, ...
    {'pred_0p2_10', 'prev_0p2_10'}, ...
    {'pred_0p1_10', 'prev_0p1_10'} ...
};

groupNames = {'No Attack', '2s/22s', '2s/19s', '2s/20s', '1s/20s',  '0.5s/20s', '2s/10s', '1s/10s', '0.5s/10s', '0.2s/10s', '0.1s/10s'};

rms_spd = nan(length(fileStruct),1);
rms_dist = nan(length(fileStruct),1);

for i = 1:length(fileStruct)
    data = readtable(fullfile(dataDir, fileStruct(i).file));
    for j = 1:30
        if data.v1_dist_m_(j) > 20
            data.v1_dist_m_(j) = 0;
        end
        data.v0_dist_m_(j) = 20;
    end
    spd_diff = data.v0_spd_m_s_ - data.v1_spd_m_s_;
    dist_diff = data.v0_dist_m_ - data.v1_dist_m_ - 3.25;
    rms_spd(i) = sqrt(mean(spd_diff.^2));
    rms_dist(i) = sqrt(mean(dist_diff.^2));
    % Lowest distance gap
    min_dist_gap(i) = min(dist_diff);
    % Number of unique collisions (contiguous regions where dist_diff <= 0.1)
    collision_mask = dist_diff <= 0.5;
    d_mask = diff([0; collision_mask(:)]);
    collisions(i) = sum(d_mask == 1);
end

% Prepare data for grouped bar chart
rms_spd_grouped = nan(length(groups),2); % 2 columns: pred, prev
rms_dist_grouped = nan(length(groups),2);

for g = 1:length(groups)
    for k = 1:2
        idx = find(strcmp({fileStruct.label}, groups{g}{k}));
        if ~isempty(idx)
            rms_spd_grouped(g,k) = rms_spd(idx);
            rms_dist_grouped(g,k) = rms_dist(idx);
            min_dist_gap_grouped(g,k) = min_dist_gap(idx);
            collisions_grouped(g,k) = collisions(idx);
        end
    end
end


legendGroup = {'w/o intention', 'w/ intention'} ;

% Plot grouped bar charts
figure;
subplot(2,1,1);
bar(rms_spd_grouped);
set(gca, 'XTickLabel', groupNames, 'XTick', 1:numel(groupNames), 'XTickLabelRotation', 45);
ylabel('RMS Speed Difference (m/s)');
legend(legendGroup, 'Location', 'northwest');
title('RMS Speed Difference by Delay/Attack Group');
grid on;

subplot(2,1,2);
bar(rms_dist_grouped);
set(gca, 'XTickLabel', groupNames, 'XTick', 1:numel(groupNames), 'XTickLabelRotation', 45);
ylabel('RMS Distance Difference (m)');
legend(legendGroup, 'Location', 'northwest');
title('RMS Distance Difference by Delay/Attack Group');
grid on;

figure;
subplot(4,1,3);
bar(min_dist_gap_grouped);
set(gca, 'XTickLabel', groupNames, 'XTick', 1:numel(groupNames), 'XTickLabelRotation', 45);
ylabel('Lowest Distance Gap (m)');
legend(legendGroup, 'Location', 'northwest');
title('Lowest Distance Gap by Delay/Attack Group');
grid on;


subplot(4,1,4);
bar(collisions_grouped);
set(gca, 'XTickLabel', groupNames, 'XTick', 1:numel(groupNames), 'XTickLabelRotation', 45);
ylabel('Number of Collisions (dist gap <= 0.1 m)');
legend(legendGroup, 'Location', 'northwest');
title('Collisions by Delay/Attack Group');
grid on;


% Calculate percent change in RMS speed and RMS distance compared to No Attack group
no_attack_spd = rms_spd_grouped(1,:);
no_attack_dist = rms_dist_grouped(1,:);
percent_change_spd = 100 * (rms_spd_grouped - no_attack_spd) ./ no_attack_spd;
percent_change_dist = 100 * (rms_dist_grouped - no_attack_dist) ./ no_attack_dist;

% Plot percent change in RMS speed
figure(gcf);
subplot(4,1,1);
bar(percent_change_spd);
set(gca, 'XTickLabel', groupNames, 'XTick', 1:numel(groupNames), 'XTickLabelRotation', 45);
ylabel('Percent Change in RMS Speed (%)');
legend(legendGroup, 'Location', 'northwest');
title('Percent Change in RMS Speed vs No Attack');
grid on;

% Plot percent change in RMS distance
subplot(4,1,2);
bar(percent_change_dist);
set(gca, 'XTickLabel', groupNames, 'XTick', 1:numel(groupNames), 'XTickLabelRotation', 45);
ylabel('Percent Change in RMS Distance (%)');
legend(legendGroup, 'Location', 'northwest');
title('Percent Change in RMS Distance vs No Attack');
grid on;


sgtitle('RMS Speed, Distance, Min Gap, and Collisions Grouped by Delay/Attack');