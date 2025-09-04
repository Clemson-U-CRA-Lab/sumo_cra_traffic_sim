clc;
close all;
clear all;


FIGPOS1 = [300,400,550,450];
FIGPOS2 = [500,200,550,600];
FIGPOS3 = [500,200,550,400];
FIGPOS4 = [500,200,1050,500];
fontSize = 12;
fontSize_subtiles = 12;




dataDir = 'd:\git_repos\sumo_cra_traffic_sim\data\2025_08_31\';

% Define your labeled files and groupings (all groups have 4: pred, prev, fbStop, fbCarry)
fileStruct = struct( ...
    'label', {
        'pred', 'prev', 'fbStop', 'fbCarry', ...
        'pred_2_22', 'prev_2_22', 'fbStop_2_22', 'fbCarry_2_22', ...
        'pred_2_19', 'prev_2_19', 'fbStop_2_19', 'fbCarry_2_19', ...
        'pred_2_20', 'prev_2_20', 'fbStop_2_20', 'fbCarry_2_20', ...
        'pred_1_20', 'prev_1_20', 'fbStop_1_20', 'fbCarry_1_20', ...
        'pred_0p5_20', 'prev_0p5_20', 'fbStop_0p5_20', 'fbCarry_0p5_20', ...
        'pred_2_10', 'prev_2_10', 'fbStop_2_10', 'fbCarry_2_10', ...
        'pred_1p5_10', 'prev_1p5_10', 'fbStop_1p5_10', 'fbCarry_1p5_10', ...
        'pred_1_10', 'prev_1_10', 'fbStop_1_10', 'fbCarry_1_10', ...
        'pred_0p75_10', 'prev_0p75_10', 'fbStop_0p75_10', 'fbCarry_0p75_10', ...
        'pred_0p5_10', 'prev_0p5_10', 'fbStop_0p5_10', 'fbCarry_0p5_10', ...
        'pred_0p2_10', 'prev_0p2_10', 'fbStop_0p2_10', 'fbCarry_0p2_10', ...
        'pred_0p1_10', 'prev_0p1_10', 'fbStop_0p1_10', 'fbCarry_0p1_10'
    }, ...
    'file', {
        'r2_sum_log_2025_08_31-02_11_09_PM.csv', 'r4_sum_log_2025_08_31-02_20_27_PM.csv', ...
                            'r4_sum_log_2025_08_31-02_20_27_PM.csv', 'r4_sum_log_2025_08_31-02_20_27_PM.csv', ...
        'r28_sumo_log_2025_08_31-04_34_59_PM.csv', 'r29_sumo_log_2025_08_31-04_37_50_PM.csv', ...
                            '..\2025_09_01\r30_sumo_log_2025_09_01-07_05_40_PM.csv', '..\2025_09_01\r18_sumo_log_2025_09_01-06_31_54_PM.csv', ...
        'r30_sumo_log_2025_08_31-04_42_17_PM.csv', 'r31_sumo_log_2025_08_31-04_45_10_PM.csv', ...
                            '..\2025_09_01\r31_sumo_log_2025_09_01-07_08_25_PM.csv', '..\2025_09_01\r19_sumo_log_2025_09_01-06_34_53_PM.csv', ...
        'r7_sumo_log_2025_08_31-02_35_41_PM.csv', '..\2025_09_01\r1_sumo_log_2025_09_01-05_26_29_PM.csv', ...
                            '..\2025_09_01\r27_sumo_log_2025_09_01-06_57_24_PM.csv', '..\2025_09_01\r15_sumo_log_2025_09_01-06_20_34_PM.csv', ...
        'r8_sumo_log_2025_08_31-02_38_42_PM.csv', '..\2025_09_01\r2_sumo_log_2025_09_01-05_30_22_PM.csv', ...
                            '..\2025_09_01\r28_sumo_log_2025_09_01-07_00_20_PM.csv', '..\2025_09_01\r16_sumo_log_2025_09_01-06_26_12_PM.csv', ...
        'r9_sumo_log_2025_08_31-02_41_53_PM.csv', '..\2025_09_01\r3_sumo_log_2025_09_01-05_34_09_PM.csv', ...
                            '..\2025_09_01\r29_sumo_log_2025_09_01-07_02_53_PM.csv', '..\2025_09_01\r17_sumo_log_2025_09_01-06_28_56_PM.csv', ...
        'r16_sumo_log_2025_08_31-03_24_43_PM.csv', 'r15_sumo_log_2025_08_31-03_04_24_PM.csv', ...
                        '..\2025_09_01\r26_sumo_log_2025_09_01-06_54_43_PM.csv', '..\2025_09_01\r14_sumo_log_2025_09_01-06_17_52_PM.csv', ...
        '..\2025_09_01\r6_sumo_log_2025_09_01-05_43_36_PM.csv' , '..\2025_09_01\r7_sumo_log_2025_09_01-05_46_45_PM.csv', ...
                         '..\2025_09_01\r25_sumo_log_2025_09_01-06_51_44_PM.csv', '..\2025_09_01\r13_sumo_log_2025_09_01-06_15_06_PM.csv', ...
        'r19a_sumo_log_2025_08_31-03_50_36_PM.csv', 'r21_sumo_log_2025_08_31-03_59_23_PM.csv', ...
                        '..\2025_09_01\r24_sumo_log_2025_09_01-06_48_58_PM.csv', '..\2025_09_01\r12_sumo_log_2025_09_01-06_12_24_PM.csv', ...
        '..\2025_09_01\r4_sumo_log_2025_09_01-05_38_17_PM.csv', '..\2025_09_01\r5_sumo_log_2025_09_01-05_40_57_PM.csv', ...
                        '..\2025_09_01\r23_sumo_log_2025_09_01-06_46_14_PM.csv', '..\2025_09_01\r11_sumo_log_2025_09_01-06_09_20_PM.csv', ...
        'r20_sumo_log_2025_08_31-03_56_17_PM.csv', 'r23_sumo_log_2025_08_31-04_05_24_PM.csv', ...
                        '..\2025_09_01\r22_sumo_log_2025_09_01-06_43_26_PM.csv', '..\2025_09_01\r10_sumo_log_2025_09_01-06_04_57_PM.csv', ...
        'r25_sumo_log_2025_08_31-04_25_19_PM.csv', 'r24_sumo_log_2025_08_31-04_21_15_PM.csv', ...
                        '..\2025_09_01\r21_sumo_log_2025_09_01-06_40_24_PM.csv', '..\2025_09_01\r9_sumo_log_2025_09_01-06_02_01_PM.csv', ...
        'r27_sumo_log_2025_08_31-04_30_25_PM.csv', 'r26_sumo_log_2025_08_31-04_28_01_PM.csv', ...
                        '..\2025_09_01\r20_sumo_log_2025_09_01-06_37_34_PM.csv', '..\2025_09_01\r8_sumo_log_2025_09_01-05_59_02_PM.csv'} ...
    );

% Grouping for plotting (cell arrays of labels for each group, all groups have 4)
groups = {
    {'pred', 'prev', 'fbStop', 'fbCarry'},
    {'pred_2_22', 'prev_2_22', 'fbStop_2_22', 'fbCarry_2_22'},
    {'pred_2_19', 'prev_2_19', 'fbStop_2_19', 'fbCarry_2_19'},
    {'pred_2_20', 'prev_2_20', 'fbStop_2_20', 'fbCarry_2_20'},
    {'pred_1_20', 'prev_1_20', 'fbStop_1_20', 'fbCarry_1_20'},
    {'pred_0p5_20', 'prev_0p5_20', 'fbStop_0p5_20', 'fbCarry_0p5_20'},
    {'pred_2_10', 'prev_2_10', 'fbStop_2_10', 'fbCarry_2_10'},
    {'pred_1p5_10', 'prev_1p5_10', 'fbStop_1p5_10', 'fbCarry_1p5_10'},
    {'pred_1_10', 'prev_1_10', 'fbStop_1_10', 'fbCarry_1_10'},
    {'pred_0p75_10', 'prev_0p75_10', 'fbStop_0p75_10', 'fbCarry_0p75_10'},
    {'pred_0p5_10', 'prev_0p5_10', 'fbStop_0p5_10', 'fbCarry_0p5_10'},
    {'pred_0p2_10', 'prev_0p2_10', 'fbStop_0p2_10', 'fbCarry_0p2_10'},
    {'pred_0p1_10', 'prev_0p1_10', 'fbStop_0p1_10', 'fbCarry_0p1_10'}
};

groupNames = {'No Attack', '2s/22s', '2s/19s', '2s/20s', '1s/20s',  '0.5s/20s', '2s/10s', '1.5s/10s', '1s/10s',  '0.75s/10s', '0.5s/10s', '0.2s/10s', '0.1s/10s'};

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
rms_spd_grouped = nan(length(groups),4); % 2 columns: pred, prev
rms_dist_grouped = nan(length(groups),4);

for g = 1:length(groups)
    for k = 1:4
        idx = find(strcmp({fileStruct.label}, groups{g}{k}));
        if ~isempty(idx)
            rms_spd_grouped(g,k) = rms_spd(idx);
            rms_dist_grouped(g,k) = rms_dist(idx);
            min_dist_gap_grouped(g,k) = min_dist_gap(idx);
            collisions_grouped(g,k) = collisions(idx);
        end
    end
end


legendGroup = {'w/o intention', 'w/ intention', 'w/ detect-Stop', 'w/ detect Carry'} ;

%%

% Plot grouped bar charts
figure(10);
subplot(2,1,1);
bar(rms_spd_grouped);
set(gca, 'XTickLabel', groupNames, 'XTick', 1:numel(groupNames), 'XTickLabelRotation', 45, 'FontSize', 12);
ylabel('RMS Speed Difference (m/s)', 'FontSize', 12);
title('RMS Speed Difference by Delay/Attack Group', 'FontSize', 14, 'FontWeight', 'bold');
grid on;

subplot(2,1,2);
bar(rms_dist_grouped);
set(gca, 'XTickLabel', groupNames, 'XTick', 1:numel(groupNames), 'XTickLabelRotation', 45, 'FontSize', 12);
ylabel('RMS Distance Difference (m)', 'FontSize', 12);
legend(legendGroup, 'Location', 'northwest');
title('RMS Distance Difference by Delay/Attack Group', 'FontSize', 14, 'FontWeight', 'bold');
grid on;



% Calculate percent change in RMS speed and RMS distance compared to No Attack group
no_attack_spd = rms_spd_grouped(1,:);
no_attack_dist = rms_dist_grouped(1,:);
percent_change_spd = 100 * (rms_spd_grouped - no_attack_spd) ./ no_attack_spd;
percent_change_dist = 100 * (rms_dist_grouped - no_attack_dist) ./ no_attack_dist;

f1 = figure('DefaultAxesFontsize', fontSize);
t1 = tiledlayout(2,1, "TileSpacing","tight","Padding","tight");

nexttile
% yline(10,"-","LineWidth",1.5,"Color",'k')
hold on
bar(percent_change_spd);
set(gca, 'YScale', 'log');
set(gca, 'XTickLabel', groupNames, 'XTick', 1:numel(groupNames), 'XTickLabelRotation', 45);
ylabel('Percent Change in RMS Speed (%)');
title('Percent Change in RMS Speed vs No Attack', 'FontSize', fontSize_subtiles, 'FontWeight', 'bold');
hold off
grid on;
box on
ax1 = gca;

nexttile    
bar(percent_change_dist);
set(gca, 'YScale', 'log');
set(gca, 'XTickLabel', groupNames, 'XTick', 1:numel(groupNames), 'XTickLabelRotation', 45);
ylabel('Percent Change in RMS Distance (%)');
title('Percent Change in RMS Distance vs No Attack', 'FontSize', fontSize_subtiles, 'FontWeight', 'bold');
grid on;
ax2 = gca;

linkaxes([ax2, ax1],'x')
set(gcf,'position',FIGPOS1)
set ( gcf, 'Color', 'white')

%%

f2 = figure('DefaultAxesFontsize', fontSize);
t2 = tiledlayout(3,1, "TileSpacing","tight","Padding","tight");

nexttile
bar(min_dist_gap_grouped);
xticklabels('');
% set(gca, 'XTickLabel', groupNames, 'XTick', 1:numel(groupNames), 'XTickLabelRotation', 45);
ylabel('Lowest Distance Gap (m)', 'FontSize', 12);
title('Lowest Distance Gap by Delay/Attack Group', 'FontSize', fontSize_subtiles, 'FontWeight', 'bold');
grid on;
ax1 = gca;


nexttile
plot(min_dist_gap_grouped, '-o', 'LineWidth', 2);
xticklabels('');
hold on;
yline(0, "LineWidth",1, "LineStyle","--","Color","k")
% set(gca, 'XTickLabel', groupNames, 'XTick', 1:numel(groupNames), 'XTickLabelRotation', 45);
ylabel('Lowest Distance Gap (m)', 'FontSize', 12);
grid on;
ax2 = gca;

nexttile
% bar(collisions_grouped);
plot(collisions_grouped, '-o', 'LineWidth', 2);

set(gca, 'XTickLabel', groupNames, 'XTick', 1:numel(groupNames), 'XTickLabelRotation', 45);
ylabel('Number of Collisions (dist gap <= 0.1 m)');
title('Collisions', 'FontSize', fontSize_subtiles, 'FontWeight', 'bold');
grid on;
legend(legendGroup, 'Location', 'northeast' );
ax3 = gca;


linkaxes([ax2, ax1, ax3],'x')
set(gcf,'position',FIGPOS2)
set ( gcf, 'Color', 'white' )

    
    % lgd2 = legend({'MPC', 'AC (PPO)', 'AC^2MPC', 'AC^3MPC'},...
    %     'NumColumns', 4, Location="southoutside", Orientation="horizontal");
    % fontsize(lgd2,10,'points')

%%
% Example: Plot ego and front vehicle speed, acceleration, and position for the first scenario
% You can change idx to plot other scenarios
idx = 1; % index in fileStruct to plot


f3 = figure('DefaultAxesFontsize', fontSize);
t3 = tiledlayout(2,1, "TileSpacing","tight","Padding","tight");
set(f3,'position',FIGPOS3)

nexttile;

for idx = [1,2]
    data = readtable(fullfile(dataDir, fileStruct(idx).file));
    N = height(data);
    time = (0:N-1)';
    
    plot(time, data.v0_spd_m_s_, '-b', 'LineWidth', 1.5); hold on;
    plot(time, data.v1_spd_m_s_, '-', 'LineWidth', 1.5);
end
ylabel('Speed (m/s)');
legend({'Front Vehicle','Ego Vehicle'},'Location','best');
title('Speed vs Time', 'FontSize', fontSize_subtiles, 'FontWeight', 'bold');
grid on;

nexttile;
for idx = [1,2]
    data = readtable(fullfile(dataDir, fileStruct(idx).file));
    N = height(data);
    time = (0:N-1)';
    plot(time, data.v0_dist_m_, '-b', 'LineWidth', 1.5); hold on;
    plot(time, data.v1_dist_m_, '-', 'LineWidth', 1.5);
end

ylabel('Position (m)');
legend({'Front Vehicle','Ego Vehicle'},'Location','best');
title('Position vs Time', 'FontSize', fontSize_subtiles, 'FontWeight', 'bold');
grid on;

set ( gcf, 'Color', 'white' )


saveas(f1,'percents.png')
saveas(f2,'gaps.png')
saveas(f3,'cycle.png')


