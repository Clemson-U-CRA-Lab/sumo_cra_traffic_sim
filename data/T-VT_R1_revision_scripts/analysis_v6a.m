%% analysis_v6a.m — TVT R1 performance plots (2veh + 3veh overlay)
%
% Usage:
%   1. Set VEH2_SOURCE, VEH3_SOURCE, SPD_YSCALE below.
%   2. Run this script.
%   3. Figures save to ./plots_performance/
%
% VEH2_SOURCE (2veh):
%   'hil'     — legacy 2025_08_31 / 2025_09_01 HIL CSVs
%   'sim2026' — data/2026_08_05 v2x_2veh_* sim sweeps
%   'hil2026' — data/2026_08_06 v2x_2veh_* HIL (UDP) sweeps
%
% VEH3_SOURCE (3veh, CAV=nv1 middle; overlay = dashed):
%   'sim2026' — data/2026_08_05 v2xconfig_* / v2x_FbStop_*
%   'hil2026' — data/2026_08_06 v2xconfig_*
%
% NOMINAL_MODE / NOMINAL_CTRL: shared baseline vs each folder's own r0.
% Attack groups share one axis via x2v_automate.yaml run ids.
% Plots: effort, speed (RMS + %δ), and performance (speed %δ + lowest gap).

clc; close all; clear all;

%% ===== User knobs =====
SPD_YSCALE  = 'linear';      % 'linear' | 'log' | 'kink'
VEH2_SOURCE = 'hil2026';  % 'hil' | 'sim2026' | 'hil2026'
VEH3_SOURCE = 'hil2026';  % 'sim2026' | 'hil2026'

% NOMINAL_MODE:
%   'shared'     — one baseline (2veh NOMINAL_CTRL No-Attack) for every %δ
%   'per_folder' — each controller uses its own folder r0 / No-Attack row
NOMINAL_MODE = 'per_folder';   % 'shared' | 'per_folder'
NOMINAL_CTRL = 'fbStop';   % used when NOMINAL_MODE='shared': 'pred' | 'prev' | 'fbStop'

% Kink y-axis (only when SPD_YSCALE='kink'): each row is a physical %δv band
% shown at full display height. Gaps between rows are compressed.
% Tweak these to change which ranges are expanded visually.
SPD_KINK_BANDS = [ ...
    -10   10; ...
    70  100; ...
   ];
SPD_KINK_GAP = 0.12;   % display height of each compressed gap between bands
SPD_KINK_BAND_H = 1;   % display height of each shown band

%% ===== Style =====
FIGPOS_2plots  = [300, 400, 550, 450];
fontSize       = 12;
fontSize_title = 12;
fontSize_legend = 9;

controllers         = {'pred', 'prev', 'fbStop'};
controllers_to_plot = 3;
legendCtrl = {'PCF', 'PCF-I', 'PCF-IDA'};
legendAvg  = [strcat(legendCtrl, ' (n=2)'), strcat(legendCtrl, ' (n=3; avg)')];

axColors   = [0.0000 0.4470 0.7410; 0.8500 0.3250 0.0980; 0.9290 0.6940 0.1250];
lineStyles = {'-', '--'};  % 2veh solid, 3veh dashed

%% ===== Paths =====
scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir), scriptDir = pwd; end
dataRoot  = fileparts(scriptDir);  % .../data
plotDir   = fullfile(scriptDir, 'plots_performance');
if ~isfolder(plotDir), mkdir(plotDir); end

expSim = fullfile(dataRoot, '2026_08_05');
expHil = fullfile(dataRoot, '2026_08_06');

ctrlDirs2sim = struct( ...
    'pred',   fullfile(expSim, 'v2x_2veh_NoPreview'), ...
    'prev',   fullfile(expSim, 'v2x_2veh_Preview'), ...
    'fbStop', fullfile(expSim, 'v2x_2veh_FbStop'));

ctrlDirs2hil = struct( ...
    'pred',   fullfile(expHil, 'v2x_2veh_NoPreview'), ...
    'prev',   fullfile(expHil, 'v2x_2veh_Preview'), ...
    'fbStop', fullfile(expHil, 'v2x_2veh_FbStop'));

% 3veh: sim folder for FbStop is v2x_FbStop_* (no "config" in name)
ctrlDirs3sim = struct( ...
    'pred',   fullfile(expSim, 'v2xconfig_NoPreview_20_0_-05'), ...
    'prev',   fullfile(expSim, 'v2xconfig_Preview_20_0_-05'), ...
    'fbStop', fullfile(expSim, 'v2x_FbStop_20_0_-05'));

ctrlDirs3hil = struct( ...
    'pred',   fullfile(expHil, 'v2xconfig_NoPreview_20_0_-05'), ...
    'prev',   fullfile(expHil, 'v2xconfig_Preview_20_0_-05'), ...
    'fbStop', fullfile(expHil, 'v2xconfig_FbStop_20_0_-05'));

switch lower(VEH3_SOURCE)
    case 'sim2026'
        ctrlDirs3 = ctrlDirs3sim;
        fprintf('3veh source: sim2026 (%s)\n', expSim);
    case 'hil2026'
        ctrlDirs3 = ctrlDirs3hil;
        fprintf('3veh source: hil2026 (%s)\n', expHil);
    otherwise
        error('VEH3_SOURCE must be ''sim2026'' or ''hil2026'', got %s', VEH3_SOURCE);
end

% groupNames order <-> yaml run id (run0 = no attack). yaml r10 (1.5/20) omitted.
groupNames = { ...
    'No Attack', '2s\22s', '2s\19s', '2s\20s', '1s\20s', '0.5s\20s', ...
    '2s\10s', '1.5s\10s', '1s\10s', '0.75s\10s', '0.5s\10s', '0.2s\10s', '0.1s\10s'};
runId_for_group = [0, 13, 12, 11, 9, 8, 7, 6, 5, 4, 3, 2, 1];
nGroups = numel(groupNames);

%% ===== Legacy 2veh HIL file map (2025) =====
% Each group: pred, prev, fbStop, fbCarry — only first 3 controllers plotted.
fileStruct2 = struct( ...
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
            '../2025_09_01/r30_sumo_log_2025_09_01-07_05_40_PM.csv', '../2025_09_01/r18_sumo_log_2025_09_01-06_31_54_PM.csv', ...
        'r30_sumo_log_2025_08_31-04_42_17_PM.csv', 'r31_sumo_log_2025_08_31-04_45_10_PM.csv', ...
            '../2025_09_01/r31_sumo_log_2025_09_01-07_08_25_PM.csv', '../2025_09_01/r19_sumo_log_2025_09_01-06_34_53_PM.csv', ...
        'r7_sumo_log_2025_08_31-02_35_41_PM.csv', '../2025_09_01/r1_sumo_log_2025_09_01-05_26_29_PM.csv', ...
            '../2025_09_01/r27_sumo_log_2025_09_01-06_57_24_PM.csv', '../2025_09_01/r15_sumo_log_2025_09_01-06_20_34_PM.csv', ...
        'r8_sumo_log_2025_08_31-02_38_42_PM.csv', '../2025_09_01/r2_sumo_log_2025_09_01-05_30_22_PM.csv', ...
            '../2025_09_01/r28_sumo_log_2025_09_01-07_00_20_PM.csv', '../2025_09_01/r16_sumo_log_2025_09_01-06_26_12_PM.csv', ...
        'r9_sumo_log_2025_08_31-02_41_53_PM.csv', '../2025_09_01/r3_sumo_log_2025_09_01-05_34_09_PM.csv', ...
            '../2025_09_01/r29_sumo_log_2025_09_01-07_02_53_PM.csv', '../2025_09_01/r17_sumo_log_2025_09_01-06_28_56_PM.csv', ...
        'r16_sumo_log_2025_08_31-03_24_43_PM.csv', 'r15_sumo_log_2025_08_31-03_04_24_PM.csv', ...
            '../2025_09_01/r26_sumo_log_2025_09_01-06_54_43_PM.csv', '../2025_09_01/r14_sumo_log_2025_09_01-06_17_52_PM.csv', ...
        '../2025_09_01/r6_sumo_log_2025_09_01-05_43_36_PM.csv', '../2025_09_01/r7_sumo_log_2025_09_01-05_46_45_PM.csv', ...
            '../2025_09_01/r25_sumo_log_2025_09_01-06_51_44_PM.csv', '../2025_09_01/r13_sumo_log_2025_09_01-06_15_06_PM.csv', ...
        'r19a_sumo_log_2025_08_31-03_50_36_PM.csv', 'r21_sumo_log_2025_08_31-03_59_23_PM.csv', ...
            '../2025_09_01/r24_sumo_log_2025_09_01-06_48_58_PM.csv', '../2025_09_01/r12_sumo_log_2025_09_01-06_12_24_PM.csv', ...
        '../2025_09_01/r4_sumo_log_2025_09_01-05_38_17_PM.csv', '../2025_09_01/r5_sumo_log_2025_09_01-05_40_57_PM.csv', ...
            '../2025_09_01/r23_sumo_log_2025_09_01-06_46_14_PM.csv', '../2025_09_01/r11_sumo_log_2025_09_01-06_09_20_PM.csv', ...
        'r20_sumo_log_2025_08_31-03_56_17_PM.csv', 'r23_sumo_log_2025_08_31-04_05_24_PM.csv', ...
            '../2025_09_01/r22_sumo_log_2025_09_01-06_43_26_PM.csv', '../2025_09_01/r10_sumo_log_2025_09_01-06_04_57_PM.csv', ...
        'r25_sumo_log_2025_08_31-04_25_19_PM.csv', 'r24_sumo_log_2025_08_31-04_21_15_PM.csv', ...
            '../2025_09_01/r21_sumo_log_2025_09_01-06_40_24_PM.csv', '../2025_09_01/r9_sumo_log_2025_09_01-06_02_01_PM.csv', ...
        'r27_sumo_log_2025_08_31-04_30_25_PM.csv', 'r26_sumo_log_2025_08_31-04_28_01_PM.csv', ...
            '../2025_09_01/r20_sumo_log_2025_09_01-06_37_34_PM.csv', '../2025_09_01/r8_sumo_log_2025_09_01-05_59_02_PM.csv'
    });

groups2 = {
    {'pred', 'prev', 'fbStop', 'fbCarry'}
    {'pred_2_22', 'prev_2_22', 'fbStop_2_22', 'fbCarry_2_22'}
    {'pred_2_19', 'prev_2_19', 'fbStop_2_19', 'fbCarry_2_19'}
    {'pred_2_20', 'prev_2_20', 'fbStop_2_20', 'fbCarry_2_20'}
    {'pred_1_20', 'prev_1_20', 'fbStop_1_20', 'fbCarry_1_20'}
    {'pred_0p5_20', 'prev_0p5_20', 'fbStop_0p5_20', 'fbCarry_0p5_20'}
    {'pred_2_10', 'prev_2_10', 'fbStop_2_10', 'fbCarry_2_10'}
    {'pred_1p5_10', 'prev_1p5_10', 'fbStop_1p5_10', 'fbCarry_1p5_10'}
    {'pred_1_10', 'prev_1_10', 'fbStop_1_10', 'fbCarry_1_10'}
    {'pred_0p75_10', 'prev_0p75_10', 'fbStop_0p75_10', 'fbCarry_0p75_10'}
    {'pred_0p5_10', 'prev_0p5_10', 'fbStop_0p5_10', 'fbCarry_0p5_10'}
    {'pred_0p2_10', 'prev_0p2_10', 'fbStop_0p2_10', 'fbCarry_0p2_10'}
    {'pred_0p1_10', 'prev_0p1_10', 'fbStop_0p1_10', 'fbCarry_0p1_10'}
};

%% ===== 2veh metrics =====
[rms_spd_g2, rms_dist_g2, min_gap_g2, collisions_g2, rms_acc_g2, sim_dur_g2, missing2] = ...
    deal(nan(nGroups, controllers_to_plot), nan(nGroups, controllers_to_plot), ...
         nan(nGroups, controllers_to_plot), nan(nGroups, controllers_to_plot), ...
         nan(nGroups, controllers_to_plot), nan(nGroups, controllers_to_plot), {});

switch lower(VEH2_SOURCE)
    case 'hil'
        fprintf('2veh source: HIL legacy (2025_08_31 / 2025_09_01)\n');
        % Resolve relative paths against data/2025_08_31
        hilBase = fullfile(dataRoot, '2025_08_31');
        n2 = numel(fileStruct2);
        [rms_spd2, rms_dist2, rms_acc2, min_gap2, col2, dur2] = deal(nan(n2, 1));
        for i = 1:n2
            fpath = fullfile(hilBase, fileStruct2(i).file);
            m = pair_metrics(sanitize_leading_gap(readtable(fpath)), 'v0', 'v1');
            rms_spd2(i) = m.rms_spd; rms_dist2(i) = m.rms_gap; rms_acc2(i) = m.rms_acc;
            min_gap2(i) = m.min_gap; col2(i) = m.collisions; dur2(i) = m.duration;
        end
        for g = 1:nGroups
            for k = 1:controllers_to_plot
                idx = find(strcmp({fileStruct2.label}, groups2{g}{k}), 1);
                if isempty(idx), continue; end
                rms_spd_g2(g, k) = rms_spd2(idx);
                rms_dist_g2(g, k) = rms_dist2(idx);
                min_gap_g2(g, k) = min_gap2(idx);
                collisions_g2(g, k) = col2(idx);
                rms_acc_g2(g, k) = rms_acc2(idx);
                sim_dur_g2(g, k) = dur2(idx);
            end
        end

    case 'sim2026'
        fprintf('2veh source: sim2026 (%s)\n', expSim);
        [rms_spd_g2, rms_dist_g2, min_gap_g2, collisions_g2, rms_acc_g2, sim_dur_g2, missing2] = ...
            load_run_metrics(ctrlDirs2sim, controllers, controllers_to_plot, nGroups, runId_for_group);

    case 'hil2026'
        fprintf('2veh source: hil2026 (%s)\n', expHil);
        [rms_spd_g2, rms_dist_g2, min_gap_g2, collisions_g2, rms_acc_g2, sim_dur_g2, missing2] = ...
            load_run_metrics(ctrlDirs2hil, controllers, controllers_to_plot, nGroups, runId_for_group);

    otherwise
        error('VEH2_SOURCE must be ''hil'', ''sim2026'', or ''hil2026'', got %s', VEH2_SOURCE);
end
warn_missing('2-veh', missing2);

% %δ = 100 * (metric - baseline) ./ baseline
switch lower(NOMINAL_MODE)
    case 'shared'
        kNom = find(strcmp(controllers, NOMINAL_CTRL), 1);
        if isempty(kNom)
            error('NOMINAL_CTRL must be one of {%s}, got %s', strjoin(controllers, ', '), NOMINAL_CTRL);
        end
        % ONE shared No-Attack baseline (NOMINAL_CTRL, 2veh) for ALL %δ
        nom_spd  = rms_spd_g2(1, kNom);
        nom_dist = rms_dist_g2(1, kNom);
        nom_acc  = rms_acc_g2(1, kNom);
        fprintf(['Nominal mode=shared (2veh %s No Attack): ', ...
            'rms_spd=%.4f  rms_gap=%.4f  rms_acc=%.4f\n'], ...
            NOMINAL_CTRL, nom_spd, nom_dist, nom_acc);
        pct_spd_g2  = 100 * (rms_spd_g2  - nom_spd)  ./ nom_spd;
        pct_dist_g2 = 100 * (rms_dist_g2 - nom_dist) ./ nom_dist;
        pct_acc_g2  = 100 * (rms_acc_g2  - nom_acc)  ./ nom_acc;

    case 'per_folder'
        fprintf('Nominal mode=per_folder (each controller uses its own No-Attack / r0)\n');
        pct_spd_g2  = 100 * (rms_spd_g2  - rms_spd_g2(1, :))  ./ rms_spd_g2(1, :);
        pct_dist_g2 = 100 * (rms_dist_g2 - rms_dist_g2(1, :)) ./ rms_dist_g2(1, :);
        pct_acc_g2  = 100 * (rms_acc_g2  - rms_acc_g2(1, :))  ./ rms_acc_g2(1, :);
        nom_spd = []; nom_dist = []; nom_acc = [];  % unused; 3veh uses own rows

    otherwise
        error('NOMINAL_MODE must be ''shared'' or ''per_folder'', got %s', NOMINAL_MODE);
end
col_rate_g2 = collisions_g2 ./ sim_dur_g2;

%% ===== 3veh metrics (front pair + avg of both pairs) =====
[rms_spd_g3, rms_dist_g3, min_gap_g3, collisions_g3, rms_acc_g3, sim_dur_g3] = ...
    deal(nan(nGroups, controllers_to_plot));
[rms_spd_g3avg, rms_dist_g3avg, min_gap_g3avg, collisions_g3avg, rms_acc_g3avg] = ...
    deal(nan(nGroups, controllers_to_plot));
missing3 = {};

for g = 1:nGroups
    runId = runId_for_group(g);
    if isnan(runId), continue; end
    for k = 1:controllers_to_plot
        ctrl = controllers{k};
        csvPath = find_run_csv(ctrlDirs3.(ctrl), runId);
        if isempty(csvPath)
            missing3{end+1} = sprintf('%s r%d (%s)', ctrl, runId, ctrlDirs3.(ctrl)); %#ok<AGROW>
            continue
        end
        data = sanitize_leading_gap(readtable(csvPath));
        mf = pair_metrics(data, 'v0', 'v1');  % front↔ego
        mr = pair_metrics(data, 'v1', 'v2');  % ego↔rear

        rms_spd_g3(g, k) = mf.rms_spd;
        rms_dist_g3(g, k) = mf.rms_gap;
        rms_acc_g3(g, k) = mf.rms_acc;
        min_gap_g3(g, k) = mf.min_gap;
        collisions_g3(g, k) = mf.collisions;
        sim_dur_g3(g, k) = mf.duration;

        rms_spd_g3avg(g, k) = mean([mf.rms_spd, mr.rms_spd], 'omitnan');
        rms_dist_g3avg(g, k) = mean([mf.rms_gap, mr.rms_gap], 'omitnan');
        min_gap_g3avg(g, k) = mean([mf.min_gap, mr.min_gap], 'omitnan');
        collisions_g3avg(g, k) = mean([mf.collisions, mr.collisions], 'omitnan');
        rms_acc_g3avg(g, k) = mean([mf.rms_acc, mr.rms_acc], 'omitnan');
    end
end
warn_missing('3-veh', missing3);

switch lower(NOMINAL_MODE)
    case 'shared'
        pct_spd_g3  = 100 * (rms_spd_g3  - nom_spd)  ./ nom_spd;
        pct_dist_g3 = 100 * (rms_dist_g3 - nom_dist) ./ nom_dist;
        pct_acc_g3  = 100 * (rms_acc_g3  - nom_acc)  ./ nom_acc;

        pct_spd_g3avg  = 100 * (rms_spd_g3avg  - nom_spd)  ./ nom_spd;
        pct_dist_g3avg = 100 * (rms_dist_g3avg - nom_dist) ./ nom_dist;
        pct_acc_g3avg  = 100 * (rms_acc_g3avg  - nom_acc)  ./ nom_acc;

    case 'per_folder'
        pct_spd_g3  = 100 * (rms_spd_g3  - rms_spd_g3(1, :))  ./ rms_spd_g3(1, :);
        pct_dist_g3 = 100 * (rms_dist_g3 - rms_dist_g3(1, :)) ./ rms_dist_g3(1, :);
        pct_acc_g3  = 100 * (rms_acc_g3  - rms_acc_g3(1, :))  ./ rms_acc_g3(1, :);

        pct_spd_g3avg  = 100 * (rms_spd_g3avg  - rms_spd_g3avg(1, :))  ./ rms_spd_g3avg(1, :);
        pct_dist_g3avg = 100 * (rms_dist_g3avg - rms_dist_g3avg(1, :)) ./ rms_dist_g3avg(1, :);
        pct_acc_g3avg  = 100 * (rms_acc_g3avg  - rms_acc_g3avg(1, :))  ./ rms_acc_g3avg(1, :);
end
col_rate_g3 = collisions_g3 ./ sim_dur_g3;
col_rate_g3avg = collisions_g3avg ./ sim_dur_g3;

% Combined for plot: cols = [2veh x3, 3veh-avg x3]
pct_spd_avg = [pct_spd_g2, pct_spd_g3avg];
rms_spd_avg = [rms_spd_g2, rms_spd_g3avg];
pct_acc_avg = [pct_acc_g2, pct_acc_g3avg];
rms_acc_avg = [rms_acc_g2, rms_acc_g3avg];
min_gap_avg = [min_gap_g2, min_gap_g3avg];

%% ===== Figures (2veh + 3veh avg only) =====
kinkCfg = struct('bands', SPD_KINK_BANDS, 'gapH', SPD_KINK_GAP, 'bandH', SPD_KINK_BAND_H);

% Fig 1: control effort (%δ + absolute RMS)
f_effort = make_effort_fig(pct_acc_avg, rms_acc_avg, ...
    groupNames, legendAvg, fontSize, fontSize_title, fontSize_legend, FIGPOS_2plots, ...
    2, axColors, lineStyles, controllers_to_plot);

% Fig 2: speed error (absolute RMS + %δ)
f_spd = make_spd_fig(rms_spd_avg, pct_spd_avg, ...
    groupNames, legendAvg, SPD_YSCALE, fontSize, fontSize_title, fontSize_legend, FIGPOS_2plots, ...
    2, axColors, lineStyles, controllers_to_plot, kinkCfg);

% Fig 3: performance (speed %δ + lowest gap) — unchanged 2-subplot layout
f_perf = make_perf_fig(pct_spd_avg, min_gap_avg, [], false, ...
    groupNames, legendAvg, SPD_YSCALE, fontSize, fontSize_title, fontSize_legend, FIGPOS_2plots, ...
    {'', ''}, ...
    2, axColors, lineStyles, controllers_to_plot, kinkCfg);

%% ===== Save =====
save_fig(f_effort, plotDir, 'R1_effort_3veh_avg.png');
save_fig(f_spd,    plotDir, 'R1_speed_3veh_avg.png');
save_fig(f_perf,   plotDir, 'R1_performance_3veh_avg_2panel.png');
fprintf('Saved figures to %s\n', plotDir);

%% ===== Summary tables =====
controllerNames = legendCtrl;
fprintf('\n=== Summary Metrics ===\n');
summaryRows = {};
for cfgTag = ["2veh", "3veh", "3veh_avg"]
    if cfgTag == "2veh"
        pct_spd = pct_spd_g2; pct_dist = pct_dist_g2; col_rate = col_rate_g2;
    elseif cfgTag == "3veh"
        pct_spd = pct_spd_g3; pct_dist = pct_dist_g3; col_rate = col_rate_g3;
    else
        pct_spd = pct_spd_g3avg; pct_dist = pct_dist_g3avg; col_rate = col_rate_g3avg;
    end
    for g = 1:nGroups
        fprintf('\n[%s] Group: %s\n', cfgTag, groupNames{g});
        fprintf('%-10s | %-12s | %-12s | %-14s\n', 'Controller', 'RMS Speed %', 'RMS Gap %', 'Collision Rate');
        fprintf('%s\n', repmat('-', 1, 56));
        for k = 1:controllers_to_plot
            fprintf('%-10s | %12.3f | %12.3f | %14.4f\n', ...
                controllerNames{k}, pct_spd(g, k), pct_dist(g, k), col_rate(g, k));
            summaryRows(end+1, :) = {char(cfgTag), groupNames{g}, controllerNames{k}, ...
                pct_spd(g, k), pct_dist(g, k), col_rate(g, k)}; %#ok<AGROW>
        end
    end
end

summaryTable = cell2table(summaryRows, ...
    'VariableNames', {'Config', 'Group', 'Controller', 'RMS_Speed_Dev_pct', 'RMS_Gap_Dev_pct', 'CollisionRate_per_s'});
disp(summaryTable)

summaryCsv = fullfile(plotDir, sprintf('R1_summary_%s_%s.csv', VEH2_SOURCE, VEH3_SOURCE));
writetable(summaryTable, summaryCsv);
fprintf('Saved summary table to %s\n', summaryCsv);

fprintf('\n=== Min / Max by Config/Controller (excluding No Attack) ===\n');
validIdx = ~strcmp(summaryTable.Group, 'No Attack');
filteredTable = summaryTable(validIdx, :);
for cfgTag = ["2veh", "3veh", "3veh_avg"]
    for k = 1:controllers_to_plot
        idx = strcmp(filteredTable.Config, char(cfgTag)) & ...
              strcmp(filteredTable.Controller, controllerNames{k});
        spdVals = filteredTable.RMS_Speed_Dev_pct(idx);
        gapVals = filteredTable.RMS_Gap_Dev_pct(idx);
        colVals = filteredTable.CollisionRate_per_s(idx);
        fprintf('\n%s / %s\n', cfgTag, controllerNames{k});
        fprintf('  RMS Speed Dev %% : min = %.3f, max = %.3f\n', ...
            min(spdVals, [], 'omitnan'), max(spdVals, [], 'omitnan'));
        fprintf('  RMS Gap Dev %%   : min = %.3f, max = %.3f\n', ...
            min(gapVals, [], 'omitnan'), max(gapVals, [], 'omitnan'));
        fprintf('  Collision Rate   : min = %.4f, max = %.4f\n', ...
            min(colVals, [], 'omitnan'), max(colVals, [], 'omitnan'));
    end
end

%% ===== Local helpers =====
function save_fig(fig, plotDir, name)
    saveas(fig, fullfile(plotDir, name));
end

function warn_missing(tag, missing)
    if isempty(missing), return; end
    warning('Missing %d %s CSV file(s); those points stay NaN:', numel(missing), tag);
    for m = 1:numel(missing)
        fprintf('  - %s\n', missing{m});
    end
end

function [rms_spd, rms_dist, min_gap, collisions, rms_acc, sim_dur, missing] = ...
        load_run_metrics(ctrlDirs, controllers, nCtrl, nGroups, runId_for_group)
    rms_spd = nan(nGroups, nCtrl);
    rms_dist = nan(nGroups, nCtrl);
    min_gap = nan(nGroups, nCtrl);
    collisions = nan(nGroups, nCtrl);
    rms_acc = nan(nGroups, nCtrl);
    sim_dur = nan(nGroups, nCtrl);
    missing = {};
    for g = 1:nGroups
        runId = runId_for_group(g);
        if isnan(runId), continue; end
        for k = 1:nCtrl
            ctrl = controllers{k};
            csvPath = find_run_csv(ctrlDirs.(ctrl), runId);
            if isempty(csvPath)
                missing{end+1} = sprintf('%s r%d (%s)', ctrl, runId, ctrlDirs.(ctrl)); %#ok<AGROW>
                continue
            end
            m = pair_metrics(sanitize_leading_gap(readtable(csvPath)), 'v0', 'v1');
            rms_spd(g, k) = m.rms_spd;
            rms_dist(g, k) = m.rms_gap;
            rms_acc(g, k) = m.rms_acc;
            min_gap(g, k) = m.min_gap;
            collisions(g, k) = m.collisions;
            sim_dur(g, k) = m.duration;
        end
    end
end

function m = pair_metrics(data, leader, ego)
%PAIR_METRICS Match analysis_v5.m exactly for one leader↔ego pair:
%   spd_diff = vL_spd - vE_spd
%   dist_diff / gap = vL_dist - vE_dist - 3.25   (bumper gap; 3.25 m vehicle length)
%   rms_spd  = sqrt(mean(spd_diff.^2))
%   rms_gap  = sqrt(mean(dist_diff.^2))
%   rms_acc  = sqrt(mean(ego_accCmd.^2))
%   min_gap  = min(dist_diff)
    spdL = data.([leader '_spd_m_s_']);
    spdE = data.([ego '_spd_m_s_']);
    distL = data.([leader '_dist_m_']);
    distE = data.([ego '_dist_m_']);
    accE = data.([ego '_accCmd_m_s2_']);

    spd_diff = spdL - spdE;
    gap = distL - distE - 3.25;
    m.rms_spd = sqrt(mean(spd_diff.^2));
    m.rms_gap = sqrt(mean(gap.^2));
    m.rms_acc = sqrt(mean(accE.^2));
    m.min_gap = min(gap);
    m.collisions = count_collision_events(gap);
    m.duration = data.SimTime_sec_(end) - data.SimTime_sec_(1);
end

function fig = make_effort_fig(pct_acc, rms_acc, groupNames, legendLabels, ...
        fontSize, fontSize_title, fontSize_legend, figPos, nCfg, axColors, lineStyles, nCtrl)
    fig = figure('DefaultAxesFontsize', fontSize);
    tiledlayout(2, 1, 'TileSpacing', 'tight', 'Padding', 'tight');

    nexttile
    plot_cfg_series(rms_acc, lineStyles, nCtrl, nCfg, axColors);
    style_group_xaxis(groupNames);
    ylabel('RMS Effort [m/s^2]');
    title('RMS Control Effort', ...
        'FontSize', fontSize_title, 'FontWeight', 'bold');
    grid on; box on; ax1 = gca;

    nexttile
    plot_cfg_series(pct_acc, lineStyles, nCtrl, nCfg, axColors);
    style_group_xaxis(groupNames);
    ylabel('% \delta Effort');
    title('Percent Change in RMS Control Effort', ...
        'FontSize', fontSize_title, 'FontWeight', 'bold');
    grid on; box on; ax2 = gca;

    lgd = legend(legendLabels, 'Location', 'southoutside', 'Orientation', 'horizontal', 'NumColumns', 3);
    lgd.FontSize = fontSize_legend;
    linkaxes([ax1, ax2], 'x');
    set(fig, 'Position', figPos, 'Color', 'white');
end

function fig = make_spd_fig(rms_spd, pct_spd, groupNames, legendLabels, ...
        spdScale, fontSize, fontSize_title, fontSize_legend, figPos, nCfg, axColors, lineStyles, nCtrl, kinkCfg)
    fig = figure('DefaultAxesFontsize', fontSize);
    tiledlayout(2, 1, 'TileSpacing', 'tight', 'Padding', 'tight');

    nexttile
    plot_cfg_series(rms_spd, lineStyles, nCtrl, nCfg, axColors);
    style_group_xaxis(groupNames);
    ylabel('RMS Speed Error [m/s]');
    title('RMS Speed Error', ...
        'FontSize', fontSize_title, 'FontWeight', 'bold');
    grid on; box on; ax1 = gca;

    nexttile
    plot_cfg_series(spd_ydata(pct_spd, spdScale, kinkCfg), lineStyles, nCtrl, nCfg, axColors);
    apply_spd_yscale(spdScale, kinkCfg);
    style_group_xaxis(groupNames);
    ylabel('% \delta v');
    title('Percent Change in RMS Speed Error', ...
        'FontSize', fontSize_title, 'FontWeight', 'bold');
    grid on; box on; ax2 = gca;

    lgd = legend(legendLabels, 'Location', 'southoutside', 'Orientation', 'horizontal', 'NumColumns', 3);
    lgd.FontSize = fontSize_legend;
    linkaxes([ax1, ax2], 'x');
    set(fig, 'Position', figPos, 'Color', 'white');
end

function fig = make_perf_fig(pct_spd, min_gap, pct_acc, withEffort, ...
        groupNames, legendLabels, spdScale, fontSize, fontSize_title, fontSize_legend, figPos, ...
        titleSuffix, nCfg, axColors, lineStyles, nCtrl, kinkCfg)
    if nargin < 14, lineStyles = {'-'}; end
    if nargin < 15, nCtrl = size(pct_spd, 2) / nCfg; end
    if nargin < 16 || isempty(kinkCfg)
        kinkCfg = struct('bands', [0 10; 50 60; 100 200], 'gapH', 0.12, 'bandH', 1);
    end
    if isempty(titleSuffix), titleSuffix = {'', '', ''}; end
    while numel(titleSuffix) < 3, titleSuffix{end+1} = ''; end %#ok<AGROW>

    nTiles = 2 + double(withEffort);
    fig = figure('DefaultAxesFontsize', fontSize);
    tiledlayout(nTiles, 1, 'TileSpacing', 'tight', 'Padding', 'tight');

    nexttile
    if nCfg == 1
        plot(spd_ydata(pct_spd, spdScale, kinkCfg), '-o', 'LineWidth', 2);
    else
        plot_cfg_series(spd_ydata(pct_spd, spdScale, kinkCfg), lineStyles, nCtrl, nCfg, axColors);
    end
    apply_spd_yscale(spdScale, kinkCfg);
    style_group_xaxis(groupNames);
    ylabel(' % \delta v');
    title(['Percent Change in RMS Speed Error' titleSuffix{1}], ...
        'FontSize', fontSize_title, 'FontWeight', 'bold');
    grid on; box on; ax1 = gca;

    nexttile
    if nCfg == 1
        plot(min_gap, '-o', 'LineWidth', 2); hold on;
    else
        plot_cfg_series(min_gap, lineStyles, nCtrl, nCfg, axColors);
    end
    yline(0, 'LineWidth', 1, 'LineStyle', '--', 'Color', 'k');
    style_group_xaxis(groupNames);
    ylabel('Lowest gap [m]');
    title(['Lowest Gap observed' titleSuffix{2}], ...
        'FontSize', fontSize_title, 'FontWeight', 'bold');
    grid on; box on; ax2 = gca;

    axs = [ax1, ax2];
    if withEffort
        nexttile
        if nCfg == 1
            plot(pct_acc, '-o', 'LineWidth', 2);
        else
            plot_cfg_series(pct_acc, lineStyles, nCtrl, nCfg, axColors);
        end
        style_group_xaxis(groupNames);
        ylabel('% \delta Effort');
        title(['Percent Change in RMS Control Effort' titleSuffix{3}], ...
            'FontSize', fontSize_title, 'FontWeight', 'bold');
        grid on; box on; axs(end+1) = gca; %#ok<AGROW>
    end

    if nCfg > 1
        lgd = legend(legendLabels, 'Location', 'southoutside', 'Orientation', 'horizontal', 'NumColumns', 3);
    else
        lgd = legend(legendLabels, 'Location', 'southoutside', 'Orientation', 'horizontal');
    end
    lgd.FontSize = fontSize_legend;
    linkaxes(axs, 'x');
    set(fig, 'Position', figPos, 'Color', 'white');
end

function style_group_xaxis(groupNames)
    set(gca, 'XTickLabel', groupNames, 'XTick', 1:numel(groupNames), 'XTickLabelRotation', 45);
end

function plot_cfg_series(Y, lineStyles, nCtrl, nCfg, axColors)
    hold on
    for c = 1:nCfg
        for k = 1:nCtrl
            col = (c - 1) * nCtrl + k;
            plot(Y(:, col), 'LineWidth', 2, 'LineStyle', lineStyles{c}, ...
                'Marker', 'o', 'Color', axColors(k, :));
        end
    end
    hold off
end

function Y = spd_ydata(pct, mode, kinkCfg)
    switch lower(mode)
        case 'log'
            Y = max(0.01, pct);
        case 'kink'
            Y = map_kink_y(pct, kinkCfg);
        otherwise
            Y = pct;
    end
end

function apply_spd_yscale(mode, kinkCfg)
    switch lower(mode)
        case 'log'
            set(gca, 'YScale', 'log');
            ylim([0.01, 200]);
            yticks([0.01, 1, 25, 200]);
        case 'kink'
            set(gca, 'YScale', 'linear');
            [ticks, labels, yLo, yHi, kinkY] = kink_axis_ticks(kinkCfg);
            ylim([yLo, yHi]);
            yticks(ticks);
            yticklabels(labels);
            draw_kink_marks(gca, kinkY);
        otherwise
            set(gca, 'YScale', 'linear');
    end
end

function yDisp = map_kink_y(y, kinkCfg)
%MAP_KINK_Y Map physical %δv onto display coords using editable bands.
% Shown bands (rows of kinkCfg.bands) get height bandH each.
% Intervals between bands are compressed to height gapH each.
    bands = kinkCfg.bands;
    gapH = kinkCfg.gapH;
    bandH = kinkCfg.bandH;
    nB = size(bands, 1);

    % Build piecewise segments: shown, then compressed (except after last)
    % edgesPhys: sorted unique physical breakpoints covering all segments
    edges = bands(1, 1);
    segIsShown = false(0, 1);
    for b = 1:nB
        edges(end+1) = bands(b, 2); %#ok<AGROW>
        segIsShown(end+1) = true; %#ok<AGROW>
        if b < nB
            edges(end+1) = bands(b+1, 1); %#ok<AGROW>
            segIsShown(end+1) = false; %#ok<AGROW>
        end
    end

    segH = zeros(numel(segIsShown), 1);
    for s = 1:numel(segIsShown)
        if segIsShown(s)
            segH(s) = bandH;
        else
            segH(s) = gapH;
        end
    end
    dispEdges = [0; cumsum(segH)];

    yDisp = nan(size(y));
    for i = 1:numel(y)
        v = y(i);
        if isnan(v)
            continue
        elseif v < edges(1)
            yDisp(i) = 0;
            continue
        elseif v > edges(end)
            yDisp(i) = dispEdges(end) + min((v - edges(end)) / max(edges(end), 1), 0.5);
            continue
        end
        for s = 1:numel(segIsShown)
            lo = edges(s);
            hi = edges(s+1);
            if v <= hi || s == numel(segIsShown)
                if hi == lo
                    frac = 0;
                else
                    frac = (v - lo) / (hi - lo);
                end
                yDisp(i) = dispEdges(s) + frac * segH(s);
                break
            end
        end
    end
end

function [ticks, labels, yLo, yHi, kinkY] = kink_axis_ticks(kinkCfg)
    bands = kinkCfg.bands;
    phys = bands(1, 1);
    for b = 1:size(bands, 1)
        phys(end+1) = bands(b, 2); %#ok<AGROW>
        if b < size(bands, 1)
            phys(end+1) = bands(b+1, 1); %#ok<AGROW>
        end
    end
    ticks = map_kink_y(phys, kinkCfg);
    labels = arrayfun(@(v) sprintf('%g', v), phys, 'UniformOutput', false);
    yLo = 0;
    yHi = ticks(end);

    % Kink marks at midpoints of compressed gaps in display coords
    kinkY = [];
    yCursor = 0;
    for b = 1:size(bands, 1)-1
        yCursor = yCursor + kinkCfg.bandH;
        kinkY(end+1) = yCursor + 0.5 * kinkCfg.gapH; %#ok<AGROW>
        yCursor = yCursor + kinkCfg.gapH;
    end
end

function draw_kink_marks(ax, kinkY)
    hold(ax, 'on');
    xl = xlim(ax);
    x0 = xl(1);
    dx = 0.015 * (xl(2) - xl(1));
    for yi = kinkY
        plot(ax, [x0 - dx, x0 + dx], [yi - 0.04, yi + 0.04], 'k-', 'LineWidth', 1.2, 'Clipping', 'off');
        plot(ax, [x0 - dx, x0 + dx], [yi - 0.01, yi + 0.07], 'k-', 'LineWidth', 1.2, 'Clipping', 'off');
    end
    hold(ax, 'off');
end

function n = count_collision_events(gap)
    collision_mask = gap <= 0.5;
    collision_mask(isnan(collision_mask)) = false;
    d_mask = diff([0; collision_mask(:)]);
    n = sum(d_mask == 1);
end

function csvPath = find_run_csv(dirPath, runId)
    csvPath = '';
    if ~isfolder(dirPath), return; end
    listing = dir(fullfile(dirPath, '*.csv'));
    targetSuffix = sprintf('_r%d.csv', runId);
    for i = 1:numel(listing)
        if endsWith(listing(i).name, targetSuffix)
            csvPath = fullfile(dirPath, listing(i).name);
            return
        end
    end
end

function data = sanitize_leading_gap(data)
% First 30 samples: force leader dist=20, clamp ego if >20 (same as analysis_v5.m).
    n = min(30, height(data));
    for j = 1:n
        if data.v1_dist_m_(j) > 20
            data.v1_dist_m_(j) = 0;
        end
        data.v0_dist_m_(j) = 20;
    end
end
