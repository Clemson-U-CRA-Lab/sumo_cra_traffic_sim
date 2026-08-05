% Plot the same column-3 minus column-2 traces as process.m using the
% explicitly configured CSV inputs below.

% For TVT DOS paper - Revision R1

clc; clear; close all;
fontSize = 12;
fontSize_subtiles = 12;
FIGPOS_1plot = [300,400,550,290];


folder = fileparts(mfilename('fullpath'));
if isempty(folder)
    folder = pwd;
end

% Input dictionary: add a run here to load a CSV from any source folder.
% Fields are: scenario name, display name, client count, packet size (B),
% packet rate (PPS), sweep, and CSV path.
augustFolder = fullfile(fileparts(fileparts(folder)), '2026_08_01');
runInputs = [ ...
    runInput('nominal', 'nominal', NaN, NaN, NaN, 'Nominal', fullfile(folder, 'run1_sumIndoorVIL_log_2026_07_24-01_02_23_PM.csv')); ...
    runInput('288B_10PPS_3C', '3', 3, 288, 10, 'Client count', fullfile(folder, 'run2_sumIndoorVIL_log_2026_07_24-01_35_10_PM.csv')); ...
    runInput('288B_10PPS_6C', '6', 6, 288, 10, 'Client count', fullfile(folder, 'run3_sumIndoorVIL_log_2026_07_24-01_41_26_PM.csv')); ...
    % runInput('288B_10PPS_9C', '9', 9, 288, 10, 'Client count', fullfile(folder, 'run4_sumIndoorVIL_log_2026_07_24-01_50_52_PM.csv')); ...
    % runInput('288B_10PPS_12C', '12', 12, 288, 10, 'Client count', fullfile(folder, 'run5_sumIndoorVIL_log_2026_07_24-01_54_57_PM.csv')); ...
    runInput('288B_10PPS_15C', '15', 15, 288, 10, 'Client count', fullfile(folder, 'run6_sumIndoorVIL_log_2026_07_24-02_01_36_PM.csv')); ...
    % runInput('288B_10PPS_18C', '18', 18, 288, 10, 'Client count', fullfile(folder, 'run7_sumIndoorVIL_log_2026_07_24-02_07_01_PM.csv')); ...
    runInput('288B_10PPS_21C', '21', 21, 288, 10, 'Client count', fullfile(folder, 'run8_sumIndoorVIL_log_2026_07_24-02_13_09_PM.csv')); ...
    % runInput('288B_10PPS_24C', '24', 24, 288, 10, 'Client count', fullfile(folder, 'run9_sumIndoorVIL_log_2026_07_24-02_17_58_PM.csv')); ...
    % runInput('288B_10PPS_27C', '27', 27, 288, 10, 'Client count', fullfile(folder, 'run10_sumIndoorVIL_log_2026_07_24-02_23_26_PM.csv')); ...
    runInput('288B_10PPS_30C', '30', 30, 288, 10, 'Client count', fullfile(folder, 'run11_sumIndoorVIL_log_2026_07_24-02_28_20_PM.csv')); ...
    runInput('400B_10PPS_15C', '400', 15, 400, 10, 'Packet size', fullfile(folder, 'run12_sumIndoorVIL_log_2026_07_24-02_40_17_PM.csv')); ...
    % runInput('500B_10PPS_15C', '500', 15, 500, 10, 'Packet size', fullfile(folder, 'run13_sumIndoorVIL_log_2026_07_24-02_44_59_PM.csv')); ...
    runInput('600B_10PPS_15C', '600', 15, 600, 10, 'Packet size', fullfile(folder, 'run14_sumIndoorVIL_log_2026_07_24-02_50_51_PM.csv')); ...
    % runInput('700B_10PPS_15C', '700', 15, 700, 10, 'Packet size', fullfile(folder, 'run15_sumIndoorVIL_log_2026_07_24-02_56_05_PM.csv')); ...
    runInput('800B_10PPS_15C', '800', 15, 800, 10, 'Packet size', fullfile(folder, 'run16_sumIndoorVIL_log_2026_07_24-03_03_04_PM.csv')); ...
    % runInput('900B_10PPS_15C', '900', 15, 900, 10, 'Packet size', fullfile(folder, 'run17_sumIndoorVIL_log_2026_07_24-03_11_50_PM.csv')); ...
    runInput('1000B_10PPS_15C', '1000', 15, 1000, 10, 'Packet size', fullfile(folder, 'run18_sumIndoorVIL_log_2026_07_24-03_17_46_PM.csv')); ...
    runInput('20PPS_15C', '20', 15, 288, 20, 'Packet rate', fullfile(folder, 'run19_sumIndoorVIL_log_2026_07_24-03_26_20_PM.csv')); ...
    runInput('30PPS_15C', '30', 15, 288, 30, 'Packet rate', fullfile(folder, 'run20_sumIndoorVIL_log_2026_07_24-03_33_18_PM.csv')); ...
    runInput('40PPS_15C', '40', 15, 288, 40, 'Packet rate', fullfile(folder, 'run21_sumIndoorVIL_log_2026_07_24-03_38_22_PM.csv')); ...
    runInput('50PPS_15C', '50', 15, 288, 50, 'Packet rate', fullfile(folder, 'run22_sumIndoorVIL_log_2026_07_24-03_53_19_PM.csv')); ...
    runInput('70PPS_15C', '70', 15, 288, 70, 'Packet rate', fullfile(folder, 'run24_sumIndoorVIL_log_2026_07_24-04_03_19_PM.csv')); ...
    runInput('90PPS_15C', '90', 15, 288, 90, 'Packet rate', fullfile(folder, 'run26_sumIndoorVIL_log_2026_07_24-04_20_34_PM.csv')); ...
    
    runInput('288B_10PPS_50C', '50', 50, 288, 10, 'Client count', fullfile(augustFolder, 'r1_sumIndoorVIL_log_2026_08_01-06_33_32_PM.csv')); ...
    runInput('288B_10PPS_60C', '60', 60, 288, 10, 'Client count', fullfile(augustFolder, 'r3_sumIndoorVIL_log_2026_08_01-06_45_19_PM.csv')); ...
    runInput('288B_10PPS_70C', '70', 70, 288, 10, 'Client count', fullfile(augustFolder, 'r4_sumIndoorVIL_log_2026_08_01-06_47_35_PM.csv')); ...
    runInput('288B_10PPS_80C', '80', 80, 288, 10, 'Client count', fullfile(augustFolder, 'r5_sumIndoorVIL_log_2026_08_01-06_51_06_PM.csv')); ...
    runInput('288B_10PPS_90C', '90', 90, 288, 10, 'Client count', fullfile(augustFolder, 'r6_sumIndoorVIL_log_2026_08_01-06_53_10_PM.csv')); ...
    runInput('288B_10PPS_100C', '100', 100, 288, 10, 'Client count', fullfile(augustFolder, 'r2_sumIndoorVIL_log_2026_08_01-06_41_19_PM.csv')); ...
    runInput('1500B_10PPS_15C', '1500', 15, 1500, 10, 'Packet size', fullfile(augustFolder, 'r7_sumIndoorVIL_log_2026_08_01-06_57_26_PM.csv')); ...
    runInput('2000B_10PPS_15C', '2000', 15, 2000, 10, 'Packet size', fullfile(augustFolder, 'r8_sumIndoorVIL_log_2026_08_01-06_59_53_PM.csv')); ...
    runInput('2500B_10PPS_15C', '2500', 15, 2500, 10, 'Packet size', fullfile(augustFolder, 'r9_sumIndoorVIL_log_2026_08_01-07_02_58_PM.csv')); ...
    runInput('3500B_10PPS_15C', '3500', 15, 3500, 10, 'Packet size', fullfile(augustFolder, 'r10_sumIndoorVIL_log_2026_08_01-07_05_08_PM.csv')); ...
    runInput('4500B_10PPS_15C', '4500', 15, 4500, 10, 'Packet size', fullfile(augustFolder, 'r11_sumIndoorVIL_log_2026_08_01-07_07_11_PM.csv')); ...
    runInput('5500B_10PPS_15C', '5500', 15, 5500, 10, 'Packet size', fullfile(augustFolder, 'r12_sumIndoorVIL_log_2026_08_01-07_09_30_PM.csv')); ...
    runInput('6500B_10PPS_15C', '6500', 15, 6500, 10, 'Packet size', fullfile(augustFolder, 'r13_sumIndoorVIL_log_2026_08_01-07_12_04_PM.csv'))];

outDir = fullfile(folder, 'plots');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

allDiffs = cell(1, numel(runInputs));
allNames = cell(1, numel(runInputs));
allScenarios = cell(1, numel(runInputs));
allSweeps = cell(1, numel(runInputs));
allClientCounts = nan(1, numel(runInputs));
allPacketSizes = nan(1, numel(runInputs));
allPacketRates = nan(1, numel(runInputs));
allAverageDelays = nan(1, numel(runInputs));
allMaxDelays = nan(1, numel(runInputs));
allRmsDelays = nan(1, numel(runInputs));
allStdDelays = nan(1, numel(runInputs));
allSampleCounts = nan(1, numel(runInputs));
validCount = 0;

for k = 1:numel(runInputs)
    csvPath = string(runInputs(k).csvPath);
    if ~isfile(csvPath)
        warning('Skipping missing CSV: %s', csvPath);
        continue;
    end

    T = readtable(csvPath);
    if height(T) < 2 || width(T) < 3
        warning('Skipping %s because it does not contain at least two rows and three columns.', ...
            csvPath);
        continue;
    end

    data = table2array(T);
    col2 = data(:, 2);
    col3 = data(:, 3);

    % Keep the startup-residual handling used by process.m.
    maxResidualSamples = 20;
    for i = 1:min(maxResidualSamples, numel(col3))
        if col3(i) > 10 && abs(col3(i) - col2(i)) > 20
            col3(i) = col2(i);
        end
    end
    diffVals = col2 - col3;

    validCount = validCount + 1;
    allDiffs{validCount} = diffVals;
    allNames{validCount} = runInputs(k).displayName;
    allScenarios{validCount} = runInputs(k).scenario;
    allSweeps{validCount} = runInputs(k).sweep;
    allClientCounts(validCount) = runInputs(k).clientCount;
    allPacketSizes(validCount) = runInputs(k).packetSizeBytes;
    allPacketRates(validCount) = runInputs(k).packetRatePps;
    finiteDiffVals = diffVals(isfinite(diffVals));
    if ~isempty(finiteDiffVals)
        allAverageDelays(validCount) = mean(finiteDiffVals);
        allMaxDelays(validCount) = max(abs(finiteDiffVals));
        allRmsDelays(validCount) = sqrt(mean(finiteDiffVals .^ 2));
        allStdDelays(validCount) = std(finiteDiffVals);
        allSampleCounts(validCount) = numel(finiteDiffVals);
    end
end

allDiffs = allDiffs(1:validCount);
allNames = allNames(1:validCount);
allScenarios = allScenarios(1:validCount);
allSweeps = allSweeps(1:validCount);
allClientCounts = allClientCounts(1:validCount);
allPacketSizes = allPacketSizes(1:validCount);
allPacketRates = allPacketRates(1:validCount);
allAverageDelays = allAverageDelays(1:validCount);
allMaxDelays = allMaxDelays(1:validCount);
allRmsDelays = allRmsDelays(1:validCount);
allStdDelays = allStdDelays(1:validCount);
allSampleCounts = allSampleCounts(1:validCount);

% Group the plot strictly by the configured sweep field, not by run number.
sweepOrder = {'Nominal', 'Client count', 'Packet size', 'Packet rate'};
runOrder = [];
for sweepIndex = 1:numel(sweepOrder)
    runOrder = [runOrder, find(strcmp(allSweeps, sweepOrder{sweepIndex}))]; %#ok<AGROW>
end
allDiffs = allDiffs(runOrder);
allNames = allNames(runOrder);
allScenarios = allScenarios(runOrder);
allSweeps = allSweeps(runOrder);
allClientCounts = allClientCounts(runOrder);
allPacketSizes = allPacketSizes(runOrder);
allPacketRates = allPacketRates(runOrder);
allAverageDelays = allAverageDelays(runOrder);
allMaxDelays = allMaxDelays(runOrder);
allRmsDelays = allRmsDelays(runOrder);
allStdDelays = allStdDelays(runOrder);
allSampleCounts = allSampleCounts(runOrder);

displayScenarioNames = allNames;

% Normalize every run against the nominal scenario so that zero represents
% nominal performance and the sweep effects can be compared directly.
nominalIdx = find(strcmpi(allScenarios, 'nominal'));
if numel(nominalIdx) ~= 1 || ~isfinite(allAverageDelays(nominalIdx))
    error('Exactly one valid nominal PCAP/CSV pair is required for normalization.');
end
relativeAverageDelays = allAverageDelays - allAverageDelays(nominalIdx);
% Reference the peak delay to the same nominal baseline as the average.
% For every run, this preserves max delay >= average delay.
relativeMaxDelays = allMaxDelays - allAverageDelays(nominalIdx);
if any(relativeMaxDelays < relativeAverageDelays)
    error('Maximum delay is below average delay for at least one run. Check the delay calculation.');
end

% Paper-oriented summary: one shared scenario axis with a separate line for
% each experimental sweep.
plotPaperDelaySweeps(allSweeps, displayScenarioNames, relativeAverageDelays, relativeMaxDelays, ...
    outDir, fontSize, FIGPOS_1plot);

% A second view positions every run by its complete offered-load product.
configurationProducts = allClientCounts .* allPacketSizes .* allPacketRates;
configurationProducts(~isfinite(configurationProducts)) = 0; % nominal baseline
plotConfigurationProduct(allSweeps, displayScenarioNames, configurationProducts, ...
    relativeAverageDelays, relativeMaxDelays, outDir, fontSize, FIGPOS_1plot);

% Copy-friendly numerical summary for review in GPT or a spreadsheet.
summaryTable = table( ...
    string(displayScenarioNames(:)), string(allScenarios(:)), ...
    string(allSweeps(:)), allClientCounts(:), allPacketSizes(:), allPacketRates(:), ...
    allSampleCounts(:), allAverageDelays(:), ...
    relativeAverageDelays(:), allRmsDelays(:), allStdDelays(:), ...
    allMaxDelays(:), relativeMaxDelays(:), ...
    'VariableNames', {'DisplayName', 'Scenario', 'Sweep', 'ClientCount', ...
    'PacketSizeBytes', 'PacketRatePps', ...
    'Samples', 'MeanDelay_s', 'MeanVsNominal_s', 'RMSDelay_s', ...
    'StdDelay_s', 'PeakAbsDelay_s', 'PeakVsNominal_s'});

fprintf('\n=== Delay Summary (copy into GPT or a spreadsheet) ===\n');
disp(summaryTable);
writetable(summaryTable, fullfile(outDir, 'delay_summary.csv'));
fprintf('Machine-readable summary saved to %s\n', fullfile(outDir, 'delay_summary.csv'));

fprintf('Finished plotting %d documented PCAP/CSV pair(s) to %s.\n', validCount, outDir);

function input = runInput(scenario, displayName, clientCount, packetSizeBytes, packetRatePps, sweep, csvPath)
    input = struct('scenario', scenario, 'displayName', displayName, ...
        'clientCount', clientCount, 'packetSizeBytes', packetSizeBytes, ...
        'packetRatePps', packetRatePps, 'sweep', sweep, 'csvPath', csvPath);
end

function plotPaperDelaySweeps(sweeps, scenarios, relativeDelays, relativeMaxDelays, outDir, fontSize, figPosition)
    sweepDefinitions = [ ...
        struct('sweep', 'Client count', 'label', 'Client-count sweep', ...
               'groupText', 'Varying number of clients', ...
               'color', [0.00 0.45 0.74]); ...
        struct('sweep', 'Packet size', 'label', 'Packet-size sweep', ...
               'groupText', 'Varying packet size (Bytes)', ...
               'color', [0.85 0.33 0.10]); ...
        struct('sweep', 'Packet rate', 'label', 'Packet-rate sweep', ...
               'groupText', 'Varying packet rate (1/s)', ...
               'color', [0.47 0.67 0.19])];

    figPaper = figure('Name', 'Average Delay Relative to Nominal', ...
        'Visible', 'on', 'Color', 'white', 'Position', figPosition);
    ax = axes(figPaper);
    % Leave sufficient room for the right-side y-axis label.
    ax.Position = [0.11 0.34 0.76 0.57];
    ax.Color = 'w';
    ax.FontSize = fontSize;
    toolbar = axtoolbar(ax);
    toolbar.Visible = 'off';
    yyaxis(ax, 'left');
    hold(ax, 'on');

    for sweepIndex = 1:numel(sweepDefinitions)
        definition = sweepDefinitions(sweepIndex);
        groupIdx = find(strcmp(sweeps, definition.sweep));
        lineHandle = plot(ax, groupIdx, relativeDelays(groupIdx), '-o', 'Color', definition.color, ...
            'DisplayName', definition.label, 'LineWidth', 1.1, ...
            'MarkerFaceColor', definition.color, 'MarkerSize', 3);
        if sweepIndex == 1
            averageLegendHandle = lineHandle;
        end
    end
    ylabel(ax, 'Average delay (s)');

    yyaxis(ax, 'right');
    for sweepIndex = 1:numel(sweepDefinitions)
        definition = sweepDefinitions(sweepIndex);
        groupIdx = find(strcmp(sweeps, definition.sweep));
        lineHandle = plot(ax, groupIdx, relativeMaxDelays(groupIdx), '--s', 'Color', definition.color, ...
            'DisplayName', sprintf('%s: maximum', definition.label), 'LineWidth', 1.1, ...
            'MarkerFaceColor', 'w', 'MarkerSize', 3);
        if sweepIndex == 1
            maximumLegendHandle = lineHandle;
        end
    end
    ylabel(ax, 'Maximum delay (s)');
    hold(ax, 'off');

    grid(ax, 'on');
    xlim(ax, [0.5 numel(scenarios) + 0.5]);
    xticks(ax, 1:numel(scenarios));
    colorScenarioTickLabels(ax, scenarios, sweeps, sweepDefinitions, fontSize);
    addSweepGroupLabels(figPaper, ax, sweeps, sweepDefinitions, fontSize);
    xlabel(ax, 'Scenarios');
    title(ax, 'Communication Delays vs Attack Parameters');
    ax.YAxis(1).Label.FontSize = fontSize;
    ax.YAxis(2).Label.FontSize = fontSize;
    ax.Title.FontSize = fontSize;
    ax.XLabel.FontSize = fontSize;
    ax.XLabel.Units = 'normalized';
    ax.XLabel.Position = [0.5 -0.50 0];
    legendHandle = legend(ax, [averageLegendHandle maximumLegendHandle], ...
        {'Average delay', 'Maximum delay'}, ...
        'Location', 'northwest', 'Interpreter', 'none');
    legendHandle.FontSize = fontSize-2;

    % set(gcf,'position',FIGPOS_1plot_wide)
    % set( gcf, 'Color', 'white' )

    % Retain a white plotting area while producing a tight PNG with a
    % transparent outer background.

    exportgraphics(figPaper, fullfile(outDir, 'sweeps.png'), ...
        'Resolution', 600, 'BackgroundColor', 'none');
    % close(figPaper);
end

function plotConfigurationProduct(sweeps, scenarios, products, relativeDelays, relativeMaxDelays, outDir, fontSize, figPosition)
    sweepDefinitions = [ ...
        struct('sweep', 'Client count', 'color', [0.00 0.45 0.74]); ...
        struct('sweep', 'Packet size', 'color', [0.85 0.33 0.10]); ...
        struct('sweep', 'Packet rate', 'color', [0.47 0.67 0.19])];
    [products, order] = sort(products);
    sweeps = sweeps(order);
    scenarios = scenarios(order);
    relativeDelays = relativeDelays(order);
    relativeMaxDelays = relativeMaxDelays(order);

    fig = figure('Name', 'Delay vs Configuration Product', 'Visible', 'on', ...
        'Color', 'white', 'Position', figPosition);
    ax = axes(fig);
    ax.Position = [0.11 0.34 0.76 0.57];
    ax.Color = 'w';
    ax.FontSize = fontSize;
    toolbar = axtoolbar(ax);
    toolbar.Visible = 'off';

    yyaxis(ax, 'left');
    hold(ax, 'on');
    for sweepIndex = 1:numel(sweepDefinitions)
        definition = sweepDefinitions(sweepIndex);
        groupIdx = find(strcmp(sweeps, definition.sweep));
        plot(ax, products(groupIdx), relativeDelays(groupIdx), '-o', ...
            'Color', definition.color, 'LineWidth', 1.1, ...
            'MarkerFaceColor', definition.color, 'MarkerSize', 3);
    end
    nominalIdx = find(strcmp(sweeps, 'Nominal'));
    plot(ax, products(nominalIdx), relativeDelays(nominalIdx), 'o', ...
        'Color', [0 0 0], 'MarkerFaceColor', [0 0 0], 'MarkerSize', 3);
    ylabel(ax, 'Average delay (s)');

    yyaxis(ax, 'right');
    for sweepIndex = 1:numel(sweepDefinitions)
        definition = sweepDefinitions(sweepIndex);
        groupIdx = find(strcmp(sweeps, definition.sweep));
        plot(ax, products(groupIdx), relativeMaxDelays(groupIdx), '--s', ...
            'Color', definition.color, 'LineWidth', 1.1, ...
            'MarkerFaceColor', 'w', 'MarkerSize', 3);
    end
    plot(ax, products(nominalIdx), relativeMaxDelays(nominalIdx), 's', ...
        'Color', [0 0 0], 'MarkerFaceColor', 'w', 'MarkerSize', 3);
    ylabel(ax, 'Maximum delay (s)');
    hold(ax, 'off');

    grid(ax, 'on');
    xlim(ax, [0 max(products) * 1.05]);
    % colorConfigurationTickLabels(ax, scenarios, sweeps, products, sweepDefinitions, fontSize);
    xlabel(ax, 'Client count \times packet size (B) \times packet rate (PPS)');
    ax.XLabel.FontSize = fontSize;
    % ax.XLabel.Units = 'normalized';
    % ax.XLabel.Position = [0.5 -0.62 0];
    title(ax, 'Communication Delays vs Configuration Product');
    exportgraphics(fig, fullfile(outDir, 'configuration_product.png'), ...
        'Resolution', 600, 'BackgroundColor', 'none');
end


function colorScenarioTickLabels(ax, scenarios, sweeps, sweepDefinitions, fontSize)
    % Native MATLAB tick labels share one color.  Replace them with text
    % objects so every scenario label can match its sweep line color.
    xticklabels(ax, repmat({''}, 1, numel(scenarios)));
    yyaxis(ax, 'left');
    yBase = ax.YLim(1);
    scenarioLabelFontSize = max(8, fontSize - 2);

    for scenarioIndex = 1:numel(scenarios)
        labelColor = [0 0 0];
        for sweepIndex = 1:numel(sweepDefinitions)
            definition = sweepDefinitions(sweepIndex);
            if strcmp(sweeps{scenarioIndex}, definition.sweep)
                labelColor = definition.color;
                break;
            end
        end

        text(ax, scenarioIndex, yBase, scenarios{scenarioIndex}, ...
            'Color', labelColor, 'FontSize', scenarioLabelFontSize, 'Interpreter', 'none', ...
            'HorizontalAlignment', 'right', 'VerticalAlignment', 'top', ...
            'Rotation', 75, 'Clipping', 'off');
    end
end

function addSweepGroupLabels(figHandle, ax, sweeps, sweepDefinitions, fontSize)
    % Add an editable label above a double-headed arrow for each sweep.
    % Figure coordinates keep the annotations aligned in the export.
    plotCount = numel(sweeps);
    arrowY = 0.10;
    labelY = 0.12;
    labelHeight = 0.1;

    for sweepIndex = 1:numel(sweepDefinitions)
        definition = sweepDefinitions(sweepIndex);
        groupIdx = find(strcmp(sweeps, definition.sweep));
        if isempty(groupIdx)
            continue;
        end

        boxLeft = ax.Position(1) + ((min(groupIdx) - 0.5) / plotCount) * ax.Position(3);
        boxRight = ax.Position(1) + ((max(groupIdx) + 0.5) / plotCount) * ax.Position(3);
        annotation(figHandle, 'doublearrow', [boxLeft boxRight], [arrowY arrowY], ...
            'Color', definition.color, 'LineWidth', 1);
        annotation(figHandle, 'textbox', [boxLeft labelY boxRight - boxLeft labelHeight], ...
            'String', definition.groupText, 'Color', definition.color, ...
            'EdgeColor', 'none', 'BackgroundColor', 'none', ...
            'FontSize', fontSize-2, 'Interpreter', 'none', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    end
end
