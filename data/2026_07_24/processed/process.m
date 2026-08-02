% Plot the same column-3 minus column-2 traces as process.m, but use the
% explicit PCAP/CSV run pairs recorded in readme1.md for plot names.

% For TVT DOS paper 

clc; clear; close all;
fontSize = 12;
fontSize_subtiles = 12;
FIGPOS_1plot = [300,400,550,290];


% Display labels, in the documented PCAP/CSV-pair order.  Edit only this
% list to rename the scenarios shown on the plots.
% scenario_names = { ...
%     'nominal', ...
%     '3C', '6C', '9C', ...
%     '12C', '15C', '18C', ...
%     '21C', '24C', '27C', ...
%     '30C', ...
%     '400B_10PPS_15C', '500B_10PPS_15C', '600B_10PPS_15C', ...
%     '700B_10PPS_15C', '800B_10PPS_15C', '900B_10PPS_15C', ...
%     '1000B_10PPS_15C', ...
%     '20PPS_15C', '30PPS_15C', '35PPS_15C', '40PPS_15C', ...
%     '50PPS_15C', '70PPS_15C'};
scenario_names = { ...
    'nominal', ...
    '3', '6', '9', ...
    '12', '15', '18', ...
    '21', '24', '27', ...
    '30', ...
    '400', '500', '600', ...
    '700', '800', '900', ...
    '1000', ...
    '20', '30', '35', '40', ...
    '50', '70'};


folder = fileparts(mfilename('fullpath'));
if isempty(folder)
    folder = pwd;
end

readmePath = fullfile(folder, 'readme.md');
if ~isfile(readmePath)
    error('Could not find %s', readmePath);
end

outDir = fullfile(folder, 'plots_readme');
if ~exist(outDir, 'dir')
    mkdir(outDir);
end

% Each documented pair consists of a PCAP table row followed by a CSV row.
% This deliberately does not infer pairings from timestamps.
pairs = readPairs(readmePath);
if isempty(pairs)
    error('No PCAP/CSV pairs were found in %s', readmePath);
end

allDiffs = cell(1, numel(pairs));
allNames = cell(1, numel(pairs));
allRuns = nan(1, numel(pairs));
allScenarios = cell(1, numel(pairs));
allAverageDelays = nan(1, numel(pairs));
allMaxDelays = nan(1, numel(pairs));
allRmsDelays = nan(1, numel(pairs));
allStdDelays = nan(1, numel(pairs));
allSampleCounts = nan(1, numel(pairs));
validCount = 0;

for k = 1:numel(pairs)
    csvPath = resolveCsvPath(folder, pairs(k).csvFile);
    if strlength(csvPath) == 0
        warning('Skipping missing CSV: %s', pairs(k).csvFile);
        continue;
    end

    T = readtable(csvPath);
    if height(T) < 2 || width(T) < 3
        warning('Skipping %s because it does not contain at least two rows and three columns.', ...
            pairs(k).csvFile);
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

    % The PCAP basename is the run name used for both the title and output.
    [~, plotName] = fileparts(pairs(k).pcapFile);
    validCount = validCount + 1;
    allDiffs{validCount} = diffVals;
    allNames{validCount} = plotName;
    allScenarios{validCount} = scenarioName(plotName);
    finiteDiffVals = diffVals(isfinite(diffVals));
    if ~isempty(finiteDiffVals)
        allAverageDelays(validCount) = mean(finiteDiffVals);
        allMaxDelays(validCount) = max(abs(finiteDiffVals));
        allRmsDelays(validCount) = sqrt(mean(finiteDiffVals .^ 2));
        allStdDelays(validCount) = std(finiteDiffVals);
        allSampleCounts(validCount) = numel(finiteDiffVals);
    end
    runToken = regexp(plotName, '^run(\d+)_', 'tokens', 'once');
    if isempty(runToken)
        error('Could not determine the run number from PCAP name: %s', pairs(k).pcapFile);
    end
    allRuns(validCount) = str2double(runToken{1});

end

allDiffs = allDiffs(1:validCount);
allNames = allNames(1:validCount);
allRuns = allRuns(1:validCount);
allScenarios = allScenarios(1:validCount);
allAverageDelays = allAverageDelays(1:validCount);
allMaxDelays = allMaxDelays(1:validCount);
allRmsDelays = allRmsDelays(1:validCount);
allStdDelays = allStdDelays(1:validCount);
allSampleCounts = allSampleCounts(1:validCount);

% The run IDs were corrected after the CSV files were written.  Sort the
% data by those corrected IDs before matching it with scenario_names.
[allRuns, runOrder] = sort(allRuns);
allDiffs = allDiffs(runOrder);
allNames = allNames(runOrder);
allScenarios = allScenarios(runOrder);
allAverageDelays = allAverageDelays(runOrder);
allMaxDelays = allMaxDelays(runOrder);
allRmsDelays = allRmsDelays(runOrder);
allStdDelays = allStdDelays(runOrder);
allSampleCounts = allSampleCounts(runOrder);

if numel(scenario_names) ~= validCount
    error(['scenario_names must contain one label for each valid documented PCAP/CSV pair. ' ...
        'Expected %d labels but found %d.'], validCount, numel(scenario_names));
end
displayScenarioNames = reshape(scenario_names, 1, []);

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
plotPaperDelaySweeps(allRuns, displayScenarioNames, relativeAverageDelays, relativeMaxDelays, ...
    outDir, fontSize, FIGPOS_1plot);

% Copy-friendly numerical summary for review in GPT or a spreadsheet.
summaryTable = table( ...
    allRuns(:), string(displayScenarioNames(:)), string(allScenarios(:)), ...
    sweepName(allRuns(:)), allSampleCounts(:), allAverageDelays(:), ...
    relativeAverageDelays(:), allRmsDelays(:), allStdDelays(:), ...
    allMaxDelays(:), relativeMaxDelays(:), ...
    'VariableNames', {'Run', 'DisplayParameter', 'Scenario', 'Sweep', ...
    'Samples', 'MeanDelay_s', 'MeanVsNominal_s', 'RMSDelay_s', ...
    'StdDelay_s', 'PeakAbsDelay_s', 'PeakVsNominal_s'});

fprintf('\n=== Delay Summary (copy into GPT or a spreadsheet) ===\n');
disp(summaryTable);
writetable(summaryTable, fullfile(outDir, 'delay_summary.csv'));
fprintf('Machine-readable summary saved to %s\n', fullfile(outDir, 'delay_summary.csv'));

% % Plot the requested run groups.
% plotGroups = [1 11; 12 18; 19 26];
% for groupIndex = 1:size(plotGroups, 1)
%     firstRun = plotGroups(groupIndex, 1);
%     lastRun = plotGroups(groupIndex, 2);
%     groupIdx = find(allRuns >= firstRun & allRuns <= lastRun);
%     if isempty(groupIdx)
%         warning('No documented CSV data found for runs %d through %d.', firstRun, lastRun);
%         continue;
%     end
% 
%     figSweep = figure('Name', sprintf('Runs %d-%d', firstRun, lastRun), 'Visible', 'on');
%     hold on;
%     for k = groupIdx
%         plot(allDiffs{k}, 'LineWidth', 1.2);
%     end
%     hold off;
%     grid on;
%     legend(allNames(groupIdx), 'Interpreter', 'none', 'Location', 'bestoutside');
%     xlabel('Index');
%     ylabel('Difference (3 - 2)');
%     title(sprintf('Documented PCAP Runs %d-%d', firstRun, lastRun));
%     saveas(figSweep, fullfile(outDir, sprintf('runs_%d_to_%d.png', firstRun, lastRun)));
%     close(figSweep);
% 
%     plotAverageDelay(displayScenarioNames(groupIdx), relativeAverageDelays(groupIdx), ...
%         sprintf('Average Delay Relative to Nominal: Runs %d-%d', firstRun, lastRun), ...
%         fullfile(outDir, sprintf('average_delay_runs_%d_to_%d.png', firstRun, lastRun)));
% end

fprintf('Finished plotting %d documented PCAP/CSV pair(s) to %s.\n', validCount, outDir);

function pairs = readPairs(readmePath)
    lines = splitlines(string(fileread(readmePath)));
    pcapPattern = '`([^`]+\.pcap)`';
    csvPattern = '`([^`]+\.csv)`';
    pairs = struct('pcapFile', {}, 'csvFile', {});
    pendingPcap = "";

    for i = 1:numel(lines)
        line = lines(i);
        pcapToken = regexp(line, pcapPattern, 'tokens', 'once', 'ignorecase');
        if ~isempty(pcapToken)
            pendingPcap = string(pcapToken{1});
            continue;
        end

        csvToken = regexp(line, csvPattern, 'tokens', 'once', 'ignorecase');
        if ~isempty(csvToken) && strlength(pendingPcap) > 0
            pairs(end + 1).pcapFile = char(pendingPcap); %#ok<AGROW>
            pairs(end).csvFile = csvToken{1};
            pendingPcap = "";
        end
    end
end

function csvPath = resolveCsvPath(folder, csvFile)
    csvPath = string(fullfile(folder, strrep(csvFile, '/', filesep)));
    if isfile(csvPath)
        return;
    end

    % CSV run prefixes can differ after a PCAP run has been renumbered.
    % The timestamp portion remains unique and preserves the documented pair.
    suffixToken = regexp(csvFile, '^run\d+_(.+)$', 'tokens', 'once');
    if isempty(suffixToken)
        csvPath = "";
        return;
    end
    matches = dir(fullfile(folder, ['run*_' suffixToken{1}]));
    if numel(matches) == 1
        csvPath = string(fullfile(matches.folder, matches.name));
    else
        csvPath = "";
    end
end

function scenario = scenarioName(pcapBaseName)
    % PCAP format: runNum_R1_HH_MM_Month_Date_Device_PacketSize_PacketRate_numClients.
    % Nominal PCAP files do not follow the full format and share one label.
    if contains(pcapBaseName, 'nominal', 'IgnoreCase', true)
        scenario = 'nominal';
        return;
    end

    nameParts = split(string(pcapBaseName), '_');
    if numel(nameParts) < 7
        warning('Could not extract scenario from PCAP name: %s', pcapBaseName);
        scenario = pcapBaseName;
        return;
    end
    scenario = char(join(nameParts(7:end), '_'));
end

function sweep = sweepName(runs)
    % Labels match the experimental groups shown in the paper figure.
    sweep = repmat("Unclassified", size(runs));
    sweep(runs >= 1 & runs <= 11) = "Client count";
    sweep(runs >= 12 & runs <= 18) = "Packet size";
    sweep(runs >= 19 & runs <= 26) = "Packet rate";
end

function plotAverageDelay(scenarios, averageDelays, plotTitle, outputPath)
    figAverage = figure('Name', plotTitle, 'Visible', 'on');
    bar(averageDelays);
    grid on;
    xticks(1:numel(scenarios));
    xticklabels(scenarios);
    xtickangle(35);
    xlabel('Scenario');
    ylabel('Average delay relative to nominal (s)');
    title(plotTitle);
    set(gca, 'TickLabelInterpreter', 'none');
    saveas(figAverage, outputPath);
    close(figAverage);
end

function plotPaperDelaySweeps(runs, scenarios, relativeDelays, relativeMaxDelays, outDir, fontSize, figPosition)
    sweepDefinitions = [ ...
        struct('runRange', [1 11], 'label', 'Client-count sweep', ...
               'groupText', 'Varying Number of clients', ...
               'color', [0.00 0.45 0.74]); ...
        struct('runRange', [12 18], 'label', 'Packet-size sweep', ...
               'groupText', 'Varying Packet size (Bytes)', ...
               'color', [0.85 0.33 0.10]); ...
        struct('runRange', [19 26], 'label', 'Packet-rate sweep', ...
               'groupText', 'Varying Packet rate (1/s)', ...
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
        groupIdx = find(runs >= definition.runRange(1) & runs <= definition.runRange(2));
        [~, order] = sort(runs(groupIdx));
        groupIdx = groupIdx(order);
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
        groupIdx = find(runs >= definition.runRange(1) & runs <= definition.runRange(2));
        [~, order] = sort(runs(groupIdx));
        groupIdx = groupIdx(order);
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
    colorScenarioTickLabels(ax, scenarios, runs, sweepDefinitions, fontSize);
    addSweepGroupLabels(figPaper, ax, runs, sweepDefinitions, fontSize);
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

function colorScenarioTickLabels(ax, scenarios, runs, sweepDefinitions, fontSize)
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
            if runs(scenarioIndex) >= definition.runRange(1) && ...
                    runs(scenarioIndex) <= definition.runRange(2)
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

function addSweepGroupLabels(figHandle, ax, runs, sweepDefinitions, fontSize)
    % Add an editable label above a double-headed arrow for each sweep.
    % Figure coordinates keep the annotations aligned in the export.
    plotCount = numel(runs);
    arrowY = 0.10;
    labelY = 0.12;
    labelHeight = 0.069;

    for sweepIndex = 1:numel(sweepDefinitions)
        definition = sweepDefinitions(sweepIndex);
        groupIdx = find(runs >= definition.runRange(1) & runs <= definition.runRange(2));
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
