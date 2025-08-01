% plot_all_signals.m
% ------------------------------------------------------------------------
% Reads a CSV log and creates one figure with:
%   • Subplot 1: all acceleration channels
%   • Subplot 2: all speed channels
%   • Subplot 3: all distance channels
%   • Subplots 4...: each "time" column in its own tile
% ------------------------------------------------------------------------

% 1) Read the data
filename = 'sumo_v2x_log2025_05_14-09_10_16_AM';
data = readtable(filename);

% 2) Get variable names and identify groups
vars     = data.Properties.VariableNames;
isAcc    = contains(vars, 'acc',    'IgnoreCase', true);
isSpd    = contains(vars, 'spd',    'IgnoreCase', true);
isDist   = contains(vars, 'dist',   'IgnoreCase', true);
isTime   = contains(vars, {'Time','runtime'}, 'IgnoreCase', true);

% 3) Choose a common time axis for plotting channels (pick the first “time” col)
timeVars = vars(isTime);
if isempty(timeVars)
    error('No time‐type column found for x‐axis');
end
t = data.(timeVars{1});

% 4) Set up the tiled layout
numTimeCols = sum(isTime);
totalTiles  = 3 + numTimeCols;
figure('Name','All Signals','NumberTitle','off');
tlo = tiledlayout(totalTiles,1, ...
    'TileSpacing','compact','Padding','compact');

% 5) Plot acceleration channels
nexttile(tlo,1);
plot(t, table2array(data(:,isAcc)), 'LineWidth',1);
legend(vars(isAcc), 'Interpreter','none','Location','best');
title('Acceleration Channels');
xlabel(timeVars{1});
ylabel('m/s^2');


% 6) Plot speed channels
nexttile(tlo,2);
plot(t, table2array(data(:,isSpd)), 'LineWidth',1);
legend(vars(isSpd), 'Interpreter','none','Location','best');
title('Speed Channels');
xlabel(timeVars{1});
ylabel('m/s');

% 7) Plot distance channels
nexttile(tlo,3);
plot(t, table2array(data(:,isDist)), 'LineWidth',1);
legend(vars(isDist), 'Interpreter','none','Location','best');
title('Distance Channels');
xlabel(timeVars{1});
ylabel('m');

nexttile(tlo,4);
plot(t, table2array(data(:,12)), 'LineWidth',1);
% legend(vars(isAcc), 'Interpreter','none','Location','best');
title("Acc Cmd Mache");
xlabel(timeVars{1});
ylabel('m/s^2');


% 8) Plot each time column in its own tile (vs. sample index)
timeNames = vars(isTime);
for k = 1:numTimeCols
    ax = nexttile(tlo, 3+k);
    col = timeNames{k};
    plot(data.(col), 'LineWidth',1);
    title(['Time Column: ' col]);
    xlabel('Sample Index');
    ylabel(col);
end

% 9) Super‐title
sgtitle('SIM V2X Log — All Signals','FontWeight','bold');
