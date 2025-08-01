% plot_all_signals.m
% ------------------------------------------------------------------------
% For every sumo_v2x_log*.csv in pwd:
%   • Reads it as a table
%   • Plots:
%       1) Acc channels + mache_mpcCmd overlaid
%       2) Speed channels
%       3) Dist channels
%       4) Each other "time" col individually
%       5) Combined simtime & reltime vs index
%   • Renames veh0→LV, veh1→Ego in all legends
%   • Saves the figure as PNG
% ------------------------------------------------------------------------
figPos      = [100, 100, 1400, 1000];  

files = dir('*.csv');

for f = 1:numel(files)
    fnameNum = files(f).name(1:4);
    fname = files(f).name;
    T     = readtable(fname);
    vars  = T.Properties.VariableNames;

    % Group definitions
    isAcc  = contains(vars,'acc','IgnoreCase',true);
    isSpd  = contains(vars,'spd','IgnoreCase',true);
    isDist = contains(vars,'dist','IgnoreCase',true);
    isTime = contains(vars,{'Time','runtime'},'IgnoreCase',true);

    % Time axis for channel plots: pick first time column
    timeCols = vars(isTime);
    if isempty(timeCols)
        error('No time column found in %s', fname);
    end
    t = T.(timeCols{1});

    % How many individual time‑cols (minus those we’ll overlay later)
    numTimeCols = numel(timeCols);

    % Total tiles = 3 groups + each time col + 1 extra overlay plot
    totalTiles =  numTimeCols ;

    % Start figure
    hFig = figure('Name',fname,'NumberTitle','off', ...
        'Visible','on','Position', figPos);
    tl  = tiledlayout(totalTiles,1, 'TileSpacing','compact','Padding','compact');

    %% 1) Acceleration + mache_mpcCmd
    nexttile(tl,1);
    p = plot(t, table2array(T(:,isAcc)), 'LineWidth',1); hold on;
    % overlay the mache_mpcCmd
    if ismember('mache_mpcCmd', vars)
        plot(t, T.mache_mpcCmd, '--','LineWidth',1.5);
        legNames = [vars(isAcc), {'mache\_mpcCmd'}];
    else
        legNames = vars(isAcc);
    end
    % rename veh0→LV, veh1→Ego
    legNames = strrep(legNames,'veh0','LV');
    legNames = strrep(legNames,'veh1','Ego');
    legend(legNames,'Interpreter','none','Location','best');
    title('Acceleration Channels + MPC Cmd');
    xlabel(timeCols{1}); ylabel('m/s^2');
    hold off;

    %% 2) Speed
    nexttile(tl,2);
    plot(t, table2array(T(:,isSpd)), 'LineWidth',1);
    legNames = vars(isSpd);
    legNames = strrep(legNames,'veh0','LV');
    legNames = strrep(legNames,'veh1','Ego');
    legend(legNames,'Interpreter','none','Location','best');
    title('Speed Channels'); xlabel(timeCols{1}); ylabel('m/s');

    %% 3) Distance
    nexttile(tl,3);
    plot(t, table2array(T(:,isDist)),'LineWidth',1);
    legNames = vars(isDist);
    legNames = strrep(legNames,'veh0','LV');
    legNames = strrep(legNames,'veh1','Ego');
    legend(legNames,'Interpreter','none','Location','best');
    title('Distance Channels'); xlabel(timeCols{1}); ylabel('m');



    %% Super title & save
    sgtitle(sprintf('run:%s', fnameNum),'FontWeight','bold');
    outname = ['run_' fnameNum, '.png'];
    saveas(hFig, outname);
    close(hFig);
end
