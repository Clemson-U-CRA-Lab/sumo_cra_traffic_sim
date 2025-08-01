clear;
clc;
addpath('/home/cra/sumo_ws/bag/custom_interface/matlab_msg_gen_ros1/glnxa64/install/m')

% bag = rosbag("2025-03-29-11-16-00.bag");
bag = rosbag("run11_2025-04-05-17-07-52.bag");
testName="run11";
%%

% Define topics to analyze
topic1 = '/v2x/obu2veh';
topic2 = '/v2x/veh2obu_mpc';
topic3 = '/bridge_to_lowlevel';

% Select messages from the topics
bagSel1 = select(bag, 'Topic', topic1);
bagSel2 = select(bag, 'Topic', topic2);
bagSel3 = select(bag, 'Topic', topic3);

% Read messages and timestamps
msgs1 = readMessages(bagSel1, 'DataFormat','struct');
msgs2 = readMessages(bagSel2);
msgs3 = readMessages(bagSel3);
%%

ulcCmd_bag = select(bag, 'Topic', '/Mach_E/ulc_cmd');
ulcCmdMsg = readMessages(ulcCmd_bag);
t_ulcCmd = ulcCmd_bag.MessageList.Time - bag.StartTime;

ulcReport_bag = select(bag, 'Topic', '/Mach_E/ulc_report');
ulcReportMsg = readMessages(ulcReport_bag);
t_ulcReport = ulcReport_bag.MessageList.Time - bag.StartTime;

%%

t_rosSimArray = bagSel1.MessageList.Time - bag.StartTime;
t_rosVehArray = bagSel2.MessageList.Time - bag.StartTime;
t_rosBridge = bagSel3.MessageList.Time - bag.StartTime;


dataCell = cellfun(@(m) m.Data, msgs1, 'UniformOutput', false);
% Convert to a matrix (assuming all messages have the same vector length)
simArray = cell2mat(cellfun(@(x) x', dataCell, 'UniformOutput', false)); % Transpose for row-wise stacking

dataCell = cellfun(@(m) m.Data, msgs2, 'UniformOutput', false);
vehArray = cell2mat(cellfun(@(x) x', dataCell, 'UniformOutput', false)); % Transpose for row-wise stacking

dataCell = cellfun(@(m) m.Data, msgs3, 'UniformOutput', false);
bridgeData = cell2mat(cellfun(@(x) x', dataCell, 'UniformOutput', false)); % Transpose for row-wise stacking

dataCell = cellfun(@(m) m.AccelCmd, ulcCmdMsg, 'UniformOutput', false);
ulcCmd = cell2mat(cellfun(@(x) x', dataCell, 'UniformOutput', false)); % Transpose for row-wise stacking

dataCell = cellfun(@(m) m.AccelMeas, ulcReportMsg, 'UniformOutput', false);
ulcReport =  cell2mat(cellfun(@(x) x', dataCell, 'UniformOutput', false)); % Transpose for row-wise stacking

dataCell = cellfun(@(m) m.AccelRef, ulcReportMsg, 'UniformOutput', false);
ulcRef =  cell2mat(cellfun(@(x) x', dataCell, 'UniformOutput', false)); % Transpose for row-wise stacking


%%
simt_end = length(simArray);
rost_end = 1100;

front_col = 'k';
ego_col = 'blue';
lw = 1.5;
vehArray_t_end = 670;
xend = 110;

clf;
tiledlayout(3, 2, "Padding","tight", "TileSpacing","compact");
set(gcf,'position',[50,500,1200,900])

nexttile
hold on
plot(t_rosVehArray, vehArray(:,3), Color=ego_col, LineWidth=lw)
% plot(t_rosSimArray, simArray(:,3)) % V_ego in sim
plot(t_rosSimArray, simArray(:,6), Color=front_col, LineWidth=lw)
% plot(t_rosBridge, bridgeData(:,4), "--") % gps V ego
legend("ego V [gps]", "front V")
xlabel("Vehicle ROS Time")
hold off
grid on
xlim([0, xend])
ylim([0, 6])

nexttile
hold on
plot(vehArray(1:vehArray_t_end,1), vehArray(1:vehArray_t_end,3), Color=ego_col, LineWidth=lw)
plot(simArray(1:simt_end,1), simArray(1:simt_end,6), Color=front_col, LineWidth=lw)
legend("ego V", "front V")
hold off
grid on
xlabel("RSPC SimTime")
xlim([0, xend])
ylim([0, 6])


% ego_s_gps1 = cumtrapz(vehArray(1:simt_end,3), vehArray(1:simt_end,1));
ego_s_gps = cumtrapz(t_rosBridge, bridgeData(:,4));

nexttile
offsetT= 8.54;
hold on
plot(t_rosSimArray(1:simt_end,1)-offsetT, simArray(1:simt_end,2), Color=ego_col, LineWidth=lw)
% plot(t_rosVehArray(1:simt_end,1), ego_s_gps1)
plot(t_rosBridge, ego_s_gps, "--", Color=ego_col, LineWidth=lw)
plot(t_rosSimArray(1:simt_end,1)-offsetT, simArray(1:simt_end,5), Color=front_col, LineWidth=lw)
legend("Ego s [sim]", "Ego s [gps] ", "front-s [sim]")
hold off
grid on
xlabel("Vehicle ROS Time")
xlim([0, xend])


nexttile
hold on
plot(simArray(1:simt_end,1), simArray(1:simt_end,2), Color=ego_col, LineWidth=lw)
plot(simArray(1:simt_end,1), simArray(1:simt_end,5), Color=front_col, LineWidth=lw)
legend("ego s", "front s")
xlabel("RSPC SimTime")
hold off
grid on
xlim([0, xend])

nexttile
hold on
plot(t_ulcCmd, ulcCmd, Color='r', LineWidth=lw)
plot(t_ulcReport, ulcReport, "--", Color=ego_col, LineWidth=lw)
% plot(t_ulcReport, ulcRef, Color=ego_col, LineWidth=lw)
% plot(t_rosSimArray(1:simt_end,1), simArray(1:simt_end,4),Color=ego_col, LineWidth=lw)
plot(t_rosSimArray(1:simt_end,1), simArray(1:simt_end,7),Color=front_col, LineWidth=lw)
legend("ULC Cmd", "ego a [GPS]", "front a")
hold off
grid on
xlabel("Vehicle ROS Time")
xlim([0, xend])
ylim([-2, 2])

nexttile
hold on
plot(vehArray(1:end,1), vehArray(1:end,4))
plot(vehArray(1:end,7), vehArray(1:end,4))
legend("ego a-sim", "ego a-vehsim")
hold off
grid on
xlabel("VehPCCTime")
xlim([0, xend])

sgtitle(testName)

% linkaxes(get(gcf, 'children'), 'x') 

%%

% figure(1)
% hold on
% plot(t_rosBridge, bridgeData(:,7),'--')
% plot(t_ulcReport, ulcReport)
% legend("GPS a", "ULC meas a")
% hold off
t_vA = [t_rosVehArray(1,1):0.05:t_rosVehArray(end,1)+0.05]' - t_rosVehArray(1,1);
ego_s_gps1 = cumtrapz(vehArray(:,3),t_vA );

% clf;
figure(2)
plot(t_vA, ego_s_gps1)

%% FUNCTIONS

function [sim_time, s_ego, v_ego, a_ego, s_f, v_f, a_f] = getSimVals(simArray)
    sim_time = simArray(1);
    s_ego = simArray(2);
    v_ego = simArray(3);
    a_ego = simArray(4);
    s_f = simArray(5);
    v_f = simArray(6);
    a_f = simArray(7);
end