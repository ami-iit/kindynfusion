baseTime = squeeze(whole_body_kinematics_logger.base_state.orientation.timestamps)';
baseTime = baseTime - baseTime(1);
baseRPY = squeeze(whole_body_kinematics_logger.base_state.orientation.data)';
basePos = squeeze(whole_body_kinematics_logger.base_state.position.data)';

gtBaseTime = squeeze(whole_body_kinematics_logger.ground_truth.base_state.orientation.timestamps)';
gtBaseTime = gtBaseTime - gtBaseTime(1);
gtBaseRPY = squeeze(whole_body_kinematics_logger.ground_truth.base_state.orientation.data)';
gtBasePos = squeeze(whole_body_kinematics_logger.ground_truth.base_state.position.data)';
gtBasePos(1, :) = gtBasePos(2, :);
gtBaseRPY(1, :) = gtBaseRPY(2, :);

nrIters = length(gtBasePos);
startIterForCalib= 2;
endIterForCalib = nrIters;
%%
[gtBasePosAligned,gtBaseRPYAligned] = alignTrajectoriesShah(gtBasePos, gtBaseRPY, basePos, baseRPY);

%% base position
color = {'r', 'g', 'b'};
title2 = {'x (m)', 'y (m)', 'z (m)'};
figure
for idx = 1:3
    subplot(3, 1, idx)
    simp = plot(gtBaseTime, gtBasePosAligned(:, idx), 'o',  'Color', 'black');
    legend_line = [];
    legendtex = {};
    legend_line = [legend_line simp];
    legendtex = [legendtex 'Vive Trackers'];
    hold on

    wbkline = plot(baseTime, basePos(:, idx), 'o',  'Color',  color{idx});
    legend_line = [legend_line wbkline];
    legendtex = [legendtex 'WBK'];

    ylabel(title2{idx}, 'FontSize', 24)
%     if (idx == 1)
        legend(legend_line,legendtex,'FontSize', 24, 'Orientation', 'Vertical', 'Location', 'NorthWest')
%     end
    if (idx == 3)
        xlabel('Time(s)', 'FontSize', 18)
    end
    set(gca,'FontSize',20, 'FontWeight', 'Bold')
    set(gca, 'GridAlpha', 0.5, 'LineWidth', 1.5)
    grid on
end
sgtitle('Base Position', ...
    'FontSize', 24, 'FontWeight', 'Bold');
drawnow;

%% base orientation
color = {'r', 'g', 'b'};
title2 = {'Roll (deg)', 'Pitch (deg)', 'Yaw (deg)'};
figure
for idx = 1:3
    subplot(3, 1, idx)
    simp = plot(gtBaseTime, rad2deg(gtBaseRPYAligned(:, idx)), 'o',  'Color', 'black');
    legend_line = [];
    legendtex = {};
    legend_line = [legend_line simp];
    legendtex = [legendtex 'Vive Trackers'];
    hold on

    wbkline = plot(baseTime, rad2deg(baseRPY(:, idx)), 'o',  'Color',  color{idx});
    legend_line = [legend_line wbkline];
    legendtex = [legendtex 'WBK'];

    ylabel(title2{idx}, 'FontSize', 24)
%     if (idx == 1)
        legend(legend_line,legendtex,'FontSize', 24, 'Orientation', 'Vertical', 'Location', 'NorthWest')
%     end
    if (idx == 3)
        xlabel('Time(s)', 'FontSize', 18)
    end
    set(gca,'FontSize',20, 'FontWeight', 'Bold')
    set(gca, 'GridAlpha', 0.5, 'LineWidth', 1.5)
    grid on
end
sgtitle('Base Orientation', ...
    'FontSize', 24, 'FontWeight', 'Bold');
drawnow;