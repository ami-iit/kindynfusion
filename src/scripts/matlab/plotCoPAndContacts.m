

light_blue = [0, 0, 0.7];
light_green = [0, 0.7, 0.];
light_red = [0.7, 0, 0.];
%%
figure
lsole0_contact = squeeze(whole_body_kinematics_logger.foot_contact.LeftSole_vertex0.contact_state.data);
lsole1_contact = squeeze(whole_body_kinematics_logger.foot_contact.LeftSole_vertex1.contact_state.data);
lsole2_contact = squeeze(whole_body_kinematics_logger.foot_contact.LeftSole_vertex2.contact_state.data);
lsole3_contact = squeeze(whole_body_kinematics_logger.foot_contact.LeftSole_vertex3.contact_state.data);

rsole0_contact = squeeze(whole_body_kinematics_logger.foot_contact.RightSole_vertex0.contact_state.data);
rsole1_contact = squeeze(whole_body_kinematics_logger.foot_contact.RightSole_vertex1.contact_state.data);
rsole2_contact = squeeze(whole_body_kinematics_logger.foot_contact.RightSole_vertex2.contact_state.data);
rsole3_contact = squeeze(whole_body_kinematics_logger.foot_contact.RightSole_vertex3.contact_state.data);

z = zeros(size(baseTime));
colormap(flipud(bone))
lsolecontact = 0.5*[lsole0_contact'; lsole1_contact'; lsole2_contact'; lsole3_contact'];
rsolecontact = 0.5*[rsole0_contact'; rsole1_contact'; rsole2_contact'; rsole3_contact'];
contactdata = [lsolecontact; rsolecontact];
im = imagesc(contactdata);
caxis([0 1])
xticklabels = baseTime(1): baseTime(end);
yticklabels = {'L1', 'L2', 'L3', 'L4', 'R1', 'R2', 'R3', 'R4'};
xticks = linspace(1, size(contactdata, 2), numel(xticklabels));
set(gca, 'XTick', xticks, 'XTickLabel', xticklabels, 'YTickLabel', yticklabels);
set(gca,'FontSize',24, 'FontWeight', 'Bold')
sgtitle('Feet Vertex Contact', ...
'FontSize', 32, 'FontWeight', 'Bold');
drawnow;


%%

globalCop = squeeze(whole_body_kinematics_logger.global_cop.data)';
lsole0_pos = squeeze(whole_body_kinematics_logger.foot_contact.LeftSole_vertex0.est_global_position.data)';
lsole1_pos = squeeze(whole_body_kinematics_logger.foot_contact.LeftSole_vertex1.est_global_position.data)';
lsole2_pos = squeeze(whole_body_kinematics_logger.foot_contact.LeftSole_vertex2.est_global_position.data)';
lsole3_pos = squeeze(whole_body_kinematics_logger.foot_contact.LeftSole_vertex3.est_global_position.data)';

rsole0_pos = squeeze(whole_body_kinematics_logger.foot_contact.RightSole_vertex0.est_global_position.data)';
rsole1_pos = squeeze(whole_body_kinematics_logger.foot_contact.RightSole_vertex1.est_global_position.data)';
rsole2_pos = squeeze(whole_body_kinematics_logger.foot_contact.RightSole_vertex2.est_global_position.data)';
rsole3_pos = squeeze(whole_body_kinematics_logger.foot_contact.RightSole_vertex3.est_global_position.data)';


figure 
copline = plot(globalCop(:, 1), globalCop(:, 2), ':', 'color',light_green, 'LineWidth', 4);
hold on;
showTrace = false;
showPolygon = true;
lfFirstPlanarContact = false;
rfFirstPlanarContact = false;
for idx =1:length(lsole0_contact)
    lfContacts = [lsole0_contact(idx); lsole1_contact(idx); lsole2_contact(idx); lsole3_contact(idx)];
    if (sum(lfContacts) == 4)
        if (~showTrace && lfFirstPlanarContact)
            % do nothing
        else
            lfFirstPlanarContact = true;
            if (lfFirstPlanarContact)
                scatter(lsole0_pos(idx, 1), lsole0_pos(idx, 2), 'filled', 'MarkerEdgeColor', light_blue,'MarkerFaceColor', light_blue)
                scatter(lsole1_pos(idx, 1), lsole1_pos(idx, 2), 'filled', 'MarkerEdgeColor', light_blue,'MarkerFaceColor', light_blue)
                scatter(lsole2_pos(idx, 1), lsole2_pos(idx, 2), 'filled', 'MarkerEdgeColor', light_blue,'MarkerFaceColor', light_blue)
                scatter(lsole3_pos(idx, 1), lsole3_pos(idx, 2), 'filled', 'MarkerEdgeColor', light_blue,'MarkerFaceColor', light_blue)
                if showPolygon
                    lsolex = [lsole2_pos(idx, 1) lsole0_pos(idx, 1) lsole1_pos(idx, 1) lsole3_pos(idx, 1)];
                    lsoley = [lsole2_pos(idx, 2) lsole0_pos(idx, 2) lsole1_pos(idx, 2) lsole3_pos(idx, 2)];
                    pgon = polyshape(lsolex, lsoley);
                    lfrect = plot(pgon,'FaceColor',light_blue,'FaceAlpha',0.1);
                end
            end
        end
    else
        lfFirstPlanarContact = false;
    end

    rfContacts = [rsole0_contact(idx); rsole1_contact(idx); rsole2_contact(idx); rsole3_contact(idx)];
    if (sum(rfContacts) == 4)
        if (~showTrace && rfFirstPlanarContact)
            % do nothing
        else
            rfFirstPlanarContact = true;
            if (rfFirstPlanarContact)
                scatter(rsole0_pos(idx, 1), rsole0_pos(idx, 2), 'filled', 'MarkerEdgeColor', light_red,'MarkerFaceColor', light_red)
                scatter(rsole1_pos(idx, 1), rsole1_pos(idx, 2), 'filled', 'MarkerEdgeColor', light_red,'MarkerFaceColor', light_red)
                scatter(rsole2_pos(idx, 1), rsole2_pos(idx, 2), 'filled', 'MarkerEdgeColor', light_red,'MarkerFaceColor', light_red)
                scatter(rsole3_pos(idx, 1), rsole3_pos(idx, 2), 'filled', 'MarkerEdgeColor', light_red,'MarkerFaceColor', light_red)
                if showPolygon
                    rsolex = [rsole2_pos(idx, 1) rsole0_pos(idx, 1) rsole1_pos(idx, 1) rsole3_pos(idx, 1)];
                    rsoley = [rsole2_pos(idx, 2) rsole0_pos(idx, 2) rsole1_pos(idx, 2) rsole3_pos(idx, 2)];
                    pgon = polyshape(rsolex, rsoley);
                    rfrect = plot(pgon,'FaceColor',light_red,'FaceAlpha',0.1);
                end
            end
        end
    else
        rfFirstPlanarContact = false;
    end
end
legend_line = [copline lfrect rfrect];
legendtex = {'Global CoP' 'Left Foot' 'Right Foot'};
legend(legend_line,legendtex,'FontSize', 22, 'Orientation', 'Vertical')
set(gca,'FontSize',24, 'FontWeight', 'Bold')
sgtitle('Estimated Contact Point Positions and Global CoP', ...
'FontSize', 32, 'FontWeight', 'Bold');
grid on
xlabel('x (m)')
ylabel('y (m)')


% %% 
% 
% figure 
% copline = plot3(globalCop(:, 1), globalCop(:, 2), globalCop(:, 3), ':', 'color',light_green, 'LineWidth', 4);
% hold on;
% showTrace = false;
% showPolygon = true;
% lfFirstPlanarContact = false;
% rfFirstPlanarContact = false;
% for idx =1:length(lsole0_contact)
%     lfContacts = [lsole0_contact(idx); lsole1_contact(idx); lsole2_contact(idx); lsole3_contact(idx)];
%     if (sum(lfContacts) == 4)
%         if (~showTrace && lfFirstPlanarContact)
%             % do nothing
%         else
%             lfFirstPlanarContact = true;
%             if (lfFirstPlanarContact)
%                 scatter3(lsole0_pos(idx, 1), lsole0_pos(idx, 2), lsole0_pos(idx, 3),'filled', 'MarkerEdgeColor', light_blue,'MarkerFaceColor', light_blue)
%                 scatter3(lsole1_pos(idx, 1), lsole1_pos(idx, 2), lsole1_pos(idx, 3),'filled', 'MarkerEdgeColor', light_blue,'MarkerFaceColor', light_blue)
%                 scatter3(lsole2_pos(idx, 1), lsole2_pos(idx, 2), lsole2_pos(idx, 3),'filled', 'MarkerEdgeColor', light_blue,'MarkerFaceColor', light_blue)
%                 scatter3(lsole3_pos(idx, 1), lsole3_pos(idx, 2), lsole3_pos(idx, 3),'filled', 'MarkerEdgeColor', light_blue,'MarkerFaceColor', light_blue)
%                 if showPolygon
%                     lsolex = [lsole2_pos(idx, 1) lsole0_pos(idx, 1) lsole1_pos(idx, 1) lsole3_pos(idx, 1)];
%                     lsoley = [lsole2_pos(idx, 2) lsole0_pos(idx, 2) lsole1_pos(idx, 2) lsole3_pos(idx, 2)];
%                     pgon = polyshape(lsolex, lsoley);
%                     lfrect = plot(pgon,'FaceColor',light_blue,'FaceAlpha',0.1);
%                 end
%             end
%         end
%     else
%         lfFirstPlanarContact = false;
%     end
% 
%     rfContacts = [rsole0_contact(idx); rsole1_contact(idx); rsole2_contact(idx); rsole3_contact(idx)];
%     if (sum(rfContacts) == 4)
%         if (~showTrace && rfFirstPlanarContact)
%             % do nothing
%         else
%             rfFirstPlanarContact = true;
%             if (rfFirstPlanarContact)
%                 scatter3(rsole0_pos(idx, 1), rsole0_pos(idx, 2), rsole0_pos(idx, 3),'filled', 'MarkerEdgeColor', light_red,'MarkerFaceColor', light_red)
%                 scatter3(rsole1_pos(idx, 1), rsole1_pos(idx, 2), rsole1_pos(idx, 3),'filled', 'MarkerEdgeColor', light_red,'MarkerFaceColor', light_red)
%                 scatter3(rsole2_pos(idx, 1), rsole2_pos(idx, 2), rsole2_pos(idx, 3),'filled', 'MarkerEdgeColor', light_red,'MarkerFaceColor', light_red)
%                 scatter3(rsole3_pos(idx, 1), rsole3_pos(idx, 2), rsole3_pos(idx, 3),'filled', 'MarkerEdgeColor', light_red,'MarkerFaceColor', light_red)
%                 if showPolygon
%                     rsolex = [rsole2_pos(idx, 1) rsole0_pos(idx, 1) rsole1_pos(idx, 1) rsole3_pos(idx, 1)];
%                     rsoley = [rsole2_pos(idx, 2) rsole0_pos(idx, 2) rsole1_pos(idx, 2) rsole3_pos(idx, 2)];
%                     pgon = polyshape(rsolex, rsoley);
%                     rfrect = plot(pgon,'FaceColor',light_red,'FaceAlpha',0.1);
%                 end
%             end
%         end
%     else
%         rfFirstPlanarContact = false;
%     end
% end
% legend_line = [copline lfrect rfrect];
% legendtex = {'Global CoP' 'Left Foot' 'Right Foot'};
% legend(legend_line,legendtex,'FontSize', 22, 'Orientation', 'Vertical')
% set(gca,'FontSize',24, 'FontWeight', 'Bold')
% sgtitle('Estimated Contact Point Positions and Global CoP', ...
% 'FontSize', 32, 'FontWeight', 'Bold');
% grid on
