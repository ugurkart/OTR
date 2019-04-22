function [rgbd_plugin] = visualize_all(rgbd_plugin, rgb_img, bb)

patch_center_x = size(rgbd_plugin.target_region, 2) / 2;
patch_center_y = size(rgbd_plugin.target_region, 1) / 2;
object_region_x = max(1, patch_center_x - (bb(3) / 2));
object_region_y = max(1, patch_center_y - (bb(4) / 2));

obj_reg = [object_region_x object_region_y bb(3) bb(4)];

if size(rgb_img,3) == 1
    rgb_img = repmat(rgb_img, [1 1 3]);
end

if (rgbd_plugin.frame_no == 1)  %first frame, create GUI
    rgbd_plugin.fig_handle = figure('Name', 'Tracking');
    set(rgbd_plugin.fig_handle, 'Position', [100, 100, 2 * size(rgb_img,2), 2 * size(rgb_img,1)]);
else
    figure(rgbd_plugin.fig_handle);
end
imshow(rgb_img);
hold on;
text(15, 25, int2str(rgbd_plugin.frame_no), 'color', [0 1 1], 'FontSize', 15, 'FontWeight', 'bold');

if(rgbd_plugin.occlusion)
    rectangle('Position',obj_reg, 'EdgeColor','r', 'LineWidth',2);
    text(10, 40, 'Occlusion', 'color', [1 0 0]);
else
    rectangle('Position', bb, 'EdgeColor','g', 'LineWidth',2);
end

axis on
hold off
drawnow
end

