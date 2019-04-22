function [rgbd_plugin] = rgbd_plugin_update(rgbd_plugin, c, bb, scaleFactor, resp_score)
rgbd_plugin.c = c;
rgbd_plugin.bb = bb;
rgbd_plugin.currentScaleFactor = scaleFactor;

% if(rgbd_plugin.occlusion == false)
%     rgbd_plugin.consistent_centers(rgbd_plugin.successful_frame_index, :) = c;
% %     rgbd_plugin.successful_frame_index = rgbd_plugin.successful_frame_index + 1;
%     
%     rgbd_plugin.response(rgbd_plugin.frame_counter) = resp_score;
%     
%     if(mod(rgbd_plugin.frame_counter, rgbd_plugin.history) == 0)
%         rgbd_plugin.frame_counter = 1;
%     else
%         rgbd_plugin.frame_counter = rgbd_plugin.frame_counter + 1;
%     end
% end

end

