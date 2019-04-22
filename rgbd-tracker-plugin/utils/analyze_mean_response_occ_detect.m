function [ occlusion_state, rgbd_plugin ] = analyze_mean_response( rgbd_plugin )
% Compare the latest  response extracted from the object region to 
% figure out possible occlusions

non_zero_element_indices = find(rgbd_plugin.response > 0.0);

mean_response = mean(mean(rgbd_plugin.response(non_zero_element_indices)));

diff_percentage = double(rgbd_plugin.current_response) / double(mean_response);

response_percentage = diff_percentage;
rgbd_plugin.current_response_percentage = response_percentage;

occlusion_state = false;
if(diff_percentage < rgbd_plugin.occ_detect_response_threshold)
    occlusion_state = true;
end

end

