function [rgbd_plugin, c, bb, resp_score, temp_params] = occlusion_recovery(rgbd_plugin, tracker, rgb_img)

[max_response, c, bb, temp_params, rgbd_plugin] = target_redetect(rgb_img, tracker, rgbd_plugin);

c = double(c);
rgbd_plugin.current_response = max_response;

rgbd_plugin.bb = bb;
resp_score = max_response;

[response_occlusion_state, rgbd_plugin] = analyze_mean_response(rgbd_plugin);

if(response_occlusion_state == false)
    rgbd_plugin.first_iteration = true;
    rgbd_plugin.occlusion = false;  
else
    if rgbd_plugin.use_median_ref
        temp_params.scale_factor = tracker.currentScaleFactor;
    end
    return;
end

end

