function [ tracker, occlusion, out_of_plane_rotation ] = analyze_depth_histograms( tracker, current_depth_histograms )
% Analyze the current depth histogram with the previous depth_histograms,
% estimate whether there's occlusion or out of plane rotation

mean_depth_histogram = sum(tracker.depth_histograms);
mean_depth_histogram = mean_depth_histogram ./ double(max(1, tracker.frame_counter - 1));
mean_depth_histogram = reshape(mean_depth_histogram, [8 9]);

diff_depth_histogram = current_depth_histograms - tracker.diff_depth_histograms(tracker.frame_counter);

tracker.diff_depth_histograms(tracker.frame_counter,:,:) = diff_depth_histogram;

occlusion = false;
out_of_plane_rotation = false;

end

