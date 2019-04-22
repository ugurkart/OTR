function [ possible_occlusion, absolute_occlusion, tracker ] = analyze_mean_response( tracker )
% Compare the latest  response extracted from the object region to 
% figure out possible occlusions

non_zero_element_indices = find(tracker.response > 0.0);

mean_response = mean(mean(tracker.response(non_zero_element_indices)));

diff_percentage = double(tracker.current_response) / double(mean_response);

response_percentage = diff_percentage;
tracker.current_response_percentage = response_percentage;

possible_occlusion = false;
if(diff_percentage < tracker.peak_percentage_response)
    possible_occlusion = true;
end

if(diff_percentage < tracker.absolute_response_threshold)
    absolute_occlusion = true;
else
    absolute_occlusion = false;
end

end

