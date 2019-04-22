function [prob] = get_color_priors(patch, foreground_hist, background_hist, nBins, tracker_bb_rect)

foreground_prob = zeros(size(patch, 2), size(patch, 1), 1);
background_prob = zeros(size(patch, 2), size(patch, 1), 1);

for c = 1:size(patch, 2) %tracker_bb_rect(2) : tracker_bb_rect(4)
    for r = 1:size(patch, 1) %tracker_bb_rect(1): tracker_bb_rect(3)
        h = patch(r,c,1);
        s = patch(r,c,2);
        v = patch(r,c,3);
        
        h_bin = floor(h * nBins / 256);
        s_bin = floor(s * nBins / 256);
        v_bin = floor(v * nBins / 256);
        
        hist_index = uint16((nBins * nBins) * (h_bin) + (s_bin) * nBins + v_bin) +1;
        foreground_prob(r,c) = double(foreground_hist(hist_index));
        background_prob(r,c) = double(background_hist(hist_index));  
        
    end
end
eps = 10e-9; 
prob = (foreground_prob + eps) ./ (foreground_prob + background_prob + 2*eps); 
end

