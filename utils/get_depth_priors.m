function [prob] = get_depth_priors(patch, foreground_hist, background_hist)

foreground_prob = zeros(size(patch));
background_prob = zeros(size(patch));

bin = floor(patch / 100) + 1;

regular_bin_indices = find(bin ~= 1);
foreground_prob(regular_bin_indices) = double(foreground_hist(bin(regular_bin_indices)));
background_prob(regular_bin_indices) = double(background_hist(bin(regular_bin_indices)));

other_bin_indices = find(bin == 1);
foreground_prob(other_bin_indices) = 0.0;
background_prob(other_bin_indices) = 1.0;

prob = (foreground_prob ) ./ (foreground_prob + background_prob);
end
