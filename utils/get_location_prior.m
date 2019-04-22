function [fg_prior, bg_prior, out_depth_priors, tracker] = get_location_prior(roi, depth_img, depth_median, target_sz, img_sz, tracker)

x1 = round(max(min(roi(1), img_sz(1)), 1));
y1 = round(max(min(roi(2), img_sz(2)), 1));
x2 = round(min(max(roi(3), 1), img_sz(1)));
y2 = round(min(max(roi(4), 1), img_sz(2)));

% make it rotationaly invariant
% target_sz = [min(target_sz) min(target_sz)];

wh_i = 1/(0.5*target_sz(1)*1.4142+1);
hh_i = 1/(0.5*target_sz(2)*1.4142+1);
cx = x1+0.5*(x2-x1);
cy = y1+0.5*(y2-y1);

fg_prior = kernelProfileMultiple((repmat((x1-cx):(x2-cx), [length(y1:y2) 1]).*wh_i).^2 + (repmat([(y1-cy):(y2-cy)]', [1 length(x1:x2)]).*hh_i).^2);

fg_prior = double(fg_prior./max(fg_prior(:)));

fg_prior(fg_prior < 0.5) = 0.5;
fg_prior(fg_prior > 0.9) = 0.9;
bg_prior = double(1-fg_prior);

out_depth_priors = fg_prior; % Dummy to make Matlab happy
end

function [val] = kernelProfileMultiple(x)
idx = x > 1;
val = (2/3.14)*(ones(size(x,1), size(x,2))-x);
val(idx) = 0;
end

function [val] = calculatePriorAccordingToDepth(patch, median_value)

% val = sqrt(1.0 ./ double((double(patch) - double(median_value)).^2 + eps('double')));

val = 1 ./ ((exp(sqrt(abs(double(patch) - repmat(double(median_value), size(patch))))) + eps('double')));
val = log(val);
inf_indices = find(val == -Inf);
val(inf_indices) = -1000;
max_depth_prior = max(max(val));
min_depth_prior = min(min(val));

val = (val - min_depth_prior) / (max_depth_prior - min_depth_prior);

end

function [foreground_val, background_val] = calculatePriorWithGaussian(patch, tracker)

foreground_prob = cdf(tracker.foreground_dist, patch(:,:)+1) - cdf(tracker.foreground_dist, patch(:,:)) + eps('double');
background_prob = cdf(tracker.background_dist, patch(:,:)+1) - cdf(tracker.background_dist, patch(:,:)) + eps('double');

foreground_val = log(foreground_prob) - log(background_prob);
background_val = log(background_prob) - log(foreground_prob);

foreground_val_negative_indices = find(foreground_val < 0);
background_val_negative_indices = find(background_val < 0);

foreground_val(foreground_val_negative_indices) = 0.0;
background_val(background_val_negative_indices) = 0.0;

max_foreground_val = max(max(foreground_val));
min_foreground_val = min(min(foreground_val));

max_background_val = max(max(background_val));
min_background_val = min(min(background_val));

foreground_val = (foreground_val - min_foreground_val) / (max_foreground_val - min_foreground_val);
background_val = (background_val - min_background_val) / (max_background_val - min_background_val);
background_val = -background_val;

val = foreground_val + background_val;

end
