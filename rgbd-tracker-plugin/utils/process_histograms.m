function [foreground_hist, background_hist] = process_histograms(foreground_hist, background_hist)

    % Assign first bin (near zero values are assumed to be at infinity due to
    % the measurement errors
    
    foreground_hist_first_bin_value = foreground_hist(1, 1);
    foreground_hist_last_bin_value = foreground_hist(1, end);
    foreground_hist(1, end) = foreground_hist_first_bin_value;
    foreground_hist(1, 1) = foreground_hist_last_bin_value;
    
    background_hist_first_bin_value = background_hist(1, 1);
    background_hist_last_bin_value = background_hist(1, end);
    background_hist(1, end) = background_hist_first_bin_value;
    background_hist(1, 1) = background_hist_last_bin_value;
    
    sum_foreground_hist = double(sum(sum(foreground_hist)));
    sum_background_hist = double(sum(sum(background_hist)));
     
    % Normalize
    foreground_hist = foreground_hist ./ sum_foreground_hist;
    background_hist = background_hist ./ sum_background_hist;

    max_foreground_hist_peak_index = find(foreground_hist == max(max(foreground_hist)));
    max_foreground_hist_peak_index = max_foreground_hist_peak_index(1);

    foreground_regularizer = 0.02;
    background_regularizer = 0.20;
    
    num_foreground_hist = double(numel(foreground_hist));
    num_background_hist = double(numel(background_hist));
    
%     percentage_for_foreground_per_bin = foreground_regularizer / num_foreground_hist;
    foreground_addition = zeros(size(foreground_hist)); 
    % Triangle shaped addition
    for i=1:num_foreground_hist
        foreground_addition(1, i) = (1.0 - (abs(i - max_foreground_hist_peak_index) / num_foreground_hist)) * foreground_regularizer;
    end

    percentage_for_background_per_bin = background_regularizer / num_background_hist;

    background_addition = zeros(size(background_hist));

%     foreground_addition(:,:) = percentage_for_foreground_per_bin;
    background_addition(:,:) = percentage_for_background_per_bin;

    foreground_hist = foreground_hist + foreground_addition;
    background_hist = background_hist + background_addition;
    
    sum_foreground_hist = double(sum(sum(foreground_hist)));
    sum_background_hist = double(sum(sum(background_hist)));
     
    % Normalize
    foreground_hist = foreground_hist ./ sum_foreground_hist;
    background_hist = background_hist ./ sum_background_hist;
   
end

