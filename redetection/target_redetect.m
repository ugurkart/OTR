function [max_response, c, out_bb, out_temp_params, rgbd_plugin] = target_redetect(img, ...
    tracker, rgbd_plugin)

% last target position when tracking was considered reliable
last_c = rgbd_plugin.last_known_position;
% enlarge search region of the tracker (exponentially growing with number
% of frames since tracking was reliable for the last time)
enlarge_factor =  rgbd_plugin.enlarge_factor ^ rgbd_plugin.num_occluded_frames; 
% assume that filter is square
resize_factor = (tracker.template_size(1) * tracker.currentScaleFactor) ./ ...
    tracker.rescale_template_size(1);

% is it correct to use currentScaleFactor of the tracker?
extraction_sz = round(tracker.currentScaleFactor .* enlarge_factor .* ...
    tracker.template_size);

% top-left and bottom-right coordinates of detector search region
x0 = round(last_c(1) - extraction_sz(1) / 2);
y0 = round(last_c(2) - extraction_sz(2) / 2);
x1 = x0 + extraction_sz(1);
y1 = y0 + extraction_sz(2);

% bound to within-image coordinates
if x0 < 1
    x0 = 1;
end
if y0 < 1
    y0 = 1;
end
if x1 > size(img,2)
    x1 = size(img,2);
end
if y1 > size(img,1)
    y1 = size(img,1);
end

% extract patch
xs = x0:x1;
ys = y0:y1;
patch = img(ys, xs, :);

if(rgbd_plugin.debug)
    rgbd_plugin.search_region = patch;
end

% resize the patch to reference size
patch_resized = imresize(patch, 1/resize_factor);

% extract features from given patch
[img_f, ~] = extract_features_only(patch_resized, [], tracker.feature_type, ...
    tracker.w2c, tracker.cell_size);

% Fourier transform features
F_img = fft2(img_f);

max_score = 0;
row_img = 0;
col_img = 0;
if(rgbd_plugin.filter_index > 1)
    for filter_index = 1:rgbd_plugin.filter_index-1
        
        h_img = zeros(size(img_f,1), size(img_f,2), size(img_f,3));
        % filter of the tracker
        current_H(:, :, :) = rgbd_plugin.filters(filter_index, :, :, :);
        %     h = real(ifft2(tracker.H));
        h = real(ifft2(current_H));
        
        % create image-sized zero-padded filter
        % calculate indexes for inserting filter into zero-paded matrix
        if size(h_img,1) > size(h,1)
            y1_img = max(1, floor(size(h_img,1) / 2 - size(h,1) / 2));
            y1_h = 1;
            y2_img = y1_img + size(h,1) - 1;
            y2_h = size(h,1);
        elseif size(h_img,1) < size(h,1)
            y1_img = 1;
            y1_h = max(1, floor(size(h,1) / 2 - size(h_img,1) / 2));
            y2_img = size(h_img,1);
            y2_h = y1_h + size(h_img,1) - 1;
        else
            y1_img = 1;
            y1_h = 1;
            y2_img = size(h_img,1);
            y2_h = size(h,1);
        end
        if size(h_img,2) > size(h,2)
            x1_img = max(1, floor(size(h_img,2) / 2 - size(h,2) / 2));
            x1_h = 1;
            x2_img = x1_img + size(h,2) - 1;
            x2_h = size(h,2);
        elseif size(h_img,2) < size(h,2)
            x1_img = 1;
            x1_h = max(1, floor(size(h,2) / 2 - size(h_img,2) / 2));
            x2_img = size(h_img,2);
            x2_h = x1_h + size(h_img,2) - 1;
        else
            x1_img = 1;
            x1_h = 1;
            x2_img = size(h_img,2);
            x2_h = size(h,2);
        end
        % insert filter and transform it to Fourier
        h_img(y1_img:y2_img, x1_img:x2_img, :) = h(y1_h:y2_h, x1_h:x2_h, :);
        H_img = fft2(h_img);
        
        % correlation of the filter and image
        response_chann_img = real(ifft2(F_img.*conj(H_img)));
        chann_w = ones(size(response_chann_img,3), 1) * ...
            (1.0 / size(response_chann_img,3));
        response_img = sum(bsxfun(@times, response_chann_img, ...
            reshape(chann_w, 1, 1, size(response_chann_img,3))), 3);
        
        % fftshift response to get actual center of the target
        response_img = fftshift(response_img);
        
        % calculate target position estimated with detector
        [tmp_row_img, tmp_col_img] = ind2sub(size(response_img), find(response_img == max(response_img(:)), 1));
        
        if(max(response_img(:)) > max_score)
            max_score = max(response_img(:));
            v_neighbors_img = response_img(mod(tmp_row_img + [-1, 0, 1] - 1, ...
                size(response_img,1)) + 1, tmp_col_img);
            h_neighbors_img = response_img(tmp_row_img, ...
                mod(tmp_col_img + [-1, 0, 1] - 1, size(response_img,2)) + 1);
            row_img = tmp_row_img + subpixel_peak(v_neighbors_img);
            col_img = tmp_col_img + subpixel_peak(h_neighbors_img);
            tracker.H = current_H;
        end
    end
else
    h_img = zeros(size(img_f,1), size(img_f,2), size(img_f,3));

    h = real(ifft2(tracker.H));
    
    % create image-sized zero-padded filter
    % calculate indexes for inserting filter into zero-paded matrix
    if size(h_img,1) > size(h,1)
        y1_img = max(1, floor(size(h_img,1) / 2 - size(h,1) / 2));
        y1_h = 1;
        y2_img = y1_img + size(h,1) - 1;
        y2_h = size(h,1);
    elseif size(h_img,1) < size(h,1)
        y1_img = 1;
        y1_h = max(1, floor(size(h,1) / 2 - size(h_img,1) / 2));
        y2_img = size(h_img,1);
        y2_h = y1_h + size(h_img,1) - 1;
    else
        y1_img = 1;
        y1_h = 1;
        y2_img = size(h_img,1);
        y2_h = size(h,1);
    end
    if size(h_img,2) > size(h,2)
        x1_img = max(1, floor(size(h_img,2) / 2 - size(h,2) / 2));
        x1_h = 1;
        x2_img = x1_img + size(h,2) - 1;
        x2_h = size(h,2);
    elseif size(h_img,2) < size(h,2)
        x1_img = 1;
        x1_h = max(1, floor(size(h,2) / 2 - size(h_img,2) / 2));
        x2_img = size(h_img,2);
        x2_h = x1_h + size(h_img,2) - 1;
    else
        x1_img = 1;
        x1_h = 1;
        x2_img = size(h_img,2);
        x2_h = size(h,2);
    end
    % insert filter and transform it to Fourier
    h_img(y1_img:y2_img, x1_img:x2_img, :) = h(y1_h:y2_h, x1_h:x2_h, :);
    H_img = fft2(h_img);
    
    % correlation of the filter and image
    response_chann_img = real(ifft2(F_img.*conj(H_img)));
    chann_w = ones(size(response_chann_img,3), 1) * ...
        (1.0 / size(response_chann_img,3));
    response_img = sum(bsxfun(@times, response_chann_img, ...
        reshape(chann_w, 1, 1, size(response_chann_img,3))), 3);
    
    % fftshift response to get actual center of the target
    response_img = fftshift(response_img);
    
    % calculate target position estimated with detector
    [row_img, col_img] = ind2sub(size(response_img), find(response_img == max(response_img(:)), 1));
    
    v_neighbors_img = response_img(mod(row_img + [-1, 0, 1] - 1, ...
        size(response_img,1)) + 1, col_img);
    h_neighbors_img = response_img(row_img, ...
        mod(col_img + [-1, 0, 1] - 1, size(response_img,2)) + 1);
    row_img = row_img + subpixel_peak(v_neighbors_img);
    col_img = col_img + subpixel_peak(h_neighbors_img); 
end
% project new target position to image coordinates
resize_factor_inv = tracker.cell_size * resize_factor;
dx = resize_factor_inv * (col_img - 1);
dy = resize_factor_inv * (row_img - 1);

% add center to the top-left coordinates of search region
% to get actual target position in the image
new_c = [x0 + dx, y0 + dy];

if rgbd_plugin.use_median_ref
    % calculate median distance of the target and calculate 
    % ration between the current distance and reference distance
    % (reference distance is calculated in the first frame)
    [depth_seg_patch, depth_valid_pixels_mask] = get_patch(rgbd_plugin.depth_img, ...
        new_c, rgbd_plugin.currentScaleFactor, rgbd_plugin.template_size);
    depth_seg_patch = double(depth_seg_patch) .* depth_valid_pixels_mask;
    non_zero_foreground_indices = find(imresize(tracker.target_dummy_mask, size(depth_seg_patch)) > 0);
    target_med_dist = median(depth_seg_patch(non_zero_foreground_indices));

    if target_med_dist > 100 && rgbd_plugin.median_dist0 > 100
        dist_ratio = rgbd_plugin.median_dist0 / target_med_dist;
        tracker.currentScaleFactor = dist_ratio;
    end
end

% re-evaluate tracker on predicted position
% Note: flag occlusion=false is set to false since we want to correct
% also position which is obtained with image-wide redetection filter
[bb, resp_score, out_temp_params] = tracker_process_frame(tracker, ...
    new_c, img, false);

max_response = resp_score;
c = out_temp_params.c;
out_bb = bb;

if rgbd_plugin.use_median_ref
    % set new scale factor
    out_temp_params.scale_factor = tracker.currentScaleFactor;
end

end  % endfunction


function delta = subpixel_peak(p)
%parabola model (2nd order fit)
delta = 0.5 * (p(3) - p(1)) / (2 * p(2) - p(3) - p(1));
if ~isfinite(delta), delta = 0; end
end  % endfunction
