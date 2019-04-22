function [rgbd_plugin] = rgbd_plugin_process_frame(rgb_img, depth_img, c, bb, rgbd_plugin, tracker)

if strcmp(rgbd_plugin.seg_colorspace, 'rgb')
    seg_img = rgb_img;
elseif strcmp(rgbd_plugin.seg_colorspace, 'hsv')
    seg_img = rgb2hsv(rgb_img);
    seg_img = seg_img * 255;
else
    error('Unknown colorspace parameter');
end

obj_reg = round([bb(1), bb(2), bb(1) + bb(3), bb(2) + bb(4)]) - [1 1 1 1];

% extract masked patch: mask out parts outside image
[seg_patch, valid_pixels_mask] = get_patch(seg_img, c, rgbd_plugin.currentScaleFactor, rgbd_plugin.template_size);
[depth_seg_patch, depth_valid_pixels_mask] = get_patch(depth_img, c, rgbd_plugin.currentScaleFactor, rgbd_plugin.template_size);

depth_seg_patch = double(depth_seg_patch) .* depth_valid_pixels_mask; % Remove padded pixels

patch_center_x = size(depth_seg_patch, 2) / 2;
patch_center_y = size(depth_seg_patch, 1) / 2;
object_region_x = max(1, patch_center_x - (bb(3) / 2));
object_region_y = max(1, patch_center_y - (bb(4) / 2));

object_region_bottom_x = min(size(depth_seg_patch, 2), object_region_x + bb(3));
object_region_bottom_y = min(size(depth_seg_patch, 1), object_region_y + bb(4));
rgbd_plugin.obj_region = [object_region_x object_region_y bb(3) bb(4)];

rgbd_plugin.target_region = hsv2rgb(seg_patch ./ 255);
if(rgbd_plugin.first_iteration == true) % Reset the depth histograms after occlusion recovery
    
    depth_seg_patch = depth_seg_patch .* depth_valid_pixels_mask; % Remove padded pixels
    
    depth_img_object_region = depth_seg_patch(object_region_y:object_region_bottom_y, object_region_x:object_region_bottom_x); % Extract object region
    
    [foreground_depth_hist, ~] = histcounts(depth_img_object_region, rgbd_plugin.edges); % Get histogram from object region
    
    depth_valid_pixels_mask(object_region_y : object_region_bottom_y, object_region_x :object_region_bottom_x) = -1000; % Mark foreground pixels
    
    background_pixels = find(depth_valid_pixels_mask == 1); % Background pixel indices on the image
    
    [background_depth_hist, ~] = histcounts(depth_seg_patch(background_pixels), rgbd_plugin.edges);
    
    [foreground_depth_hist, background_depth_hist] = process_histograms(foreground_depth_hist, background_depth_hist);
    
    rgbd_plugin.prev_foreground_depth_hist = foreground_depth_hist;
    rgbd_plugin.prev_background_depth_hist = background_depth_hist;
    
    rgbd_plugin.foreground_depth_hist = foreground_depth_hist;
    rgbd_plugin.background_depth_hist = background_depth_hist;
end

tracker_bb_rect = uint16([object_region_y object_region_x object_region_bottom_y object_region_bottom_x]);

[rgbd_plugin.depth_p] = get_depth_priors(depth_seg_patch, rgbd_plugin.foreground_depth_hist, rgbd_plugin.background_depth_hist);

[fg_p, bg_p] = get_location_prior([1 1 size(seg_patch, 2) size(seg_patch, 1)], rgbd_plugin.currentScaleFactor * rgbd_plugin.base_target_sz, [size(seg_patch,2), size(seg_patch, 1)]);

seg_patch = double(seg_patch) .* valid_pixels_mask; % Remove padded pixels

foreground_priors = rgbd_plugin.depth_p .* fg_p;
background_priors = (1.0 - rgbd_plugin.depth_p) .* bg_p;

[~, fg, ~] = mex_segment(seg_patch, rgbd_plugin.hist_fg, rgbd_plugin.hist_bg, rgbd_plugin.nbins, foreground_priors, background_priors);

% cut out regions outside from image
mask = single(fg).*single(valid_pixels_mask);
mask = binarize_softmask(mask);

bb_area = ((object_region_bottom_y - object_region_y) * (object_region_bottom_x - object_region_x));

tracker_bb_region = zeros(size(mask));
tracker_bb_region(object_region_y : object_region_bottom_y, object_region_x : object_region_bottom_x) = 1;

mask = mask .* tracker_bb_region;
rgbd_plugin.mask = mask;

non_zero_foreground_indices = find(mask > 0); % Check if we were able to segment enough

rgbd_plugin.tracker_update_flag = true;
if((numel(non_zero_foreground_indices) < bb_area * rgbd_plugin.occlusion_threshold))
    rgbd_plugin.tracker_update_flag = false;
end

[ occlusion_state, rgbd_plugin ] = analyze_mean_response(rgbd_plugin);

if(rgbd_plugin.occlusion == true)
    if(occlusion_state && (numel(non_zero_foreground_indices) < bb_area * rgbd_plugin.occlusion_threshold))
        rgbd_plugin.occlusion = true;
        return;
    else
        rgbd_plugin.first_iteration = true;
        rgbd_plugin.occlusion = false;
    end
else
    if(occlusion_state && (numel(non_zero_foreground_indices) < bb_area * rgbd_plugin.occlusion_threshold))
        if(rgbd_plugin.first_iteration == false)
            if(((rgbd_plugin.frame_no - rgbd_plugin.occlusion_recovery_frame_no) > rgbd_plugin.occlusion_frame_recovery_limit) || (rgbd_plugin.frame_no <= rgbd_plugin.occlusion_frame_recovery_limit))
                rgbd_plugin.last_known_position = c;
                rgbd_plugin.last_bb = bb;
                rgbd_plugin.consistent_velocity = zeros(1, 2);
                rgbd_plugin.num_occluded_frames = 0;
                rgbd_plugin.successful_frame_index = 1;
                rgbd_plugin.consistent_centers = [];
                rgbd_plugin.consistent_centers(rgbd_plugin.successful_frame_index, :) = c;
                rgbd_plugin.successful_frame_index = rgbd_plugin.successful_frame_index + 1;
                rgbd_plugin.filters(rgbd_plugin.filter_index, :, :, :) = zeros(size(tracker.H));
                rgbd_plugin.filters(rgbd_plugin.filter_index, :, : ,:) = tracker.H; % Add the latest filter as well
                rgbd_plugin.filter_index = rgbd_plugin.filter_index + 1;
            end
        end
        rgbd_plugin.occlusion = true;
        
        return;
    end
end

if(rgbd_plugin.debug == false)
    search_region = 0;
    color_p = 0;
    color_priors = 0;
else
    if(rgbd_plugin.occlusion == false)
        search_region = hsv2rgb(seg_patch./255);
    end
    color_p = get_color_priors(seg_patch, rgbd_plugin.hist_fg, rgbd_plugin.hist_bg, rgbd_plugin.nbins, tracker_bb_rect);
    color_p = color_p .* valid_pixels_mask;
    rgbd_plugin.color_p = color_p;
end

rgbd_plugin.first_iteration =  false;

if(rgbd_plugin.successful_frame_index > 1)
    tmp_center_dist = sqrt((c(1) - rgbd_plugin.consistent_centers(rgbd_plugin.successful_frame_index - 1, 1))^2 + (c(2) - rgbd_plugin.consistent_centers(rgbd_plugin.successful_frame_index - 1, 2))^2);
else
    tmp_center_dist = sqrt((c(1) - rgbd_plugin.consistent_centers(rgbd_plugin.successful_frame_limit, 1))^2 + (c(2) - rgbd_plugin.consistent_centers(rgbd_plugin.successful_frame_limit, 2))^2);
end

if(tmp_center_dist <= rgbd_plugin.consistency_radius)
    % Count the consistent trackings
    rgbd_plugin.consistent_centers(rgbd_plugin.successful_frame_index, :) = c;
    rgbd_plugin.successful_frame_index = rgbd_plugin.successful_frame_index + 1;
    
    if(rgbd_plugin.successful_frame_index > rgbd_plugin.successful_frame_limit)
        rgbd_plugin.successful_frame_index = 1;
    end
    
else % Reset counter
    rgbd_plugin.successful_frame_index = 1;
    rgbd_plugin.consistent_centers = [];
    rgbd_plugin.consistent_centers(rgbd_plugin.successful_frame_index, :) = c;
    rgbd_plugin.successful_frame_index = rgbd_plugin.successful_frame_index + 1;
end

if((numel(non_zero_foreground_indices) > bb_area * rgbd_plugin.occlusion_threshold)) % Update the histograms only if there's enough fg pixels
    
    [foreground_depth_hist, ~] = histcounts(depth_seg_patch(non_zero_foreground_indices), rgbd_plugin.edges); % Get histogram from the segmented region
    
    depth_valid_pixels_mask(object_region_y : object_region_bottom_y, object_region_x : object_region_bottom_x) = -1000; % Mark foreground pixels
    
    background_pixels = find(depth_valid_pixels_mask == 1); % Background pixel indices on the image
    
    [background_depth_hist, ~] = histcounts(depth_seg_patch(background_pixels), rgbd_plugin.edges);
    
    [foreground_depth_hist, background_depth_hist] = process_histograms(foreground_depth_hist, background_depth_hist);
    
    rgbd_plugin.prev_foreground_depth_hist = rgbd_plugin.foreground_depth_hist;
    rgbd_plugin.prev_background_depth_hist = rgbd_plugin.background_depth_hist;
    
    max_foreground_depth_hist = max(max(rgbd_plugin.foreground_depth_hist));
    max_foreground_depth_hist_index = find(rgbd_plugin.foreground_depth_hist == max_foreground_depth_hist);
    
    max_gauss_index = find(rgbd_plugin.gauss_win == max(max(rgbd_plugin.gauss_win)));
    max_gauss_index = max_gauss_index(1);
    
    shift_offset = numel(rgbd_plugin.foreground_depth_hist) - max_gauss_index + max_foreground_depth_hist_index;
    
    current_gauss_win = circshift(rgbd_plugin.gauss_win, shift_offset);
    
    rgbd_plugin.foreground_depth_hist = (1-rgbd_plugin.depth_hist_lr)*rgbd_plugin.foreground_depth_hist + ((rgbd_plugin.depth_hist_lr*foreground_depth_hist) .* current_gauss_win');
    rgbd_plugin.background_depth_hist = (1-rgbd_plugin.depth_hist_lr)*rgbd_plugin.background_depth_hist + rgbd_plugin.depth_hist_lr*background_depth_hist;
end
rgbd_plugin.current_rgb_foreground = zeros(size(rgb_img));
rgb_mask = zeros(size(rgbd_plugin.mask, 1), size(rgbd_plugin.mask, 2), 3);
rgb_mask(:,:,1) = rgbd_plugin.mask; rgb_mask(:,:,2) = rgbd_plugin.mask; rgb_mask(:,:,3) = rgbd_plugin.mask;
masked_seg_patch = seg_patch .* rgb_mask;

rgbd_plugin.current_csr_foreground = hsv2rgb(masked_seg_patch./255);

binary_mask = rgbd_plugin.mask;
[rows, cols, values] = find(binary_mask > 0);

written_binary_mask = zeros(size(rgb_img, 1), size(rgb_img, 2), 1);

x_diff = c(1) - patch_center_x;
y_diff = c(2) - patch_center_y;

shifted_rows = uint16(rows+y_diff);
shifted_cols = uint16(cols+x_diff);

%% Make sure everything is inside the boundaries
valid_shifted_row_indices = find((shifted_rows > 0) & (shifted_rows <= size(rgb_img, 1)));
valid_shifted_col_indices = find((shifted_cols > 0) & (shifted_cols <= size(rgb_img, 2)));

logical_shifted_row_indices = zeros(length(shifted_rows), 1);
logical_shifted_col_indices = logical_shifted_row_indices;

logical_shifted_row_indices(valid_shifted_row_indices) = 1;
logical_shifted_col_indices(valid_shifted_col_indices) = 1;

logical_shifted_common_indices = logical_shifted_row_indices .* logical_shifted_col_indices;
valid_same_indices = find(logical_shifted_common_indices == 1);

linearInd = sub2ind(size(rgb_img), shifted_rows(valid_same_indices), shifted_cols(valid_same_indices));

written_binary_mask(linearInd) = 1;

imwrite(uint8(written_binary_mask), [rgbd_plugin.mask_folder 'Mask_' num2str(rgbd_plugin.frame_no, '%08d') '.png']);
fprintf(rgbd_plugin.tcp_connection, "PROCESS_FRAME");
while(rgbd_plugin.tcp_connection.BytesAvailable == 0)
end
received_data = fread(rgbd_plugin.tcp_connection, rgbd_plugin.tcp_connection.BytesAvailable, 'char');
received_data = char(received_data);

fprintf(rgbd_plugin.tcp_connection, num2str(rgbd_plugin.frame_no));

while(rgbd_plugin.tcp_connection.BytesAvailable == 0)
end

received_data = fread(rgbd_plugin.tcp_connection, rgbd_plugin.tcp_connection.BytesAvailable, 'char');
received_data = char(received_data);

while(strcmp(received_data', ['Frame_' num2str(rgbd_plugin.frame_no)]) == false)
    while(rgbd_plugin.tcp_connection.BytesAvailable == 0)
    end
    received_data = fread(rgbd_plugin.tcp_connection, rgbd_plugin.tcp_connection.BytesAvailable, 'char');
    received_data = char(received_data);
end
disp(received_data');
disp('Frame processed on C++ side');

icp_error = 0;
if(rgbd_plugin.frame_no > 2)
    fileID = fopen(rgbd_plugin.icp_error_file, 'r');
    icp_error = fscanf(fileID, '%f');
    fclose(fileID);
end

rgbd_plugin.icp_error = icp_error;

rgbd_plugin.use_csr_mask = false;

projected_patch = imread([rgbd_plugin.projections_folder '/projected_img_' num2str(rgbd_plugin.frame_no) '.png']);

if(rgbd_plugin.frame_no > 1)
    if((icp_error < rgbd_plugin.icp_error_threshold) & (length(find(projected_patch > 0)) > 500)) % Make sure projected mask isn't empty
        % Shift the center of the projection image to the center of the
        % segmentation patch for consistency
        
        optimal_projected_bb = fit_bbox_on_mask(projected_patch, 'ratio');
        rgbd_plugin.optimal_projected_bb = optimal_projected_bb;
        rgbd_plugin.optimal_bb_aspect_ratio = double(optimal_projected_bb(3)) / double(optimal_projected_bb(4));
        
        [D, valid_] = get_patch(projected_patch, c, rgbd_plugin.currentScaleFactor, rgbd_plugin.template_size);
        rgbd_plugin.current_projected_foreground = hsv2rgb(seg_patch ./ 255) .* double(repmat(D,[1,1,3]));
        int_mask = uint8(D);
        rgbd_plugin.current_projected_mask = int_mask;
    else
        rgbd_plugin.use_csr_mask = true;
        int_mask = uint8(rgbd_plugin.mask);
    end
else
    rgbd_plugin.use_csr_mask = true;
    int_mask = uint8(rgbd_plugin.mask);
    rgbd_plugin.optimal_bb_aspect_ratio
end

rgbd_plugin.update_mask = int_mask;
seg_patch_obj_reg =  [object_region_x object_region_y object_region_bottom_x object_region_bottom_y];

if(rgbd_plugin.use_csr_mask == false)
    hist_fg = mex_extractforeground(seg_img, obj_reg, rgbd_plugin.nbins, uint8(projected_patch), 1);
    hist_bg = mex_extractbackground(seg_img, obj_reg, rgbd_plugin.nbins);
else
    hist_fg = mex_extractforeground(seg_patch, seg_patch_obj_reg, rgbd_plugin.nbins, int_mask, 1);
    hist_bg = mex_extractbackground(seg_img, obj_reg, rgbd_plugin.nbins);
end
rgbd_plugin.hist_fg = (1-rgbd_plugin.hist_lr) * rgbd_plugin.hist_fg + rgbd_plugin.hist_lr * hist_fg;
rgbd_plugin.hist_bg = (1-rgbd_plugin.hist_lr) * rgbd_plugin.hist_bg + rgbd_plugin.hist_lr * hist_bg;

end

