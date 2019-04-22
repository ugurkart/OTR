function [rgbd_plugin] = rgbd_plugin_init(rgb_img, depth_img, init_BB, visualize, debug)

rgbd_plugin.debug = debug;
rgbd_plugin.visualize = visualize;
rgbd_plugin.occlusion = false;

% segmentation parameters
rgbd_plugin.hist_lr = 0.04;
rgbd_plugin.nbins = 8;  % N bins for segmentation
rgbd_plugin.seg_colorspace = 'hsv';     % 'rgb' or 'hsv'
rgbd_plugin.use_segmentation = true;  % false to disable use of segmentation
rgbd_plugin.mask_diletation_type = 'disk';  % for function strel (square, disk, ...)
rgbd_plugin.mask_diletation_sz = 1;

rgbd_plugin.padding = 3;
rgbd_plugin.currentScaleFactor = 1.0;

% Adaptive occlusion recovery
rgbd_plugin.frame_counter = 1; % It will count the number of frames
rgbd_plugin.history = 30;
rgbd_plugin.response = zeros(rgbd_plugin.history, 1);
rgbd_plugin.response_threshold = 0.67;
rgbd_plugin.occ_detect_response_threshold = 0.67;
rgbd_plugin.occlusion = false;
rgbd_plugin.current_response = 1.0;
rgbd_plugin.detection_stride_coarse = 30;
rgbd_plugin.first_iteration = true;
rgbd_plugin.occlusion_threshold = 0.11;
rgbd_plugin.successful_frame_limit = 10;
rgbd_plugin.successful_frame_index = 1;
rgbd_plugin.consistency_radius = 50;
rgbd_plugin.consistent_velocity = [0 0];
rgbd_plugin.num_occluded_frames = 0;
rgbd_plugin.last_known_position = [];
rgbd_plugin.default_velocity = [5 5];
rgbd_plugin.occlusion_recovery_frame_no = 0;
rgbd_plugin.occlusion_frame_recovery_limit = 5;

rgbd_plugin.depth_hist_lr = 0.15; 
rgbd_plugin.background_extension = 20;
rgbd_plugin.edges = 0:100:10100;

% Multi-view DCF
rgbd_plugin.filter_index = 1;
rgbd_plugin.mdcf_period = 5;
rgbd_plugin.enlarge_factor = 1.07;

if(rgbd_plugin.seg_colorspace)
    seg_img = rgb2hsv(rgb_img);
    seg_img = seg_img * 255;
end

bb = init_BB;

bb = round(bb);

obj_reg = [bb(1), bb(2), bb(1)+bb(3), bb(2)+bb(4)] - [1 1 1 1];

base_target_sz = [bb(3), bb(4)];
rgbd_plugin.base_target_sz = base_target_sz;

% reference template size: [w, h], does not change during tracking
template_size = floor(base_target_sz + rgbd_plugin.padding*sqrt(prod(base_target_sz)));
template_size = mean(template_size);
template_size = [template_size, template_size];
rgbd_plugin.template_size = template_size;

c = bb([1,2]) + base_target_sz/2;

% Pass a dummy to make C++ happy
dummy_mask = uint8(zeros(size(seg_img, 1), size(seg_img, 2)));

rgbd_plugin.hist_fg = mex_extractforeground(seg_img, obj_reg, rgbd_plugin.nbins, dummy_mask, 0);
rgbd_plugin.hist_bg = mex_extractbackground(seg_img, obj_reg, rgbd_plugin.nbins);

% extract masked patch: mask out parts outside image
[seg_patch, valid_pixels_mask] = get_patch(seg_img, c, rgbd_plugin.currentScaleFactor, template_size);
[depth_seg_patch, depth_valid_pixels_mask] = get_patch(depth_img, c, rgbd_plugin.currentScaleFactor, template_size);

% Find the object bounding box region in the search region
patch_center_x = size(depth_seg_patch, 2) / 2;
patch_center_y = size(depth_seg_patch, 1) / 2;
object_region_x = max(1, patch_center_x - (bb(3) / 2));
object_region_y = max(1, patch_center_y - (bb(4) / 2));

object_region_bottom_x = min(size(depth_seg_patch, 2), object_region_x + bb(3));
object_region_bottom_y = min(size(depth_seg_patch, 1), object_region_y + bb(4));
rgbd_plugin.obj_region = [object_region_x object_region_y bb(3) bb(4)];

rgbd_plugin.target_region = hsv2rgb(seg_patch ./ 255);

depth_seg_patch = double(depth_seg_patch) .* double(depth_valid_pixels_mask); % Remove padded pixels

depth_img_object_region = depth_seg_patch(object_region_y:object_region_bottom_y, object_region_x:object_region_bottom_x); % Extract object region

[foreground_depth_hist, ~] = histcounts(depth_img_object_region, rgbd_plugin.edges); % Get histogram from object region

depth_valid_pixels_mask(object_region_y : object_region_bottom_y, object_region_x : object_region_bottom_x) = -1000; % Mark foreground pixels

background_pixels = find(depth_valid_pixels_mask == 1); % Background pixel indices on the image (padded pixels excluded)

[background_depth_hist, ~] = histcounts(depth_seg_patch(background_pixels), rgbd_plugin.edges);

[foreground_depth_hist, background_depth_hist] = process_histograms(foreground_depth_hist, background_depth_hist); % Add constant and normalize

rgbd_plugin.prev_foreground_depth_hist = foreground_depth_hist;
rgbd_plugin.prev_background_depth_hist = background_depth_hist;

[rgbd_plugin.depth_p] = get_depth_priors(depth_seg_patch, foreground_depth_hist, background_depth_hist); % Using the obtained histograms, estimate new depth priors
[fg_p, bg_p] = get_location_prior([1 1 size(seg_patch, 2) size(seg_patch, 1)], base_target_sz, [size(seg_patch,2), size(seg_patch, 1)]);

seg_patch = seg_patch .* valid_pixels_mask; % Remove padded pixels

tracker_bb_rect = uint16([object_region_y object_region_x object_region_bottom_y object_region_bottom_x]);

if(rgbd_plugin.debug == false)
    rgbd_plugin.search_region = 0;
    rgbd_plugin.color_p = 0;
else
    rgbd_plugin.search_region = hsv2rgb(seg_patch./255);
    color_p = get_color_priors(seg_patch, rgbd_plugin.hist_fg, rgbd_plugin.hist_bg, rgbd_plugin.nbins, tracker_bb_rect);
    rgbd_plugin.color_p = color_p .* valid_pixels_mask;
end

foreground_priors = rgbd_plugin.depth_p .* fg_p;
background_priors = (1.0 - rgbd_plugin.depth_p) .* bg_p;

[~, fg, ~] = mex_segment(seg_patch, rgbd_plugin.hist_fg, rgbd_plugin.hist_bg, rgbd_plugin.nbins, foreground_priors, background_priors);

% cut out regions outside from image
mask = single(fg).*single(valid_pixels_mask);
rgbd_plugin.mask = binarize_softmask(mask);

tracker_bb_region = zeros(size(rgbd_plugin.mask));
tracker_bb_region(object_region_y : object_region_bottom_y, object_region_x : object_region_bottom_x) = 1;

rgbd_plugin.mask = rgbd_plugin.mask .* tracker_bb_region;

non_zero_mask_indices = find(rgbd_plugin.mask > 0);

[foreground_depth_hist, ~] = histcounts(depth_seg_patch(non_zero_mask_indices), rgbd_plugin.edges); % Update foreground historgram

[foreground_depth_hist, background_depth_hist] = process_histograms(foreground_depth_hist, background_depth_hist);

rgbd_plugin.foreground_depth_hist = foreground_depth_hist;
rgbd_plugin.background_depth_hist = background_depth_hist;

% median distance of the object will be used as a reference distance
rgbd_plugin.use_median_ref = true;
if rgbd_plugin.use_median_ref
    rgbd_plugin.median_dist0 = median(depth_seg_patch(non_zero_mask_indices));
end

% Smooth depth updates
rgbd_plugin.gauss_win = gausswin(numel(rgbd_plugin.foreground_depth_hist), 15);

% Refine the histograms using the mask
int_mask = uint8(rgbd_plugin.mask);
seg_patch_obj_reg =  [object_region_x object_region_y object_region_bottom_x object_region_bottom_y];
rgbd_plugin.hist_fg = mex_extractforeground(seg_patch, seg_patch_obj_reg, rgbd_plugin.nbins, int_mask, 1);
rgbd_plugin.hist_bg = mex_extractbackground(seg_img, obj_reg, rgbd_plugin.nbins);

rgbd_plugin.first_iteration = false;

rgbd_plugin.c = c;
rgbd_plugin.bb = bb;

rgbd_plugin.consistent_centers(rgbd_plugin.successful_frame_index, :) = [rgbd_plugin.c(1) rgbd_plugin.c(2)];
rgbd_plugin.successful_frame_index = rgbd_plugin.successful_frame_index + 1;
rgbd_plugin.frame_no = 1;

original_coordinates_top_x = uint16(max((rgbd_plugin.c(1) - bb(3)/2), 1));
original_coordinates_top_y = uint16(max((rgbd_plugin.c(2) - bb(4)/2), 1));

original_coordinates_bottom_x = uint16(min((rgbd_plugin.c(1) + bb(3)/2), size(rgb_img, 1)));
original_coordinates_bottom_y = uint16(min((rgbd_plugin.c(2) + bb(4)/2), size(rgb_img, 2)));

masked_depth_seg_patch = depth_seg_patch.* rgbd_plugin.mask;
rgbd_plugin.prev_depth_foreground = zeros(size(depth_img));

rgbd_plugin.prev_rgb_foreground = zeros(size(rgb_img));
rgb_mask = zeros(size(rgbd_plugin.mask, 1), size(rgbd_plugin.mask, 2), 3);
rgb_mask(:,:,1) = rgbd_plugin.mask; rgb_mask(:,:,2) = rgbd_plugin.mask; rgb_mask(:,:,3) = rgbd_plugin.mask;
masked_seg_patch = seg_patch .* rgb_mask;

height = original_coordinates_bottom_y - original_coordinates_top_y;
width = original_coordinates_bottom_x - original_coordinates_top_x;

object_region_y = uint16(object_region_y);
object_region_x = uint16(object_region_x);

rgbd_plugin.prev_depth_foreground(original_coordinates_top_y:original_coordinates_top_y + height, original_coordinates_top_x:original_coordinates_top_x + width)...
    = masked_depth_seg_patch(object_region_y:object_region_y + height, object_region_x:object_region_x + width);

rgbd_plugin.prev_rgb_foreground(original_coordinates_top_y:original_coordinates_top_y + height, original_coordinates_top_x:original_coordinates_top_x + width, :)...
    = masked_seg_patch(object_region_y:object_region_y + height, object_region_x:object_region_x + width, :);

rgbd_plugin.prev_rgb_foreground = hsv2rgb(rgbd_plugin.prev_rgb_foreground./255);
rgbd_plugin.current_csr_foreground = hsv2rgb(masked_seg_patch./255);
rgbd_plugin.binary_mask = rgbd_plugin.prev_depth_foreground;
rgbd_plugin.binary_mask(find(rgbd_plugin.binary_mask > 0)) = 1;
rgbd_plugin.binary_mask = imresize(rgbd_plugin.binary_mask, [size(rgb_img, 1) size(rgb_img, 2)]);

end
