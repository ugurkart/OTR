function [ histograms ] = get_depth_histograms( depth_img_roi, tracker )

% Calculate histograms for each block

num_cell_rows = tracker.depth_num_cells_vertical;
num_cell_cols = tracker.depth_num_cells_horizontal;

img_rows = size(depth_img_roi, 1);
img_cols = size(depth_img_roi, 2);

pad_rows = mod(img_rows, num_cell_rows);
pad_cols = mod(img_cols, num_cell_cols);

depth_img_roi = padarray(depth_img_roi, [pad_rows pad_cols], 'replicate');

cell_row_size = uint16(img_rows / num_cell_rows);
cell_col_size = uint16(img_cols / num_cell_cols);

fun = @(block_struct) histcounts(block_struct.data, tracker.depth_hist_bins);

histograms = blockproc(depth_img_roi, [cell_row_size cell_col_size], fun);

end

