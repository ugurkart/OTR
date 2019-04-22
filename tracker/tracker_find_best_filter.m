function [max_score, c, output_BB, temp_params, rgbd_plugin, tracker] = tracker_find_best_filter(img, tracker, rgbd_plugin, last_c)

f = get_csr_features(img, last_c, tracker.currentScaleFactor, tracker.template_size, ...
    tracker.rescale_template_size, tracker.cos_win, tracker.feature_type, tracker.w2c, tracker.cell_size);

max_score = 0;

if(rgbd_plugin.filter_index > 1)
    % Add current tracker to the evaluated filters list
    rgbd_plugin.filters(rgbd_plugin.filter_index, :, :, :) = zeros(size(tracker.H));
    rgbd_plugin.filters(rgbd_plugin.filter_index, :, : ,:) = tracker.H;
    current_filter_index =  rgbd_plugin.filter_index;
    rgbd_plugin.filter_index = rgbd_plugin.filter_index + 1;
    
    chosen_filter_index = 0;
    
    for filter_index = 1:rgbd_plugin.filter_index-1
        
        % filter of the tracker
        current_H(:, :, :) = rgbd_plugin.filters(filter_index, :, :, :);
        
        if ~tracker.use_channel_weights
            response = real(ifft2(sum(fft2(f).*conj(current_H), 3)));
        else
            response_chann = real(ifft2(fft2(f).*conj(current_H)));
            response = sum(bsxfun(@times, response_chann, reshape(tracker.chann_w, 1, 1, size(response_chann,3))), 3);
        end
        
        [row, col] = ind2sub(size(response),find(response == max(response(:)), 1));
        
        max_resp_score = max(response(:));
        
        if(max_resp_score > max_score)
            
            max_score = max_resp_score;
            
            if tracker.use_channel_weights
                temp_params.channel_discr = ones(1, size(response_chann, 3));
                for i = 1:size(response_chann, 3)
                    norm_response = normalize_img(response_chann(:, :, i));
                    local_maxs_sorted = localmax_nonmaxsup2d(squeeze(norm_response(:, :)));
                    
                    if local_maxs_sorted(1) == 0, continue; end;
                    temp_params.channel_discr(i) = 1 - (local_maxs_sorted(2) / local_maxs_sorted(1));
                    
                    % sanity checks
                    if temp_params.channel_discr(i) < 0.5, temp_params.channel_discr(i) = 0.5; end;
                end
            end
            
            % subpixel accuracy: response map is smaller than image patch -
            % due to HoG histogram (cell_size > 1)
            v_neighbors = response(mod(row + [-1, 0, 1] - 1, size(response,1)) + 1, col);
            h_neighbors = response(row, mod(col + [-1, 0, 1] - 1, size(response,2)) + 1);
            row = row + subpixel_peak(v_neighbors);
            col = col + subpixel_peak(h_neighbors);
            
            % wrap around
            if row > size(response,1) / 2,
                row = row - size(response,1);
            end
            if col > size(response,2) / 2,
                col = col - size(response,2);
            end
            
            % displacement
            d = tracker.currentScaleFactor * tracker.cell_size * (1/tracker.rescale_ratio) * [col - 1, row - 1];
            
            % new object center
            c = (last_c) + d;
            
            region = [c - tracker.currentScaleFactor * tracker.base_target_sz/2, tracker.currentScaleFactor * tracker.base_target_sz];
            
            %do a scale space search aswell
            xs = get_scale_subwindow(img, c([2,1]), tracker.base_target_sz([2,1]),  tracker.currentScaleFactor * tracker.scaleSizeFactors, ...
                tracker.scale_window, tracker.scale_model_sz([2,1]), []);
            xsf = fft(xs,[],2);
            % scale correlation response
            scale_response = real(ifft(sum(tracker.sf_num .* xsf, 1) ./ (tracker.sf_den + 1e-2) ));
            % max(scale_response(:))
            % tracker.current_response = max(scale_response(:));
            recovered_scale = ind2sub(size(scale_response),find(scale_response == max(scale_response(:)), 1));
            %set the scale
            currentScaleFactor = tracker.currentScaleFactor * tracker.scaleFactors(recovered_scale);
            
            % check for min/max scale
            if currentScaleFactor < tracker.min_scale_factor
                currentScaleFactor = tracker.min_scale_factor;
            elseif currentScaleFactor > tracker.max_scale_factor
                currentScaleFactor = tracker.max_scale_factor;
            end
            
            % new tracker scale
            temp_params.currentScaleFactor = currentScaleFactor;
            
            % put new object location into the tracker structure
            temp_params.c = c;
            temp_params.bb = region;
            output_BB = temp_params.bb;
            tracker.H = current_H;
            
            chosen_filter_index = filter_index;
        end
    end
    
    if(chosen_filter_index == current_filter_index)
        rgbd_plugin.filter_index = rgbd_plugin.filter_index - 1;
    end
else
    if ~tracker.use_channel_weights
        response = real(ifft2(sum(fft2(f).*conj(tracker.H), 3)));
    else
        response_chann = real(ifft2(fft2(f).*conj(tracker.H)));
        response = sum(bsxfun(@times, response_chann, reshape(tracker.chann_w, 1, 1, size(response_chann,3))), 3);
    end
    
    [row, col] = ind2sub(size(response),find(response == max(response(:)), 1));
    
    max_resp_score = max(response(:));
    max_score = max_resp_score;
    % max_resp_score
    % calculate detection-based weights
    if tracker.use_channel_weights
        temp_params.channel_discr = ones(1, size(response_chann, 3));
        for i = 1:size(response_chann, 3)
            norm_response = normalize_img(response_chann(:, :, i));
            local_maxs_sorted = localmax_nonmaxsup2d(squeeze(norm_response(:, :)));
            
            if local_maxs_sorted(1) == 0, continue; end;
            temp_params.channel_discr(i) = 1 - (local_maxs_sorted(2) / local_maxs_sorted(1));
            
            % sanity checks
            if temp_params.channel_discr(i) < 0.5, temp_params.channel_discr(i) = 0.5; end;
        end
    end
    
    % subpixel accuracy: response map is smaller than image patch -
    % due to HoG histogram (cell_size > 1)
    v_neighbors = response(mod(row + [-1, 0, 1] - 1, size(response,1)) + 1, col);
    h_neighbors = response(row, mod(col + [-1, 0, 1] - 1, size(response,2)) + 1);
    row = row + subpixel_peak(v_neighbors);
    col = col + subpixel_peak(h_neighbors);
    
    % wrap around
    if row > size(response,1) / 2,
        row = row - size(response,1);
    end
    if col > size(response,2) / 2,
        col = col - size(response,2);
    end
    
    % displacement
    d = tracker.currentScaleFactor * tracker.cell_size * (1/tracker.rescale_ratio) * [col - 1, row - 1];
    
    % new object center
    c = (last_c) + d;
    
    % c = double(c);
    % object bounding-box
    region = [c - tracker.currentScaleFactor * tracker.base_target_sz/2, tracker.currentScaleFactor * tracker.base_target_sz];
    
    %do a scale space search aswell
    xs = get_scale_subwindow(img, c([2,1]), tracker.base_target_sz([2,1]),  tracker.currentScaleFactor * tracker.scaleSizeFactors, tracker.scale_window, tracker.scale_model_sz([2,1]), []);
    xsf = fft(xs,[],2);
    % scale correlation response
    scale_response = real(ifft(sum(tracker.sf_num .* xsf, 1) ./ (tracker.sf_den + 1e-2) ));
    % max(scale_response(:))
    % tracker.current_response = max(scale_response(:));
    recovered_scale = ind2sub(size(scale_response),find(scale_response == max(scale_response(:)), 1));
    %set the scale
    currentScaleFactor = tracker.currentScaleFactor * tracker.scaleFactors(recovered_scale);
    
    % check for min/max scale
    if currentScaleFactor < tracker.min_scale_factor
        currentScaleFactor = tracker.min_scale_factor;
    elseif currentScaleFactor > tracker.max_scale_factor
        currentScaleFactor = tracker.max_scale_factor;
    end
    
    % new tracker scale
    temp_params.currentScaleFactor = currentScaleFactor;
    
    % put new object location into the tracker structure
    temp_params.c = c;
    temp_params.bb = region;
    output_BB = temp_params.bb;
    
end

end

function delta = subpixel_peak(p)
%parabola model (2nd order fit)
delta = 0.5 * (p(3) - p(1)) / (2 * p(2) - p(3) - p(1));
if ~isfinite(delta), delta = 0; end
end

function [local_max] = localmax_nonmaxsup2d(response)
BW = imregionalmax(response);
CC = bwconncomp(BW);

local_max = [max(response(:)) 0];
if length(CC.PixelIdxList) > 1
    local_max = zeros(length(CC.PixelIdxList));
    for i = 1:length(CC.PixelIdxList)
        local_max(i) = response(CC.PixelIdxList{i}(1));
    end
    local_max = sort(local_max, 'descend');
end
end  % endfunction

function out = normalize_img(img)
min_val = min(img(:));
max_val = max(img(:));
if (max_val - min_val) > 0
    out = (img - min_val)/(max_val - min_val);
else
    out = zeros(size(img));
end
end  % endfunction