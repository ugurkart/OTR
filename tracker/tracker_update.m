function [tracker] = tracker_update(tracker, rgb_img, temp_params, mask)

mask = imresize(mask, size(tracker.Y), 'nearest');

% check if mask is too small (probably segmentation is not ok then)
if mask_normal(mask, tracker.target_dummy_area)
    if tracker.mask_diletation_sz > 0
        D = strel(tracker.mask_diletation_type, tracker.mask_diletation_sz);
        mask = imdilate(mask, D);
    end
end

f = get_csr_features(rgb_img, tracker.c, tracker.currentScaleFactor, tracker.template_size, tracker.rescale_template_size, tracker.cos_win, tracker.feature_type, tracker.w2c, tracker.cell_size);

H_new = create_csr_filter(f, tracker.Y, single(mask));

% calculate per-channel feature weights
if tracker.use_channel_weights
    w_lr = tracker.weight_lr;
    response = real(ifft2(fft2(f).*conj(H_new)));
    chann_w = max(reshape(response, [size(response,1)*size(response,2), size(response,3)]), [], 1) .* temp_params.channel_discr;
    chann_w = chann_w / sum(chann_w);
    tracker.chann_w = (1-w_lr)*tracker.chann_w + w_lr*chann_w;
    tracker.chann_w = tracker.chann_w / sum(tracker.chann_w);
end

lr = tracker.learning_rate;
tracker.H = (1-lr)*tracker.H + lr*H_new;

% make a scale search model aswell
xs = get_scale_subwindow(rgb_img, tracker.c([2,1]), tracker.base_target_sz([2,1]), tracker.currentScaleFactor * tracker.scaleSizeFactors, tracker.scale_window, tracker.scale_model_sz([2,1]), []);
% fft over the scale dim
xsf = fft(xs,[],2);
new_sf_num = bsxfun(@times, tracker.ysf, conj(xsf));
new_sf_den = sum(xsf .* conj(xsf), 1);
% auto-regressive scale filters update
slr = tracker.scale_lr;
tracker.sf_den = (1 - slr) * tracker.sf_den + slr * new_sf_den;
tracker.sf_num = (1 - slr) * tracker.sf_num + slr * new_sf_num;
end

