function [ tracker, f, max_response ] = detect_best_response( img, tracker, top_point, bottom_point )

stride = uint16(tracker.bb(3) / 3);

response_size = max(size(tracker.cos_win));

x_values = top_point(1, 1):stride:bottom_point(1, 1);
y_values = top_point(1, 2):stride:bottom_point(1, 2);

size_y = max(size(y_values));
size_x = max(size(x_values));

all_responses = zeros(size_y, size_x, response_size, response_size);

cos_win = double(ones(response_size, response_size));

parfor y=1:size_y
    current_responses = zeros(1, size_x, response_size, response_size);
    for x=1:size_x
        f = get_csr_features(img, [x_values(x) y_values(y)], tracker.currentScaleFactor, tracker.template_size, tracker.rescale_template_size, tracker.cos_win, tracker.feature_type, tracker.w2c, tracker.cell_size);
        
        if ~tracker.use_channel_weights
            response = real(ifft2(sum(fft2(f).*conj(tracker.H), 3)));
        else
            response_chann = real(ifft2(fft2(f).*conj(tracker.H)));
            response = sum(bsxfun(@times, response_chann, reshape(tracker.chann_w, 1, 1, size(response_chann,3))), 3);
        end
        current_responses(1, x, :, :) = response;
    end
    all_responses(y,:,:,:) = current_responses;
end

max_response = max(max(max(max(all_responses))));

[I1,I2,I3,I4] = ind2sub(size(all_responses),find(all_responses == max_response));

start_y = y_values(I1(1));
start_x = x_values(I2(1));

clear y_values;
clear x_values;
clear size_y; 
clear size_x;
clear all_responses;
clear I1;
clear I2;
clear I3;
clear I4;

y_values_min = max((start_y - stride), top_point(1, 2));
y_values_max = min((start_y + stride), bottom_point(1, 2));
y_values = y_values_min : (stride / 2) : y_values_max;

x_values_min = max((start_x - stride), top_point(1, 1));
x_values_max = min((start_x + stride), bottom_point(1, 1));
x_values = x_values_min : (stride / 2) : x_values_max;

clear start_x;
clear start_y;

stride = stride / 2;

size_y = max(size(y_values));
size_x = max(size(x_values));

all_responses = zeros(size_y, size_x, response_size, response_size);

parfor y=1:size_y
    current_responses = zeros(1, size_x, response_size, response_size);
    for x=1:size_x
        f = get_csr_features(img, [x_values(x) y_values(y)], tracker.currentScaleFactor, tracker.template_size, tracker.rescale_template_size, tracker.cos_win, tracker.feature_type, tracker.w2c, tracker.cell_size);
        
        if ~tracker.use_channel_weights
            response = real(ifft2(sum(fft2(f).*conj(tracker.H), 3)));
        else
            response_chann = real(ifft2(fft2(f).*conj(tracker.H)));
            response = sum(bsxfun(@times, response_chann, reshape(tracker.chann_w, 1, 1, size(response_chann,3))), 3);
        end
        current_responses(1, x, :, :) = response;
    end
    all_responses(y,:,:,:) = current_responses;
end

max_response = max(max(max(max(all_responses))));

[I1,I2,I3,I4] = ind2sub(size(all_responses),find(all_responses == max_response));

start_y = y_values(I1(1));
start_x = x_values(I2(1));

clear y_values;
clear x_values;
clear size_y; 
clear size_x;
clear all_responses;
clear I1;
clear I2; 
clear I3;
clear I4;


y_values = (start_y - 5) : (start_y + 5);
x_values = (start_x - 5) : (start_x + 5);

clear start_x;
clear start_y;

size_y = max(size(y_values));
size_x = max(size(x_values));

all_responses = zeros(size_y, size_x, response_size, response_size);

f = double(zeros(response_size, response_size));

parfor y=1:size_y
    current_responses = zeros(1, size_x, response_size, response_size);
    for x=1:size_x      
        f = get_csr_features(img, [x_values(x) y_values(y)], tracker.currentScaleFactor, tracker.template_size, tracker.rescale_template_size, tracker.cos_win, tracker.feature_type, tracker.w2c, tracker.cell_size);
        
        if ~tracker.use_channel_weights
            response = real(ifft2(sum(fft2(f).*conj(tracker.H), 3)));
        else
            response_chann = real(ifft2(fft2(f).*conj(tracker.H)));
            response = sum(bsxfun(@times, response_chann, reshape(tracker.chann_w, 1, 1, size(response_chann,3))), 3);
        end
        current_responses(1, x, :, :) = response;
    end
    all_responses(y,:,:,:) = current_responses;
end

max_response = max(max(max(max(all_responses))));

[I1,I2,I3,I4] = ind2sub(size(all_responses),find(all_responses == max_response));

start_y = y_values(I1(1));
start_x = x_values(I2(1));

tracker.c = [start_x start_y];

tracker.current_response = max_response;

end

