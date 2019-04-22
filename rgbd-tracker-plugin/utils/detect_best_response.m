function [max_response, c, out_bb, out_temp_params] = detect_best_response(img, tracker, top_point, bottom_point)

stride = uint16(tracker.bb(3) / 3);

x_values = top_point(1, 1):stride:bottom_point(1, 1);
y_values = top_point(1, 2):stride:bottom_point(1, 2);

size_y = max(size(y_values));
size_x = max(size(x_values));

all_responses = zeros(size_y, size_x);

parfor y=1:size_y
    current_responses = zeros(1, size_x);
    for x=1:size_x
        tmp_targetPosition = ([x_values(x) y_values(y)]);
        [bb, resp_score, temp_params] = tracker_process_frame(tracker, tmp_targetPosition, img, true);
        current_responses(1, x) = resp_score;
    end
    all_responses(y,:) = current_responses;
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

all_responses = zeros(size_y, size_x);

parfor y=1:size_y
    current_responses = zeros(1, size_x);
    for x=1:size_x
        tmp_targetPosition = ([x_values(x) y_values(y)]);
        [bb, resp_score, temp_params] = tracker_process_frame(tracker, tmp_targetPosition, img, true);
        current_responses(1, x) = resp_score;
    end
    all_responses(y,:) = current_responses;
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

all_responses = zeros(size_y, size_x);
all_bb = cell(size_y, size_x);
all_temp_params = cell(size_y, size_x);

parfor y=1:size_y
    current_responses = zeros(1, size_x);
    for x=1:size_x
        tmp_targetPosition = ([x_values(x) y_values(y)]);
        [bb, resp_score, temp_params] = tracker_process_frame(tracker, tmp_targetPosition, img, true);
        current_responses(1, x) = resp_score;
        all_bb{y, x} = bb;
        all_temp_params{y, x} = temp_params;
    end
    all_responses(y,:) = current_responses;
end

max_response = max(max(max(max(all_responses))));

[I1,I2,I3,I4] = ind2sub(size(all_responses),find(all_responses == max_response));

start_y = y_values(I1(1));
start_x = x_values(I2(1));

c = [start_x start_y];
out_bb = all_bb{I1(1), I2(1)};
out_temp_params = all_temp_params{I1(1), I2(1)};

end

