function [output_bb] = optimal_bb(input_mask, input_bb, centroid, step, fg_perc)

input_x = input_bb(1); input_y = input_bb(2);
input_width = input_bb(3); input_height = input_bb(4);

fg_in_bb = input_mask(input_y : input_y + input_height, input_x : input_x + input_width);

init_num_fg_pixels = length(find(fg_in_bb > 0));

bb_area = input_bb(3) * input_bb(4);

init_perc = double(init_num_fg_pixels) / double(bb_area);

if(init_perc < fg_perc)
    % Center of mass should be closer to the edges that have more pixels,
    % so reduce the size of the other edges
    % Find the quadrant where the centroid locates. Clockwise starting from
    % top left
    input_bb_center_x = (input_x + input_width) / 2;
    input_bb_center_y = (input_y + input_height) / 2;
    
    quadrant = 0;
    
    if((centroid(1) < input_bb_center_x) && (centroid(2) < input_bb_center_y))
        quadrant = 1;
        % Reduce right & bottom edges, keep top left corner same
        current_top_x = uint32(input_x);
        current_top_y = uint32(input_y);
        current_bottom_x = uint32(current_top_x + input_width);
        current_bottom_y = uint32(current_top_y + input_height);
        
        current_perc = init_perc;
        prev_perc = init_perc;
        
        while((current_perc < fg_perc) && (current_perc - prev_perc >= 0))
            prev_perc = current_perc;
            current_bottom_x  = current_bottom_x - step;
            current_bottom_y = current_bottom_y - step;
            
            current_fg_in_bb = input_mask(current_top_y : current_bottom_y, current_top_x : current_bottom_x);
            current_num_fg_pixels = length(find(current_fg_in_bb > 0));
            current_perc = double(current_num_fg_pixels) / double((current_bottom_x - current_top_x) * (current_bottom_y - current_top_y));
        end
        output_bb = [current_top_x current_top_y (current_bottom_x - current_top_x) (current_bottom_y - current_top_y)];
        
    elseif((centroid(1) >= input_bb_center_x) && (centroid(2) < input_bb_center_y))
        quadrant = 2;
        % Reduce left & bottom edges
        current_perc = init_perc;
        current_top_x = uint32(input_x);
        current_top_y = uint32(input_y);
        current_bottom_x = uint32(current_top_x + input_width);
        current_bottom_y = uint32(current_top_y + input_height);

        prev_perc = init_perc;
        while((current_perc < fg_perc) && (current_perc - prev_perc >= 0))
            prev_perc = current_perc;
            current_top_x = current_top_x + step;
            current_bottom_y = current_bottom_y - step;
            current_fg_in_bb = input_mask(current_top_y : current_bottom_y, current_top_x : current_bottom_x);
            current_num_fg_pixels = length(find(current_fg_in_bb > 0));
            current_perc = double(current_num_fg_pixels) / double((current_bottom_x - current_top_x) * (current_bottom_y - current_top_y));
        end
        
        output_bb = [current_top_x current_top_y (current_bottom_x - current_top_x) (current_bottom_y - current_top_y)];
        
    elseif((centroid(1) >= input_bb_center_x) && (centroid(2) >= input_bb_center_y))
        quadrant = 3;
        % Reduce top & left edges
        current_perc = init_perc;
        current_top_x = uint32(input_x);
        current_top_y = uint32(input_y);
        current_bottom_x = uint32(current_top_x + input_width);
        current_bottom_y = uint32(current_top_y + input_height);
        
        prev_perc = init_perc;
        while((current_perc < fg_perc) && (current_perc - prev_perc >= 0))
            prev_perc = current_perc;
            current_top_x = current_top_x + step;
            current_top_y = current_top_y + step;
            
            current_fg_in_bb = input_mask(current_top_y : current_bottom_y, current_top_x : current_bottom_x);
            current_num_fg_pixels = length(find(current_fg_in_bb > 0));
            current_perc = double(current_num_fg_pixels) / double((current_bottom_x - current_top_x) * (current_bottom_y - current_top_y));
        end
        output_bb = [current_top_x current_top_y (current_bottom_x - current_top_x) (current_bottom_y - current_top_y)];
        
    else
        quadrant = 4;
        % Reduce top & right edges
        current_perc = init_perc;
        current_top_x = uint32(input_x);
        current_top_y = uint32(input_y);
        current_bottom_x = uint32(current_top_x + input_width);
        current_bottom_y = uint32(current_top_y + input_height);
        
        
        prev_perc = init_perc;
        while((current_perc < fg_perc) && (current_perc - prev_perc >= 0))
            prev_perc = current_perc;
            current_top_y = current_top_y + step;
            current_bottom_x = current_bottom_x - step;
            
            current_fg_in_bb = input_mask(current_top_y : current_bottom_y, current_top_x : current_bottom_x);
            current_num_fg_pixels = length(find(current_fg_in_bb > 0));
            current_perc = double(current_num_fg_pixels) / double((current_bottom_x - current_top_x) * (current_bottom_y - current_top_y));
            
        end
        output_bb = [current_top_x current_top_y (current_bottom_x - current_top_x) (current_bottom_y - current_top_y)];
        
    end
else
    output_bb = input_bb;
end

end

