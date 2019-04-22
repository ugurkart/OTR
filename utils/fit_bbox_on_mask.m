function bb = fit_bbox_on_mask(mask, method)

bb = [];

switch method
    case 'min_max'
        
        x_sum = sum(mask, 1);
        y_sum = sum(mask, 2);
        x0 = min(find(x_sum>0));
        y0 = min(find(y_sum>0));
        x1 = max(find(x_sum>0));
        y1 = max(find(y_sum>0));
        bb = [x0, y0, x1 - x0 + 1, y1 - y0 + 1];
        
    case {'ratio', 'iou'}
        
        if strcmp(method, 'ratio')
            cost_f = @ratio_cost;
        elseif strcmp(method, 'iou')
            cost_f = @iou_cost;
        end
        
        bb_outter = fit_bbox_on_mask(mask, 'min_max');
        bb_mod = bb_outter;
        c = cost_f(mask, bb_outter);
        c_mod = c;
        
        changes = [1,0,-1,0; 0,1,0,-1; 0,0,-1,0; 0,0,0,-1];
        
        while true
            
            % flag if bbox was modified in current iteration
            modified = false;
            
            for i=1:size(changes,1)
                bb_ = bb_mod;
                bb_ = bb_ + changes(i,:);
                c_ = cost_f(mask, bb_);
                if c_ > c_mod
                    c_mod = c_;
                    bb_mod = bb_;
                    modified = true;
                end
            end
            
            if ~modified
                break;
            end
            
        end
        
        bb = bb_mod;
                
    otherwise
        
        error('Unknown fit method.');
end


end  % endfunction


function cost = ratio_cost(mask, bb)

x0 = round(bb(1));
y0 = round(bb(2));
x1 = x0 + round(bb(3)) - 1;
y1 = y0 + round(bb(4)) - 1;

in = mask(y0:y1, x0:x1);
positive_in = sum(in(:));
negative_in = numel(in) - positive_in;

m_out = mask;
m_out(y0:y1, x0:x1) = 0;
positive_out = sum(m_out(:));

cost = positive_in / (negative_in + positive_out);

end  % endfunction


function cost = iou_cost(mask, bb)

x0 = round(bb(1));
y0 = round(bb(2));
x1 = x0 + round(bb(3)) - 1;
y1 = y0 + round(bb(4)) - 1;

in = mask(y0:y1, x0:x1);
positive_in = sum(in(:));

cost = positive_in / numel(in);

end  % endfunction