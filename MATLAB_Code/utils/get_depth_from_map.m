function depth_list = get_depth_from_map(events_raw,ts_list,depth_map, intrinsics)
    depth_list = zeros(size(events_raw,1),1);
    for interval_i = 1:size(ts_list, 1) - 1
        first_img = depth_map{interval_i, 1};
        second_img = depth_map{interval_i + 1, 1};
        events_raw(:,2) = round(events_raw(:,2));
        events_raw(:,3) = round(events_raw(:,3));

        first_depth = first_img(sub2ind(intrinsics.img_size, events_raw(:, 3), events_raw(:, 2)));
        second_depth = second_img(sub2ind(intrinsics.img_size, events_raw(:, 3), events_raw(:, 2)));

        event_inside_id = events_raw(:, 1) < ts_list(interval_i + 1) & events_raw(:, 1) > ts_list(interval_i);
        event_valid_id = event_inside_id & first_depth ~= 0 & second_depth ~= 0;

        events_time_inside = events_raw(event_valid_id, 1);
        first_depth = first_depth(event_valid_id, 1);
        second_depth = second_depth(event_valid_id, 1);

        if sum(event_valid_id) == 0
            continue;
        end
    
        depth_list(event_valid_id) = (first_depth .* (ts_list(interval_i + 1) - events_time_inside) + second_depth .* (events_time_inside - ts_list(interval_i))) / ...
                                      (ts_list(interval_i + 1) - ts_list(interval_i));
    end

%     depth_list = zeros(size(events_raw,1),1);
%     for i = 1:size(events_raw,1)
%         timestamp = events_raw(i,1);
% %         timestamp - ts_list(1,1)
%         j = find(timestamp>=ts_list(:,1),1,"last");
%         k = find(timestamp<=ts_list(:,1),1);
%         if isempty(j) || isempty(k)
% %             'empty'
%             depth_list(i)=-1;
%             continue;
%         end
%         %% Spatial bilinear interpolation
%         y = events_raw(i,3);
%         x = events_raw(i,2);
%         % Check if the coordinates (x, y) are within the valid range of the depth map dimensions
%         if x < 1 || x > size(depth_map, 2) || y < 1 || y > size(depth_map, 1)
%             depth1 = NaN; % Set depth as NaN for out-of-bounds coordinates
%         else
%             % Extract the integer and fractional parts of the coordinates (x, y)
%             x_int = floor(x);
%             y_int = floor(y);
%             x_frac = x - x_int;
%             y_frac = y - y_int;
% 
%             % Check if the integer coordinates are at the boundary
%             if x_int == size(depth_map, 2)
%                 x_int = x_int - 1;
%             end
%             if y_int == size(depth_map, 1)
%                 y_int = y_int - 1;
%             end
% 
%             % Retrieve the depth values of the four neighboring pixels
%             depth_tl = depth_map(y_int, x_int,ts_list(j,2)+1);
%             depth_tr = depth_map(y_int, x_int + 1,ts_list(j,2)+1);
%             depth_bl = depth_map(y_int + 1, x_int,ts_list(j,2)+1);
%             depth_br = depth_map(y_int + 1, x_int + 1,ts_list(j,2)+1);
%             % Perform bilinear interpolation to calculate the interpolated depth
%             depth1 = (1 - x_frac) * (1 - y_frac) * depth_tl + x_frac * (1 - y_frac) * depth_tr + (1 - x_frac) * y_frac * depth_bl + x_frac * y_frac * depth_br;
%         end
%         % Check if the coordinates (x, y) are within the valid range of the depth map dimensions
%         if x < 1 || x > size(depth_map, 2) || y < 1 || y > size(depth_map, 1)
%             depth2 = NaN; % Set depth as NaN for out-of-bounds coordinates
%         else
%             % Extract the integer and fractional parts of the coordinates (x, y)
%             x_int = floor(x);
%             y_int = floor(y);
%             x_frac = x - x_int;
%             y_frac = y - y_int;
% 
%             % Check if the integer coordinates are at the boundary
%             if x_int == size(depth_map, 2)
%                 x_int = x_int - 1;
%             end
%             if y_int == size(depth_map, 1)
%                 y_int = y_int - 1;
%             end
% 
%             % Retrieve the depth values of the four neighboring pixels
%             depth_tl = depth_map(y_int, x_int,ts_list(k,2)+1);
%             depth_tr = depth_map(y_int, x_int + 1,ts_list(k,2)+1);
%             depth_bl = depth_map(y_int + 1, x_int,ts_list(k,2)+1);
%             depth_br = depth_map(y_int + 1, x_int + 1,ts_list(k,2)+1);
%             % Perform bilinear interpolation to calculate the interpolated depth
%             depth2 = (1 - x_frac) * (1 - y_frac) * depth_tl + x_frac * (1 - y_frac) * depth_tr + (1 - x_frac) * y_frac * depth_bl + x_frac * y_frac * depth_br;
%         end
% %         depth1 = depth_map(events_raw(i,3),events_raw(i,2),ts_list(j,2)+1);
% %         depth2 = depth_map(events_raw(i,3),events_raw(i,2),ts_list(k,2)+1);
%         if isnan(depth1) || isnan(depth2) || depth1==0|| depth2==0
% %             'nan'
%             depth_list(i)=-1;
%             continue;
%         end
%         %% Temporal linear interpolation
%         t_ratio = (timestamp - ts_list(j,1))/(ts_list(k,1)-ts_list(j,1));
%         if isnan(t_ratio)
%             depth_list(i) = depth1;
%             continue;
%         end
%         depth_list(i)=depth1*(1-t_ratio)+depth2*t_ratio;
%     end
end