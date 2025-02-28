function [normal_flow_corr, events_corr] = get_valid_normal_flow(distortion_mode,events, epoch_time, plane_fit_thres, camera_param, window_size, undistort_map)
%GET_VALID_NORMAL_FLOW 此处显示有关此函数的摘要
%   此处显示详细说明
    normal_flow_corr = [];
    events_corr = [];
    events_undistort = events;

    if(distortion_mode)
        events_undistort(:,2:3) = undistort_map(sub2ind(camera_param.img_size, events_undistort(:, 3), events_undistort(:, 2)), :);
    end
    inside_id = events_undistort(:, 2) > 1 & events_undistort(:, 2) < camera_param.img_size(2) &...
        events_undistort(:, 3) > 1 & events_undistort(:, 3) < camera_param.img_size(1);
    events_undistort = events_undistort(inside_id, :);
    
    ev_id = events_undistort(:, 1) <= epoch_time.max_time & events_undistort(:, 1) >= epoch_time.min_time;
    events_epoch = events_undistort(ev_id, :);
    
    for id = 1:size(events_epoch, 1)
        x = events_epoch(id, 2);
        y = events_epoch(id, 3);
        events_neighbour_id = events_epoch(:,2)>=x-window_size/2 & events_epoch(:,2)<=x+window_size/2 & ...
            events_epoch(:,3)>=y-window_size/2 & events_epoch(:,3)<=y+window_size/2;
        events_neighbour = events_epoch(events_neighbour_id,:);

        if ~isempty(events_neighbour)
%             fx = camera_param.K(1,1);
%             fy = camera_param.K(2,2);
%             cx = camera_param.K(1,3);
%             cy = camera_param.K(2,3);
%             xData = (events_neighbour(:,2)-cx)/fx;
%             yData = (events_neighbour(:,3)-cy)/fy;

            xData = events_neighbour(:,2);
            yData = events_neighbour(:,3);
            %           Normalize
%             xData = (events_neighbour(:,2)-cx)/camera_param.img_size(2);        
%             yData = (events_neighbour(:,3)-cy)/camera_param.img_size(1);
            zData = events_neighbour(:,1)-events_neighbour(1,1);
 

            A = [xData, yData, ones(size(xData))]; % Forming [x, y, 1] matrix
%             ax + by + d = t;
%           TODO: Add inlier set and optimize
            [coefficients, ransac_inlier_ratio] = RANSAC(A, zData, 7, plane_fit_thres,30);
            if ~isempty(coefficients) && ransac_inlier_ratio > 0.2
                % Extract coefficients for the plane equation

                A_coeff = coefficients(1);
                B_coeff = coefficients(2);
                deriv_vec = -[A_coeff,B_coeff];
                nflow = deriv_vec/(A_coeff.^2+B_coeff.^2);
%                 nflow = nflow./[fx,fy];
%                 nflow = [1/A_coeff,1/B_coeff]/sqrt(1/A_coeff.^2+1/B_coeff.^2);
                normal_flow_corr = [normal_flow_corr; nflow];
                events_corr = [events_corr; events_epoch(id,:)];
                %% Visualization  
                % Define grid for plotting the plane
%                 C_coeff = -1; % C coefficient for the equation Ax + By + Cz + D = 0
%                 D_coeff = coefficients(3);
%                 [xGrid, yGrid] = meshgrid(linspace(min(xData), max(xData), 100), linspace(min(yData), max(yData), 100));
%                 zGrid = -(A_coeff * xGrid + B_coeff * yGrid + D_coeff) / C_coeff; % Compute z values for the plane
% 
% %                 Plot the original data and the fitted plane
%                 figure;
%                 scatter3(xData, yData, zData, 'filled'); % Scatter plot of data points
%                 hold on;
%                 surf(xGrid, yGrid, zGrid, 'FaceAlpha', 0.5); % Plot the fitted plane
% 
%                 % Customize the plot
%                 xlabel('X');
%                 ylabel('Y');
%                 zlabel('Z');
%                 title('Fitted Plane');
             end
        end
    end
end

