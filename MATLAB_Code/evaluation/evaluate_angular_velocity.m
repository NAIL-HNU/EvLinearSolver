function [RMSE, error_avg] =  evaluate_angular_velocity(motion_est, imu_gt)

        

    error_all = zeros(length(motion_est),3);
    motion_gt = zeros(length(motion_est),3);
    for i = 1:length(motion_est)
        
        angular_velocity_gt = getAngularVelocityAt(motion_est(i,1), imu_gt)';
        motion_gt(i,:) = angular_velocity_gt;
        angular_velocity_est = motion_est(i,2:4);
        error = abs(angular_velocity_est - angular_velocity_gt)*180/pi;
        error_all(i,:) = error;
    end
    RMSE = sqrt(mean(error_all.^2,'all'));
    error_avg = mean(error_all,'all');


    %% Plot
    figure;
    gt = motion_gt/pi*180;
    result(:,2:4) = motion_est(:,2:4)/pi*180;
    result(:,1) = motion_est(:,1);
    t_start = result(1,1);
    t_end = result(end,1);
%     index_start = 1;
%     index_end = end;
    plot(result(:,1),gt(1:end,1),'b--');
    hold on
    p1 = plot(result(:,1),result(:,2),'b');
    hold on
    plot(result(:,1),gt(1:end,2),'g--');
    hold on
    p2 = plot(result(:,1),result(:,3),'g');
    hold on
    plot(result(:,1),gt(1:end,3),'r--');
    hold on
    p3 = plot(result(:,1),result(:,4),'r');
    xlabel('Time (s)')
    ylabel('Angular velocity (degree/s)')
%     lgd_handle = legend([p1 p2 p3],'tilt','pan','roll');
%     lgd_handle.Location =  'northwest';
%     lgd_handle.FontSize = 12;
end
