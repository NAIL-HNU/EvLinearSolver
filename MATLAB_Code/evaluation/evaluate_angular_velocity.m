function [RMSE, error_avg] =  evaluate_angular_velocity(motion_est, imu_gt)
    % Time alignment
    error_all = zeros(length(motion_est),3);
    for i = 1:length(motion_est)
        angular_velocity_gt = getAngularVelocityAt(motion_est(i,1), imu_gt);
        angular_velocity_est = motion_est(i,2:4);
        error = angular_velocity_est - angular_velocity_gt;
        error_all(i,:) = error;
    end
    RMSE = sqrt(mean(error_all.^2));
    error_avg = mean(error_all);
end
