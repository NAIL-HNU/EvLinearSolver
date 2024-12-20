function [RMSE, error_avg] =  evaluate_velocity(motion_est, motion_gt)
    % Time alignment
    error_all = zeros(length(motion_est),3);
    for i = 1:length(motion_est)
        velocity_gt = getVelocityAt(motion_est(i,1), motion_gt);
        velocity_est = motion_est(i,2:4);
        error = velocity_est - velocity_gt;
        error_all(i,:) = error;
    end
    % 计算误差的范数
    RMSE = sqrt(mean(error_all.^2));
    error_avg = mean(error_all);
end
