function [RMSE, error_avg] =  evaluate_velocity(motion_est, pose_gt, dataset_name)
    % Time alignment
    error_all = zeros(length(motion_est),3);
    for i = 1:length(motion_est)
        velocity_gt = getVelocityAt(motion_est(i,1), pose_gt, dataset_name);
        velocity_est = motion_est(i,2:4);
        error = velocity_est - velocity_gt;
        error_all(i,:) = error;
    end
    % 计算误差的范数
    RMSE = sqrt(mean(error_all.^2));
    error_avg = mean(error_all);
end
