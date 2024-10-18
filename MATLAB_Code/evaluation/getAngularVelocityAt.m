function av = getAngularVelocityAt(timestamp, gt)
    gt_index = find(gt(:,1)==timestamp);
    if length(gt_index)>1
        gt_index = gt_index(1);
    end
    av = gt(gt_index,5:7)';
%     index_b = find(gt(:,1) >= timestamp);
%     time_pre = index_a(length(index_a));
%     time_after = index_b(1);
%     
%     % Calculate velocity in world frame
%     v = (pose(time_after,2:4) - pose(time_pre,2:4))/(pose(time_after,1)-pose(time_pre,1));
%     p = (timestamp-pose(time_pre,1))/(pose(time_after,1)-pose(time_pre,1)); 
%     
%     q1 = quaternion(pose(time_pre,8),pose(time_pre,5),pose(time_pre,6),pose(time_pre,7));
%     q2 = quaternion(pose(time_after,8),pose(time_after,5),pose(time_after,6),pose(time_after,7));
%     q = slerp(q1, q2, p);
%     R = quat2rotm(q);
%     v = R0*R * v';
end