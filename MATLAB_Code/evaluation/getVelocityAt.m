function [v,omega] = getVelocityAt(timestamp, pose, dataset_name)

    % Get recent pose;
    index_a = find(pose(:,1) < timestamp,1,"last");
    index_b = find(pose(:,1) >=timestamp,1,"first");
%     time_pre = index_a(length(index_a))
%     time_after = index_b(1)
    time_pre = index_a;
    time_after = index_b; 
    
    % Calculate velocity in world frame
    v_mocap = (pose(time_after,2:4) - pose(time_pre,2:4))/(pose(time_after,1)-pose(time_pre,1));
    p = (timestamp-pose(time_pre,1))/(pose(time_after,1)-pose(time_pre,1)); 
    % Transfer velocity to body frame
    q1 = quaternion(pose(time_pre,8),pose(time_pre,5),pose(time_pre,6),pose(time_pre,7));
%     q1.norm
    q2 = quaternion(pose(time_after,8),pose(time_after,5),pose(time_after,6),pose(time_after,7));
%     q2.norm
%     q_array = quaternion(pose(time_pre:time_after,8),pose(time_pre:time_after,5),pose(time_pre:time_after,6),pose(time_pre:time_after,7));
%     dq = normalize(q2-q1);
    
    dt = pose(time_after,1)-pose(time_pre,1);
    omega_mocap = quat2angvel(q1,q2,dt);
%     omega_compare = angvel(q_array,dt,"point")
%     omega_mocap = omega_body(2,:)';
    q = slerp(q1, q2, p);
    R = quat2rotm(q);
    v_body = R * v_mocap';
    omega_body = R * omega_mocap;
    % Transfer velocity to event camera frame
    switch dataset_name
        case 'VECtor'
            T_lcam_body = [-0.857137023976571  ,  0.03276713258773897, -0.5140451703406658 ,  0.09127742788053987;
                0.01322063096422759, -0.9962462506036175 , -0.08554895133864114, -0.02255409664008403;
                -0.5149187674240416 , -0.08012317505073682,  0.853486344222504  , -0.02986309837992267;
                0                  ,  0                  ,  0                  ,  1                  ];
            T_lcam_lev= [ 0.9999407352369797  , 0.009183655542749752,  0.005846920950435052,  0.0005085820608404798;
                -0.009131364645448854, 0.9999186289230431  , -0.008908070070089353, -0.04081979450823404  ;
                -0.005928253827254812, 0.008854151768176144,  0.9999432282899994  , -0.0140781304960408   ;
                0                   , 0                   ,  0                   ,  1                    ];
%             R_lcam_body = T_lcam_body(1:3,1:3);
%             R_lcam_lev = T_lcam_lev(1:3,1:3);
            T_lev_body = T_lcam_lev \ T_lcam_body;
            R_lev_body = T_lev_body(1:3,1:3);
            v = R_lev_body * v_body;
            omega = R_lev_body * omega_body;
        case 'MVSEC'
            v = v_mocap';
            omega = omega_mocap;
        otherwise
            "Warning: No Coordinates Transform Found"
            v = v_body;
            omega = omega_body;
    end
end

function omega = quat2angvel(q1,q2,dt)
    [w1,x1,y1,z1]=q1.parts();
    [w2,x2,y2,z2]=q2.parts();
    omega = 2/dt *[ w1*x2- x1*w2- y1*z2+ z1*y2;
             w1*y2+ x1*z2- y1*w2- z1*x2;
             w1*z2- x1*y2+ y1*x2- z1*w2];
end

% function omega = get_w_from_q(a,b,dt)
%     %     dt = t_b - t_a;
%     % Calculate relative quaternion
%     dq = b * conj(a);
%     dq = normalize(dq);
%     % Assuming dq is normalized and unit quaternion
%     if abs(norm(dq) - 1) > 1e-10
%         error('Quaternion not normalized');
%     end
% 
%     % Extract the vector part and scalar part
%     v = [dq(2), dq(3), dq(4)]; % Vector part
%     s = dq(1); % Scalar part
% 
%     % Calculate the angular velocity
%     % Theta is the angle of rotation
%     theta = 2 * acos(s);
% 
%     % Avoid division by zero or very small numbers
%     if theta > 1e-6
%         % Normalizing the vector part to get the rotation axis
%         rotation_axis = v / sin(theta/2);
%         omega = theta / dt * rotation_axis; % Angular velocity
%     else
%         omega = [0, 0, 0]; % No significant rotation
%     end
% end