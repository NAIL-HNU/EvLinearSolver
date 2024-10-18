function [omega, velocity, A, m] = tracking_estimation(N,X,camera_param,depth_list,ransac_thres)
    fx = camera_param.K(1,1);
    cx = camera_param.K(1,3);
    fy = camera_param.K(2,2);
    cy = camera_param.K(2,3);
    NO2 = length(N);
    A = [];
    m = [];
    for i =1:NO2
        x = (X(i,2)-cx)/fx;
        y = (X(i,3)-cy)/fy;
        a = [-1,0,x;
             0,-1,y];
        if depth_list(i)>0
            inverse_depth = 1/depth_list(i);
            a = a*inverse_depth;
            b = [x*y, -(1+x*x), y;
                1+y*y, -x*y, -x];
            d = N(i,:)*[a,b];
            A = [A; d/norm(N(i,:))];
            m = [m;norm(N(i,:))];
        end
    end
%     if ~isempty(A)
%         length(A)
%     end
    if length(A)<6
        velocity=[];
        omega = [];
        A =[];
        m =[];
        return;
    end
  
%     [Seg_id_list, D, nondeg_id_list] = NormalFlowSeg(N, 10);
%     e = RANSAC_adaptive(A,m,6,(0.1/fx), Seg_id_list, nondeg_id_list);
    [e, solver_inlier_ratio]=RANSAC(A,m,6,ransac_thres);
    solver_inlier_ratio
    if isempty(e)
        velocity = [];
        omega = [];
        A =[];
        m =[];
    else
        velocity = e(1:3);
        omega = e(4:6);
        crtErrors = A * e - m;
        crtInliersInd = crtErrors .* crtErrors < (2/fx) * 2;
%         LS_inlier_ratio = length(crtInliersInd)/length(A)
        A = A(crtInliersInd, :);
        m = m(crtInliersInd, :);
    end
end