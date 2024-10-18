function [omega,A,m] = rotational_estimation(N,X,camera_param,ransac_thres)
    fx = camera_param.K(1,1);
    cx = camera_param.K(1,3);
    fy = camera_param.K(2,2);
    cy = camera_param.K(2,3);
    NO2 = length(N);
    A = [];
    m = [];
    for i =1:NO2
        x = (X(i,1)-cx)/fx;
        y = (X(i,2)-cy)/fy;
        b = [x*y, -(1+x*x), y;
            1+y*y, -x*y, -x];
        A = [A;N(i,:)*b/norm(N(i,:))];
        m = [m;norm(N(i,:))];
    end
    if length(m)>=3
        [omega, bestSize]=RANSAC(A,m,3, ransac_thres, 200);
        crtErrors = A * omega - m;
        crtInliersInd = crtErrors .* crtErrors < (1/fx) * 2;
        A = A(crtInliersInd, :);
        m = m(crtInliersInd, :);
    end
end