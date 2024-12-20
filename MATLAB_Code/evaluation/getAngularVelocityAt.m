function av = getAngularVelocityAt(timestamp, gt)

    index_a = find(gt(:,1) <= timestamp,1,"last");
    index_b = find(gt(:,1) >=timestamp,1,"first");
    time_pre = index_a;
    time_after = index_b; 
    if gt(time_pre,1) ~= gt(time_after,1)
        ratio = (timestamp - gt(time_pre,1))/(gt(time_after,1)-gt(time_pre,1));
        av = gt(time_pre,5:7)'*(1-ratio)+ratio*gt(time_after,5:7)';
    else
        av = gt(time_pre,5:7)';
    end
end