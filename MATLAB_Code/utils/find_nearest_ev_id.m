function index = find_nearest_ev_id(event,events_corr)
%FIND_NEAREST_EV_ID 此处显示有关此函数的摘要
%   此处显示详细说明
    t = event(1);
    x = round(event(2));
    y = round(event(3));
    index = 0;
    for i = 1:size(events_corr,1)
        if round(events_corr(i,2))==x && round(events_corr(i,3))==y
            if (t - events_corr(i,1))< 1e-3
                index = i;
                break
            end
        end
    end
end

