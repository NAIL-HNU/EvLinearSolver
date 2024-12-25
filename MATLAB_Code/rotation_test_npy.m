clear; clc; close all; warning off;
addpath(genpath(pwd))
addpath(genpath('D:\Code\Mat\Utils\yamlmatlab'))
addpath(genpath('D:\Code\Mat\Utils\npy-matlab'))

hyper_para = yaml.ReadYaml('config\settings_rotation.yaml');
basic_path = 'D:\Dataset\';
dataset_name = 'IJRR\';
sequence_name = 'dynamic_rotation';
events_timestamp_folder = strcat(basic_path,dataset_name,sequence_name,'\events_timestamp\');
events_folder = strcat(basic_path,dataset_name,sequence_name,'\events\');
flow_folder = strcat(basic_path,dataset_name,sequence_name,'\flow_prediction\');


ransac_thres = hyper_para.ransac_thres;
sample_number = hyper_para.sample_number;
loop = hyper_para.loop;
distortion_mode = 1;
basic_dataset_path = hyper_para.basic_dataset_path;
dataset_name = hyper_para.dataset_name;
sequence_name = hyper_para.sequence_name;
dataset_path = strcat(basic_dataset_path,dataset_name,sequence_name);
gt = load(strcat(dataset_path,'imu.txt')); % This is only for evaluation

% load D:/Experiments/LinearSolver/RealDataResults/normal_flow/rotation_shapes_rotation.mat


dataset_name = 'IJRR/';
undistort_map_path = strcat('config\undistort_map_',dataset_name(1:end-1),'.mat');
undistort_map = load(undistort_map_path, 'undistort_map');
undistort_map = undistort_map.undistort_map;
intrinsics_path = strcat('config\calibration_',dataset_name(1:end-1),'.mat');
camera_param = load(intrinsics_path);
fx = camera_param.K(1,1);
fy = camera_param.K(2,2);



fileFolder=fullfile(events_folder);
ev_filelist=dir(fullfile(fileFolder,'*.txt'));

fileFolder=fullfile(flow_folder);
filelist=dir(fullfile(fileFolder,'*.npy'));

epoch_time_span = 0.02;

result = [];
for i = 1:length(filelist)
    flow_sub = readNPY(strcat(flow_folder,filelist(i).name));
    events_sub = readmatrix(strcat(events_folder,ev_filelist(i).name));
    events_sub(:,2:3) = events_sub(:,2:3)+1;
%   
    valid_flow_id = find(~isnan(flow_sub(:,1)));
    events_sub = events_sub(valid_flow_id,:);
    flow_sub = flow_sub(valid_flow_id,:);

    event_epoch_undistort = events_sub;
    event_epoch_undistort(:, 2:3) = undistort_map(sub2ind(camera_param.img_size, events_sub(:, 3), events_sub(:, 2)), :);
    time_ref = mean(event_epoch_undistort(:,1));
    min_time = time_ref - epoch_time_span/2;
    max_time = time_ref + epoch_time_span/2;

    corr_id = event_epoch_undistort(:,1) >= min_time & event_epoch_undistort(:,1) <= max_time;
    events_corr = event_epoch_undistort(corr_id,:);
%     nflow_corr = flow_sub(corr_id, :)./[fx, fy];
    nflow_corr = flow_sub(corr_id, :);

    if size(nflow_corr,1) > sample_number
        omega_loop = zeros(loop,3);
        time_loop = zeros(loop,1);
        for loop_i = 1:loop
            % ---------- linear solver ----------
            tic
            [omega,A,m] = rotational_estimation(nflow_corr, events_corr(:,2:3), camera_param, ransac_thres);
            time_loop(loop_i) = toc;
            omega_loop(loop_i,:) =  omega';
        end
%         time_ref
        omega_est = median(omega_loop,1);
        result = [result;time_ref,omega_est];
%         omega_gt = getAngularVelocityAt(time_ref, gt)
        events_corr = [];
        nflow_corr = [];
    end

end
save("result\boxes_rotation.mat", "result");
[RMSE, error_avg] =  evaluate_angular_velocity(result, gt);








