clear; clc; close all; warning off;
addpath(genpath(pwd))
addpath(genpath('D:\Code\Mat\Utils\yamlmatlab'))
addpath(genpath('D:\Code\Mat\Utils\npy-matlab'))

task = 'run';
hyper_para = yaml.ReadYaml('config\settings_tracking.yaml');

basic_dataset_path = hyper_para.basic_dataset_path;
dataset_name = hyper_para.dataset_name;
sequence_name = hyper_para.sequence_name;
distortion_mode = hyper_para.distortion_mode;

events_timestamp_folder = strcat(basic_dataset_path,dataset_name,sequence_name,'\events_timestamp\');
events_folder = strcat(basic_dataset_path,dataset_name,sequence_name,'\events\');
flow_folder = strcat(basic_dataset_path,dataset_name,sequence_name,'\flow_prediction\');


% ---------- hyper_para load ----------
sample_number = hyper_para.sample_number;
sample_gap = hyper_para.sample_gap;
loop = hyper_para.loop;
window_size = hyper_para.window_size;
ransac_thres = hyper_para.ransac_thres;
depth_scale_factor = hyper_para.depth_scale_factor;


% ---------- path settings ----------
dataset_path = strcat(basic_dataset_path,dataset_name,sequence_name);
depth_root_path = strcat(dataset_path,'\depth\');
intrinsics_path = strcat('config\calibration_',dataset_name(1:end-1),'.mat');
undistort_map_path = strcat('config\undistort_map_',dataset_name(1:end-1),'.mat');
img_timestamp_path = strcat(dataset_path,'\depth\img_timestamp.txt');
event_timestamp_path = strcat(dataset_path,'\event_timestamp.txt');

% -------------- load ---------------

fileFolder=fullfile(events_folder);
ev_filelist=dir(fullfile(fileFolder,'*.txt'));
fileFolder=fullfile(flow_folder);
filelist=dir(fullfile(fileFolder,'*.npy'));
gt = importdata(strcat(dataset_path,'imu.txt'));
pose = importdata(strcat(dataset_path,'groundtruth_new.txt'));
img_timestamp = readmatrix(img_timestamp_path);
event_timestamp = readmatrix(event_timestamp_path);
undistort_map = load(undistort_map_path, 'undistort_map');
undistort_map = undistort_map.undistort_map;
camera_param = load(intrinsics_path);
fx = camera_param.K(1,1);
fy = camera_param.K(2,2);

depth_folder = strcat(dataset_path,'depth\');
depth_files = dir(fullfile(depth_folder, '*.png'));
depth_files = {depth_files.name};
depth_files = sort(depth_files);
% for i = 1:length(depth_files)
%     depth_img = imread(strcat(depth_folder,depth_files{i}));
%     depth_img = double(depth_img);
%     depth_img = depth_img / depth_scale_factor;
% end

%% ---------- main ----------
result = [];
epoch_time_span = 0.02;
switch task
    case 'run'
        result = [];
        first_ev = readmatrix(strcat(events_folder,ev_filelist(1).name));
        time_shift = first_ev(1,1);
        img_timestamp = img_timestamp+time_shift;
        j = 1;
        for i = 1:length(filelist)
            flow_sub = readNPY(strcat(flow_folder,filelist(i).name));
            events_sub = readmatrix(strcat(events_folder,ev_filelist(j).name));
            j = j+1;
            if events_sub(end,1)-events_sub(1,1)>0.04
            else
               events_sub = [events_sub; readmatrix(strcat(events_folder,ev_filelist(j).name))] ;
            end

            events_sub(:,2:3) = events_sub(:,2:3)+1;
            %
            valid_flow_id = find(~isnan(flow_sub(:,1)));
            events_sub = events_sub(valid_flow_id,:);
            flow_sub = flow_sub(valid_flow_id,:);

            event_epoch_undistort = events_sub;
            event_epoch_undistort(:, 2:3) = undistort_map(sub2ind(camera_param.img_size, events_sub(:, 3), events_sub(:, 2)), :);
            
            inside_id = event_epoch_undistort(:, 2) > 1 & event_epoch_undistort(:, 2) < camera_param.img_size(2) &...
                event_epoch_undistort(:, 3) > 1 & event_epoch_undistort(:, 3) < camera_param.img_size(1);
            event_epoch_undistort = event_epoch_undistort(inside_id, :);
            flow_sub = flow_sub(inside_id, :);

            time_ref = mean(event_epoch_undistort(:,1));
            min_time = time_ref - epoch_time_span/2;
            max_time = time_ref + epoch_time_span/2;

            corr_id = event_epoch_undistort(:,1) >= min_time & event_epoch_undistort(:,1) <= max_time;
            events_corr = event_epoch_undistort(corr_id,:);
            nflow_corr = flow_sub(corr_id, :);

           
            first_depth_map_id = find(img_timestamp <= min_time, 1, 'last');
            second_depth_map_id = find(img_timestamp >= max_time, 1, "first");
            depth_map_list = cell(second_depth_map_id - first_depth_map_id + 1,1);
            depth_map_time_list = [];
            for id = first_depth_map_id:second_depth_map_id
                depth_map_time_list = [depth_map_time_list; img_timestamp(id)];
                depth_map_sub = double(imread(strcat(depth_folder,depth_files{i}))) / depth_scale_factor;
                depth_map_list{id - first_depth_map_id + 1, 1} = depth_map_sub;
            end

            depth_corr = get_depth_from_map(events_corr, depth_map_time_list, depth_map_list, camera_param);
            corr_id = depth_corr > 0;
            events_corr = events_corr(corr_id,:);
            nflow_corr = nflow_corr(corr_id,:);
            depth_corr = depth_corr(corr_id,:);
            
            if size(nflow_corr,1) > sample_number
                motion_para_loop = zeros(loop,6);
                time_loop = zeros(loop,1);
                for loop_i = 1:loop
                    tic
                    [omega, velocity, A, m] = tracking_estimation(nflow_corr,events_corr,camera_param, depth_corr,ransac_thres);
                    if isempty(omega) || omega(1)==0
                        continue;
                    end
                    time_loop(loop_i) = toc;
                    motion_para_loop(loop_i,:) = [omega',velocity'];
                end
                motion_para = median(motion_para_loop,1)';
                omega = motion_para(1:3)';
                velocity = motion_para(4:6)';
                result = [result;time_ref,omega,velocity];
            end
            save(strcat('result\',sequence_name(1:end-1),"result"));
        end
    case 'evaluation'
        result = load(strcat("result\",sequence_name(1:end-1),".mat"));
        result = result.result;
        [RMSE_ang, AvgErr_ang] = evaluate_angular_velocity(result, gt);
        [RMSE_lnr, AvgErr_lnr] = evaluate_velocity(result,pose,'VECtor');
end

