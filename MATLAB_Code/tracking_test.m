clear; clc; close all; warning off;
hyper_para = yaml.ReadYaml('config\settings_tracking.yaml');
load D:/Experiments/LinearSolver/RealDataResults/normal_flow/tracking_corner.mat

% ---------- path load ----------
basic_dataset_path = hyper_para.basic_dataset_path;
dataset_name = hyper_para.dataset_name;
sequence_name = hyper_para.sequence_name;
distortion_mode = hyper_para.distortion_mode;
dataset_path = strcat(basic_dataset_path,dataset_name,sequence_name);
% ---------- hyper_para load ----------
sample_number = hyper_para.sample_number;
sample_gap = hyper_para.sample_gap;
epoch_time_span = hyper_para.epoch_time_span;
loop = hyper_para.loop;
window_size = hyper_para.window_size;
ransac_thres = hyper_para.ransac_thres;
plane_fit_thres = hyper_para.plane_fit_thres;
depth_scale_factor = hyper_para.depth_scale_factor;

% ---------- path settings ----------
depth_root_path = strcat(dataset_path,'depth\');
intrinsics_path = strcat('config\calibration_',dataset_name(1:end-1),'.mat');
undistort_map_path = strcat('config\undistort_map_',dataset_name(1:end-1),'.mat');
img_timestamp_path = strcat(dataset_path,'depth\img_timestamp.txt');
event_timestamp_path = strcat(dataset_path,'\event_timestamp.txt');

% -------------- load ---------------
gt = importdata(strcat(dataset_path,'imu.txt'));
pose = importdata(strcat(dataset_path,'groundtruth_new.txt'));
img_timestamp = readmatrix(img_timestamp_path);
event_timestamp = readmatrix(event_timestamp_path);
camera_param = load(intrinsics_path);
fx = camera_param.K(1,1);
fy = camera_param.K(2,2);

%% Load undistort map


%% ---------- main ----------
time_ref = nflow_all(1, 1);
events_corr = [];
nflow_corr = [];
depth_corr = [];
for i = 1: length(nflow_all)
    if nflow_all(i, 1) == time_ref
        nflow_corr = [nflow_corr;nflow_all(i,4:5)];
        events_corr = [events_corr; nflow_all(i, 1:3)];
        depth_corr = [depth_corr; nflow_all(i,6)];
    else
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
            [v_gt, omega_ref] = getVelocityAt(time_ref, pose,'VECtor');
            omega_gt = getAngularVelocityAt(time_ref,gt)
            omega = motion_para(1:3)
            velocity = motion_para(4:6)
            v_gt
        end
        events_corr = [];
        nflow_corr = [];
        depth_corr = [];
        time_ref = nflow_all(i, 1);
    end
    
end






