clear; clc; close all; warning off;
hyper_para = yaml.ReadYaml('config\settings_rotation.yaml');
load D:/Experiments/LinearSolver/RealDataResults/normal_flow/rotation_shapes_rotation.mat
 
ransac_thres = hyper_para.ransac_thres;
sample_number = hyper_para.sample_number;
loop = hyper_para.loop;
distortion_mode = 1;
basic_dataset_path = hyper_para.basic_dataset_path;
dataset_name = hyper_para.dataset_name;
sequence_name = hyper_para.sequence_name;
dataset_path = strcat(basic_dataset_path,dataset_name,sequence_name);
gt = load(strcat(dataset_path,'imu.txt')); % This is only for evaluation

intrinsics_path = strcat('config\calibration_',dataset_name(1:end-1),'.mat');
camera_param = load(intrinsics_path);
fx = camera_param.K(1,1);
fy = camera_param.K(2,2);

undistort_map_path = strcat('config\undistort_map_',dataset_name(1:end-1),'.mat');
undistort_map = load(undistort_map_path, 'undistort_map');
undistort_map = undistort_map.undistort_map;
time_ref = nflow_all(1, 1);
events_corr = [];
nflow_corr = [];
for i = 1: length(nflow_all)
    if nflow_all(i, 1) == time_ref
        nflow_corr = [nflow_corr;nflow_all(i,4:5)];
        events_corr = [events_corr; nflow_all(i, 1:3)];
    else
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
        time_ref
        omega_est = median(omega_loop,1)'
        omega_gt = getAngularVelocityAt(time_ref, gt)
        events_corr = [];
        nflow_corr = [];
        time_ref = nflow_all(i, 1);
    end
    end
    
end

