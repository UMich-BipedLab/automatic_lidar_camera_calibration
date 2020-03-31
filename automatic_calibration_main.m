%{
 * Copyright (C) 2013-2020, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/) 
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 * 
 * AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu)
 * WEBSITE: https://www.brucerobot.com/
%}

clc, clear, close all
%%%%%%%%%%%%%%%%%%%%%
%%% camera parameters
%%%%%%%%%%%%%%%%%%%%%
intrinsic_matrix = [616.3681640625, 0.0,            319.93463134765625;
                    0.0,            616.7451171875, 243.6385955810547;
                    0.0, 0.0, 1.0];
opt.intrinsic_matrix = intrinsic_matrix;            
distortion_param = [0.099769, -0.240277, 0.002463, 0.000497, 0.000000];

% Initial guess of LiDAR to camera transformation
opt.H_LC.rpy_init = [90 0 90];

% train data id from getBagData.m
trained_ids = [3 4 5]; % 
skip_indices = []; %% skip non-standard 

% validate the calibration result if one has validation dataset(s)
% (Yes:1; No: 0)
% Note: A validation dataset is the same as training set, i.e. it has to
% have calibration targets in the scene; However, a testing set does not
% need targets in the scene. 
validation_flag = 1; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% You usually do not need change setting below %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% parameters of user setting
%%% optimizeAllCorners (0/1): optimize all lidar targets vertices for
%                             different datasets
%   NOTE: this usually only needs to be done ONCE.
%%% skip (0/1/2):
%        0: optimize lidar target's corners 
%           and then calibrate 
%        1: skip optimize lidar target's corners
%        2: just shown calibration results
%%% debug (0/1): print more stuff at the end to help debugging

%%% base_line_method (1/2): 
%                   1: ransac edges seperately and the intersect edges to
%                      estimate corners
%                   2: apply geometry contrain to estimate the corners
% base_line.edge_method (1/2/3):
%                   1: JWG's method
%                   2: Manual pick edge points 
%                      -- top-left, bottom-left, top-right, bottom-left
%                   3: L1-cost to assign edge points
% base_line.more_tags (0/1): if use all tags in a scene for the baseline
%%% calibration_method: 
%                     "4 points"
%                     "IoU"
%%% path.load_dir: directory of saved files
%%% load_all_vertices: pre-calculated vertices (pick the top-5 consistent)
%%% bag_file_path: bag files of images 
%%% mat_file_path: mat files of extracted lidar target's point clouds
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
opts.optimizeAllCorners = 0;
opts.refineAllCorners = 0;
opts.use_top_consistent_vertices = 0;
opts.randperm_to_fine_vertices = 0;
skip = 0; 
debug = 0;
opts.base_line.optimized_method = 1;
opts.base_line.edge_method = 3;
opts.base_line.more_tags = 1;
opts.base_line.show_results = 0;
opts.base_line.L1_cleanup = 0;
opts.base_line.num_scan = 5; % how many scans accumulated to optimize one LiDARTag pose (3)
opts.calibration_method = "4 points";
% opts.calibration_method = "IoU";

path.bag_file_path = 'all/'; 
path.event_name = '';

% save into results into folder         
path.save_name = "IROS2020";
diary Debug % save terminal outputs

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% show figures
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
show_image_refinement = 0;
show_pnp_numerical_result = 0; % show numerical results
show_lidar_target = 0;
% show.lidar_target_optimization = 1;
show_camera_target = 0;
show_training_results = 0; % 1
show_validation_results = 0; %1 
show_testing_results = 0; %1
show_baseline_results = 0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% parameters for optimization of lidar targets
% num_refinement: how many rounds of refinement
% num_lidar_target_pose: how many lidar target poses to optimize H_LC 
% num_scan: accumulate how many scans to optimize a lidar target's corners
% correspondance_per_pose (int): how many correspondance on a target
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
opts.num_refinement = 5 ; % 4 rounds of refinement
opts.num_lidar_target_pose = 1; % (5) how many LiDARTag poses to optimize H_LC (5) (2)
opts.num_scan = 5; % how many scans accumulated to optimize one LiDARTag pose (3)
opts.correspondance_per_pose = 4; % 4 correspondance on a target

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% We have tried several methods to recover the unobserable lidar target's 
%%% corners, those will be added soon
% method:
%        Constraint Customize: Using proposed method stated in the paper
%        Customize: coming soon
%        Coherent Point Drift: coming soon
%        Iterative Closest Point (point): coming soon
%        Iterative Closest Point (plane): coming soon
%        Normal-distributions Transform: coming soon
%        GICP-SE3: coming soon
%        GICP-SE3 (plane): coming soon
%        GICP-SE3-costimized: coming soon
%        Two Hollow Strips: coming soon
%        Project: coming soon
%%% optimization parameters
%   H_TL: optimization for LiDAR target to ideal frame to get corners

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
opt.H_TL.rpy_init = [45 2 3];
opt.H_TL.T_init = [2, 0, 0];
opt.H_TL.H_init = eye(4);
opt.H_TL.method = "Constraint Customize"; 
opt.H_TL.UseCentroid = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% training, validation and testing datasets
%%% random_select (0/1): randomly select training sets
%%% trained_ids: a list of ids of training sets
% training sets (targets included): 
%  -- used all of them to optimize a H_LC
% validation sets (targets included): 
%  -- used the optimized H_LC to validate the results
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
random_select = 0;
BagData = getSceneData(path.bag_file_path, '*.bag');
bag_with_tag_list  = [BagData(:).bagfile];
bag_testing_list = [];
test_pc_mat_list = [];
opts.num_training = length(trained_ids);  
opts.num_validation = length(bag_with_tag_list) - length(skip_indices) - opts.num_training;    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% disp("Refining corners of camera targets ...")
% BagData = refineImageCorners(path.bag_file_path, BagData, skip_indices, show_image_refinement);

% create figure handles
training_img_fig_handles = createFigHandle(opts.num_training, "training_img");
training_pc_fig_handles = createFigHandle(opts.num_training, "training_pc");
validation_fig_handles = createFigHandle(opts.num_validation, "validation_img");
CoF_validation_fig_handles = createFigHandle(opts.num_validation + 1, "confidence_of_range (validation)"); % the +1 is for summary
CoF_training_fig_handles = createFigHandle(opts.num_training + 1, "confidence_of_range (training)"); % the +1 is for summary
validation_pc_fig_handles = createFigHandle(opts.num_validation, "validation_pc");
testing_fig_handles = createFigHandle(size(bag_testing_list, 2), "testing");
opts.base_line.img_hangles = createFigHandle(6, "base_line_vis"); %% don't change

if random_select
    % get training indices
    bag_training_indices = randi([1, length(bag_with_tag_list)], 1, opts.num_training);

    % make sure they are not the same and not consists of undesire index
    while length(unique(bag_training_indices)) ~=  length(bag_training_indices) || ...
            any(ismember(bag_training_indices, skip_indices)) 
        bag_training_indices = randi([1, length(bag_with_tag_list)], 1, opts.num_training);
    end
    
    % get validation indices
    bag_validation_indices = randi(length(bag_with_tag_list), 1, opts.num_validation);

    % make sure they are not the same and not consists of undesire index
    while length(unique(bag_validation_indices)) ~=  length(bag_validation_indices) || ...`
          any(ismember(bag_validation_indices, skip_indices)) || ...
          any(ismember(bag_validation_indices, bag_training_indices)) 
       bag_validation_indices = randi(length(bag_with_tag_list), 1, opts.num_validation);
    end
else
    % overwrite
    bag_training_indices = trained_ids;
    bag_validation_indices = linspace(1, length(bag_with_tag_list), length(bag_with_tag_list));
    bag_validation_indices([trained_ids skip_indices]) = [];
end
bag_chosen_indices = [bag_training_indices, bag_validation_indices];

ans_error_big_matrix = [];
ans_counting_big_matrix = [];

if skip
    load(path.load_dir + 'saved_chosen_indices.mat');
    load(path.load_dir + 'saved_parameters.mat');
end

disp("********************************************")
disp(" Chosen dataset")
disp("********************************************")
disp("-- Skipped: ")
disp(bag_with_tag_list(skip_indices))
disp("-- Training set: ")
disp(bag_with_tag_list(bag_training_indices))            
disp("-- Validation set: ")
disp([bag_with_tag_list(bag_validation_indices)])
disp("-- Chosen set: ")
disp(bag_with_tag_list(bag_chosen_indices))

disp("********************************************")
disp(" Chosen parameters")
disp("********************************************")
fprintf("-- validation flag: %i \n", validation_flag)
fprintf("-- number of training set: %i\n", size(bag_training_indices, 2))
fprintf("-- number of validation set: %i\n", size(bag_validation_indices, 2))
fprintf("-- number of refinement: %i\n", opts.num_refinement)
fprintf("-- number of LiDARTag's poses: %i\n", opts.num_lidar_target_pose)
fprintf("-- number of scan to optimize a LiDARTag pose: %i\n", opts.num_scan)
c = datestr(datetime); 
path.save_dir = path.save_name + "/" + c + "/";


if ~skip
    mkdir(path.save_dir);
    save(path.save_dir + 'saved_parameters.mat', 'opts', 'validation_flag');
    save(path.save_dir + 'saved_chosen_indices.mat', 'skip_indices', 'bag_training_indices', 'bag_validation_indices', 'bag_chosen_indices');
else
    load(path.load_dir + "X_base_line.mat");
    load(path.load_dir + "X_train.mat");
    load(path.load_dir + "Y.mat")
    load(path.load_dir + "save_validation.mat")
    load(path.load_dir + "array.mat")
    load(path.load_dir + "BagData.mat")
end


% loading training image
for k = 1:opts.num_training
    current_index = bag_training_indices(k);
    loadBagImg(training_img_fig_handles(k), [], BagData(current_index).scans(1).image, "not display", "not clean");
    
%     if skip==1 || skip == 2
%         for j = 1:BagData(current_index).num_tag
%             for i = 1:size(BagData(current_index).lidar_target(j).scan(:))
%                 showLinedLiDARTag(training_pc_fig_handles(k), ...
%                                   BagData(current_index).bagfile, ...
%                                   BagData(current_index).lidar_target(j).scan(i), show_lidar_target);
%                 showLinedAprilTag(training_img_fig_handles(k), ...
%                                   BagData(current_index).camera_target(j), show_camera_target);
%             end
%         end
%     end
end

if validation_flag
    for k = 1:opts.num_validation
        current_index = bag_validation_indices(k);
        loadBagImg(validation_fig_handles(k), [], BagData(current_index).scans(1).image, "no display", "not clean");

%         if skip==1 || skip == 2
%             for j = 1:BagData(current_index).num_tag
%                 for i = 1:size(BagData(current_index).lidar_target(j).scan(:))
%                     showLinedLiDARTag(validation_pc_fig_handles(k), ...
%                                       BagData(current_index).bagfile, ...
%                                       BagData(current_index).lidar_target(j).scan(i), show_lidar_target);
%                     showLinedAprilTag(validation_fig_handles(k), ...
%                                       BagData(current_index).camera_target(j), show_camera_target);
%                 end
%             end
%         end
    end
end

disp("All data loaded!")
%%
if skip == 0
    disp("********************************************")
    disp(" Optimizing LiDAR Target Corners")
    disp("********************************************")
    X_train = []; % training corners of lidar targets in 3D
    Y_train = []; % training corners of image targets in 2D
    train_num_tag_array = []; % number of tag in each training data (need to be used later)
    train_tag_size_array = []; % size of tag in each training data (need to be used later)
    H_LT_big = [];
    X_base_line_edge_points = [];
    X_base_line = [];
    Y_base_line = [];
    
    X_validation = []; % validation corners of lidar targets in 3D
    Y_validation = []; % validation corners of image targets in 2D
    X_base_line_edge_points_validation = [];
    X_base_line_validation = [];
    Y_base_line_validation = [];

    
    for k = 1:length(bag_chosen_indices)
        current_index = bag_chosen_indices(k);
        fprintf("Working on %s -->\n", bag_with_tag_list(current_index))
        BagData(current_index) = get4CornersFromAllScans(opt, opts, BagData(current_index));

        % skip undesire index
        if any(ismember(current_index, skip_indices))
            continue
        end
        % if don't want to get validation set, skip
        % everything else but the traing set
        if ~validation_flag
            if ~any(ismember(bag_training_indices, current_index))
                continue;
            end
        end
        
        if any(ismember(bag_training_indices, current_index))
            %% training set
            % 4 x M*i, M is correspondance per scan, i is scan
            X_train = [X_train, BagData(current_index).array.L1_inspired.training_x]; 

            % 3 x M*i, M is correspondance per image, i is image
            Y_train = [Y_train, BagData(current_index).array.L1_inspired.training_y]; 
            H_LT_big = [H_LT_big, BagData(current_index).array.L1_inspired.target_H_LT];
            train_tag_size_array = [train_tag_size_array, BagData(current_index).array.L1_inspired.tag_size];
            train_num_tag_array = [train_num_tag_array, BagData(current_index).array.L1_inspired.num_tag];
            fprintf("--- Got training set: %s\n\n", bag_with_tag_list(current_index))
            
            if isfield(BagData(current_index).array, 'ransac_normal')
                X_base_line = [X_base_line, BagData(current_index).array.ransac_normal.training_x];
                Y_base_line = [Y_base_line, BagData(current_index).array.ransac_normal.training_y];
                X_base_line_edge_points = [X_base_line_edge_points, BagData(current_index).array.ransac_normal.edges];
            end
       
        else 
            %% validation set
            % 4 x M*i, M is correspondance per scan, i is scan
            X_validation = [X_validation, BagData(current_index).array.L1_inspired.training_x]; 

            % 3 x M*i, M is correspondance per image, i is image
            Y_validation = [Y_validation, BagData(current_index).array.L1_inspired.training_y]; 
            
            if isfield(BagData(current_index).array, 'ransac_normal')
                X_base_line_validation = [X_base_line_validation, BagData(current_index).array.ransac_normal.training_x];
                Y_base_line_validation = [Y_base_line_validation, BagData(current_index).array.ransac_normal.training_y];
                X_base_line_edge_points_validation = [X_base_line_edge_points_validation, BagData(current_index).array.ransac_normal.edges];
            end
            fprintf("--- Got validation set: %s\n\n", bag_with_tag_list(current_index))
        end
    end
    drawnow
    save(path.save_dir + 'X_base_line.mat', 'X_base_line');
    save(path.save_dir + 'X_train.mat', 'X_train', 'H_LT_big', 'X_base_line_edge_points');
    save(path.save_dir + 'array.mat', 'train_num_tag_array', 'train_tag_size_array');
    save(path.save_dir + 'Y.mat', 'Y_train', 'Y_base_line');
    save(path.save_dir + 'BagData.mat', 'BagData');
    save(path.save_dir + 'save_validation.mat', 'X_validation', 'Y_validation', 'X_base_line_validation', ...
                         'Y_base_line_validation', 'X_base_line_edge_points_validation');
end

disp("Vertices optimized!")
%%
if ~(skip == 2)
    X_square_no_refinement = X_train;
    X_not_square_refinement = X_base_line;
    disp("********************************************")
    disp(" Calibrating...")
    disp("********************************************")
    switch opts.calibration_method
        case "4 points"
            %%%  one shot calibration (*-NR)
            % square withOUT refinement
            disp('---------------------')
            disp('SNR ...')
            disp('---------------------')
%             [SNR_H_LC, SNR_P, SNR_opt_total_cost] = optimize4Points(opt.H_LC.rpy_init,...
%                                                                     X_square_no_refinement, Y_train, ...
%                                                                     intrinsic_matrix, display);
            [SNR_H_LC, SNR_P, SNR_opt_total_cost, SNR_final, SNR_All] = optimize4Points(opt.H_LC.rpy_init,...
                                                                    X_square_no_refinement, Y_train, ...
                                                                    intrinsic_matrix, show_pnp_numerical_result);                                                    
            calibration(1).H_SNR = SNR_H_LC;
            calibration(1).P_SNR = SNR_P;
            calibration(1).RMSE.SNR = SNR_opt_total_cost;
            calibration(1).All.SNR = SNR_All; 
            
            % NOT square withOUT refinement
            if ~isempty(X_base_line)
                disp('---------------------')
                disp('NSNR ...')
                disp('---------------------')
                [NSNR_H_LC, NSNR_P, NSNR_opt_total_cost, NSNR_final, NSNR_All] = optimize4Points(opt.H_LC.rpy_init, ...
                                                                       X_base_line, Y_base_line, ... 
                                                                       intrinsic_matrix, show_pnp_numerical_result); 
            else
                [NSNR_H_LC, NSNR_P, NSNR_opt_total_cost, NSNR_final, NSNR_All] = deal(zeros(4,4), zeros(3,4), 0, 0, 0);
            end
            calibration(1).H_NSNR = NSNR_H_LC;
            calibration(1).P_NSNR = NSNR_P;
            calibration(1).RMSE_NSNR = NSNR_opt_total_cost;
            calibration(1).All.NSNR = NSNR_All;

            
            for scene = 0: opts.num_refinement-1
                disp('---------------------')
                disp(' Optimizing H_LC ...')
                disp('---------------------')
                
                
                disp('---------------------')
                disp('--- SR_H_LC ...')
                disp('---------------------')
                % square with refinement
                [SR_H_LC, SR_P, SR_opt_total_cost, SR_final, SR_All] = optimize4Points(opt.H_LC.rpy_init, ...
                                                                                       X_train, Y_train, ... 
                                                                                       intrinsic_matrix, show_pnp_numerical_result); 
                calibration(1).H_SR = SR_H_LC;
                calibration(1).P_SR = SR_P;
                calibration(1).RMSE_SR = SR_opt_total_cost;
                calibration(1).All.SR = SR_All;

                % NOT square with refinement
                if ~isempty(X_not_square_refinement)
                    [NSR_H_LC, NSR_P, NSR_opt_total_cost, NSR_final, NSR_All] = optimize4Points(opt.H_LC.rpy_init, ...
                                                                                                X_not_square_refinement, Y_base_line, ...
                                                                                                intrinsic_matrix, show_pnp_numerical_result); 

                else
                    [NSR_H_LC, NSR_P, NSR_opt_total_cost, NSR_final, NSR_All] = deal(zeros(4,4), zeros(3,4), 0, 0, 0);
                end
                calibration(1).H_NSR = NSR_H_LC;
                calibration(1).P_NSR = NSR_P;
                calibration(1).RMSE_NSR = NSR_opt_total_cost;
                calibration(1).All.NSR = NSR_All;
                
                if scene == opts.num_refinement-1
                    break;
                else
                    disp('------------------')
                    disp(' Refining SR_H_LC ...')
                    disp('------------------')
                    X_train = regulizedFineTuneLiDARTagPose(train_tag_size_array, ...
                                                            X_train, Y_train, H_LT_big, SR_P, ...
                                                            opts.correspondance_per_pose, show_pnp_numerical_result);
                    if ~isempty(X_not_square_refinement)
                        X_not_square_refinement = regulizedFineTuneKaessCorners(X_not_square_refinement, Y_base_line,...
                                                                            X_base_line_edge_points, NSR_P, ...
                                                                            opts.correspondance_per_pose, show_pnp_numerical_result);
                    end
                end
            end

        case "IoU"
            % one shot calibration (*-NR)
            [SNR_H_LC, SNR_P, SNR_opt_total_cost, ~, SNR_All] = optimizeIoU(opt.H_LC.rpy_init, ...
                                                                            X_square_no_refinement, Y_train, ...
                                                                            intrinsic_matrix, show_pnp_numerical_result); % square withOUT refinement
            calibration(1).H_SNR = SNR_H_LC;
            calibration(1).P_SNR = SNR_P;
            calibration(1).RMSE.SNR = SNR_opt_total_cost;
            calibration(1).All.SNR = SNR_All;
            
            if ~isempty(X_base_line)
                [NSNR_H_LC, NSNR_P, NSNR_opt_total_cost, ~, NSNR_All] = optimizeIoU(opt.H_LC.rpy_init, ...
                                                                                X_base_line, Y_base_line, ...
                                                                                intrinsic_matrix, show_pnp_numerical_result); % NOT square withOUT refinement
            else
                [NSNR_H_LC, NSNR_P, NSNR_opt_total_cost, ~, NSNR_All] = deal(zeros(4,4), zeros(3,4), 0, 0, 0);
            end
            calibration(1).H_NSNR = NSNR_H_LC;
            calibration(1).P_NSNR = NSNR_P;
            calibration(1).RMSE_NSNR = NSNR_opt_total_cost;
            calibration(1).All.NSNR = NSNR_All;
            
            for scene = 1: opts.num_refinement
                disp('---------------------')
                disp(' Optimizing H_LC ...')
                disp('---------------------')

                [SR_H_LC, SR_P, SR_opt_total_cost, ~, SR_All] = optimizeIoU(opt.H_LC.rpy_init, ...
                                                                 X_train, Y_train, ... 
                                                                 intrinsic_matrix, show_pnp_numerical_result); % square with refinement
                calibration(1).H_SR = SR_H_LC;
                calibration(1).P_SR = SR_P;
                calibration(1).RMSE_SR = SR_opt_total_cost;
                calibration(1).All.SR = SR_All;

                if ~isempty(X_not_square_refinement)
                    [NSR_H_LC, NSR_P, NSR_opt_total_cost, ~, NSR_All] = optimizeIoU(opt.H_LC.rpy_init, ...
                                                                     X_not_square_refinement, Y_base_line, ...
                                                                     intrinsic_matrix, show_pnp_numerical_result); % NOT square with refinement
                else
                    [NSR_H_LC, NSR_P, NSR_opt_total_cost, ~, NSR_All] = deal(zeros(4,4), zeros(3,4), 0, 0, 0);
                end
                calibration(1).H_NSR = NSR_H_LC;
                calibration(1).P_NSR = NSR_P;
                calibration(1).RMSE_NSR = NSR_opt_total_cost;
                calibration(1).All.NSR = NSR_All;

                if scene == opts.num_refinement
                    break;
                else
                    disp('------------------')
                    disp(' Refining H_LT ...')
                    disp('------------------')

                    X_train = regulizedFineTuneLiDARTagPose(train_tag_size_array, ...
                                                            X_train, Y_train, H_LT_big, SR_P, ...
                                                            opts.correspondance_per_pose, show_pnp_numerical_result);
                    if ~isempty(X_not_square_refinement)
                        X_not_square_refinement = regulizedFineTuneKaessCorners(X_not_square_refinement, ...
                                                            Y_base_line, X_base_line_edge_points, NSR_P, ...
                                                            opts.correspondance_per_pose, show_pnp_numerical_result);
                    end
                end
            end
            
    end
    if skip == 0
        save(path.save_dir + 'calibration.mat', 'calibration');
        save(path.save_dir + 'save_validation.mat', 'X_validation', 'Y_validation');
        save(path.save_dir + 'NSNR.mat', 'NSNR_H_LC', 'NSNR_P', 'NSNR_opt_total_cost');
        save(path.save_dir + 'SNR.mat', 'SNR_H_LC', 'SNR_P', 'SNR_opt_total_cost');
        save(path.save_dir + 'NSR.mat', 'NSR_H_LC', 'NSR_P', 'NSR_opt_total_cost');
        save(path.save_dir + 'SR.mat',  'SR_H_LC', 'SR_P', 'SR_opt_total_cost');
    elseif skip == 1
        save(path.load_dir + 'calibration.mat', 'calibration');
        save(path.load_dir + 'save_validation.mat', 'X_validation', 'Y_validation');
        save(path.load_dir + 'NSNR.mat', 'NSNR_H_LC', 'NSNR_P', 'NSNR_opt_total_cost');
        save(path.load_dir + 'SNR.mat', 'SNR_H_LC', 'SNR_P', 'SNR_opt_total_cost');
        save(path.load_dir + 'NSR.mat', 'NSR_H_LC', 'NSR_P', 'NSR_opt_total_cost');
        save(path.load_dir + 'SR.mat',  'SR_H_LC', 'SR_P', 'SR_opt_total_cost');
    end
else
    % load saved data
    load(path.load_dir + 'calibration.mat');
    load(path.load_dir + "NSNR.mat");
    load(path.load_dir + "SNR.mat");
    load(path.load_dir + "NSR.mat");
    load(path.load_dir + "SR.mat");
    load(path.load_dir + "save_validation.mat")
end

disp("Done calibrating!")
%%
disp("****************** NSNR-training ******************")
disp('NSNR_H_LC: ')
disp(' R:')
disp(NSNR_H_LC(1:3, 1:3))
disp(' RPY (XYZ):')
disp(rad2deg(rotm2eul(NSNR_H_LC(1:3, 1:3), "XYZ")))
disp(' T:')
disp(-inv(NSNR_H_LC(1:3, 1:3))*NSNR_H_LC(1:3, 4))
disp("========= Error =========")
disp(' Training Total Error (pixel)')
disp(sqrt(NSNR_opt_total_cost))
disp(' Training Error Per Corner (pixel)')
disp(sqrt(NSNR_opt_total_cost/size(Y_base_line, 2)))
calibration(1).error_struc.training_results.id = [bag_training_indices(:)]';
calibration(1).error_struc.training_results.name = [BagData(bag_training_indices(:)).bagfile];
calibration(1).error_struc.training_results.NSNR_RMSE = [sqrt(NSNR_opt_total_cost/size(Y_base_line, 2))];

disp("****************** NSR-training ******************")
disp('NSR_H_LC: ')
disp(' R:')
disp(NSR_H_LC(1:3, 1:3))
disp(' RPY (XYZ):')
disp(rad2deg(rotm2eul(NSR_H_LC(1:3, 1:3), "XYZ")))
disp(' T:')
disp(-inv(NSR_H_LC(1:3, 1:3))*NSR_H_LC(1:3, 4))
disp("========= Error =========")
disp(' Training Total Error (pixel)')
disp(sqrt(NSR_opt_total_cost))
disp(' Training Error Per Corner (pixel)')
disp(sqrt(NSR_opt_total_cost/size(Y_base_line, 2))) 
calibration(1).error_struc.training_results.NSR_RMSE = [sqrt(NSR_opt_total_cost/size(Y_base_line, 2))];

disp("****************** SNR-training ******************")
disp('SNR_H_LC: ')
disp(' R:')
disp(SNR_H_LC(1:3, 1:3))
disp(' RPY (XYZ):')
disp(rad2deg(rotm2eul(SNR_H_LC(1:3, 1:3), "XYZ")))
disp(' T:')
disp(-inv(SNR_H_LC(1:3, 1:3))*SNR_H_LC(1:3, 4))
disp("========= Error =========")
disp(' Training Total Error (pixel)')
disp(sqrt(SNR_opt_total_cost))
disp(' Training Error Per Corner (pixel)')
disp(sqrt(SNR_opt_total_cost/size(Y_train, 2)))
calibration(1).error_struc.training_results.SNR_RMSE = [sqrt(SNR_opt_total_cost/size(Y_train, 2))];

disp("****************** SR-training ******************")
disp('H_LC: ')
disp(' R:')
disp(SR_H_LC(1:3, 1:3))
disp(' RPY (XYZ):')
disp(rad2deg(rotm2eul(SR_H_LC(1:3, 1:3), "XYZ")))
disp(' T:')
disp(-inv(SR_H_LC(1:3, 1:3))*SR_H_LC(1:3, 4))
disp("========= Error =========")
disp(' Training Total Error (pixel)')
disp(sqrt(SR_opt_total_cost))
disp(' Training Error Per Corner (pixel)')
disp(sqrt(SR_opt_total_cost/size(Y_train, 2)))
calibration(1).error_struc.training_results.SR_RMSE = [sqrt(SR_opt_total_cost/size(Y_train, 2))];


% with refinement and without refinement (inandout comparision and RMSE)
%%%%% training
%%%% 1) L1-inspired
%%% inandout
[t_SNR_count, t_SR_count] = inAndOutBeforeAndAfter_v02(bag_training_indices, BagData, SNR_P, SR_P, 'L1_inspired');
%%% RMSE
SR_training_cost = verifyCornerAccuracyWRTDataset_v02(bag_training_indices, BagData, SR_P, 'L1_inspired', 'refinement');
SNR_training_cost = verifyCornerAccuracyWRTDataset_v02(bag_training_indices, BagData, SNR_P, 'L1_inspired', 'no_refinement');

%%%% 2) baseline                                           
if ~isempty(X_not_square_refinement) || ~isempty(X_base_line)
    %%% InandOut
    [t_NSNR_count, t_NSR_count] = inAndOutBeforeAndAfter_v02(bag_training_indices, BagData, NSNR_P, NSR_P, 'ransac_normal');
    %%% RMSE
    zeroCells = num2cell(zeros(opts.num_training, 1));
    NSR_training_cost = struct('RMSE', zeroCells);
%     NSR_training_cost = verifyCornerAccuracyWRTDataset_v02(bag_training_indices, BagData, NSR_P, 'ransac_normal', 'refinement');
    NSNR_training_cost = verifyCornerAccuracyWRTDataset_v02(bag_training_indices, BagData, NSNR_P, 'ransac_normal', 'no_refinement');
else
    [t_NSNR_count, t_NSR_count] = deal(-1);
    zeroCells = num2cell(zeros(opts.num_training,1));
    NSR_training_cost = struct('RMSE', zeroCells);
    NSNR_training_cost = struct('RMSE', zeroCells); 
end

% validation
% L1-inspired
%%% verify corner accuracy
if validation_flag
    % inandout
    [SNR_count, SR_count] = inAndOutBeforeAndAfter_v02(bag_validation_indices, BagData, SNR_P, SR_P, 'L1_inspired');
    % RMSE                                       
    SR_validation_cost = verifyCornerAccuracyWRTDataset_v02(bag_validation_indices, BagData, SR_P, 'L1_inspired', 'refinement');
    SNR_validation_cost = verifyCornerAccuracyWRTDataset_v02(bag_validation_indices, BagData, SNR_P, 'L1_inspired', 'no_refinement');

    if ~isempty(X_not_square_refinement) || ~isempty(X_base_line)    
        [NSNR_count, NSR_count] = inAndOutBeforeAndAfter_v02(bag_validation_indices, BagData, NSNR_P, NSR_P, 'ransac_normal');
        zeroCells = num2cell(zeros(opts.num_validation, 1));
        NSR_validation_cost = struct('RMSE', zeroCells);
%         NSR_validation_cost = verifyCornerAccuracyWRTDataset_v02(bag_validation_indices, BagData, NSR_P, 'ransac_normal', 'refinement');
        NSNR_validation_cost = verifyCornerAccuracyWRTDataset_v02(bag_validation_indices, BagData, NSNR_P, 'ransac_normal', 'no_refinement');
    else
%         mat2cell(zeros(3,1), ones(1,3), [1])
        zeroCells = num2cell(zeros(opts.num_validation, 1));
        NSR_validation_cost = struct('RMSE', zeroCells);
        NSNR_validation_cost = struct('RMSE', zeroCells);
        [NSNR_count, NSR_count] = deal(-1);
    end

    for scene = 1:opts.num_validation
        calibration(1).error_struc.validation(scene).id = bag_validation_indices(scene);   
        calibration(1).error_struc.validation(scene).name = extractBetween(BagData(bag_validation_indices(scene)).bagfile,"",".bag");
        calibration(1).error_struc.validation(scene).NSNR_RMSE = [NSNR_validation_cost(scene).RMSE];
        calibration(1).error_struc.validation(scene).NSR_RMSE = [NSR_validation_cost(scene).RMSE];
        calibration(1).error_struc.validation(scene).SNR_RMSE = [SNR_validation_cost(scene).RMSE];
        calibration(1).error_struc.validation(scene).SR_RMSE = [SR_validation_cost(scene).RMSE];
    end
end


calibration(1).count.training.SNR = t_SNR_count;
calibration(1).count.training.SR = t_SR_count;
calibration(1).count.training.NSR = t_NSR_count;
calibration(1).count.training.NSNR = t_NSNR_count;

calibration(1).count.validation.SNR = SNR_count;
calibration(1).count.validation.SR = SR_count;
calibration(1).count.validation.NSR = NSR_count;
calibration(1).count.validation.NSNR = NSNR_count;

for scene = 1:opts.num_training
        disp('------')
        current_index = bag_training_indices(scene);
        fprintf("---dataset: %s\n", bag_with_tag_list(current_index))
        calibration(1).error_struc.training(scene).id = bag_training_indices(scene);   
        calibration(1).error_struc.training(scene).name = extractBetween(BagData(bag_training_indices(scene)).bagfile,"",".bag");
        disp("-- RMS Error Per Corner (pixel)")
        disp(' NSNR training RMS Error Per Corner (pixel)')
        disp(NSNR_training_cost(scene).RMSE)
        calibration(1).error_struc.training(scene).NSNR_RMSE = [NSNR_training_cost(scene).RMSE];
        disp(' NSR training RMS Error Per Corner (pixel)')
        disp(NSR_training_cost(scene).RMSE)
        calibration(1).error_struc.training(scene).NSR_RMSE = [NSR_training_cost(scene).RMSE];
        disp(' SNR training RMS Error Per Corner (pixel)')
        disp(SNR_training_cost(scene).RMSE)
        calibration(1).error_struc.training(scene).SNR_RMSE = [SNR_training_cost(scene).RMSE];
        disp(' SR training RMS Error Per Corner (pixel)')
        disp(SR_training_cost(scene).RMSE)
        calibration(1).error_struc.training(scene).SR_RMSE = [SR_training_cost(scene).RMSE];
end
%
disp("***************************************************************************************")
disp("***************************************************************************************")
% disp("------------------")
disp(" training results")
% disp("------------------")
disp(struct2table(calibration(1).error_struc.training_results))
[calibration(1).error_struc.training_results.NSNR_RMSE; calibration(1).error_struc.training_results.NSR_RMSE; calibration(1).error_struc.training_results.SNR_RMSE; calibration(1).error_struc.training_results.SR_RMSE]
% disp("------------------")
% disp(" training error")
% disp("------------------")
% disp(struct2table(calibration(1).error_struc.training))
%
if validation_flag
%     disp("------------------")
    disp(" validation error")
%     disp("------------------")
    disp(struct2table(calibration(1).error_struc.validation))
%     disp("NSNR_RMSE--validata on its own method")
%     [calibration(1).error_struc.validation.baseline.NSNR_RMSE]
    disp("NSR_RMSE")
    [calibration(1).error_struc.validation.NSR_RMSE]
    disp("SNR_RMSE")
    [calibration(1).error_struc.validation.SNR_RMSE]
    disp("SR_RMSE")
    [calibration(1).error_struc.validation.SR_RMSE]
    
    disp("------ALL info-------")
    [calibration(1).error_struc.validation.NSNR_RMSE;
     calibration(1).error_struc.validation.NSR_RMSE;
     calibration(1).error_struc.validation.SNR_RMSE;
     calibration(1).error_struc.validation.SR_RMSE]
    disp("------paper info-------")
    disp("-- training")
    training_res = [calibration(1).error_struc.training_results.NSNR_RMSE;
                    calibration(1).error_struc.training_results.SNR_RMSE]

    disp("-- validating")
    validating_res = [calibration(1).error_struc.validation.NSNR_RMSE;
                      calibration(1).error_struc.validation.SNR_RMSE]
    disp('summary')
    validating_mean = mean(validating_res')'
    validating_std = std(validating_res')'
end

   
%% Draw results (by projecting points back to an image)
% SNR with L1-inspired (training)
pause_each_scan = 1;
start_scan = 1;
plotProjectedPointOnImage(SNR_P, BagData, bag_training_indices, training_img_fig_handles, ...
                          "L1_inspired", "training_{SR}", show_training_results, ...
                          pause_each_scan, start_scan)
if validation_flag
    plotProjectedPointOnImage(SNR_P, BagData, bag_validation_indices, validation_fig_handles, ...
                              "L1_inspired", "validation_{SR}", 1, ...
                              pause_each_scan, start_scan)
end


disp("********************************************") 
disp("---- Projected using:")
disp(SR_P)
disp('--- H_LC: ')
disp('-- R:')
disp(SR_H_LC(1:3, 1:3))
disp('-- RPY (XYZ):')
disp(rad2deg(rotm2eul(SR_H_LC(1:3, 1:3), "XYZ")))
disp('-- T:')
disp(-inv(SR_H_LC(1:3, 1:3))*SR_H_LC(1:3, 4))
disp("********************************************")

if skip == 0
    save(path.save_dir + 'calibration.mat', 'calibration');
elseif skip == 1
    save(path.load_dir + 'calibration.mat', 'calibration');
end

%% plot CoR
confidence_of_range = computeConfidenceOfRange(SR_P, BagData, bag_validation_indices, "L1_inspired", "no_refinement");
confidence_of_range_training = computeConfidenceOfRange(SR_P, BagData, bag_training_indices, "L1_inspired", "no_refinement");
plotConfidenceOfRange(CoF_validation_fig_handles, confidence_of_range, "CoR of validation sets")
plotConfidenceOfRange(CoF_training_fig_handles, confidence_of_range_training, "CoR of training set")
% training_targets_statistics = summarizeTargets(BagData, bag_training_indices);
% validation_targets_statistics = summarizeTargets(BagData, bag_validation_indices);

%% project testing results
% load testing images and testing pc mat
% testing_set_pc = loadTestingMatFiles(path.mat_file_path, test_pc_mat_list);
% for scene = 1: size(bag_testing_list, 2)
%     loadBagImg(testing_fig_handles(scene), path.bag_file_path, bag_testing_list(scene), "not display", "Not clean"); 
%     projectBackToImage(testing_fig_handles(scene), SR_P, testing_set_pc(scene).mat_pc, 3, 'g.', "testing", show_testing_results, "Not-Clean");
% end



