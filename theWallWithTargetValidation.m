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
clc

train = 1;
if train
    clear
    train = 1;
    img_clean = 0;
    use_data = 3; 
else
    img_clean = 1;
    use_data = 4; 
end


intrinsic_matrix = [616.3681640625, 0.0,            319.93463134765625;
                    0.0,            616.7451171875, 243.6385955810547;
                    0.0, 0.0, 1.0];
intrinsic_matrix = [616.3681640625, 0.0,            319.93463134765625;
                    0.0,            616.7451171875, 243.6385955810547;
                    0.0, 0.0, 1.0];
                
                
path.load_all_vertices = "ICRA2020/10-Dec-2019 21:10:26/";
% parameters
opts.num_scan = 10;
opts.num_corner = 1;
opts.num_boundary = 2;
opts.pc_iter = 1;
opts.num_beams = 32;
opts.min_points = 10;
opts.display_img = 1;


opts.num_lidar_target_pose = 2;
opts.optimizeAllCorners = 0;
opts.use_top_consistent_vertices = 1;

% optimization initial guess
% opt.T = [-28, 6, 0];
opt.H_LC.rpy_init = [90 0 90];
opt.H_LC.T_init = [0.1 0.02 -0.19];
opt.H_LC.method = 3; % 1: PnP; 2: L2L; 3: L2LandPnP

opt.H_TL.rpy_init = [90 0 90];
opt.H_TL.T_init = [2, 0, 0];
opt.H_TL.UseCentroid = 1;
opt.H_TL.method = "Constraint Customize"; 

c = datestr(datetime); 
data.lidar.lidar_target.save_name = "ICRA2020";
data.lidar.lidar_target.save_dir = data.lidar.lidar_target.save_name + "/" + c + "/";

% load point cloud
if use_data == 1
    data.image.name = "the-wall-2.bag";
    data.lidar.lidar_target.path = "/home/brucebot/workspace/griztag/src/matlab/matlab/";
    data.image.camera_target.corners = [490 404 522;
                                        235 259 331
                                        1   1   1];
    data.lidar_target.name = "velodyne_points-the-wall-dataset--2019-11-17-15-50.mat";                                
    data.lidar.lidar_target.corners = [28.200 28.086 28.006 28.020 28.0137 28.0106 28.0059 28.0058;
                                       -0.571 -2.299 -6.750 -7.174 -7.5901 -7.4792 -7.4308 -6.7495;
                                        2.303  3.460  3.537  2.361  0.8447  1.1812 1.6874 3.5371;
                                        1      1      1      1       1      1      1      1];
    show_line_fitting = 1;
    data.lidar.lidar_target.lines(1).line = best_fit_3D_line(data.lidar.lidar_target.corners(:,1:2), show_line_fitting);
    data.lidar.lidar_target.lines(2).line = best_fit_3D_line(data.lidar.lidar_target.corners(:,3:end), show_line_fitting);
elseif use_data == 2
    data.image.name = "theWall-botom-right-edge.bag";
    data.image.camera_target.corners = [317 196 348;
                                        287 324 379;   
                                        1   1   1];
    data.lidar.lidar_target.path = "/home/brucebot/workspace/griztag/src/matlab/matlab/";
    data.lidar_target.name = "velodyne_points-the-wall-clearnerGround--2019-12-04-10-52.mat";  
    data.lidar.lidar_target.corners = [28.5912    3.4549    1.6772;
                                       28.2443    4.8177    1.1673;
                                       27.8830    6.0693    0.8305;
                                       27.7872    6.5481    0.6643;
                                       27.6538    7.0232    0.4980;
                                       27.4863    7.5865    0.3320;

                                       29.1586    1.0794    1.6993;
                                       29.1676    0.9727    1.1890;
                                       29.1424    0.8802    0.8485;
                                       29.1577    0.7788    0.6787;
                                       29.1533    0.7736    0.5091;
                                       29.1506    0.6564    0.3395;
                                       29.1497    0.5801    0.1695;
                                       29.1722    0.4736         0;
                                       29.1459    0.5698   -0.1694;
                                       29.1544    0.4580   -0.3395;
                                       29.2143    0.2753   -0.5100;
                                       29.1418    0.3662   -0.6782;
                                       29.1585    0.2596   -0.8486;
                                       29.1891    0.2598   -1.0193;
                                       29.1753    0.1731   -1.1887;
                                       29.1723    0.0611   -1.3589;
                                       29.1519    0.0611   -1.5278;
                                       29.1546    0.0509   -1.6979;
                                       29.1482   -0.0203   -1.8681;
                                       29.1445   -0.1373   -2.0380]';
    data.lidar.lidar_target.corners = [data.lidar.lidar_target.corners; ones(1, size(data.lidar.lidar_target.corners,2))];
    show_line_fitting = 1;
    data.lidar.lidar_target.lines(1).line = best_fit_3D_line(data.lidar.lidar_target.corners(:,1:6), show_line_fitting);
    data.lidar.lidar_target.lines(2).line = best_fit_3D_line(data.lidar.lidar_target.corners(:,7:end), show_line_fitting);
    
elseif use_data == 3
    data.image.name = "the-wall-intrinsic-upper1.bag";
    % wall
    data.image.camera_target(1).corners = [308 197 325;
                                           326 350 398;   
                                           1   1   1];
    data.lidar.lidar_target.path = "/home/brucebot/workspace/griztag/src/matlab/matlab/";
    data.lidar.lidar_target.name = "velodyne_points-the-wall-with-2-tags--2019-12-10-14-26.mat";
    data.lidar.lidar_target.load_all_vertices = "ICRA2020/10-Dec-2019 21:10:26/";
%     reshape(permute([topEdge.Position],[2,1]), 3,[])'
    data.lidar.lidar_target.corners = [28.4603    7.9730   -0.6878
                                       28.6799    7.4011   -0.5170
                                       28.8963    6.4803   -0.3448
                                       29.1634    5.4894   -0.1725
                                       29.3034    4.7881         0
                                       29.5088    4.0947    0.1731
                                       29.7393    3.3726    0.3484
                                       30.0270    2.2525    0.5256

                                       29.8565    0.7923   -3.8017
                                       29.9043    0.7987   -3.2223
                                       29.9607    0.8840   -2.7980
                                       29.9756    0.9996   -2.4484
                                       30.0058    1.0059   -2.0994
                                       29.9975    1.1209   -1.9238
                                       30.0131    1.2002   -1.7493
                                       30.0266    1.2060   -1.5749
                                       30.0828    1.1136   -1.4023
                                       30.0741    1.2237   -1.2263
                                       30.0852    1.3083   -1.0516
                                       30.0904    1.3190   -0.8766
                                       30.0949    1.3192   -0.7010
                                       30.0940    1.3297   -0.5258
                                       30.1007    1.4143   -0.3508
                                       30.1049    1.5251   -0.1752
                                       30.1460    1.4322         0
                                       30.1320    1.5422    0.1754
                                       30.1461    1.6274    0.3515
                                       30.1592    1.6334    0.5272]';
    data.lidar.lidar_target.corners = [data.lidar.lidar_target.corners; ones(1, size(data.lidar.lidar_target.corners,2))];
    show_line_fitting = 1;
    data.lidar.lidar_target.lines(1).line = best_fit_3D_line(data.lidar.lidar_target.corners(:,1:8), show_line_fitting);
    data.lidar.lidar_target.lines(2).line = best_fit_3D_line(data.lidar.lidar_target.corners(:,9:end), show_line_fitting);
    
    % tags
    bag_num = 1;
    bag_data(1).num_tag = 2;
    bag_data(1).bagfile = data.image.name;
    bag_data(1).lidar_target(1).pc_file = 'velodyne_points-upper1-LargeTag--2019-12-05-20-14.mat'; %% payload
    bag_data(1).lidar_target(1).tag_size = 1.216;
    
    bag_data(1).lidar_target(2).pc_file = 'velodyne_points-upper1-SmallTag--2019-12-05-20-13.mat'; %% payload
    bag_data(1).lidar_target(2).tag_size = 0.8051;
    bag_data(1).lidar_full_scan = "";
    
%     reshape(permute([bigTag.Position],[2,1]), 2,[])
    bag_data(1).camera_target(1).corners = [386   349   411   372
                                         361   375   404   417;
                                         1, 1, 1, 1];
    bag_data(1).camera_target(2).corners = [601   515   622   536
                                         309   330   391   408;
                                         1, 1, 1, 1];
elseif use_data == 4
    data.image.name = "the-wall-intrinsic-upper2.bag";
    data.image.camera_target(1).corners = [];
    data.lidar.lidar_target.path = "/home/brucebot/workspace/griztag/src/matlab/matlab/LiDARTag_data/";
    data.lidar.lidar_target.name = "velodyne_points-upper2-SmallTag--2019-12-05-20-16.mat";
    data.lidar.lidar_target.load_all_vertices = "ICRA2020/11-Dec-2019 13:16:02/";
    
    % tags
    bag_num = 2;
    bag_data(2).num_tag = 1;
    bag_data(2).bagfile = data.image.name;
    bag_data(2).lidar_target(1).pc_file = 'velodyne_points-upper1-LargeTag--2019-12-05-20-14.mat'; %% payload
    bag_data(2).lidar_target(1).tag_size = 1.216;
    
%     reshape(permute([bigTag.Position],[2,1]), 2,[])
    bag_data(2).camera_target(1).corners = [172   282   293   175
                                            361   352   247   246
                                            1, 1, 1, 1];
elseif use_data == 5
    %% not yet finish! 
    data.image.name = "wavefield-2tags-proper.bag";
    % wall
    data.image.camera_target(1).corners = [308 197 325;
                                           326 350 398;   
                                           1   1   1];
    data.lidar.lidar_target.path = "/home/brucebot/workspace/griztag/src/matlab/matlab/";
    data.lidar.lidar_target.name = "velodyne_points-the-wall-with-2-tags--2019-12-10-14-26.mat";
    data.lidar.lidar_target.load_all_vertices = "ICRA2020/10-Dec-2019 21:10:26/";
%     reshape(permute([topEdge.Position],[2,1]), 3,[])'
    data.lidar.lidar_target.corners = [28.4603    7.9730   -0.6878
                                       28.6799    7.4011   -0.5170
                                       28.8963    6.4803   -0.3448
                                       29.1634    5.4894   -0.1725
                                       29.3034    4.7881         0
                                       29.5088    4.0947    0.1731
                                       29.7393    3.3726    0.3484
                                       30.0270    2.2525    0.5256

                                       29.8565    0.7923   -3.8017
                                       29.9043    0.7987   -3.2223
                                       29.9607    0.8840   -2.7980
                                       29.9756    0.9996   -2.4484
                                       30.0058    1.0059   -2.0994
                                       29.9975    1.1209   -1.9238
                                       30.0131    1.2002   -1.7493
                                       30.0266    1.2060   -1.5749
                                       30.0828    1.1136   -1.4023
                                       30.0741    1.2237   -1.2263
                                       30.0852    1.3083   -1.0516
                                       30.0904    1.3190   -0.8766
                                       30.0949    1.3192   -0.7010
                                       30.0940    1.3297   -0.5258
                                       30.1007    1.4143   -0.3508
                                       30.1049    1.5251   -0.1752
                                       30.1460    1.4322         0
                                       30.1320    1.5422    0.1754
                                       30.1461    1.6274    0.3515
                                       30.1592    1.6334    0.5272]';
    data.lidar.lidar_target.corners = [data.lidar.lidar_target.corners; ones(1, size(data.lidar.lidar_target.corners,2))];
    show_line_fitting = 1;
    data.lidar.lidar_target.lines(1).line = best_fit_3D_line(data.lidar.lidar_target.corners(:,1:8), show_line_fitting);
    data.lidar.lidar_target.lines(2).line = best_fit_3D_line(data.lidar.lidar_target.corners(:,9:end), show_line_fitting);
    
    % tags
    bag_num = 1;
    bag_data(1).num_tag = 2;
    bag_data(1).bagfile = data.image.name;
    bag_data(1).lidar_target(1).pc_file = 'velodyne_points-upper1-LargeTag--2019-12-05-20-14.mat'; %% payload
    bag_data(1).lidar_target(1).tag_size = 1.216;
    
    bag_data(1).lidar_target(2).pc_file = 'velodyne_points-upper1-SmallTag--2019-12-05-20-13.mat'; %% payload
    bag_data(1).lidar_target(2).tag_size = 0.8051;
    bag_data(1).lidar_full_scan = "";
    
%     reshape(permute([bigTag.Position],[2,1]), 2,[])
    bag_data(1).camera_target(1).corners = [386   349   411   372
                                         361   375   404   417;
                                         1, 1, 1, 1];
    bag_data(1).camera_target(2).corners = [601   515   622   536
                                         309   330   391   408;
                                         1, 1, 1, 1];                                        
  
end


data.lidar.lidar_target.mat_file_path = "/home/brucebot/workspace/griztag/src/matlab/matlab/LiDARTag_data/";
pc = loadPointCloud(data.lidar.lidar_target.path, data.lidar.lidar_target.name);
data.lidar.lidar_points = splitPointsBasedOnRing(opts, pc);
data.lidar.lidar_target.points = [data.lidar.lidar_points(:).points];
figure(1)
scatter3(data.lidar.lidar_target.points(1,:), data.lidar.lidar_target.points(2,:), data.lidar.lidar_target.points(3,:), '.')
hold on
scatter3(0,0,0,50, 'b.')
axis 'equal'
origin = [0 0 0 0 0 1;
          0 1 0 0 0 0;
          0 0 0 1 0 0];
plot3(origin(1,:), origin(2,:), origin(3,:))
fig_pc = gca;
hold off

% load images
data.image.path = "/home/brucebot/workspace/griztag/src/griz_tag/bagfiles/matlab/";
if  ~isempty(data.image.camera_target(1).corners)
    data.image = refineImageRightCorner(opts, data.image, opts.display_img);
end

 
% Optimize LiDAR vertices
X_train = [];
Y_train = [];
H_LT_big = [];

X_training_tmp = [];
Y_training_tmp = [];
H_LT_tmp = [];
train_num_tag_array = [];
train_tag_size_array = [];
bag_data(bag_num) = refineImageCorners(data.image.path, bag_data(bag_num), [], 1, img_clean);

for j = 1:bag_data(bag_num).num_tag
    % optimize lidar targets corners
    [bag_data(bag_num), H_LT] = getAll4CornersReturnHLT(j, opt, ...
                                         data.lidar.lidar_target, bag_data(bag_num), ...
                                         opts);
    % draw camera targets 
    bag_data(bag_num).camera_target(j).four_corners_line = ...
    point2DToLineForDrawing(bag_data(bag_num).camera_target(j).corners);
    showAllLinedLiDARTag(fig_pc, ...
                         bag_data(bag_num).bagfile, ...
                         bag_data(bag_num).lidar_target(j), 1);
    figure(1000);
    fig = gca;
    showLinedAprilTag(fig, ...
                      bag_data(bag_num).camera_target(j), 1);
    X_training_tmp = [X_training_tmp, bag_data(bag_num).lidar_target(j).scan(:).corners];
    Y_training_tmp = [Y_training_tmp, repmat(bag_data(bag_num).camera_target(j).corners, 1, opts.num_lidar_target_pose)];
    H_LT_tmp = [H_LT_tmp, H_LT];
    train_num_tag_array = [train_num_tag_array, repmat(bag_data(bag_num).num_tag, 1, opts.num_lidar_target_pose)];
    train_tag_size_array = [train_tag_size_array, repmat(bag_data(bag_num).lidar_target(j).tag_size, 1, opts.num_lidar_target_pose)];
end

% 4 x M*i, M is correspondance per scan, i is scan
data.lidar.X_train = [X_train, X_training_tmp]; 

% 3 x M*i, M is correspondance per image, i is image
data.image.Y_train = [Y_train, Y_training_tmp]; 
H_LT_big = [H_LT_big, H_LT_tmp];


if train
    show_pnp_numerical_result = 1;
    [SR_H_LC, SR_P, SR_opt_total_cost, SR_final, SR_All] = optimizePnPandL2L(opt.H_LC, ...
                                                                             data.lidar, data.image, ... 
                                                                             intrinsic_matrix, show_pnp_numerical_result);
end
figure(1000);
fig = gca;
if  ~isempty(data.image.camera_target(1).corners)
    projectBackToImage(fig, SR_P, data.lidar.lidar_target.corners, 30, 'y*', "results", "display", "Not-Clean");
end
projectBackToImage(fig, SR_P, data.lidar.X_train, 30, 'r.', "results", "display", "Not-Clean");
disp("Done")
disp("Projected using:")
disp("-- P: ")
disp(SR_P)
% disp("-- H:")
% disp(SR_H_LC)
