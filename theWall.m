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
clc, clear
intrinsic_matrix = [616.3681640625, 0.0,            319.93463134765625;
                    0.0,            616.7451171875, 243.6385955810547;
                    0.0, 0.0, 1.0];

use_data = 2; 
% parameters
opts.num_scan = 10;
opts.num_corner = 1;
opts.num_boundary = 2;
opts.pc_iter = 1;
opts.num_beams = 32;
opts.min_points = 10;
opts.display_img = 1;
% optimization initial guess
opt.T = [-28, 6, 0];
opt.H_LC.rpy_init = [90 0 90];

% load point cloud
if use_data == 1
    
    data.image.name = "the-wall-2.bag";
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
end

data.lidar_target.path = "/home/brucebot/workspace/griztag/src/matlab/matlab/";
pc = loadPointCloud(data.lidar_target.path, data.lidar_target.name);
data.lidar_points = splitPointsBasedOnRing(opts, pc);
data.lidar_target.points = [data.lidar_points(:).points];
figure(1)
scatter3(data.lidar_target.points(1,:), data.lidar_target.points(2,:), data.lidar_target.points(3,:), '.')
hold on
scatter3(0,0,0,50, 'b.')
axis 'equal'
origin = [0 0 0 0 0 1;
          0 1 0 0 0 0;
          0 0 0 1 0 0];
plot3(origin(1,:), origin(2,:), origin(3,:))
hold off

% load images

data.image.path = "/home/brucebot/workspace/griztag/src/griz_tag/bagfiles/matlab/";
data.image = refineImageRightCorner(opts, data.image, opts.display_img);

 
% optimization
% opt = optimizeBoundaries(opt, opts, data.lidar_target.points);
% opt.boundaries
% opt.corners




show_pnp_numerical_result = 1;
[SR_H_LC, SR_P, SR_opt_total_cost, SR_final, SR_All] = optimizeL2L(opt.H_LC.rpy_init, ...
                                                                 data.lidar.lidar_target, data.image.camera_target, ... 
                                                                 intrinsic_matrix, show_pnp_numerical_result);
figure(1000);
fig = gca;
projectBackToImage(fig, SR_P, data.lidar.lidar_target.corners, 30, 'y*', "results", "display", "Not-Clean");
disp("done")
