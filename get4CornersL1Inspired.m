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

function bag_data = get4CornersL1Inspired(opt, opts, bag_data, tag_num)

    X = bag_data.lidar_target(tag_num).payload_points;
    target_len = bag_data.lidar_target(tag_num).tag_size;

    % clean data
    [X_clean, X_ref, X_std, L_infinity, N] = cleanLiDARTargetCore(opt.H_TL, X, target_len);
    bag_data.lidar_target(tag_num).L1_inspired.clean_up.std = N*(X_std(1));
    bag_data.lidar_target(tag_num).L1_inspired.clean_up.L_infinity = L_infinity;
    bag_data.lidar_target(tag_num).L1_inspired.clean_up.L_1 = sum(X_ref(1:3,:), 1);

    % cost
    opt_tmp = optimizeCost(opt.H_TL, X_clean, target_len, ...
                           bag_data.lidar_target(tag_num).L1_inspired.clean_up.std/2);
    target_lidar = [0 -target_len/2 -target_len/2 1; % x y z 1
                    0 -target_len/2  target_len/2 1;
                    0  target_len/2  target_len/2 1;
                    0  target_len/2 -target_len/2 1]';
                
    % reject if not enough coverage
    X_projected = opt_tmp.H_opt * X_clean;
    [~, area_frame] = makeConvexHull(target_lidar(2:3,:));
    [index, area_2D] = convhull(X_projected(2:3,:)');
    ratio = area_2D/area_frame;

    if ratio < 0.8
        bag_data.lidar_target(tag_num).L1_inspired.corners = [];
        bag_data.lidar_target(tag_num).L1_inspired.four_corners_line = [];
        bag_data.lidar_target(tag_num).L1_inspired.pc_points_original = [];
        bag_data.lidar_target(tag_num).L1_inspired.pc_points = [];
        bag_data.lidar_target(tag_num).L1_inspired.centroid = [];
        bag_data.lidar_target(tag_num).L1_inspired.normal_vector = [];
        bag_data.lidar_target(tag_num).L1_inspired.H_LT = [];
        
        bag_data.camera_target(tag_num).L1_inspired.corners = [];
    else
        corners3D = opt_tmp.H_opt \ target_lidar;
        corners3D = sortrows(corners3D', 3, 'descend')';
        [centroid, normals] = computeCentroidAndNormals(corners3D);
        bag_data.lidar_target(tag_num).L1_inspired.corners = corners3D;
        bag_data.lidar_target(tag_num).L1_inspired.four_corners_line = point3DToLineForDrawing(corners3D);
        bag_data.lidar_target(tag_num).L1_inspired.pc_points_original = X;
        bag_data.lidar_target(tag_num).L1_inspired.pc_points = X_clean;
        bag_data.lidar_target(tag_num).L1_inspired.centroid = centroid;
        bag_data.lidar_target(tag_num).L1_inspired.normal_vector = normals;
        bag_data.lidar_target(tag_num).L1_inspired.H_LT = inv(opt_tmp.H_opt);
        
        bag_data.camera_target(tag_num).L1_inspired.corners =  bag_data.camera_target(tag_num).corners;
    end
    

%     figure(999)
%     cla
%     X_projected = opt_tmp.H_opt * X_clean;
%     scatter3(X_projected(1,:), X_projected(2,:), X_projected(3,:))
%     hold on, axis equal,
%     
% 
%     plot3(X_projected(1,index), X_projected(2,index), X_projected(3,index))
% %     scatter3()
%     four_corners_line = point3DToLineForDrawing(target_lidar);
%     plot3(four_corners_line(1,:), four_corners_line(2,:), four_corners_line(3,:))
end
