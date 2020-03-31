%{
 * Copyright (C) 2013-2025, The Regents of The University of Michigan.
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS AND
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


function bag_data = getRefined4CornersL1Inspired(opt, opts, bag_data, tag_num)
    X_train = [bag_data.lidar_target(tag_num).L1_inspired.corners];
    Y_train = [bag_data.camera_target(tag_num).corners];
    H_LT = [bag_data.lidar_target(tag_num).L1_inspired.H_LT];
    tag_size = bag_data.lidar_target(tag_num).tag_size;
    show_pnp_numerical_result = 0;
    
    for i = 1: opts.num_refinement
        if opts.calibration_method == "4 points"
            [SR_H_LC, SR_P, ~, ~, ~] = optimize4Points(opt.H_LC.rpy_init, ...
                                           X_train, Y_train, ... 
                                           opt.intrinsic_matrix, ...
                                           show_pnp_numerical_result);
        elseif opts.calibration_method == "IoU"
            [SR_H_LC, SR_P, ~, ~, ~] = optimizeIoU(opt.H_LC.rpy_init, ...
                                         X_train, Y_train, ... 
                                         opt.intrinsic_matrix, ...
                                         show_pnp_numerical_result); % square with refinement
        else
            error("This refinement method is not yet implemented: %s",...
                    opts.calibration_method);
        end

        if i == opts.num_refinement
            break;
        else
            X_train = regulizedFineTuneEachLiDARTagPose(tag_size, ...
                        X_train, Y_train, H_LT, SR_P, ...
                        show_pnp_numerical_result);
        end
    end

    [centroid, normals] = computeCentroidAndNormals(X_train);
    bag_data.lidar_target(tag_num).L1_inspired.refined_centroid = centroid;
    bag_data.lidar_target(tag_num).L1_inspired.refined_normals = normals;
    bag_data.lidar_target(tag_num).L1_inspired.refined_corners = X_train;
    bag_data.lidar_target(tag_num).L1_inspired.refined_H_LC = SR_H_LC;
    bag_data.lidar_target(tag_num).L1_inspired.refined_P = SR_P;
end
