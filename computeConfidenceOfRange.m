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


function confidence_of_range = computeConfidenceOfRange(P, bag_data, validation_indices, method, refinement)
    num_senes = length(validation_indices);
    
    for scene = 1:num_senes % which dataset
        current_index = validation_indices(scene);
        num_scan = length(bag_data(current_index).scans(:));
        confidence_of_range(scene).bagfile = bag_data(current_index).bagfile;
        
        for scan_num = 1:num_scan
            num_tag = size(bag_data(current_index).scans(scan_num).lidar_target, 2);
            
            for tag_num = 1:num_tag % which tag in this dataset
                % refinement and no-refinement should be the same
                if isempty(bag_data(current_index).scans(scan_num).lidar_target(tag_num).(method).corners) 
                    continue
                else
                    if strcmpi(refinement, 'no_refinement')
                        current_corners_X = [bag_data(current_index).scans(scan_num).lidar_target(tag_num).(method).corners];
                        H_LT = bag_data(current_index).scans(scan_num).lidar_target(tag_num).(method).H_LT;
                        
                    else
                        current_corners_X = [bag_data(current_index).scans(scan_num).lidar_target(tag_num).(method).refined_corners];
                        H_LT = bag_data(current_index).scans(scan_num).lidar_target(tag_num).(method).refined_H_LC;
                    end
                    num_corners = size(H_LT, 2) / 4;
                    
                    current_corners_Y = [bag_data(current_index).scans(scan_num).camera_target(tag_num).(method).corners];
                    squared_sum = verifyCornerAccuracy(current_corners_X(:, 1:4), current_corners_Y(:, 1:4), P);
                    
                    confidence_of_range(scene).scans(scan_num).targets(tag_num).RMSE = sqrt(squared_sum / num_corners);
                    confidence_of_range(scene).scans(scan_num).targets(tag_num).distance  = norm(H_LT(1:3, 4));
                    confidence_of_range(scene).scans(scan_num).targets(tag_num).rpy = rad2deg(rotm2eul(H_LT(1:3, 1:3), 'xyz'));
                end
            end
        end
    end
end
