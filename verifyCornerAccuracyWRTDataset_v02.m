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

function cost = verifyCornerAccuracyWRTDataset_v02(indices, bag_data, P, method, refinement)
    for i = 1:length(indices) % which dataset
        current_index = indices(i);
        num_scans = length(bag_data(current_index).scans);
        num_corners = 0;
        square_summed_error = 0;
        for scan_num = 1:num_scans % which scan in this dataset
            num_tag = size(bag_data(current_index).scans(scan_num).lidar_target, 2);
            for tag_num = 1:num_tag % which tag in this dataset
                % refinement and no-refinement should be the same
                if isempty(bag_data(current_index).scans(scan_num).lidar_target(tag_num).(method).corners) 
                    continue
                else
                    if strcmpi(refinement, 'no_refinement')
                        current_corners_X = [bag_data(current_index).scans(scan_num).lidar_target(tag_num).(method).corners];
                    else
                        current_corners_X = [bag_data(current_index).scans(scan_num).lidar_target(tag_num).(method).refined_corners];
                    end
                    current_corners_Y = [bag_data(current_index).scans(scan_num).camera_target(tag_num).(method).corners];

                    num_corners = num_corners + length(current_corners_Y);
                    scan_cost = verifyCornerAccuracy(current_corners_X(:, 1:4), current_corners_Y(:, 1:4), P);
                    square_summed_error = square_summed_error + scan_cost;
                end
            end
        end
        cost(i).name = bag_data(current_index).bagfile;
        cost(i).square_summed_error = square_summed_error;
        cost(i).num_corners = num_corners;
        cost(i).RMSE = sqrt(square_summed_error/num_corners); % total cost of this dataset
%         cost(i).std = std(cost_array'); % std of cost of each scan
    end
end
