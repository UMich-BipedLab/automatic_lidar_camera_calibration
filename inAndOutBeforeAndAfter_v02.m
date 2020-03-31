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

function [count_no_refinement, count_refinement] = inAndOutBeforeAndAfter_v02(bag_indices, bag_data, P1, P2, method)
    if ~exist('method', 'var')
        method = "L1_inspired";
    end

	for bag = 1:length(bag_indices) % which dataset is this
        current_index = bag_indices(bag);
        name = bag_data(current_index).bagfile;
        num_scans = length(bag_data(current_index).scans);
        
        data_total_points = bag_data(current_index).total_points;
        
        count_array_no_refinement = zeros(2, num_scans); % 1 is on and in; 2 is outside
        count_array_refinement = zeros(2, num_scans); % 1 is on and in; 2 is outside

        for scan_num = 1:num_scans % which scan in this dataset
            NR_counter = 0; % no refinement counter for inside and on the polygon
            WR_counter = 0; % with refinement counter for inside and on the polygon
            total_points = 0;
            num_tag = size(bag_data(current_index).scans(scan_num).lidar_target, 2);
            for tag_num = 1:num_tag % which tag in this dataset
                if isempty(bag_data(current_index).scans(scan_num).lidar_target(tag_num).(method).corners)
                    continue
                else
                    current_camera_corners = [bag_data(current_index).scans(scan_num).camera_target(tag_num).(method).corners];
                    current_camera_corners = makeConvexHull(current_camera_corners);
                    current_camera_corners = checkHomogeneousCorners(current_camera_corners);

                    current_lidar_target_pc = [bag_data(current_index).scans(scan_num).lidar_target(tag_num).payload_points];
                    current_lidar_target_pc = checkHomogeneousCorners(current_lidar_target_pc);
                    total_points = total_points + size(current_lidar_target_pc, 2);

                    projection_before_refinement = projectionMap(current_lidar_target_pc, P1);
                    [in_before, on_before] = inpolygon(projection_before_refinement(1,:)', projection_before_refinement(2,:)', ...
                                                       current_camera_corners(1,:)' ,current_camera_corners(2,:)');

                    projection_after_refinement = projectionMap(current_lidar_target_pc, P2);
                    [in_after, on_after] = inpolygon(projection_after_refinement(1,:)', projection_after_refinement(2,:)', ...
                                                     current_camera_corners(1,:)' ,current_camera_corners(2,:)');
                    NR_counter = NR_counter + numel(projection_before_refinement(in_before)) + numel(projection_before_refinement(on_before));
                    WR_counter = WR_counter + numel(projection_after_refinement(in_after)) + numel(projection_after_refinement(on_after));

                end 
            end
            % in and on the polygon
            count_array_no_refinement(1, scan_num) = NR_counter;
            count_array_refinement(1, scan_num) = WR_counter;
            
            % outside of the polygon 
            count_array_no_refinement(2, scan_num) = total_points - NR_counter;
            count_array_refinement(2, scan_num) = total_points - WR_counter;
        end
        count_no_refinement(bag).name = name;
        count_no_refinement(bag).count = sum(count_array_no_refinement(1,:)); % total count of this dataset
        count_no_refinement(bag).count_outside = sum(count_array_no_refinement(2,:)); % total count of this dataset
        count_no_refinement(bag).std = std(count_array_no_refinement'); % std of count of each scan
        count_no_refinement(bag).fraction = count_no_refinement(bag).count / data_total_points; 
        count_no_refinement(bag).fraction_outside = count_no_refinement(bag).count_outside / data_total_points; 
        count_no_refinement(bag).total_num_point = data_total_points; 
        
        count_refinement(bag).name = name;
        count_refinement(bag).count = sum(count_array_refinement(1,:)); % total count of this dataset
        count_refinement(bag).count_outside = sum(count_array_refinement(2,:)); % total count of this dataset
        count_refinement(bag).std = std(count_array_refinement'); % std of count of each scan
        count_refinement(bag).fraction = count_refinement(bag).count / data_total_points; 
        count_refinement(bag).fraction_outside = count_refinement(bag).count_outside / data_total_points; 
        count_refinement(bag).total_num_point = data_total_points; 
	end
end
