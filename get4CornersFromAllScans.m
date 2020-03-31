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


function bag_data = get4CornersFromAllScans(opt, opts, bag_data)
%     save(path.save_dir + extractBetween(bag_data.bagfile,"",".bag") + '_' + tag_num + '_' + path.event_name + '_all_scan_refined_corners.mat', 'refinement_scan_total');    
    num_scan = length(bag_data.scans(:));
    scans_t(num_scan).all = struct();
%         fprintf(['
' repmat('.',1,num_scan) '

']);
    
    parforProgress(num_scan);
    parfor scan = 1:num_scan
%         if scan == 1 || mod(scan, floor(num_scan/10)) == 0 || scan == num_scan
%             fprintf("----working on scan #%i/%i
", scan, num_scan)
%         end
        if bag_data.scans(scan).num_tag.original == 0
            warning("skipped scan_num: %i in training set due to no tag detected.", scan)
            continue;
        end
        % passing scan number only for warning if any scan is skipped
        scans_t(scan).all = get4CornersFromAScan(opt, opts, bag_data.scans(scan));
        parforProgress;
%             fprintf('|
');
    end
    parforProgress(0);

    bag_data.array.L1_inspired.training_x = []; % will be used for other functions
    bag_data.array.L1_inspired.target_H_LT = []; % will be used for other functions
    bag_data.array.L1_inspired.training_y = []; % will be used for other functions
    bag_data.array.L1_inspired.num_tag = []; % will be used for other functions
    bag_data.array.L1_inspired.tag_size = []; % will be used for other functions
    
    if isfield(scans_t(1).all.lidar_target(1), 'ransac_normal')
        flag_use_rn = 1;
        bag_data.array.ransac_normal.training_x = []; % will be used for other functions
        bag_data.array.ransac_normal.training_y = []; % will be used for other functions
        bag_data.array.ransac_normal.edges = []; % will be used for other functions
        bag_data.array.ransac_normal.num_tag = []; % will be used for other functions
        bag_data.array.ransac_normal.tag_size = []; % will be used for other functions
    else
        flag_use_rn = 0;   
    end
    
    for scan = 1:num_scan
        if size([scans_t(scan).all], 2) == 0
            continue;
        end
        bag_data.scans(scan) = scans_t(scan).all;
%         figure(100)
%         cla
        
        for tag = 1:bag_data.scans(scan).num_tag.original
            if isempty(scans_t(scan).all.lidar_target(tag).L1_inspired.corners)
                warning("Scan#%i, tag#%i/%i has been skipped using L1_inspired method.", scan, tag, bag_data.scans(scan).num_tag.original)
                bag_data.scans(scan).num_tag.L1_inspired = bag_data.scans(scan).num_tag.L1_inspired - 1;
                continue
            else     
                bag_data.array.L1_inspired.training_x = [bag_data.array.L1_inspired.training_x, scans_t(scan).all.lidar_target(tag).L1_inspired.corners];
                bag_data.array.L1_inspired.target_H_LT = [bag_data.array.L1_inspired.target_H_LT, scans_t(scan).all.lidar_target(tag).L1_inspired.H_LT];
                bag_data.array.L1_inspired.training_y = [bag_data.array.L1_inspired.training_y, scans_t(scan).all.camera_target(tag).L1_inspired.corners];
                bag_data.array.L1_inspired.tag_size = [bag_data.array.L1_inspired.tag_size, bag_data.scans(scan).lidar_target(tag).tag_size];
                bag_data.array.L1_inspired.num_tag = [bag_data.array.L1_inspired.num_tag, bag_data.scans(scan).num_tag];
%                 scatter3(scans_t(scan).all.lidar_target(tag).L1_inspired.corners(1,:),scans_t(scan).all.lidar_target(tag).L1_inspired.corners(2,:), scans_t(scan).all.lidar_target(tag).L1_inspired.corners(3,:) )
%                 hold on
%                 scatter3(bag_data.scans(scan).lidar_target(tag).L1_inspired.pc_points(1, :), ...
%                          bag_data.scans(scan).lidar_target(tag).L1_inspired.pc_points(2, :), ...
%                          bag_data.scans(scan).lidar_target(tag).L1_inspired.pc_points(3, :))
            end
        end
%         title("scan#" + num2str(scan))
%         axis equal
%         hold off
%         pause(0.5)
        
        for tag = 1:bag_data.scans(scan).num_tag.original   
            if flag_use_rn
                if isempty(scans_t(scan).all.lidar_target(tag).ransac_normal.corners)
                    bag_data.scans(scan).num_tag.ransac_normal = bag_data.scans(scan).num_tag.ransac_normal - 1;
                    warning("Scan#%i, tag#%i has been skipped using baseline method.", scan, tag)
                    continue
                else
                    bag_data.array.ransac_normal.training_x = [bag_data.array.ransac_normal.training_x, scans_t(scan).all.lidar_target(tag).ransac_normal.corners];
                    bag_data.array.ransac_normal.edges = [bag_data.array.ransac_normal.edges, scans_t(scan).all.lidar_target(tag).ransac_normal.edges];
                    bag_data.array.ransac_normal.training_y = [bag_data.array.ransac_normal.training_y, scans_t(scan).all.camera_target(tag).ransac_normal.corners];
                    bag_data.array.ransac_normal.tag_size = [bag_data.array.ransac_normal.tag_size, bag_data.scans(scan).lidar_target(tag).tag_size];
                    bag_data.array.ransac_normal.num_tag = [bag_data.array.ransac_normal.num_tag, bag_data.scans(scan).num_tag];
                end
            end
        end

    end
    fprintf("Data set:
 --- L1-inspired: %i
 --- ransac_normal: %i
", size([bag_data.array.L1_inspired.training_x], 2), size([bag_data.array.ransac_normal.training_x], 2))
end
