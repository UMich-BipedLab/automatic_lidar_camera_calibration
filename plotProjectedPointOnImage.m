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


function plotProjectedPointOnImage(P, bag_data, indices, fig_handles, method, fig_title, show_training_results, pause_each_scan, start_scan)

if ~exist('start_scan', 'var')
    start_scan = 1;
end
num_senes = length(indices);

for scene = 1:num_senes % which dataset
    current_index = indices(scene);
    num_scan = length(bag_data(current_index).scans(:));
    start_scan = min(start_scan, num_scan);
    
    for scan_num = start_scan:num_scan
        loadBagImg(fig_handles(scene), [], bag_data(current_index).scans(scan_num).image, "not display", "clean");
        
        num_tag = size(bag_data(current_index).scans(scan_num).lidar_target, 2);
        for tag_num = 1:num_tag % which tag in this dataset
            if isempty(bag_data(current_index).scans(scan_num).lidar_target(tag_num).(method).corners)
                continue
            else
                current_corners_SR = [bag_data(current_index).scans(scan_num).lidar_target(tag_num).(method).corners];
                current_X_SR = [bag_data(current_index).scans(scan_num).lidar_target(tag_num).(method).pc_points];
        %         if show_baseline_results
        %             projectBackToImage(training_img_fig_handles(i), NSNR_P, current_corners_SR, 5, 'kd', "training_SR", "not display", "Not-Clean");
        %         end

                projectBackToImage(fig_handles(scene), P, current_corners_SR, 50, 'g*', fig_title, "not display", "Not-Clean");
                projectBackToImage(fig_handles(scene), P, current_X_SR, 3, 'r.', fig_title, "not display", "Not-Clean"); 
                showLinedAprilTag(fig_handles(scene), bag_data(current_index).scans(scan_num).camera_target(tag_num), show_training_results);
            end
        end
        title_txt = fig_title + " [Scene: " + bag_data(current_index).bagfile + " at scan #" + num2str(scan_num) + "/" + num2str(num_scan) + "]";
        drawnow
        title(fig_handles(scene), title_txt)
        if pause_each_scan
            pause
        else
            break;
        end
    end
end
