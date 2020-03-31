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


% path = '/home/chenxif/Documents/me590/Calibration/automatic_calibration/';
% clc, clear
% path = 'moving_bags/';
% % BagData = t_getSceneData(path,'*.bag', 4)
% BagData = t_getSceneData(path,'*.bag');
% disp("done")

function BagData = getSceneData(path, ext, scene, pair_num)

    files_from_a_folder = dir(fullfile(path, ext));
    if exist('scene', 'var')
        if scene > length(files_from_a_folder)
            error("Wrong scene number: %i/%i, scene", scene ,length(files_from_a_folder))
        else
            start_scene = scene;
            num_scene = scene;
        end
    else 
        start_scene = 1;
        num_scene = length(files_from_a_folder);
    end
    
    
    for scene = start_scene:num_scene
        selected_file = convertCharsToStrings(path) + convertCharsToStrings(files_from_a_folder(scene).name);
        RawData = getData(selected_file);
        BagData(scene).meta = files_from_a_folder(scene);
        BagData(scene).bagfile = convertCharsToStrings(files_from_a_folder(scene).name);
        BagData(scene).array = [];
        total_points = 0;
        if exist('pair_num', 'var')
            start_scan = pair_num;
            num_scan = pair_num;
        else
            start_scan = 1;
            num_scan = size(RawData, 1);
        end

        
        % prepare for parfor loop
        scans(num_scan).num_tag = [];
        scans(num_scan).image = [];
        scans(num_scan).lidar_target = [];
        scans(num_scan).camera_target = [];
        
        parfor scan = start_scan : num_scan

            scans(scan).num_tag.original = length(RawData{scan}.Detections);
            scans(scan).num_tag.ransac_normal = scans(scan).num_tag.original;
            scans(scan).num_tag.L1_inspired = scans(scan).num_tag.original;
            if scans(scan).num_tag.original == 0
                continue
            end
            scans(scan).image = getImagefromStruct(RawData{scan}.Detections(1).Image); 

            for i =1:scans(scan).num_tag.original
                [scans(scan).lidar_target(i).payload_points,...
                 scans(scan).lidar_target(i).XYZIR_points ] = getPointsfromStruct(RawData{scan}.Detections(i).LidartagDetection.Points); % [x;y;z;1]
                
                total_points = total_points + size(scans(scan).lidar_target(i).payload_points,2);
                scans(scan).lidar_target(i).tag_size = RawData{scan}.Detections(i).LidartagDetection.Size;

                camera_corners = [RawData{scan}.Detections(i).ApriltagDetection.OuterCorners.X
                                  RawData{scan}.Detections(i).ApriltagDetection.OuterCorners.Y];
                camera_corners = sortrows(camera_corners', 2)';
                scans(scan).camera_target(i).corners = refineCameraCorners(camera_corners, scans(scan).image.image, "not display", 1);
                scans(scan).camera_target(i).ransac_normal.corners = scans(scan).camera_target(i).corners; % baseline might not be able to find corners at some scan
            end
        end
        
        BagData(scene).scans = scans;
        BagData(scene).total_points = total_points;
        clear scans;
    end
end
