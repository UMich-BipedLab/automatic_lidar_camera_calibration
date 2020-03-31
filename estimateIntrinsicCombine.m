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


clear; clc;

% Single variable called point_cloud
% mat_file_path = '../repo/LiDARTag_data/velodyne_points-lab8-closer-big--2019-09-06-15-28.mat';

path = "/home/chenxif/Documents/me590/Calibration/IntrinsicCalibration/extracted_tags/";
mat_file_path = {path+'velodyne_points-Intrinsic-LargeTag--2019-11-21-22-04.mat',...
                 path+'velodyne_points-Intrinsic-SmallTag--2019-11-21-22-00.mat',...
                 path+'velodyne_points-Intrinsic4-SmallTag--2019-11-22-22-54.mat',...
                 path+'velodyne_points-Intrinsic5-LargeTag--2019-11-22-23-02.mat',...
                 path+'velodyne_points-Intrinsic5-SmallTag--2019-11-22-23-00.mat',...
                 path+'velodyne_points-Intrinsic-further-LargeTag--2019-11-22-23-05.mat',...
                 path+'velodyne_points-Intrinsic-further-SmallTag--2019-11-22-23-09.mat',...
                 path+'velodyne_points-Intrinsic-further2-LargeTag--2019-11-22-23-15.mat',...
                 path+'velodyne_points-Intrinsic-further2-SmallTag--2019-11-22-23-17.mat',...
                 path+'velodyne_points-upper1-SmallTag--2019-12-05-20-13.mat',...
                 path+'velodyne_points-upper2-SmallTag--2019-12-05-20-16.mat',...
                 path+'velodyne_points-upper3-SmallTag--2019-12-05-20-19.mat',...
                 path+'velodyne_points-upper4-SmallTag--2019-12-05-20-22.mat',...
                 path+'velodyne_points-upper5-SmallTag--2019-12-05-20-23.mat',...
                 path+'velodyne_points-upper6-SmallTag--2019-12-05-20-26.mat',...
                 path+'velodyne_points-upper7-SmallTag--2019-12-05-20-29.mat',...
                 path+'velodyne_points-upper8-SmallTag--2019-12-05-20-29.mat'};
% mat_file_path = {'/home/brucebot/workspace/griztag/src/matlab/matlab/LiDARTag_data/velodyne_points-wavefield3-big--2019-09-07-19-04.mat'};
num_beams = 32;
num_scans = 1;
num_targets = length(mat_file_path);
num_iters = 10; % user defined iterations

pc = cell(1,num_targets);
for t = 1:num_targets
 pc{t} = loadPointCloud(mat_file_path{t});
end


for i = 1: num_scans
    data = cell(1,num_targets);% XYZIR 
    for t = 1:num_targets
        data{t} = getPayload(pc{t}, i , 1);
    end
end
%%
opt_formulation = ["Lie","Spherical"]; % Lie or Spherical

method = 1;

if (opt_formulation(method) == "Lie")
    data_split_with_ring = cell(1,num_targets);
    for t = 1:num_targets
        data_split_with_ring{t} = splitPointsBasedOnRing(data{t}, num_beams);
    end 
    
    distance = cell(1, num_iters);
    for k = 1: num_iters
        [delta, plane] = estimateIntrinsicLie(num_beams, num_targets, num_scans, data_split_with_ring);
        data_split_with_ring = updateDataRaw(num_beams, num_targets, data_split_with_ring, delta, opt_formulation(method));
        distance{k} = point2PlaneDistance(data_split_with_ring, plane, num_beams, num_targets); 
    end
    
    disp('done')
    plotSanityCheckLie(num_targets, plane, data, data_split_with_ring);
    
elseif (opt_formulation(method) == "Spherical")
    % preprocess the data
    spherical_data = cell(1,num_targets);
    data_split_with_ring = cell(1,num_targets);
    data_split_with_ring_raw = cell(1,num_targets);
    for t = 1:num_targets
        spherical_data{t} = Cartesian2Spherical(data{t});
        data_split_with_ring{t} = splitPointsBasedOnRing(spherical_data{t}, num_beams);
        data_split_with_ring_raw{t} = splitPointsBasedOnRing(data{t}, num_beams);
    end
    
    % iteratively optimize the intrinsic parameters
    distance = cell(1, num_iters); 
    for k = 1: num_iters
        [delta, plane] = estimateIntrinsicSpherical(num_beams, num_targets, num_scans, data_split_with_ring, data_split_with_ring_raw);
        % update the corrected points
        data_split_with_ring = updateDataSpherical(num_beams, num_targets, data_split_with_ring, delta);
        data_split_with_ring_raw = updateDataRaw(num_beams, num_targets, data_split_with_ring, delta, opt_formulation(method));
        distance{k} = point2PlaneDistance(data_split_with_ring_raw, plane, num_beams, num_targets); 
    end
    disp('done')
    plotSanityCheckSpherical(num_targets, plane, data_split_with_ring_raw, data);   
end
