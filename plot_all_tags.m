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


% figure(888)
% clf(888)
% data1 = rand(1,20)./2;      %# Sample data set 1
% data2 = 0.3+rand(1,20)./2;  %# Sample data set 2
% hAxes = axes('NextPlot','add',...           %# Add subsequent plots to the axes,
%              'DataAspectRatio',[1 1 1],...  %#   match the scaling of each axis,
%              'XLim',[0 10],...               %#   set the x axis limit,
%              'YLim',[0 eps],...             %#   set the y axis limit (tiny!),
%              'Color','none');               %#   and don't use a background color
% plot(data1,0,'r*','MarkerSize',10);  %# Plot data set 1
% plot(data2,0,'b.','MarkerSize',10);  %# Plot data set 2
%% 
clear, clc, close all
path = "../../LiDARTag_data/";
[BagData, TestData] = getBagData();
 % target placements
bagfile_names = ["lab3-closer-cleaner.bag", ... 
                 "lab4-closer-cleaner.bag", ...
                 "lab5-closer-cleaner.bag", ...
                 "lab7-closer-cleaner.bag", ...
                 "lab8-closer-cleaner.bag", ...
                 "wavefield3-tag.bag", ...
                 "wavefield5-tag.bag"];
pc_iter = 1;
num_scan = 5;
flipped = -1;
for index = 1:length(bagfile_names)             
    current_bag = find(strcmp(bagfile_names(index), [BagData(:).bagfile]));
    for tag = 1:BagData(current_bag).num_tag 
        pc = loadPointCloud(path, BagData(current_bag).lidar_target(tag).pc_file);
        X = getPayload(pc, pc_iter, num_scan);
        hAxes = axes('NextPlot','add',...           % Add subsequent plots to the axes,
                     'DataAspectRatio',[1 1 1],...  % mtch the scaling of each axis,
                     'XLim',[0 10],...              % set the x axis limit,
                     'YLim',[0 eps],...             % set the y axis limit (tiny!),
                     'Color','none');               % and don't use a background color
                 hold on
        % title("target placement")
        distance_to_lidar = norm(mean(X'));
        
        d = BagData(current_bag).lidar_target(tag).tag_size;
        if d < 0.5 % small tag
            plot(distance_to_lidar, 0,'r*','MarkerSize',10);  %# Plot data set 1
            text(distance_to_lidar, flipped*100*eps, "t_{" + num2str(current_bag) + "}")
%             text(distance_to_lidar, flipped*100*eps, "t_" + num2str(index) + "(" + num2str(current_bag) + ")")
        else
            % large tag
            plot(distance_to_lidar, 0,'b.','MarkerSize',10);  %# Plot data set 2
            text(distance_to_lidar, flipped*100*eps, "T_{" + num2str(current_bag) + "}")
        end
    end
    flipped = flipped * -1;
end
xlabel('Target distance to lidar [m]')
