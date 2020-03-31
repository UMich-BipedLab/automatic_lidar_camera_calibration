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


clc, clear
use_big_tag = 1;

if use_big_tag == 1
    load big_diamond2.mat
    m = atan(ring.line_model(1));
    target_size = 0.8051;
else
    load small_diamond2.mat
    m = atan(ring.line_model(1));
    target_size = 0.158;
end




[cluster, center] = t_distance_KMeans(4, target_size, m, all_points');
edge1 = all_points(cluster==1, :)';
edge2 = all_points(cluster==2, :)';
edge3 = all_points(cluster==3, :)';
edge4 = all_points(cluster==4, :)';


% ploting
 

function [cluster, line] = t_distance_KMeans(k, target_size, delta_theta, x)

%kMeans Clusters data points into k clusters.
%   Input args: k: number of clusters; 
%   points: m-by-n matrix of n m-dimensional data points.
%   Output args: cluster: 1-by-n array with values of 0,...,k-1
%   representing in which cluster the corresponding point lies in
%   centr: m-by-k matrix of the m-dimensional centroids of the k clusters


numP = size(x,2); % number of points
dimP = size(x,1); % dimension of points
centroid = mean(x, 2);

%% initialize 4 lines
bias = pi/4;
initial_theta = 0;
diamond_points = [-target_size*cos(initial_theta + bias), 0;
                   0, -target_size*sin(initial_theta + bias); 
                   target_size*cos(initial_theta + bias), 0
                   0,  target_size*sin(initial_theta + bias)]' + centroid;
               
current_theta = initial_theta - wrapTo2Pi(delta_theta);
R = [cos(current_theta) -sin(current_theta); sin(current_theta) cos(current_theta)];
rotated_diamond_points = R * diamond_points;

% plotting
fig_hangle = figure(8000);
clf(fig_hangle)
x_plotting = linspace(-target_size, target_size);
scatter(diamond_points(1,:), diamond_points(2,:), 'ob')
hold on
scatter(x(1,:), x(2,:), '.k')
axis equal
xlabel('x')
ylabel('y')
scatter(rotated_diamond_points(1,:), rotated_diamond_points(2,:), 'or')


% line 1
line(1).x = [rotated_diamond_points(1, 1), rotated_diamond_points(1, 4)];
line(1).y = [rotated_diamond_points(2, 1), rotated_diamond_points(2, 4)];
line(1).coefficients = polyfit(line(1).x, line(1).y, 1);
y1 = polyval(line(1).coefficients , line(1).x);
plot(line(1).x,y1, '-r')

% line 2 
line(2).x = [rotated_diamond_points(1, 1), rotated_diamond_points(1, 2)];
line(2).y = [rotated_diamond_points(2, 1), rotated_diamond_points(2, 2)];
line(2).coefficients = polyfit(line(2).x, line(2).y, 1);
y2 = polyval(line(2).coefficients , line(2).x);
plot(line(2).x, y2, '-g')

% line 3
line(3).x = [rotated_diamond_points(1, 2), rotated_diamond_points(1, 3)];
line(3).y = [rotated_diamond_points(2, 2), rotated_diamond_points(2, 3)];
line(3).coefficients = polyfit(line(3).x, line(3).y, 1);
y3 = polyval(line(3).coefficients , line(3).x);
plot(line(3).x, y3, '-b')

% line 4
line(4).x = [rotated_diamond_points(1, 3), rotated_diamond_points(1, 4)];
line(4).y = [rotated_diamond_points(2, 3), rotated_diamond_points(2, 4)];
line(4).coefficients = polyfit(line(4).x, line(4).y, 1);
y_temp = polyval(line(4).coefficients , line(4).x);
plot(line(4).x, y_temp, '-m')


%% Repeat until stopping criterion is met

% init cluster array
cluster = zeros(1, numP);
dist = zeros(numP, 4);

% init previous cluster array clusterPrev (for stopping criterion)
clusterPrev = cluster;

% for reference: count the iterations
iterations = 0;

% init stopping criterion
stop = false; % if stopping criterion met, it changes to true
threshold = 0.0001;
plot_lines_list = ["-r", "-g", "-b", "-m"];
plot_dots_list = [".r", ".g", ".b", ".m"];
plot_inliers_list = ["or", "og", "ob", "om"];
while stop == false
    
    % for each data point 
    for idxP = 1:numP
        % init distance array dist
%         idxP
        dist = zeros(1,k);
        point = x(:,idxP);
%         scatter(point(1), point(2), 'MarkerEdgeColor',[0 .5 .5], ...
%                'MarkerFaceColor',[0 .7 .7])
        
        % compute distance to each line
        for idxL=1:k
            v1 = [line(idxL).x(1), line(idxL).y(1)];
            v2 = [line(idxL).x(2), line(idxL).y(2)];
            dist(idxL) = pointToLineDistance(x(:,idxP)', v1, v2);
        end
        % find index of closest centroid (= find the cluster)
        [~, clusterP] = min(dist);
        cluster(idxP) = clusterP;
        scatter(point(1), point(2), plot_dots_list(clusterP));
        hold on
    end
    
    % Recompute centroids using current cluster memberships:
    % for every cluster compute a new line
    for idxL = 1:k
        current_points = x(:, cluster==idxL)';
%         threshold = sqrt(norm(std(current_points))^2);
        threshold = (norm(std(current_points))^2)/10;
        % find the points in cluster number idxC and compute row-wise mean
        [line(idxL).x, line(idxL).y, line(idxL).coefficients, line(idxL).inliers] = ransacLineWithInlier(current_points, threshold, 0.3);
        y_temp = polyval(line(idxL).coefficients , line(idxL).x);
        plot(line(idxL).x, y_temp, plot_lines_list(idxL));
        scatter(line(idxL).inliers(:,1), line(idxL).inliers(:,2), 50, plot_inliers_list(idxL));
    end
    
    % Checking for stopping criterion: Clusters do not chnage anymore
    if clusterPrev==cluster
        stop = true;
    end
    % update previous cluster clusterPrev
    clusterPrev = cluster;
    
    iterations = iterations + 1
%     disp('pausing...')
%     pause
    fig_hangle = figure(8000);
    cla(fig_hangle)
    drawnow
    if iterations > 100
        return;
    end
end


% for reference: print number of iterations
fprintf('kMeans.m used %d iterations of changing centroids.
',iterations);
end
