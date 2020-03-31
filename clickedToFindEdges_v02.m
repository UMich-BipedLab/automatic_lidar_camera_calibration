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
 * AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu) and Jessy Grizzle
 * WEBSITE: https://www.brucerobot.com/
%}

function [Uc, meanClean, LEupper, LElower, REupper, RElower, PayLoadClean, PayLoadClean2D] = clickedToFindEdges_v02(base_line, payload, d)

% pnts is the pioint cloud structure that Bruce builds up


%U as in [U,S,V] to determined the normal to the pointcloud. 
%U(:,3) is the normal to the point cloud
% center is the mean of the pointcould
% center + U*[0;y;z] puts a point back in teh coordinates of the pointcloud
%
%
%LE = ([y;z] corrdinates) x Rings x scans
%RE = ([y;z] corrdinates) x Rings x scans


%LEavg = (2-(y,z) corrdinates) x Rings because the values are averaged
%over all scans
%REavg = (2-(y,z) corrdinates) x Rings because the values are averaged
%over all scans

%Rings is the list of rings in LE and RE
%RingsAvg is the list of rings in LEavg REavg
%



%% Find Rings
FR=min(payload(5,:));
LR=max(payload(5,:));
RingNumbers=[];
for i=FR:LR
    K=find(payload(5,:)==i);
    if length(K)>0
        RingNumbers=[RingNumbers,i];
    end
end

if base_line.show_results
    current_img_handle = base_line.img_hangles(1);
    hold(current_img_handle, 'on');
    for i = 1:LR
        ring_points = payload(:, (payload(5, :)==i));
        if size(ring_points, 2) > 0         
            scatter3(current_img_handle, ring_points(1,:), ring_points(2,:), ring_points(3,:), '.'), 
            view(current_img_handle, -90,3)
            txt_x = mean(ring_points(1, :));
            txt_y = mean(ring_points(2, :));
            txt_z = mean(ring_points(3, :));
            text(current_img_handle, txt_x, txt_y, txt_z, num2str(i))
        end
    end
    set(get(current_img_handle, 'parent'),'visible','on');% show the current axes
    axis(current_img_handle, 'equal')
    xlabel(current_img_handle, 'x')
    ylabel(current_img_handle, 'y')
    zlabel(current_img_handle, 'z')
    title(current_img_handle, 'Original Data')
    hold(current_img_handle, 'off');
end


%% Clean Data 
meanData=mean(payload(1:3,:),2);
error=abs(payload(1:3,:)-meanData);
distance=sum(error,1);
K=find(distance < d*1.025);
PayLoadClean=payload(:, K);
meanClean=mean(PayLoadClean(1:3,:),2);

if base_line.L1_cleanup 
    opt.H_TL.rpy_init = [45 2 3];
    opt.H_TL.T_init = [2, 0, 0];
    opt.H_TL.H_init = eye(4);
    opt.H_TL.method = "Constraint Customize"; 
    opt.H_TL.UseCentroid = 1;
    [~, ~, clean_up_indices, ~] = cleanLiDARTargetWithOneDataSetWithIndices(PayLoadClean, d/sqrt(2), opt.H_TL);
    PayLoadClean=payload(:, clean_up_indices);
end

% Check for entire rings being removed
FirstRing=min(PayLoadClean(5,:));
LastRing=max(PayLoadClean(5,:));

RingNumbers=[FirstRing:1:LastRing];
NRings=length(RingNumbers);

if base_line.show_results
    current_img_handle = base_line.img_hangles(2);
    hold(current_img_handle, 'on');
    scatter3(current_img_handle, PayLoadClean(1,:), PayLoadClean(2,:), PayLoadClean(3,:), '.')
    view(current_img_handle, -90,3)
    axis(current_img_handle, 'equal')
    xlabel(current_img_handle, 'x')
    ylabel(current_img_handle, 'y')
    zlabel(current_img_handle, 'z')
    title(current_img_handle, 'Cleaned Up Data')
    hold(current_img_handle, 'off');
    set(get(current_img_handle, 'parent'),'visible','on');% show the current axes
end
%% Build a projection to a plane that will be used to find Edge Data
K=find( and(( PayLoadClean(6,:) > IndScans(1) ),( PayLoadClean(6,:) < IndScans(end))  ));

XYZ=PayLoadClean(1:3, K);
meanXYZ=mean(XYZ,2);
[Uc, Sc, Vc]=svd(XYZ-meanXYZ);
[Uc, Vc] = FixSignsRotation(Uc,Vc);

%Sc(:,1:3),Uc

if abs(Uc(2,1)) > abs(Uc(3,1))
    Ind2D=[1,2];
else
    Ind2D=[2,1];
end
% Ind2D
NScans=max(PayLoadClean(6,:))- min(PayLoadClean(6,:));

% Uc; is used for the projection;

%% Project to a plane, find ring lines and the edges of the target edges
Data=PayLoadClean(1:3,:);
temp=Uc'*(Data-mean(Data,2));
PayLoadClean2D=temp(Ind2D,:); %Project out the distance component

if base_line.show_results
    current_img_handle = base_line.img_hangles(3);
    hold(current_img_handle, 'on');
    scatter(current_img_handle, PayLoadClean2D(1,:), PayLoadClean2D(2,:), '.b')
    set(get(current_img_handle, 'parent'),'visible','on');% show the current axes
    view(current_img_handle, -180, 90)
    axis(current_img_handle, 'equal')
    xlabel(current_img_handle, 'x')
    ylabel(current_img_handle, 'y')
    title(current_img_handle, 'Projected 2D points')
    hold(current_img_handle, 'off');
end

disp("select polygons of interests")
disp("choosing LEupper")
[LU_in, LU_on] = getDataFromClickedPointsOnAImage(current_img_handle, 4, PayLoadClean2D, 'display');
LEupper = PayLoadClean2D(:, [LU_in, LU_on]);

disp("choosing LElower")
[LL_in, LL_on] = getDataFromClickedPointsOnAImage(current_img_handle, 4, PayLoadClean2D, 'display');
LElower = PayLoadClean2D(:, [LL_in, LL_on]);


disp("choosing REupper")
[RU_in, RU_on] = getDataFromClickedPointsOnAImage(current_img_handle, 4, PayLoadClean2D, 'display');
REupper = PayLoadClean2D(:, [RU_in, RU_on]);

disp("choosing RElower")
[RL_in, RL_on] = getDataFromClickedPointsOnAImage(current_img_handle, 4, PayLoadClean2D, 'display');
RElower = PayLoadClean2D(:, [RL_in, RL_on]);
end

function [U,V] = FixSignsRotation(U,V)
%Fix the signs
Temp=abs(U);
[junk,I]=max(Temp,[],1);
%[sign(U(I(1),1)),sign(U(I(2),2)),sign(U(I(3),3))]
Signs=diag([sign(U(I(1),1)),sign(U(I(2),2)),sign(U(I(3),3))]);
U=U*Signs;
V(:,1:3)=V(:,1:3)*Signs;
end


