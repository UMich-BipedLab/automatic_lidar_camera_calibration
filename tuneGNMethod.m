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

% clc, clear, close all
clc, clear
path = "../../LiDARTag_data/";
use_bag = 2;
use_big_tag = 1;


if use_bag == 1
    if use_big_tag==1
        pc_mat = 'velodyne_points-lab3-closer-big--2019-09-06-08-38.mat';
        target_size = 0.8051;
    else 
        pc_mat = 'velodyne_points-lab3-closer-small--2019-09-06-08-35.mat';
        target_size = 0.158;
    end
elseif use_bag == 2
    if use_big_tag==1
        pc_mat = 'velodyne_points-lab4-closer-big--2019-09-06-13-49.mat';
        target_size = 0.8051;
    else 
        pc_mat = 'velodyne_points-lab4-closer-small--2019-09-06-13-38.mat';
        target_size = 0.158;
    end
elseif use_bag == 3
    if use_big_tag==1
        pc_mat = 'velodyne_points-lab5-closer-bag--2019-09-06-14-27.mat';
        target_size = 0.8051;
    else 
        pc_mat = 'velodyne_points-lab5-closer-small--2019-09-06-14-23.mat';
        target_size = 0.158;
    end
end

pc = load(string(path) + string(pc_mat)); 
pnts = pc.point_cloud; % [scan, point, [X, Y, X, I, R]]
ransac_threshold = 0.02;

d=target_size*sqrt(2);% Large Target size
% img_fig_handles = createFigHandleWithNumber(3, 1, "vis");

base_line.optimized_method = 2;
base_line.edge_method = 3;
base_line.more_tags = 1;
base_line.show_results = 1;
base_line.L1_cleanup = 0;
base_line.img_hangles = createFigHandleWithNumber(4, 4, "base_line_vis vis"); %% don't change

if base_line.edge_method == 1
    [U,center,~,~,~,~,LEupper,LElower,REupper,RElower,~,~,~, flag_changed] = LeftRightEdges_v02(base_line, pnts, d);
elseif base_line.edge_method == 2
    [U, center, LEupper, LElower, REupper, RElower, ~, ~, flag_changed] = clickedToFindEdges(base_line, pnts, d);
elseif base_line.edge_method == 3
    [U, center, LEupper, LElower, REupper, RElower, PayLoadClean, ~, flag_changed] = L1CostToFindEdges(base_line, pnts, d);
end

% %% Plot out the Left and Right Edges of the TARGET
% [nL1,nL2,nL3]=size(LE);
% LEall=reshape(LE,nL1,nL2*nL3);
% I=find( (LEall(1,:)~= 10) & (LEall(2,:)~= 10) );
% LeftTargetEdges=U*[1 0; 0 1;0 0]*LEall(:,I) + center;
% [nR1,nR2,nR3]=size(RE);
% REall=reshape(RE,nR1,nR2*nR3);
% I=find( (REall(1,:)~= 10) & (REall(2,:)~= 10) );
% RightTargetEdges=U*[1 0; 0 1; 0 0]*REall(:,I) + center;
% 
% figure(100)
% scatter3(LeftTargetEdges(1,:), LeftTargetEdges(2,:), LeftTargetEdges(3,:),'.b'), hold on, grid on, axis equal
% scatter3(RightTargetEdges(1,:), RightTargetEdges(2,:), RightTargetEdges(3,:),'.r'), hold off
% 
% disp(['Ring numbers used in figure 1 are:']), disp(RingNumbers)
% 
% 
% LeftTargetEdgesAveraged=U*[1 0; 0 1; 0 0]*LEavg + center;
% RightTargetEdgesAveraged=U*[1 0; 0 1; 0 0]*REavg + center;
% 
% figure(101)
% scatter3(LeftTargetEdgesAveraged(1,:), LeftTargetEdgesAveraged(2,:), LeftTargetEdgesAveraged(3,:),'.b'), hold on, grid on, axis equal
% scatter3(RightTargetEdgesAveraged(1,:), RightTargetEdgesAveraged(2,:), RightTargetEdgesAveraged(3,:),'.r'), hold off

%% Plotting the Edge Points in the LiDAR Frame
[nL1,nL2,nL3]=size(LEupper);
LEupperall=reshape(LEupper,nL1,nL2*nL3);
I=find( (LEupperall(1,:)~= 10) & (LEupperall(2,:)~= 10) );
LEupperall=LEupperall(:,I);
LEupperTargetEdges=U*[1 0; 0 1;0 0]*LEupperall + center;

[nL1,nL2,nL3]=size(LElower);
LElowerall=reshape(LElower,nL1,nL2*nL3);
I=find( (LElowerall(1,:)~= 10) & (LElowerall(2,:)~= 10) );
LElowerall=LElowerall(:,I);
LElowerTargetEdges=U*[1 0; 0 1;0 0]*LElowerall + center;

[nR1,nR2,nR3]=size(REupper);
REupperall=reshape(REupper,nR1,nR2*nR3);
I=find( (REupperall(1,:)~= 10) & (REupperall(2,:)~= 10) );
REupperall=REupperall(:,I);
REupperTargetEdges=U*[1 0; 0 1; 0 0]*REupperall + center;

[nR1,nR2,nR3]=size(RElower);
RElowerall=reshape(RElower,nR1,nR2*nR3);
I=find( (RElowerall(1,:)~= 10) & (RElowerall(2,:)~= 10) );
RElowerall=RElowerall(:,I);
RElowerTargetEdges=U*[1 0; 0 1; 0 0]*RElowerall + center;


%% Fitting a square to the edges

figure(104)
scatter3(LEupperTargetEdges(1,:), LEupperTargetEdges(2,:), LEupperTargetEdges(3,:),'.r'), hold on,
scatter3(RElowerTargetEdges(1,:), RElowerTargetEdges(2,:), RElowerTargetEdges(3,:),'.k'), hold on,
scatter3(LElowerTargetEdges(1,:), LElowerTargetEdges(2,:), LElowerTargetEdges(3,:),'.b'), hold on,
scatter3(REupperTargetEdges(1,:), REupperTargetEdges(2,:), REupperTargetEdges(3,:),'.g'), hold on, grid on, axis equal

[~,nLU]=size(LEupperall);[~,nRL]=size(RElowerall);[~,nLL]=size(LElowerall);[~,nRU]=size(REupperall);

PhiEdges=[LEupperall(1,:)', zeros(nLU,1), zeros(nLU,1), zeros(nLU,1), ones(nLU,1), zeros(nLU,1), zeros(nLU,1), zeros(nLU,1);
    zeros(nRU,1), REupperall(1,:)', zeros(nRU,1), zeros(nRU,1), zeros(nRU,1), ones(nRU,1), zeros(nRU,1), zeros(nRU,1);...
    zeros(nRL,1), zeros(nRL,1), RElowerall(1,:)', zeros(nRL,1), zeros(nRL,1), zeros(nRL,1) ones(nRL,1), zeros(nRL,1); ...
    zeros(nLL,1), zeros(nLL,1), zeros(nLL,1), LElowerall(1,:)', zeros(nLL,1), zeros(nLL,1), zeros(nLL,1), ones(nLL,1)];

YEdges=[LEupperall(2,:), REupperall(2,:), RElowerall(2,:), LElowerall(2,:)]';


%% %SymbolicMathDiamond.m
% 
Beta=PhiEdges\YEdges;
mu1=Beta(1);mu2=-1/Beta(1);
mu3=mu1;mu4=mu2;
b1=Beta(5);b2=Beta(6);b3=Beta(7);b4=Beta(8);
% TargetGeometry='DoSquare';
TargetGeometry='DoRectangle';
for k = 1:100    
    %Set up for rectangle; then add more for a square.
    %First line is m1*m2=-1
    %Next two are opposite sides are parallel.
    Aeq=[mu2 mu1 0 0 zeros(1,4); ...
        1    0 -1 0 zeros(1,4); ...
        0    1  0 -1 zeros(1,4)];
    beq=[-1+mu1*mu2; 0; 0];
    switch TargetGeometry
        case 'DoSquare'
            L1sq =((mu2^2 + 1)*(b1 - b3)^2)/(mu1 - mu2)^2; %Length of side 1
            L2sq =((mu1^2 + 1)*(b2 - b4)^2)/(mu1 - mu2)^2; %Length of side 2
            %
            jac_L1sq =[ -(2*(mu2^2 + 1)*(b1 - b3)^2)/(mu1 - mu2)^3, ...
                (2*(b1 - b3)^2*(mu1*mu2 + 1))/(mu1 - mu2)^3, ...
                -(2*(mu2^2 + 1)*(b1 - b3)^2)/(mu1 - mu2)^3, (2*(b1 - b3)^2*(mu1*mu2 + 1))/(mu1 - mu2)^3,...
                ((mu2^2 + 1)*(2*b1 - 2*b3))/(mu1 - mu2)^2, 0, -((mu2^2 + 1)*(2*b1 - 2*b3))/(mu1 - mu2)^2, 0];
            jac_L2sq =[ -(2*(b2 - b4)^2*(mu1*mu2 + 1))/(mu1 - mu2)^3, (2*(mu1^2 + 1)*(b2 - b4)^2)/(mu1 - mu2)^3,...
                -(2*(b2 - b4)^2*(mu1*mu2 + 1))/(mu1 - mu2)^3, (2*(mu1^2 + 1)*(b2 - b4)^2)/(mu1 - mu2)^3,...
                0, ((mu1^2 + 1)*(2*b2 - 2*b4))/(mu1 - mu2)^2, 0, -((mu1^2 + 1)*(2*b2 - 2*b4))/(mu1 - mu2)^2];
            %
            Aeq=[Aeq;jac_L1sq;jac_L2sq];
            x=[mu1 mu2 mu3 mu4 b1 b2 b3 b4]';
            beq=[beq;d^2-L1sq+jac_L1sq*x;d^2-L2sq+jac_L2sq*x];
        otherwise
    end
    
    if 1 % L2 <----% Does well in all cases
        if 0
            Beta=PhiEdges\YEdges;
            text='L2 no constraints';
        else
            Q=PhiEdges'*PhiEdges;
            f=-(YEdges')*PhiEdges;
            Beta = quadprog(Q,f,[],[],Aeq,beq);
            text='L2 with constraints';
        end
    elseif 0% L1 %Slow, but works fine.
        [nr,nc]=size(PhiEdges);
        [ne,~]=size(Aeq);
        f=[zeros(1,nc), ones(1,nr)];
        Ain=[PhiEdges, -eye(nr); -PhiEdges -eye(nr)];
        bin=[YEdges;-YEdges];
        Aeq=[Aeq,zeros(ne,nr)];
        Beta=linprog(f,Ain,bin,Aeq,beq);
        Beta=Beta(1:nc);
        text='L1 with constraints';
    else % L-inf  
        [nr,nc]=size(PhiEdges);
        f=[zeros(1,nc), 1];
        Ain=[PhiEdges, -ones(nr,1); -PhiEdges -ones(nr,1)];
        bin=[YEdges;-YEdges];
        Aeq=[Aeq,0*beq];
        Beta=linprog(f,Ain,bin,Aeq,beq);
        Beta=Beta(1:nc);
        text='L-inf with constraints';
    end
    
    mu1=Beta(1); mu2=Beta(2); mu3=Beta(3); mu4=Beta(4); b1=Beta(5);b2=Beta(6);b3=Beta(7);b4=Beta(8);
    
    V1=[mu1 -1; mu2 -1]\[b1;b2];
    V2=[mu2 -1; mu3 -1]\[b2;b3];
    V3=[mu3 -1; mu4 -1]\[b3;b4];
    V4=[mu4 -1; mu1 -1]\[b4;b1];
    
    e1=norm(V1-V2)-d;
    e2=norm(V2-V3)-d;
    e3=norm(V3-V4)-d;
    e4=norm(V4-V1)-d;
    
    switch TargetGeometry
        case   'DoSquare'
            if and(abs(1+mu1*mu2)<1e-4, max(abs([e1 e4]))< 1e-5)
                break
            end
        case 'DoRectangle'
            if abs(1+mu1*mu2)<1e-5,
                break
            end
    end
end
disp(['Number of interations =',num2str(k)])
mLU=mu1;mRU=mu2;mRL=mu3;mLL=mu4;
bLU=b1;bRU=b2;bRL=b3;bLL=b4;

disp(text)
disp(['mu1*mu2 = ']), disp(mu1*mu2)
Error_Edges=PhiEdges*Beta-YEdges;
RMSerror_Edges=sqrt( ( norm(Error_Edges)^2 ) / length(YEdges) )



%Let's plot the lines of the square target or the edge points as a sanity
%check

edges.LU= U*[1 0; 0 1;0 0]*([1 0;mLU 0] *LEupperall +  [0;bLU] ) + center;
edges.RU= U*[1 0; 0 1;0 0]*([1 0;mRU 0] *REupperall +  [0;bRU] ) + center;
edges.RL= U*[1 0; 0 1;0 0]*([1 0;mRL 0] *RElowerall +  [0;bRL] ) + center;
edges.LL= U*[1 0; 0 1;0 0]*([1 0;mLL 0] *LElowerall +  [0;bLL] ) + center;

figure(99), hold on
scatter3(edges.LU(1,:), edges.LU(2,:), edges.LU(3,:),'xr'), hold on
scatter3(edges.RL(1,:), edges.RL(2,:), edges.RL(3,:),'xg'), hold on,
scatter3(edges.LL(1,:), edges.LL(2,:), edges.LL(3,:),'xb'), hold on,
scatter3(edges.RU(1,:), edges.RU(2,:), edges.RU(3,:),'xm'), grid on, axis equal

view(90,0)

% Vertices

% V1=[mu1 -1; mu2 -1]\[b1;b2];6
% V2=[mu2 -1; mu3 -1]\[b2;b3];
% V3=[mu3 -1; mu4 -1]\[b3;b4];
% V4=[mu4 -1; mu1 -1]\[b4;b1];

LengthSides=[norm(V1-V2), norm(V2-V3), norm(V3-V4), norm(V4-V1)]
% Vertices
V1_3d = U*[1 0; 0 1;0 0]*([1 0;mLU 0] * V2 +  [0;bLU] ) + center;
V2_3d = U*[1 0; 0 1;0 0]*([1 0;mRU 0] * V3 +  [0;bRU] ) + center;
V3_3d = U*[1 0; 0 1;0 0]*([1 0;mRL 0] * V4 +  [0;bRL] ) + center;
V4_3d = U*[1 0; 0 1;0 0]*([1 0;mLL 0] * V1 +  [0;bLL] ) + center;

cross_big_3d = [V1_3d, V2_3d, V3_3d, V4_3d; 
                ones(1, 4)];
cross_big_3d = sortrows(cross_big_3d', 3, 'descend')';
scatter3(cross_big_3d(1,:), cross_big_3d(2,:), cross_big_3d(3,:), 50,'or')
scatter3(PayLoadClean(1,:), PayLoadClean(2,:), PayLoadClean(3,:), '.k')
title("vertices estimated")
view(90,0), hold off

%%
% [x_TL, y_TL, modelInliers_TL] = ransacLine(LEupperal_new(1:2, :)', ransac_threshold);
% [x_BL, y_BL, modelInliers_BL] = ransacLine(LElowerall_new(1:2, :)', ransac_threshold);
% [x_TR, y_TR, modelInliers_TR] = ransacLine(REupperall_new(1:2, :)', ransac_threshold);
% [x_BR, y_BR, modelInliers_BR] = ransacLine(RElowerall_new(1:2, :)', ransac_threshold);
% 
% cross_L=intersection(modelInliers_TL, modelInliers_BL);
% cross_R=intersection(modelInliers_TR, modelInliers_BR);
% cross_T=intersection(modelInliers_TL, modelInliers_TR);
% cross_B=intersection(modelInliers_BR, modelInliers_BL);
% 
% cross_big_2d = [cross_L, cross_R, cross_T, cross_B];
% 
% %%
% cross_big_3d = U*[1 0; 0 1;0 0]*cross_big_2d + center;
% cross_big_3d = [cross_big_3d; ones(1,size(cross_big_3d,2))];
% %
% if base_line.show_results
% % if 1
% %     current_img_handle = base_line.img_hangles(3);
% %     hold(current_img_handle, 'on')
% %     grid(current_img_handle,  'on')
% %     axis(current_img_handle, 'equal')
% %     scatter(current_img_handle, RElowerall_new(1,:), RElowerall_new(2,:), '.m'),
% %     plot(current_img_handle, x_BL, y_BL, 'r-')
% %     plot(current_img_handle, x_TL, y_TL, 'b-')
% %     plot(current_img_handle, x_TR, y_TR, 'g-')
% %     plot(current_img_handle, x_BR, y_BR, 'k-')
% %     plot(current_img_handle, [cross_L(1) cross_B(1)], [cross_L(2) cross_B(2)], 'r-')
% %     plot(current_img_handle, [cross_L(1) cross_T(1)], [cross_L(2) cross_T(2)], 'b-')
% %     plot(current_img_handle, [cross_R(1) cross_T(1)], [cross_R(2) cross_T(2)], 'g-')
% %     plot(current_img_handle, [cross_R(1) cross_B(1)], [cross_R(2) cross_B(2)], 'm-')
% %     scatter(current_img_handle, cross_big_2d(1,:), cross_big_2d(2,:), 'd')
%     
%     
%     current_img_handle = base_line.img_hangles(4);
%     scatter(current_img_handle, LEupperal_new(1,:), LEupperal_new(2,:),'.r')
%     hold(current_img_handle, 'on')
%     scatter(current_img_handle, LElowerall_new(1,:), LElowerall_new(2,:), '.b')
%     scatter(current_img_handle, REupperall_new(1,:), REupperall_new(2,:), '.g')
%     grid(current_img_handle,  'on')
%     axis(current_img_handle, 'equal')
%     scatter(current_img_handle, RElowerall_new(1,:), RElowerall_new(2,:), '.m'),
%     plot(current_img_handle, x_BL, y_BL, 'r-')
%     plot(current_img_handle, x_TL, y_TL, 'b-')
%     plot(current_img_handle, x_TR, y_TR, 'g-')
%     plot(current_img_handle, x_BR, y_BR, 'k-')
%     plot(current_img_handle, [cross_L(1) cross_B(1)], [cross_L(2) cross_B(2)], 'r-')
%     plot(current_img_handle, [cross_L(1) cross_T(1)], [cross_L(2) cross_T(2)], 'b-')
%     plot(current_img_handle, [cross_R(1) cross_T(1)], [cross_R(2) cross_T(2)], 'g-')
%     plot(current_img_handle, [cross_R(1) cross_B(1)], [cross_R(2) cross_B(2)], 'm-')
%     scatter(current_img_handle, cross_big_2d(1,:), cross_big_2d(2,:), 'd')
%     title(current_img_handle, "regressed edges")
%     set(get(current_img_handle, 'parent'),'visible','on');
%     
%     
%     current_img_handle = base_line.img_hangles(2);
%     hold(current_img_handle, 'on')
%     scatter3(current_img_handle, cross_big_3d(1,1), cross_big_3d(2,1), cross_big_3d(3,1), 'or')
%     scatter3(current_img_handle, cross_big_3d(1,2), cross_big_3d(2,2), cross_big_3d(3,2), 'og')
%     scatter3(current_img_handle, cross_big_3d(1,3), cross_big_3d(2,3), cross_big_3d(3,3), 'ob')
%     scatter3(current_img_handle, cross_big_3d(1,4), cross_big_3d(2,4), cross_big_3d(3,4), 'om')
%     
%     scatter3(current_img_handle, edges.LU(1,:), edges.LU(2,:), edges.LU(3,:), 'sr')
%     scatter3(current_img_handle, edges.LL(1,:), edges.LL(2,:), edges.LL(3,:), 'sg')
%     scatter3(current_img_handle, edges.RU(1,:), edges.RU(2,:), edges.RU(3,:), 'sb')
%     scatter3(current_img_handle, edges.RL(1,:), edges.RL(2,:), edges.RL(3,:), 'sm')
% end
disp('done')