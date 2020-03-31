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



%% Initialize stuff
clc, clear, close all
OffSet=0;

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
d=target_size*sqrt(2);

[n1,n2,n3]=size(pnts);
base_line.optimized_method = 2;
base_line.edge_method = 3;
base_line.more_tags = 1;
base_line.show_results = 1;
base_line.L1_cleanup = 0;
base_line.img_hangles = createFigHandleWithNumber(4, 4, "base_line_vis vis"); %% don't change
ExpNmbr=21;
[U,center,LE,RE,LEavg,REavg,LEupper,LElower,REupper,RElower,RingNumbers,NScans,PayLoadClean] = LeftRightEdges_v02(base_line, pnts, d, ExpNmbr);


%% Plot out the Left and Right Edges of the TARGET
[nL1,nL2,nL3]=size(LE);
LEall=reshape(LE,nL1,nL2*nL3);
I=find( (LEall(1,:)~= 10) & (LEall(2,:)~= 10) );
LeftTargetEdges=U*[1 0; 0 1;0 0]*LEall(:,I) + center;
[nR1,nR2,nR3]=size(RE);
REall=reshape(RE,nR1,nR2*nR3);
I=find( (REall(1,:)~= 10) & (REall(2,:)~= 10) );
RightTargetEdges=U*[1 0; 0 1; 0 0]*REall(:,I) + center;

figure(100)
scatter3(LeftTargetEdges(1,:), LeftTargetEdges(2,:), LeftTargetEdges(3,:),'.b'), hold on, grid on, axis equal
scatter3(RightTargetEdges(1,:), RightTargetEdges(2,:), RightTargetEdges(3,:),'.r'), hold off

disp(['Ring numbers used in figure 1 are:']), disp(RingNumbers)


LeftTargetEdgesAveraged=U*[1 0; 0 1; 0 0]*LEavg + center;
RightTargetEdgesAveraged=U*[1 0; 0 1; 0 0]*REavg + center;

figure(101)
scatter3(LeftTargetEdgesAveraged(1,:), LeftTargetEdgesAveraged(2,:), LeftTargetEdgesAveraged(3,:),'.b'), hold on, grid on, axis equal
scatter3(RightTargetEdgesAveraged(1,:), RightTargetEdgesAveraged(2,:), RightTargetEdgesAveraged(3,:),'.r'), hold off

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
%TargetGeometry='DoSquare';
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
    
    V1=-[mu1 -1; mu2 -1]\[b1;b2];
    V2=-[mu2 -1; mu3 -1]\[b2;b3];
    V3=-[mu3 -1; mu4 -1]\[b3;b4];
    V4=-[mu4 -1; mu1 -1]\[b4;b1];
    
    
    e1=norm(V1-V2)-d;
    e2=norm(V2-V3)-d;
    e3=norm(V3-V4)-d;
    e4=norm(V4-V1)-d;
    
    delta=1e-6;
    switch TargetGeometry
        case   'DoSquare'
            if and(abs(1+mu1*mu2)<delta, max(abs([e1 e4]))< delta)
                break
            end
        case 'DoRectangle'
            if abs(1+mu1*mu2)<delta,
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

V=[V1,V2,V3,V4]

%%
m=length(Beta);
n=length(YEdges);
sampleSize = floor(max(10*m,n/20)) % minimum number of points to sample per trial
maxDistance = d/15; % max allowable distance for inliers
points=[PhiEdges,YEdges];
model=Beta;

fitRectangleFcn = @(points) iQP(points,model); % personal fitFunction
evalRectangleFcn = ...   % distance evaluation function
  @(model, points) abs( points(:,1:m)*model-points(:,end) );

[modelRANSAC, inlierIdx] = ransac(points,fitRectangleFcn,evalRectangleFcn, ...
  sampleSize,maxDistance);

Beta=modelRANSAC
 mu1=Beta(1); mu2=Beta(2); mu3=Beta(3); mu4=Beta(4); b1=Beta(5);b2=Beta(6);b3=Beta(7);b4=Beta(8);
    
    V1=-[mu1 -1; mu2 -1]\[b1;b2];
    V2=-[mu2 -1; mu3 -1]\[b2;b3];
    V3=-[mu3 -1; mu4 -1]\[b3;b4];
    V4=-[mu4 -1; mu1 -1]\[b4;b1];
    
    Vransac=[V1,V2,V3,V4]

%%

%Let's plot the lines of the square target or the edge points as a sanity
%check

LEupperTargetLines= U*[1 0; 0 1;0 0]*([1 0;mLU 0] *LEupperall +  [0;bLU] ) + center;
REupperTargetLines= U*[1 0; 0 1;0 0]*([1 0;mRU 0] *REupperall +  [0;bRU] ) + center;
RElowerTargetLines= U*[1 0; 0 1;0 0]*([1 0;mRL 0] *RElowerall +  [0;bRL] ) + center;
LElowerTargetLines= U*[1 0; 0 1;0 0]*([1 0;mLL 0] *LElowerall +  [0;bLL] ) + center;

figure(1), hold on
scatter3(LEupperTargetLines(1,:), LEupperTargetLines(2,:),LEupperTargetLines(3,:),'*r'), hold on
scatter3(RElowerTargetLines(1,:), RElowerTargetLines(2,:),RElowerTargetLines(3,:),'*k'), hold on,
scatter3(LElowerTargetLines(1,:), LElowerTargetLines(2,:),LElowerTargetLines(3,:),'*b'), hold on,
scatter3(REupperTargetLines(1,:), REupperTargetLines(2,:),REupperTargetLines(3,:),'*g'), grid on, axis square, hold off

view(90,0), hold off

figure(105)
scatter3(PayLoadClean(1,:), PayLoadClean(2,:), PayLoadClean(3,:), '.'),  hold on, view(-90,3)
title('Cleaned Up Data Plus Corners')
V13D= U*[1 0; 0 1;0 0]*V1+ center;
V23D= U*[1 0; 0 1;0 0]*V2 + center;
V33D= U*[1 0; 0 1;0 0]*V3 + center;
V43D= U*[1 0; 0 1;0 0]*V4 + center;
plot3(V13D(1,:), V13D(2,:),V13D(3,:),'or'), hold on
plot3(V23D(1,:), V23D(2,:),V23D(3,:),'ok'), hold on,
plot3(V33D(1,:), V33D(2,:),V33D(3,:),'ob'), hold on,
plot3(V43D(1,:), V43D(2,:),V43D(3,:),'og'), grid on, axis square, hold off

LengthSides=[norm(V1-V2), norm(V2-V3), norm(V3-V4), norm(V4-V1)]



