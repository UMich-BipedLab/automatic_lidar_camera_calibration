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

function [cross_big_3d, edges]= KaessNewConstraintCorners_v5(base_line, target_size, path, pc_mat, pc_iter)
    d=target_size;

    pc = load(string(path) + string(pc_mat)); 
    pnts = pc.point_cloud; % [scan, point, [X, Y, X, I, R]]
    ransac_threshold = 0.02;

    if base_line.edge_method == 1
        [U,center,LE,RE,LEavg,REavg,LEupper,LElower,REupper,RElower,RingNumbers,NScans,PayLoadClean, PayLoadClean2D] = LeftRightEdges_v02(base_line, pnts, d*sqrt(2), pc_iter);
    elseif base_line.edge_method == 2
        [U, center, LEupper, LElower, REupper, RElower, PayLoadClean, PayLoadClean2D] = clickedToFindEdges(base_line, pnts, d*sqrt(2), pc_iter);
    elseif base_line.edge_method == 3
        [U, center, LEupper, LElower, REupper, RElower, PayLoadClean, PayLoadClean2D] = L1CostToFindEdges(base_line, pnts, d*sqrt(2), pc_iter);
    end
    
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
    TargetGeometry='DoSquare';
%     TargetGeometry='DoRectangle';
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
        elseif 1% L1 %Slow, but works fine.
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
                if and(abs(1+mu1*mu2)<1e-4, max(abs([e1 e4]))< 1e-4)
                    break
                end
            case 'DoRectangle'
                if abs(1+mu1*mu2)<1e-4
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
    RMSerror_Edges=sqrt( ( norm(Error_Edges)^2 ) / length(YEdges))
    
    
    
    edges.LU=U*[1 0; 0 1;0 0]*([1 0;mu1 0] *LEupperall + [0;bLU] ) + center;
    edges.LL=U*[1 0; 0 1;0 0]*([1 0;mu2 0] *LElowerall + [0;bLL] ) + center;
    edges.RU=U*[1 0; 0 1; 0 0]*([1 0;mu2 0] *REupperall +  [0;bRU])  + center;
    edges.RL=U*[1 0; 0 1; 0 0]*([1 0;mu1 0] *RElowerall +  [0;bRL] ) + center;

    LEupperall_new = ([1 0;mu1 0] *LEupperall + [0;bLU]);
    LElowerall_new = ([1 0;mu2 0] *LElowerall + [0;bLL]);
    REupperall_new = ([1 0;mu2 0] *REupperall + [0;bRU]);
    RElowerall_new = ([1 0;mu1 0] *RElowerall + [0;bRL]);

    modelInliers_TL = polyfit(LEupperall_new(1,:)', LEupperall_new(2,:)', 1);
    modelInliers_BL = polyfit(LElowerall_new(1,:)', LElowerall_new(2,:)', 1);

    modelInliers_TR = polyfit(REupperall_new(1,:)', REupperall_new(2,:)', 1);
    modelInliers_BR = polyfit(RElowerall_new(1,:)', RElowerall_new(2,:)', 1);

    cross_L=intersection(modelInliers_TL, modelInliers_BL);

    cross_R=intersection(modelInliers_TR, modelInliers_BR);

    cross_T=intersection(modelInliers_TL, modelInliers_TR);

    cross_B=intersection(modelInliers_BR, modelInliers_BL);

    cross_big_2d = [cross_T, cross_L, cross_R, cross_B];
    if base_line.show_results
        current_img_handle = base_line.img_hangles(4);
        hold(current_img_handle, 'on')
        scatter(current_img_handle, PayLoadClean2D(1,:), PayLoadClean2D(2,:), '.b')
        scatter(current_img_handle, cross_big_2d(1,:), cross_big_2d(2,:), '*g')
        plot(current_img_handle, LEupperall_new(1, :), LEupperall_new(2, :))
        plot(current_img_handle, LElowerall_new(1, :), LElowerall_new(2, :))
        plot(current_img_handle, REupperall_new(1, :), REupperall_new(2, :))
        plot(current_img_handle, RElowerall_new(1, :), RElowerall_new(2, :))
        axis(current_img_handle,'equal');
        xlabel(current_img_handle, 'x')
        ylabel(current_img_handle, 'y')
        title(current_img_handle, '2D regressed edges')
        set(get(current_img_handle, 'parent'),'visible','on');
        hold(current_img_handle, 'off');
    end
    cross_big_3d = U*[1 0; 0 1;0 0]*cross_big_2d + center;
    cross_big_3d = [cross_big_3d; ones(1,size(cross_big_3d,2))];
    cross_big_3d = sortrows(cross_big_3d', 3, 'descend')';

    if base_line.show_results
        current_img_handle = base_line.img_hangles(5);
        hold(current_img_handle, 'on')
        scatter3(current_img_handle, PayLoadClean(1, :), PayLoadClean(2, :), PayLoadClean(3, :), '.b')
        scatter3(current_img_handle, cross_big_3d(1, :), cross_big_3d(2, :), cross_big_3d(3, :), '*g')
        set(get(current_img_handle, 'parent'),'visible','on');
        axis(current_img_handle,'equal');
%         zlim(current_img_handle, [min(cross_big_3d(3,:))-1, max(cross_big_3d(3, :)+1)]);
        xlabel(current_img_handle, 'x')
        ylabel(current_img_handle, 'y')
        zlabel(current_img_handle, 'z')
        title(current_img_handle, '3D regressed edges')
        hold(current_img_handle, 'off');
    end
 end
