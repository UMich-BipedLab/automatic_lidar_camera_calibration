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


function line = best_fit_3D_line(pc, display)
    X = pc(1:3,:)';
%     X = pc;
    N=size(X,1);
    
    % Find line of best fit (in least-squares sense) through X
    % -------------------------------------------------------------------------
    X_ave=mean(X,1);            % mean; line of best fit will pass through this point
    dX=bsxfun(@minus,X,X_ave);  % residuals
    C=(dX'*dX)/(N-1);           % variance-covariance matrix of X
    [R,D]=svd(C,0);             % singular value decomposition of C; C=R*D*R'
    
    % Coefficient of determination; R^2 = (explained variance)/(total variance)
    D=diag(D);
    R2=D(1)/sum(D);
    line.dX = dX;
    line.m = R(:,1)';
    line.X0 = X_ave;
    line.R2 = R2;
    if checkDisplay(display)
        % Visualize X and line of best fit
        % -------------------------------------------------------------------------
        % End-points of a best-fit line (segment); used for visualization only
        x=dX*R(:,1);    % project residuals on R(:,1)
        x_min=min(x);
        x_max=max(x);
        dx=x_max-x_min;
        Xa=(x_min-0.05*dx)*R(:,1)' + X_ave;
        Xb=(x_max+0.05*dx)*R(:,1)' + X_ave;
        X_end=[Xa;Xb];
        figure(3000)
        axis equal
        hold on
        Hl=plot3(X_end(:,1),X_end(:,2),X_end(:,3),'-r','LineWidth',3); % best fit line
        Hg=plot3(X(:,1),X(:,2),X(:,3),'.k','MarkerSize',13);         
        set(get(gca,'Title'),'String',sprintf('R^2 = %.3f',R2),'FontSize',25,'FontWeight','normal')
        xlabel('X','FontSize',20,'Color','k')
        ylabel('Y','FontSize',20,'Color','k')
        zlabel('Z','FontSize',20,'Color','k')
        view([20 20])
        drawnow
        % Display line parameters
        % -------------------------------------------------------------------------
        fprintf('Best fit line : L(t) = Xo + t*r, where
')
        fprintf('Xo  = [ '); fprintf('%.4f ',X_ave);   fprintf(']
') 
        fprintf('r   = [ '); fprintf('%.4f ',R(:,1)'); fprintf(']
') 
        fprintf('R^2 = %.4f
',R2)
    end
end
