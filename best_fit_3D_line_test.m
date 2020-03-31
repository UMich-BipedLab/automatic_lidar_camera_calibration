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


function guesses=david_script
% Receiver positions & times
p1=[-1  0  0];
p2=[ 0 -1  0];
p3=[ 1  0  1];
p4=[ 0  1  0];
p5=[ 0  0  1];
t1=0.0031;
t2=0.0056;
t3=0.0019;
t5=0.0022;
% Generate a list of plausible guesses
percent = 0.01;
[x,y,z]=deal(cell(201,1));
[x_i,y_i]=meshgrid(single(-800:800),single(-800:800));
x_i=x_i(:);
y_i=y_i(:);
z_i=ones(size(x_i),'single');
fprintf('Generating plausible guesses
')
for i=0:200    
    [x{i+1},y{i+1},z{i+1}]=get_plausible_guesses(x_i(:),y_i(:),single(i)*z_i(:));
    fprintf('Iteration %3u/201
',i+1)
end
x=cell2mat(x);
y=cell2mat(y);
z=cell2mat(z);
guesses=[x y z];
guessNumber=size(guesses,1); 
fprintf('

Number of plausible guesses = %u

',guessNumber)
% Get parameters of best-fit-thought the guesses and visulize
% -------------------------------------------------------------------------
[Hg,Hl]=best_fit_3D_line(guesses);
% Also visulize receiver positions
% -------------------------------------------------------------------------
P={p1 p2 p3 p4 p5};
Hp=zeros(1,5);
for i=1:5
    Hp(i)=plot3(P{i}(1),P{i}(2),P{i}(3),'.','MarkerSize',30);
end
legend([Hg Hl Hp],{'guesses' 'best-fit line' 'receiver 1' 'receiver 2' 'receiver 3' 'receiver 4' 'receiver 5'})
if nargout<1, clear guesses; end
      function [x,y,z]=get_plausible_guesses(x,y,z)
      % Remove grid points based on specified contraints
      % ---------------------------------------------------------------------
      % Constraint # 1
      idx=x>=y;
      x(idx)=[];
      y(idx)=[];
      z(idx)=[];
      % Constraint # 2
      idx=z<=2*x;
      x(idx)=[];
      y(idx)=[];
      z(idx)=[];
      % Distance of the two receivers with the lowest tdoa
      D4=(x-p4(1)).^2 + (y-p4(2)).^2 + (z-p4(3)).^2;
      D3=(x-p3(1)).^2 + (y-p3(2)).^2 + (z-p3(3)).^2;
      idx=D4>=D3;
      x(idx)=[];
      y(idx)=[];
      z(idx)=[];
      D3(idx)=[];
      D4(idx)=[];
      % Distance of the receiver with the next lowest tdoa
      D5=(x-p5(1)).^2 + (y-p5(2)).^2 + (z-p5(3)).^2;
      idx=D3>=D5;
      x(idx)=[];
      y(idx)=[];
      z(idx)=[];
      D3(idx)=[];
      D4(idx)=[];
      D5(idx)=[];
      % Distance of the receiver with the next lowest tdoa
      D1=(x-p1(1)).^2 + (y-p1(2)).^2 + (z-p1(3)).^2;
      idx=D5>=D1;
      x(idx)=[];
      y(idx)=[];
      z(idx)=[];
      D1(idx)=[];
      D3(idx)=[];
      D4(idx)=[];
      D5(idx)=[];
      % Distance of the receiver with the next lowest tdoa
      D2=(x-p2(1)).^2 + (y-p2(2)).^2 + (z-p2(3)).^2;
      idx=D1>=D2;
      x(idx)=[];
      y(idx)=[];
      z(idx)=[];
      D1(idx)=[];
      D2(idx)=[];
      D3(idx)=[];
      D4(idx)=[];
      D5(idx)=[];
      % Enforce additional constraints
      % ---------------------------------------------------------------------
      [D1,D2,D3,D4,D5]=deal(sqrt(D1),sqrt(D2),sqrt(D3),sqrt(D4),sqrt(D5));
      tdoa1 = (D1-D4)/343;
      tdoa2 = (D2-D4)/343;
      tdoa3 = (D3-D4)/343;
      tdoa5 = (D5-D4)/343;
      clear D1 D2 D3 D4 D5
      % If the calculated value of any tdoa deviates more than 5% of the ideal,
      % then this is not a plausible solution
      idx= (tdoa1>=(t1+(percent*t1))) | (tdoa1<=(t1-(percent*t1))) | ...
          (tdoa2>=(t2+(percent*t2))) | (tdoa2<=(t2-(percent*t2))) | ...
          (tdoa3>=(t3+(percent*t3))) | (tdoa3<=(t3-(percent*t3))) | ...
          (tdoa5>=(t5+(percent*t5))) | (tdoa5<=(t5-(percent*t5)));
      % Plausible guesses
      x(idx)=[];
      y(idx)=[];
      z(idx)=[];
      end
end
function [Hg,Hl]=best_fit_3D_line(X)
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
figure('color','w')
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
