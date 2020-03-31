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


function stop = outfun(in, optimValues, state, idx, X, X_reference)
     persistent history searchdir fhistory
     stop = false;
 
     switch state
         case 'init'
             hold on
             history = [];
             fhistory = [];
             searchdir = [];
         case 'iter'
         % Concatenate current point and objective function
         % value with history. in must be a row vector.
           fhistory = [fhistory; optimValues.fval];
           history = [history; in(:)']; % Ensure in is a row vector
         % Concatenate current search direction with 
         % searchdir.
           searchdir = [searchdir;... 
                        optimValues.searchdirection(:)'];
           plot(in(idx.x),in(idx.y),'o');
         % Label points with iteration number and add title.
         % Add .15 to idx.x to separate label from plotted 'o'
           text(in(idx.x)+.15,in(idx.y),... 
                num2str(optimValues.iteration));
           title('Sequence of Points Computed by fmincon');
         case 'done'
             hold off
             assignin('base','optimhistory',history);
             assignin('base','searchdirhistory',searchdir);
             assignin('base','functionhistory',fhistory);
         otherwise
     end
 end
