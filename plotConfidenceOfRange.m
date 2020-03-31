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


function plotConfidenceOfRange(fig_handles, confidence_of_range, t_title)
    num_scene = size(confidence_of_range, 2);
    x_plot_all = [];
    y_plot_all = [];
    title_all = t_title + ": ";
    for scene = 1:num_scene % which dataset
        num_scans = size(confidence_of_range(scene).scans, 2);
        x_plot = [];
        y_plot = [];
        for scan_num = 1:num_scans
            num_targets = size(confidence_of_range(scene).scans(scan_num).targets, 2);
            if num_targets ~= 0
                for tag_num = 1:num_targets
                    x_plot = [x_plot, confidence_of_range(scene).scans(scan_num).targets(tag_num).distance];
                    y_plot = [y_plot, confidence_of_range(scene).scans(scan_num).targets(tag_num).RMSE];
                end
            end
        end
        [x_plot, sorted_indices] = sort(x_plot);
        plot(fig_handles(scene), x_plot, y_plot(sorted_indices), '-x')
        title_txt = t_title + ": " + confidence_of_range(scene).bagfile;
        title(fig_handles(scene), title_txt)
        xlabel(fig_handles(scene), "distance [m]")
        ylabel(fig_handles(scene), "RMSE [pixel]")
        set(get(fig_handles(scene),'parent'),'visible','on');
        hold(fig_handles(scene), 'off')
        
        x_plot_all = [x_plot_all, x_plot];
        y_plot_all = [y_plot_all, y_plot(sorted_indices)];
        title_all = title_all + " -- "+confidence_of_range(scene).bagfile;
    end
    [x_plot_all, sorted_indices] = sort(x_plot_all);
    plot(fig_handles(num_scene+1), x_plot_all, y_plot_all(sorted_indices), '-x')
    title(fig_handles(num_scene+1), title_all)
    xlabel(fig_handles(num_scene+1), "distance [m]")
    ylabel(fig_handles(num_scene+1), "RMSE [pixel]")
    set(get(fig_handles(num_scene+1),'parent'),'visible','on');
    hold(fig_handles(num_scene+1), 'off')
end
