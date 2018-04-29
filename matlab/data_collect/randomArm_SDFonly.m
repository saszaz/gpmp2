% planar arm obstacle avoidance with reaching a goal
% @author Jing Dong
% @date Nov 23, 2015

clc
close all
% clear all;

import gtsam.*
import gpmp2.*
addpath('/usr/local/gtsam_toolbox/')

if ~exist('seed')
    seed = 4;
end

% rng(seed);
data_num = 5000;
map_dim = [256,256];

% data directory
date = datestr(now,'mmm-dd-yy_HH:MM:SS');
dir_name = strcat(sprintf('datasize_%d_',data_num),date);
data_path = fullfile(pwd,'data_costMap',dir_name);
mkdir(data_path);

% obstacle cost settings
% cost_sigma = 0.1;
epsilon_dist = 0.1;

for iter=1:data_num
    
    if mod(iter,50) == 0
        fprintf('iter: %d\n\n',iter);
    end
    
    %% small dataset
    generate = true;
    while generate
        dataset = generateRandom2D(map_dim);
        rows = dataset.rows;
        cols = dataset.cols;
        cell_size = dataset.cell_size;
        if all(dataset.map(rows/2+1, cols/2+1:end) == 0) ...
                && dataset.map(rows/2+1.1/cell_size+1, cols/2+1) == 0
            generate = false;
        end
    end
    origin_point2 = Point2(dataset.origin_x, dataset.origin_y);

    % signed distance field
    field = signedDistanceField2D(dataset.map, cell_size);
    sdf = PlanarSDF(origin_point2, cell_size, field);

    % Get hinge loss
    loss = -1.0 * field + epsilon_dist;
    hinge = field <= epsilon_dist;
    cost_map = hinge .* loss;
    
    % plot sdf
%     figure(2)
%     plotSignedDistanceField2D(field, dataset.origin_x, dataset.origin_y, dataset.cell_size);
%     title('Signed Distance Field')

    % plot cost
    figure(3)
    I=mat2gray(cost_map);
    imshow(I)
%     colorbar
    set(gca,'Ydir','Normal')
    
    % Initialize figures
    figure(4)
    set(gca,'Position',[0 0 1 1]);
    set(gcf,'Position',[500 500 map_dim(1) map_dim(2)]);
    a(1) = gca;
    
    % plot obstacles
    figure(4), cla
    hold on
    plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
    pause(0.01); hold off;

    fr = getframe(4);
    Im_obst = frame2im(fr);

    case_path = fullfile(data_path, sprintf('%05d',iter));
    mkdir(case_path);
    
    % Save Obstacles
    Im_obst = im2bw(Im_obst); %#ok<IM2BW>
    img_file = fullfile(case_path,'obstacles.png');
    imwrite(Im_obst,img_file);
    
    % Save Cost Map
    cost_file = fullfile(case_path,'cost.mat');
    save(cost_file,'cost_map');
    
%         pause;
end