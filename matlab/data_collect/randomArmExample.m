% planar arm obstacle avoidance with reaching a goal

close all
% clear all
% clearvars -except seed

if ~exist('seed')
    seed = 123;
end

import gtsam.*
import gpmp2.*
addpath('/usr/local/gtsam_toolbox/');

rng(seed);

map_dim = [256,256];

% data directory
date = datestr(now,'mmm-dd-yy_HH:MM:SS');
data_path = fullfile(pwd,'data',date);
mkdir(data_path);

%% Settings

    % Number of planning attempts
    num_plan = 40000;

    total_time_sec = 20.0;
    total_time_step = 100;
    total_check_step = 100;
    delta_t = total_time_sec / total_time_step;
    check_inter = total_check_step / total_time_step - 1;

    % use GP interpolation
    use_GP_inter = true;

    % GP
    Qc = 0.01*eye(3);
    Qc_model = noiseModel.Gaussian.Covariance(Qc); 

    % obstacle cost settings
    cost_sigma = 0.1;
    epsilon_dist = 0.1;

    % prior model on start conf/velocity
    pose_fix = noiseModel.Isotropic.Sigma(3, 0.0001);
    vel_fix = noiseModel.Isotropic.Sigma(3, 0.0001);
    
    % final goal noise model
    goal_fix = noiseModel.Isotropic.Sigma(3, 0.0001);
    
    % start veolicty
    start_vel = [0, 0, 0]';
    
    % end velocity
    end_vel = [0, 0, 0]';
    %avg_vel = ((end_conf_init - start_conf) / total_time_step) / delta_t;
    
    % plot settings
    plot_smooth = false;
    plot_inter = 10;

%% Generate arm model

    arm = generateArm('SimpleThreeLinksArm');

%% Main loop
    
iter = 0;
while iter < num_plan
        
    % start and end conf
    jt2_min=-0.5*pi; jt2_max= 0.5*pi; 
    jt3_min=-0.5*pi; jt3_max= 0.5*pi; 
    
    start_conf = [rand*2*pi, ...                         
                      jt2_min+rand*(jt2_max-jt2_min),...
                      jt3_min+rand*(jt3_max-jt3_min)]' ;
    
    end_conf_inits = repmat([rand*2*pi, ...                          % CCW
                            jt2_min+rand*(jt2_max-jt2_min),...
                            jt3_min+rand*(jt3_max-jt3_min)],...
                            2,1);      
                        
    end_conf_inits(2,1) = end_conf_inits(1,1) - 2*pi;                 % CW
    end_conf_inits = end_conf_inits';
    
    start_jts_xy = arm.fk_model().forwardKinematicsPosition(start_conf);
    goal_jts_xy = arm.fk_model().forwardKinematicsPosition(end_conf_inits(:,1));
    goal = goal_jts_xy(:,end);
    
    % small dataset
    dataset = generateRandom2D(map_dim, start_jts_xy, goal_jts_xy);
    rows = dataset.rows;
    cols = dataset.cols;
    cell_size = dataset.cell_size;
    
    % Create origin and goal point objects
    origin_point2 = Point2(dataset.origin_x, dataset.origin_y);
    goal_point3 = Point3([goal; 0]);
    
    % signed distance field
    field = signedDistanceField2D(dataset.map, cell_size);
    sdf = PlanarSDF(origin_point2, cell_size, field);

    % plot sdf
    % figure(2)
    % plotSignedDistanceField2D(field, dataset.origin_x, dataset.origin_y, dataset.cell_size);
    % title('Signed Distance Field')
    
    % plot start configuration / goal point
    figure(1), hold on
    plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, dataset.cell_size);
    plotPlanarArm(arm.fk_model(), start_conf, 'b', 2);
    plot(goal(1), goal(2), 'r*');
    title('Layout')
    hold off


    %% init optimization
    for i = 1:2
        end_conf_init = end_conf_inits(:,i);
        avg_vel = ((end_conf_init - start_conf) / total_time_step) / delta_t;
        graph = NonlinearFactorGraph;
        init_values = Values;

        for i = 0 : total_time_step
            key_pos = symbol('x', i);
            key_vel = symbol('v', i);

            pose = start_conf * (total_time_step-i)/total_time_step + end_conf_init * i/total_time_step;
            vel = avg_vel;
            init_values.insert(key_pos, pose);
            init_values.insert(key_vel, vel);

            % priors
            if i==0
                graph.add(PriorFactorVector(key_pos, start_conf, pose_fix));
                graph.add(PriorFactorVector(key_vel, start_vel, vel_fix));
            elseif i==total_time_step
                graph.add(GoalFactorArm(key_pos, goal_fix, arm.fk_model(), goal_point3));
                graph.add(PriorFactorVector(key_vel, end_vel, vel_fix));
            end

            % GP priors and cost factor
            if i > 0
                key_pos1 = symbol('x', i-1);
                key_pos2 = symbol('x', i);
                key_vel1 = symbol('v', i-1);
                key_vel2 = symbol('v', i);
                graph.add(GaussianProcessPriorLinear(key_pos1, key_vel1, ...
                    key_pos2, key_vel2, delta_t, Qc_model));

                % cost factor
                graph.add(ObstaclePlanarSDFFactorArm(...
                    key_pos, arm, sdf, cost_sigma, epsilon_dist));

                % GP cost factor
                if use_GP_inter && check_inter > 0
                    for j = 1:check_inter
                        tau = j * (total_time_sec / total_check_step);
                        graph.add(ObstaclePlanarSDFFactorGPArm( ...
                            key_pos1, key_vel1, key_pos2, key_vel2, ...
                            arm, sdf, cost_sigma, epsilon_dist, ...
                            Qc_model, delta_t, tau));
                    end
                end
            end
        end

        % %% plot initial values
        % for i=0:total_time_step
        %     figure(3), hold on
        %     title('Initial Values')
        %     % plot world
        %     plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
        %     % plot arm
        %     conf = init_values.atVector(symbol('x', i));
        %     plotPlanarArm(arm, conf, 'b', 2);
        %     pause(pause_time), hold off
        % end


        %% optimize!
        use_LM = false;
        use_trustregion_opt = true;

        if use_LM
            parameters = LParams;
            parameters.setVerbosity('ERROR');
            optimizer = DoglegOptimizer(graph, init_values, parameters);
        elseif use_trustregion_opt
            parameters = DoglegParams;
            parameters.setVerbosity('ERROR');
            optimizer = DoglegOptimizer(graph, init_values, parameters);
        else
            parameters = GaussNewtonParams;
            parameters.setVerbosity('ERROR');
            optimizer = GaussNewtonOptimizer(graph, init_values, parameters);
        end

        tic
        optimizer.optimize();
        toc

        result = optimizer.values();
        % result.print('Final results')


        %% plot final values
        % interpolation for smooth traj to plot

        if plot_smooth
            % smooth version
            total_plot_step = total_time_step * plot_inter;
            plot_values = interpolateArm3Traj(result, Qc_model, delta_t, plot_inter-1);
            pause_time = total_time_sec / (total_time_step * plot_inter);
        else
            % non-smooth version
            total_plot_step = total_time_step;
            plot_values = result;
            pause_time = total_time_sec / total_time_step;
        end

        % generate traj line
        % each two rows are end eff pos
        opt_traj_line = [6, total_time_step+1];
        for i=0:total_time_step
            conf = result.atVector(symbol('x', i));
            position = arm.fk_model().forwardKinematicsPosition(conf);
            for j=1:3
                opt_traj_line(j*2-1:j*2, i+1) = position(1:2, j);
            end
        end

        smooth_traj_line = [6, total_plot_step+1];
        for i=0:total_plot_step
            conf = plot_values.atVector(symbol('x', i));
            position = arm.fk_model().forwardKinematicsPosition(conf);
            for j=1:3
                smooth_traj_line(j*2-1:j*2, i+1) = position(1:2, j);
            end
        end

        traj_settings = TrajOptimizerSetting(3);
        if CollisionCost2DArm(arm, sdf, result, traj_settings) == 0
            break;
        end
    end

    if CollisionCost2DArm(arm, sdf, result, traj_settings) == 0
        
        % Initialize figures
        figure(4)
        set(gca,'Position',[0 0 1 1]);
        set(gcf,'Position',[500 500 map_dim(1) map_dim(2)]);
        a(1) = gca;

        figure(5)
        set(gca,'Position',[0 0 1 1]);
        set(gcf,'Position',[500+map_dim(1) 500 map_dim(1) map_dim(2)]);
        set(gca,'XColor','none');
        set(gca,'YColor','none');
        a(2) = gca;
        linkaxes(a, 'xy');
        
        iter = iter + 1;
        
        % Extract joint angles and velocities.
        % x = joint angles, size [3, time steps]
        % u = joint velocities, size [3, time steps]
        x = zeros(3, total_time_step+1);
        u = zeros(3, total_time_step+1);
        Im = zeros(map_dim(1), map_dim(2), 3, total_time_step+1);
        
        for t = 0:total_time_step
            x(:,t+1) = result.atVector(symbol('x', t));
            u(:,t+1) = result.atVector(symbol('v', t));
        end 
        
        % plot obstacles
        figure(4), cla
        hold on
        plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
        pause(0.01); hold off;
        
        fr = getframe(4);
        Im_obst = frame2im(fr);
        
        % goal image
        axes=get(gcf,'CurrentAxes');
        Im_goal = getGoalImage(goal, axes, map_dim);
        
        for i=0:total_plot_step
            
%            plot arm
            figure(5), cla
            hold on
            conf = plot_values.atVector(symbol('x', i));
            plotPlanarArm(arm.fk_model(), conf, 'b', 4); % 2);
            pause(0.01); hold off
            
            fr = getframe(5);
            Im(:,:,:,i+1) = frame2im(fr);
        end
        
        
        case_path = fullfile(data_path, sprintf('%05d',iter));
        mkdir(case_path);
        saveData(x,u,Im,Im_obst,Im_goal,case_path);
        
%         pause;
    end
end