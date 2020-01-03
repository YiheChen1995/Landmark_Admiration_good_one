
clear; format short; clc; close all;
dbstop if error

addpath('utils/functions')
addpath('utils/classes')
addpath('utils/constraints/')
addpath('data/')
addpath('cut_road')

re_initialize = false;

while (~re_initialize) ||(~estimator.goal_is_reached && epoch <= params.num_epochs_sim)
 
for map_i= 10:10 
    
% seed the randomness
% rng(map_i)
    
% create objects
params= ParametersClass("simulation_fg_offline_SS");
estimator= EstimatorClassFgSimOffSS(params);
im= IntegrityMonitoringClassFgSimOffSS(params, estimator);
data_obj= DataClass(params.num_epochs_sim, params.num_epochs_sim, params);
counters= CountersClass([], [], params);
if(re_initialize == true)
   estimator.landmark_map=map;
   params.landmark_map=map;
   estimator.num_landmarks= size(estimator.landmark_map, 1);
end

% initialize time index
epoch= 1;
% ----------------------------------------------------------
% -------------------------- LOOP --------------------------


while ~estimator.goal_is_reached && epoch <= params.num_epochs_sim
    disp(strcat('Epoch -> ', num2str(epoch)));
     
    % ------------- Odometry -------------
    estimator.compute_steering(params)
    estimator.odometry_update(params);
    % -------------------------------
    
    % ----------------- LIDAR ----------------
    %while(1)
     if params.SWITCH_LIDAR_UPDATE

         % build the jacobian landmarks in the field of view
         estimator.compute_lidar_H_k( params );
            
         % main function for factor graphs integrity monitoring
         im.monitor_integrity(estimator, counters, data_obj,  params);
         
         %check if need to start from beginning
        if(im.modified_q)% && epoch > 30)
            re_initialize = true;
            map=estimator.landmark_map;
            estimator.num_landmarks= size(estimator.landmark_map, 1);
           break;
        end

         % Store data
         counters.k_update=...
             data_obj.store_update_fg(counters.k_update, estimator, counters.time_sim, params);
         
         % increase integrity counter
         counters.increase_integrity_monitoring_counter();
     end
     
    % -----------------------------------------
    % increase time
    counters.increase_time_sum_sim(params);
    counters.increase_time_sim(params);
    epoch= epoch + 1;

    
   
end

% ------------------------- END LOOP -------------------------
% ------------------------------------------------------------

% Store data for last epoch
data_obj.delete_extra_allocated_memory(counters)


% save workspace
%save(strcat( params.path_sim_fg, 'results/density_001/map_', num2str(map_i), '/offline' ));

end

end


% -------------------------- PLOTS --------------------------
data_obj.plot_map_localization_sim_fg(estimator, params)
data_obj.plot_number_of_landmarks_fg_sim(params);
data_obj.plot_integrity_risk(params);
data_obj.plot_alert_limit_over_sig_hat(params)
% ------------------------------------------------------------
