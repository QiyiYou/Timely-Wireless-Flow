n_flow=3
next_conf=1
filePath = sprintf('fig/comparision_scheduling_policies_many_flows/n_flow=%d',n_flow);
matfile = sprintf('%s/comparision_scheduling_flow_%d.mat', filePath, next_conf);
load(matfile);

fileID = fopen(sprintf('%s/conf_comparision_%d.txt',filePath, next_conf),'a');

optimal_utility_RAC_approx 
optimal_utility_RAC 


n_rate_instance=10
T=10000

%% doing scheduling
Rec_rate_RAC =  zeros(n_flow,n_rate_instance);
Rec_rate_LDF =  zeros(n_flow,n_rate_instance);
Rec_rate_EPDF =  zeros(n_flow,n_rate_instance);
Rec_rate_LLDF =  zeros(n_flow,n_rate_instance);
Rec_rate_RAC_approx_old =  zeros(n_flow,n_rate_instance);
Rec_rate_RAC_approx2 =  zeros(n_flow,n_rate_instance);


for rate_instance=1:n_rate_instance
    
    rate_instance
    
    fprintf(fileID, '===================================================================\n');
    fprintf(fileID, 'rate_instance=%d\n', rate_instance);
    
    fprintf('begin to do RACSchedule...\n');
    %for both NUM and fesiblity-fulfilling
    tic;
    [successful_transmission_RAC, state_action_distribution_RAC, system_state_RAC, system_action_RAC, state_action_per_slot_RAC ] ...
        = RACSchedule(obj, T, optimal_policy_RAC);
    fprintf(fileID, 'Finish RACSchedule with time %f seconds\n', toc);
    
    
    fprintf('begin to do RelaxedRACSchedule_old...\n');
    %for NUM
    tic;
    [successful_transmission_RAC_approx_old, state_action_distribution_RAC_approx_old, system_state_RAC_approx_old, system_action_RAC_approx_old, state_action_per_slot_RAC_approx_old] ...
        = RelaxedRACSchedule_old(obj, T, optimal_policy_RAC_approx, optimal_action_distribution_RAC_approx);
    fprintf(fileID, 'Finish RelaxedRACSchedule_old for NUM with time %f seconds\n', toc);
    
    fprintf('begin to do RelaxedRACSchedule2...\n');
    %for NUM
    tic;
    [successful_transmission_RAC_approx2, state_action_distribution_RAC_approx2, system_state_RAC_approx2, system_action_RAC_approx2, state_action_per_slot_RAC_approx2] ...
        = RelaxedRACSchedule(obj, T, optimal_policy_RAC_approx, optimal_action_distribution_RAC_approx);
    fprintf(fileID, 'Finish RelaxedRACSchedule2 for NUM with time %f seconds\n', toc);
    
    fprintf('begin to do LDF...\n');
    tic;
    [successful_transmission_LDF, virtual_arrival_LDF, virtual_server_capacity_LDF, virtual_departure_LDF,  virtual_queue_LDF, state_action_distribution_LDF, ...
        Rec_system_state_LDF, Rec_system_action_LDF, Rec_state_action_per_slot_LDF] = ...
        LDF(obj, T, strict_throughput_per_flow);
    fprintf(fileID, 'Finish LDF with time %f seconds\n', toc);
    
    fprintf('begin to do EPDF...\n');
    tic;
    [successful_transmission_EPDF, virtual_arrival_EPDF, virtual_server_capacity_EPDF, virtual_departure_EPDF,  virtual_queue_EPDF, state_action_distribution_EPDF, ...
        Rec_system_state_EPDF, Rec_system_action_EPDF, Rec_state_action_per_slot_EPDF] = ...
        EPDF(obj, T, strict_throughput_per_flow);
    fprintf(fileID, 'Finish EPDF with time %f seconds\n', toc);
    
    fprintf('begin to do LLDF...\n');
    tic;
    [successful_transmission_LLDF, virtual_arrival_LLDF, virtual_server_capacity_LLDF, virtual_departure_LLDF,  virtual_queue_LLDF, state_action_distribution_LLDF, ...
        Rec_system_state_LLDF, Rec_system_action_LLDF, Rec_state_action_per_slot_LLDF] = ...
        LLDF(obj, T, strict_throughput_per_flow);
    fprintf(fileID, 'Finish LLDF with time %f seconds\n', toc);
    
    %% get statistics of the scheduling
    Rec_rate_LDF(:, rate_instance) = mean(successful_transmission_LDF,2);
    Rec_rate_EPDF(:, rate_instance) = mean(successful_transmission_EPDF,2);
    Rec_rate_LLDF(:, rate_instance) = mean(successful_transmission_LLDF,2);
    Rec_rate_RAC(:, rate_instance) = mean(successful_transmission_RAC,2);
    Rec_rate_RAC_approx_old(:, rate_instance) = mean(successful_transmission_RAC_approx_old,2);
    Rec_rate_RAC_approx2(:, rate_instance) = mean(successful_transmission_RAC_approx2,2);
end

fprintf(fileID, '===================================================================\n');

empirical_rate_LDF = mean(Rec_rate_LDF,2);
empirical_rate_EPDF = mean(Rec_rate_EPDF,2);
empirical_rate_LLDF = mean(Rec_rate_LLDF,2);
empirical_rate_RAC = mean(Rec_rate_RAC,2)
empirical_rate_RAC_approx_old = mean(Rec_rate_RAC_approx_old,2);
empirical_rate_RAC_approx2 = mean(Rec_rate_RAC_approx2,2);


fprintf(fileID, 'empirical_rate_RAC=[');
for ii=1:n_flow
    fprintf(fileID, '%f,', empirical_rate_RAC(ii));
end
fprintf(fileID, ']\n');

%for NUM
%empirical utility for RAC scheduler and RAC-Approx scheduler
empirical_utility_RAC = 0;
empirical_utility_RAC_approx_old = 0;
empirical_utility_RAC_approx2 = 0;
for kk=1:obj.n_flow
    if(isequal(utility_form, 'weighted_sum'))
        %weighted sum
        empirical_utility_RAC = empirical_utility_RAC + utility_coeff(kk)*empirical_rate_RAC(kk);
        empirical_utility_RAC_approx_old = empirical_utility_RAC_approx_old + utility_coeff(kk)*empirical_rate_RAC_approx_old(kk);
        empirical_utility_RAC_approx2 = empirical_utility_RAC_approx2 + utility_coeff(kk)*empirical_rate_RAC_approx2(kk);
    elseif (isequal(utility_form, 'weighted_log_sum'))
        %weighted log sum
        empirical_utility_RAC = empirical_utility_RAC + utility_coeff(kk)*log(empirical_rate_RAC(kk));
        empirical_utility_RAC_approx_old = empirical_utility_RAC_approx_old + utility_coeff(kk)*log(empirical_rate_RAC_approx_old(kk));
        empirical_utility_RAC_approx2 = empirical_utility_RAC_approx2 + utility_coeff(kk)*log(empirical_rate_RAC_approx2(kk));
    else
        error('wrong input utility_form, can only be ''weighted_sum'' or ''weighted_log_sum''');
    end
end

diff_utility_RAC = (empirical_utility_RAC - optimal_utility_RAC);
diff_utility_RAC_approx_old = (empirical_utility_RAC_approx_old - optimal_utility_RAC);
diff_utility_RAC_approx2 = (empirical_utility_RAC_approx2 - optimal_utility_RAC);
diff_utility_RAC_approx = min(diff_utility_RAC_approx_old, diff_utility_RAC_approx2);

delta_utility_RAC = (empirical_utility_RAC - optimal_utility_RAC)/optimal_utility_RAC;
delta_utility_RAC_approx_old = (empirical_utility_RAC_approx_old - optimal_utility_RAC)/optimal_utility_RAC;
delta_utility_RAC_approx2 = (empirical_utility_RAC_approx2 - optimal_utility_RAC)/optimal_utility_RAC;
delta_utility_RAC_approx = min(delta_utility_RAC_approx_old, delta_utility_RAC_approx2);

%for feasibility-fulfilling
diff_LDF = sum(max(strict_throughput_per_flow - empirical_rate_LDF, 0));
diff_EPDF = sum(max(strict_throughput_per_flow - empirical_rate_EPDF(:,end), 0));
diff_LLDF = sum(max(strict_throughput_per_flow - empirical_rate_LLDF(:,end), 0));
diff_RAC = sum(max(strict_throughput_per_flow - empirical_rate_RAC(:,end), 0));
diff_RAC_approx_old = sum(max(strict_throughput_per_flow - empirical_rate_RAC_approx_old(:,end), 0));
diff_RAC_approx2 = sum(max(strict_throughput_per_flow - empirical_rate_RAC_approx2(:,end), 0));
diff_RAC_approx = min(diff_RAC_approx_old, diff_RAC_approx2);


delta_LDF = sum(max(strict_throughput_per_flow - empirical_rate_LDF, 0))/sum(strict_throughput_per_flow);
delta_EPDF = sum(max(strict_throughput_per_flow - empirical_rate_EPDF(:,end), 0))/sum(strict_throughput_per_flow);
delta_LLDF = sum(max(strict_throughput_per_flow - empirical_rate_LLDF(:,end), 0))/sum(strict_throughput_per_flow);
delta_RAC = sum(max(strict_throughput_per_flow - empirical_rate_RAC(:,end), 0))/sum(strict_throughput_per_flow);
delta_RAC_approx_old = sum(max(strict_throughput_per_flow - empirical_rate_RAC_approx_old(:,end), 0))/sum(strict_throughput_per_flow);
delta_RAC_approx2 = sum(max(strict_throughput_per_flow - empirical_rate_RAC_approx2(:,end), 0))/sum(strict_throughput_per_flow);
delta_RAC_approx = min(delta_RAC_approx_old, delta_RAC_approx2);


fprintf(fileID, '\n');
fprintf(fileID, 'diff_utility_RAC=%f\n', diff_utility_RAC);
fprintf(fileID, 'diff_utility_RAC_approx_old=%f\n', diff_utility_RAC_approx_old);
fprintf(fileID, 'diff_utility_RAC_approx2=%f\n', diff_utility_RAC_approx2);
fprintf(fileID, 'diff_utility_RAC_approx=%f\n', diff_utility_RAC_approx);

fprintf(fileID, '\n');
fprintf(fileID, 'delta_utility_RAC=%f\n', delta_utility_RAC);
fprintf(fileID, 'delta_utility_RAC_approx_old=%f\n', delta_utility_RAC_approx_old);
fprintf(fileID, 'delta_utility_RAC_approx2=%f\n', delta_utility_RAC_approx2);
fprintf(fileID, 'delta_utility_RAC_approx=%f\n', delta_utility_RAC_approx);

fprintf(fileID, '\n');
fprintf(fileID, 'diff_LDF=%f\n', diff_LDF);
fprintf(fileID, 'diff_EPDF=%f\n', diff_EPDF);
fprintf(fileID, 'diff_LLDF=%f\n', diff_LLDF);
fprintf(fileID, 'diff_RAC=%f\n', diff_RAC);
fprintf(fileID, 'diff_RAC_approx_old=%f\n', diff_RAC_approx_old);
fprintf(fileID, 'diff_RAC_approx2=%f\n', diff_RAC_approx2);
fprintf(fileID, 'diff_RAC_approx=%f\n', diff_RAC_approx);

fprintf(fileID, '\n');
fprintf(fileID, 'delta_LDF=%f\n', delta_LDF);
fprintf(fileID, 'delta_EPDF=%f\n', delta_EPDF);
fprintf(fileID, 'delta_LLDF=%f\n', delta_LLDF);
fprintf(fileID, 'delta_RAC=%f\n', delta_RAC);
fprintf(fileID, 'delta_RAC_approx_old=%f\n', delta_RAC_approx_old);
fprintf(fileID, 'delta_RAC_approx2=%f\n', delta_RAC_approx2);
fprintf(fileID, 'delta_RAC_approx=%f\n', delta_RAC_approx);

fprintf(fileID, '\nFinish this instance with time %f seconds\n', toc(begin_time_stamp));



