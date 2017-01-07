n_flow=3
next_conf=1
filePath = sprintf('fig/comparision_scheduling_policies_many_flows/n_flow=%d',n_flow);
matfile = sprintf('%s/comparision_scheduling_flow_%d.mat', filePath, next_conf);
load(matfile);

fileID = fopen(sprintf('%s/conf_comparision_%d.txt',filePath, next_conf),'a');

optimal_utility_RAC_approx
optimal_utility_RAC

%% doing scheduling
T=1000
fprintf('begin to do RACSchedule...\n');
%for both NUM and fesiblity-fulfilling
tic;
[successful_transmission_RAC, state_action_distribution_RAC, system_state_RAC, system_action_RAC, state_action_per_slot_RAC ] ...
    = RACSchedule(obj, T, optimal_policy_RAC);
fprintf(fileID, '\nFinish RACSchedule with time %f seconds\n', toc);


fprintf(fileID, 'begin to do RelaxedRACSchedule...\n');
%for NUM
tic;
[successful_transmission_RAC_approx, state_action_distribution_RAC_approx, system_state_RAC_approx, system_action_RAC_approx, state_action_per_slot_RAC_approx] ...
    = RelaxedRACSchedule_old(obj, T, optimal_policy_RAC_approx, optimal_action_distribution_RAC_approx);
fprintf(fileID, '\nFinish RelaxedRACSchedule for NUM with time %f seconds\n', toc);

fprintf(fileID, 'begin to do RelaxedRACSchedule2...\n');
%for NUM
tic;
[successful_transmission_RAC_approx2, state_action_distribution_RAC_approx2, system_state_RAC_approx2, system_action_RAC_approx2, state_action_per_slot_RAC_approx2] ...
    = RelaxedRACSchedule(obj, T, optimal_policy_RAC_approx, optimal_action_distribution_RAC_approx);
fprintf( fileID, '\nFinish RelaxedRACSchedule2 for NUM with time %f seconds\n', toc);


tic;
[successful_transmission_LDF, virtual_arrival_LDF, virtual_server_capacity_LDF, virtual_departure_LDF,  virtual_queue_LDF, state_action_distribution_LDF, ...
    Rec_system_state_LDF, Rec_system_action_LDF, Rec_state_action_per_slot_LDF] = ...
    LDF(obj, T, strict_throughput_per_flow);
fprintf(fileID,  '\nFinish LDF with time %f seconds\n', toc);

tic;
[successful_transmission_EPDF, virtual_arrival_EPDF, virtual_server_capacity_EPDF, virtual_departure_EPDF,  virtual_queue_EPDF, state_action_distribution_EPDF, ...
    Rec_system_state_EPDF, Rec_system_action_EPDF, Rec_state_action_per_slot_EPDF] = ...
    EPDF(obj, T, strict_throughput_per_flow);
fprintf(fileID,  '\nFinish EPDF with time %f seconds\n', toc);

tic;
[successful_transmission_LLDF, virtual_arrival_LLDF, virtual_server_capacity_LLDF, virtual_departure_LLDF,  virtual_queue_LLDF, state_action_distribution_LLDF, ...
    Rec_system_state_LLDF, Rec_system_action_LLDF, Rec_state_action_per_slot_LLDF] = ...
    LLDF(obj, T, strict_throughput_per_flow);
fprintf(fileID,  '\nFinish LLDF with time %f seconds\n', toc);

%% get statistics of the scheduling
empirical_rate_LDF = sum(successful_transmission_LDF,2)/T;
empirical_rate_EPDF = sum(successful_transmission_EPDF,2)/T;
empirical_rate_LLDF = sum(successful_transmission_LLDF,2)/T;
empirical_rate_RAC = sum(successful_transmission_RAC,2)/T;
empirical_rate_RAC_approx = sum(successful_transmission_RAC_approx,2)/T;

empirical_rate_RAC_approx2 = sum(successful_transmission_RAC_approx2,2)/T;



%for NUM
%empirical utility for RAC scheduler and RAC-Approx scheduler
empirical_utility_RAC = 0;
empirical_utility_RAC_approx = 0;
empirical_utility_RAC_approx2 = 0;
for kk=1:obj.n_flow
    if(isequal(utility_form, 'weighted_sum'))
        %weighted sum
        empirical_utility_RAC = empirical_utility_RAC + utility_coeff(kk)*empirical_rate_RAC(kk);
        empirical_utility_RAC_approx = empirical_utility_RAC_approx + utility_coeff(kk)*empirical_rate_RAC_approx(kk);
        empirical_utility_RAC_approx2 = empirical_utility_RAC_approx2 + utility_coeff(kk)*empirical_rate_RAC_approx2(kk);
    elseif (isequal(utility_form, 'weighted_log_sum'))
        %weighted log sum
        empirical_utility_RAC = empirical_utility_RAC + utility_coeff(kk)*log(empirical_rate_RAC(kk));
        empirical_utility_RAC_approx = empirical_utility_RAC_approx + utility_coeff(kk)*log(empirical_rate_RAC_approx(kk));
        empirical_utility_RAC_approx2 = empirical_utility_RAC_approx2 + utility_coeff(kk)*log(empirical_rate_RAC_approx2(kk));
    else
        error('wrong input utility_form, can only be ''weighted_sum'' or ''weighted_log_sum''');
    end
end

delta_utility_RAC = (empirical_utility_RAC - optimal_utility_RAC)/optimal_utility_RAC;
delta_utility_RAC_approx = (empirical_utility_RAC_approx - optimal_utility_RAC)/optimal_utility_RAC;
delta_utility_RAC_approx2 = (empirical_utility_RAC_approx2 - optimal_utility_RAC)/optimal_utility_RAC;


%for feasibility-fulfilling
delta_LDF = sum(max(strict_throughput_per_flow - empirical_rate_LDF, 0))/sum(strict_throughput_per_flow);
delta_EPDF = sum(max(strict_throughput_per_flow - empirical_rate_EPDF(:,end), 0))/sum(strict_throughput_per_flow);
delta_LLDF = sum(max(strict_throughput_per_flow - empirical_rate_LLDF(:,end), 0))/sum(strict_throughput_per_flow);
delta_RAC = sum(max(strict_throughput_per_flow - empirical_rate_RAC(:,end), 0))/sum(strict_throughput_per_flow);
delta_RAC_approx = sum(max(strict_throughput_per_flow - empirical_rate_RAC_approx(:,end), 0))/sum(strict_throughput_per_flow);

delta_RAC_approx2 = sum(max(strict_throughput_per_flow - empirical_rate_RAC_approx2(:,end), 0))/sum(strict_throughput_per_flow);



if(delta_RAC < 1) %there are some wrong-solve cases such that delta_RAC ect. are very large
    Rec_delta_LDF(nn) = delta_LDF;
    Rec_delta_EPDF(nn) = delta_EPDF;
    Rec_delta_LLDF(nn) = delta_LLDF;
    Rec_delta_RAC(nn) = delta_RAC;
    Rec_delta_RAC_approx(nn) = delta_RAC_approx;
    
    Rec_delta_RAC_approx2(nn) = delta_RAC_approx2;
    
    Rec_delta_utility_RAC(nn) = delta_utility_RAC;
    Rec_delta_utility_RAC_approx(nn) = delta_utility_RAC_approx;
    Rec_delta_utility_RAC_approx2(nn) = delta_utility_RAC_approx2;
end

%display the average result up to the current instance
mean_delta_utility_RAC = mean(Rec_delta_utility_RAC(1:nn))
mean_delta_utility_RAC_approx = mean(Rec_delta_utility_RAC_approx(1:nn))
mean_delta_utility_RAC_approx2 = mean(Rec_delta_utility_RAC_approx2(1:nn))

mean_delta_LDF = mean(Rec_delta_LDF(1:nn))
mean_delta_EPDF = mean(Rec_delta_EPDF(1:nn))
mean_delta_LLDF = mean(Rec_delta_LLDF(1:nn))
mean_delta_RAC = mean(Rec_delta_RAC(1:nn))
mean_delta_RAC_approx = mean(Rec_delta_RAC_approx(1:nn))
mean_delta_RAC_approx2 = mean(Rec_delta_RAC_approx2(1:nn))


fprintf(fileID,  '\n mean_delta_utility_RAC=%f \n mean_delta_utility_RAC_approx=%f', ...
    mean_delta_utility_RAC, min(mean_delta_utility_RAC_approx, mean_delta_utility_RAC_approx2));
fprintf(fileID,  '\n mean_delta_RAC=%f  \n mean_delta_LLDF=%f \n mean_delta_LDF=%f \n mean_delta_RAC_approx=%f \n mean_delta_EPDF=%f\n', ...
    mean_delta_RAC, mean_delta_LLDF, mean_delta_LDF, min(mean_delta_RAC_approx, mean_delta_RAC_approx2), mean_delta_EPDF);



