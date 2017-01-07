clear all; close all;

%remember to creates a different seed each time
rng shuffle
% see http://cvxr.com/cvx/doc/solver.html
%select gurobi to solve the LP, much faster than the default SDP3
%cvx_solver gurobi_3  %for LP
%cvx_solver SDPT3  %for convex
cvx_solver SeDuMi
cvx_save_prefs
cvx_precision best

n_flow=2
filePath = sprintf('fig/comparision_scheduling_policies_many_flows/n_flow=%d',n_flow);

%find the next number for the configuration file
next_conf = 1;
while(1)
    if(exist(sprintf('%s/conf_comparision_%d.txt',filePath, next_conf), 'file') == 2)
        next_conf = next_conf + 1;
    else
        break;
    end
end
fileID = fopen(sprintf('%s/conf_comparision_%d.txt',filePath, next_conf),'w');


n_instance=1
n_rate_instance=10
T=10000
fprintf(fileID, 'n_flow=%d, n_instance=%d, T=%d\n', n_flow, n_instance, T);





Rec_delta_LDF = zeros(n_instance,1);
Rec_delta_EPDF = zeros(n_instance,1);
Rec_delta_LLDF = zeros(n_instance,1);
Rec_delta_RAC = zeros(n_instance,1);
Rec_delta_RAC_approx = zeros(n_instance,1);
Rec_delta_RAC_approx2 = zeros(n_instance,1);

Rec_delta_utility_RAC = zeros(n_instance,1);
Rec_delta_utility_RAC_approx = zeros(n_instance,1);
Rec_delta_utility_RAC_approx2 = zeros(n_instance,1);

for nn=1:n_instance
    
    
    begin_time_stamp=tic;
    nn
    fprintf(fileID, '====================================================================\n');
    fprintf(fileID, '\nInstance %d\n', nn);
    
    %% construct the flow instance
    flow_array = cell(n_flow,1);
    for ii=1:n_flow
        flow = NonOverlappedFlowInstance();
        flow.offset = 0;
        %flow.offset = randi([1,5]);
        %according to my observation RAC_Approx has bad performance with period/delay=1, and less arrival_prob*success_prob
        flow.period = randi([2,5]); %let period at least 2
        flow.delay = randi([2,flow.period]); %let delay at least 2
        flow.arrival_prob = randi([50,100])/100; % at least 0.5
        flow.success_prob = randi([50,100])/100; % at least 0.5
        
        flow.constructEverything();
        flow_array{ii} = flow;
    end
    
    
    for ii=1:n_flow
        fprintf(fileID, 'Flow %d: (offset, period, delay, success_prob, arrival_prob) = (%d, %d, %d, %f, %f)\n', ii, flow_array{ii}.offset, ...
            flow_array{ii}.period, flow_array{ii}.delay, flow_array{ii}.success_prob,  flow_array{ii}.arrival_prob(1));
    end
    
    obj = DownlinkAPInstance();
    obj.n_flow = n_flow;
    obj.flow_array = flow_array;
    obj.constructEverything();
    obj.stateSanityCheck();
    
    %utility_coeff =  rand(n_flow,1);
    utility_coeff =  100*ones(n_flow,1); %enlarge the coeff such that the solver can get more accurate reuslt
    %utility_coeff = utility_coeff./sum(utility_coeff);
    utility_form = 'weighted_log_sum';
    %utility_form = 'weighted_sum';
    
    fprintf(fileID, '\n%s, utility_coeff=[', utility_form);
    for ii=1:n_flow
        fprintf(fileID, '%f,',utility_coeff(ii));
    end
    fprintf(fileID, ']\n');
    
    %% solve the optimization problems RAC and RAC-Approx
    %first solve getApproximateSolutionRAC, if status is not Solved, we can
    %terminate soon
    tic;
    [optimal_policy_RAC_approx, optimal_action_distribution_RAC_approx, optimal_utility_RAC_approx, optimal_throughput_per_flow_RAC_approx, status] = ...
        getApproximateSolutionRAC(obj, utility_coeff, utility_form);
    fprintf(fileID, '\nFinish getApproximateSolutionRAC with time %f seconds\n', toc);
    fprintf(fileID, '\nstatus=%s\n', status);
    
    
    status
    if(~strcmp(status, 'Solved'))
        fprintf('getApproximateSolutionRAC not solved, igonre this instance\n');
        continue;
    end
    
    
    tic;
    [optimal_policy_RAC, optimal_utility_RAC, optimal_throughput_per_flow_RAC, status] = ...
        getOptimalSolutionRAC_v(obj, utility_coeff, utility_form);
    fprintf(fileID, '\nFinish getOptimalSolutionRAC with time %f seconds\n', toc);
    fprintf(fileID, '\nstatus=%s\n', status);
    
    status
    if(~strcmp(status, 'Solved'))
        fprintf('getOptimalSolutionRAC_v not solved, igonre this instance\n');
        continue;
    end
    
    
    %RAC or RAC_Approx are not correctly solved
    if(sum(isnan(optimal_throughput_per_flow_RAC)) > 0 || sum(isnan(optimal_throughput_per_flow_RAC_approx)) > 0)
        fprintf('RAC or RAC_approx are not correctly solved\n');
        continue;
    end
    
    
    epsilon = 0;
    %strict_throughput_per_flow = max(throughput_per_flow - epsilon, 0);
    % let us use optimal throughput from RAC optmization problem
    strict_throughput_per_flow = max(optimal_throughput_per_flow_RAC - epsilon, 0)
    
    
    %% compare RAC and RAC_approx
    %     for tt=1:obj.period_lcm
    %         for ss=1:obj.n_state
    %             ss_vec = obj.getVectorState(ss);
    %
    %             fprintf('tt=%d, ss=%d, ss_vec=[', tt, ss);
    %             for kk=1:obj.n_flow
    %                 fprintf('%d, ', ss_vec(kk));
    %             end
    %             fprintf(']\n');
    %
    %             action_prob_RAC = zeros(1, obj.n_flow);
    %             state_prob_temp = sum(squeeze(optimal_policy_RAC(tt,ss,:)));
    %             if(state_prob_temp < 1e-7)
    %                 action_prob_RAC(:) = 0;
    %             else
    %                 action_prob_RAC(:) = optimal_policy_RAC(tt,ss,:)./state_prob_temp;
    %             end
    %
    %             action_prob_RAC_approx_per_flow = zeros(obj.n_flow, obj.n_action);
    %             for kk=1:obj.n_flow
    %                 action_prob_RAC_approx_per_flow(kk, :) = squeeze(optimal_policy_RAC_approx(kk, tt, ss_vec(kk),:))'/sum(optimal_policy_RAC_approx(kk, tt, ss_vec(kk),:));
    %                 fprintf('action_prob_RAC_approx_per_flow(%d,:)=[', kk);
    %                 for aa=1:obj.n_action
    %                     fprintf('%f, ', action_prob_RAC_approx_per_flow(kk, aa));
    %                 end
    %                 fprintf(']\n');
    %             end
    %
    %             action_prob_RAC_approx = ones(1, obj.n_flow);
    %             for aa=1:obj.n_action
    %                 for kk=1:obj.n_flow
    %                     state_aa_prob_temp = optimal_policy_RAC_approx(kk, tt, ss_vec(kk),aa);
    %                     state_prob_temp = sum(optimal_policy_RAC_approx(kk, tt, ss_vec(kk),:));
    %                     if(state_prob_temp < 1e-7) %flow k will not be in this state
    %                         action_prob_RAC_approx(aa) = 0;
    %                         break;
    %                     else
    %                         action_prob_RAC_approx(aa) = action_prob_RAC_approx(aa)*10*state_aa_prob_temp/state_prob_temp; %note that we scale it up by 10x to avoid small action_prob
    %                     end
    %                 end
    %             end
    %
    %
    %             %normalized to conditional distribution
    %             if(sum(action_prob_RAC_approx) < 1e-9) %this is empty state, and sum(action_prob) = 0
    %                 %action_prob_RAC_approx = ones(1,obj.n_action)/obj.n_action;
    %                 action_prob_RAC_approx(:) = 0;
    %             else
    %                 action_prob_RAC_approx = action_prob_RAC_approx./sum(action_prob_RAC_approx);
    %             end
    %
    %
    %             fprintf('action_prob=           [');
    %             for kk=1:obj.n_flow
    %                 fprintf('%f, ', action_prob_RAC(kk));
    %             end
    %             fprintf(']\n');
    %
    %             fprintf('action_prob_RAC_approx=[');
    %             for kk=1:obj.n_flow
    %                 fprintf('%f, ', action_prob_RAC_approx(kk));
    %             end
    %             fprintf(']\n');
    %
    %         end
    %     end
    
    
    
    
    %% doing scheduling
    Rec_rate_RAC =  zeros(2,n_rate_instance);
    Rec_rate_LDF =  zeros(2,n_rate_instance);
    Rec_rate_EPDF =  zeros(2,n_rate_instance);
    Rec_rate_LLDF =  zeros(2,n_rate_instance);
    Rec_rate_RAC_approx =  zeros(2,n_rate_instance);
    Rec_rate_RAC_approx2 =  zeros(2,n_rate_instance);
        
    for rate_instance=1:n_rate_instance
        
        rate_instance
        
        fprintf('begin to do RACSchedule...\n');
        %for both NUM and fesiblity-fulfilling
        tic;
        [successful_transmission_RAC, state_action_distribution_RAC, system_state_RAC, system_action_RAC, state_action_per_slot_RAC ] ...
            = RACSchedule(obj, T, optimal_policy_RAC);
        fprintf(fileID, '\nFinish RACSchedule with time %f seconds\n', toc);
        
        
        fprintf('begin to do RelaxedRACSchedule_old...\n');
        %for NUM
        tic;
        [successful_transmission_RAC_approx, state_action_distribution_RAC_approx, system_state_RAC_approx, system_action_RAC_approx, state_action_per_slot_RAC_approx] ...
            = RelaxedRACSchedule_old(obj, T, optimal_policy_RAC_approx, optimal_action_distribution_RAC_approx);
        fprintf(fileID, '\nFinish RelaxedRACSchedule_old for NUM with time %f seconds\n', toc);
        
        fprintf('begin to do RelaxedRACSchedule2...\n');
        %for NUM
        tic;
        [successful_transmission_RAC_approx2, state_action_distribution_RAC_approx2, system_state_RAC_approx2, system_action_RAC_approx2, state_action_per_slot_RAC_approx2] ...
            = RelaxedRACSchedule(obj, T, optimal_policy_RAC_approx, optimal_action_distribution_RAC_approx);
        fprintf(fileID, '\nFinish RelaxedRACSchedule2 for NUM with time %f seconds\n', toc);
        
        fprintf('begin to do LDF...\n');
        tic;
        [successful_transmission_LDF, virtual_arrival_LDF, virtual_server_capacity_LDF, virtual_departure_LDF,  virtual_queue_LDF, state_action_distribution_LDF, ...
            Rec_system_state_LDF, Rec_system_action_LDF, Rec_state_action_per_slot_LDF] = ...
            LDF(obj, T, strict_throughput_per_flow);
        fprintf(fileID, '\nFinish LDF with time %f seconds\n', toc);
        
        fprintf('begin to do EPDF...\n');
        tic;
        [successful_transmission_EPDF, virtual_arrival_EPDF, virtual_server_capacity_EPDF, virtual_departure_EPDF,  virtual_queue_EPDF, state_action_distribution_EPDF, ...
            Rec_system_state_EPDF, Rec_system_action_EPDF, Rec_state_action_per_slot_EPDF] = ...
            EPDF(obj, T, strict_throughput_per_flow);
        fprintf(fileID, '\nFinish EPDF with time %f seconds\n', toc);
        
        fprintf('begin to do LLDF...\n');
        tic;
        [successful_transmission_LLDF, virtual_arrival_LLDF, virtual_server_capacity_LLDF, virtual_departure_LLDF,  virtual_queue_LLDF, state_action_distribution_LLDF, ...
            Rec_system_state_LLDF, Rec_system_action_LLDF, Rec_state_action_per_slot_LLDF] = ...
            LLDF(obj, T, strict_throughput_per_flow);
        fprintf(fileID, '\nFinish LLDF with time %f seconds\n', toc);
        
        %% get statistics of the scheduling
        Rec_rate_LDF(:, rate_instance) = mean(successful_transmission_LDF,2);
        Rec_rate_EPDF(:, rate_instance) = mean(successful_transmission_EPDF,2);
        Rec_rate_LLDF(:, rate_instance) = mean(successful_transmission_LLDF,2);
        Rec_rate_RAC(:, rate_instance) = mean(successful_transmission_RAC,2);
        Rec_rate_RAC_approx(:, rate_instance) = mean(successful_transmission_RAC_approx,2);
        Rec_rate_RAC_approx2(:, rate_instance) = mean(successful_transmission_RAC_approx2,2);
        
    end
    
    empirical_rate_LDF = mean(Rec_rate_LDF,2);
    empirical_rate_EPDF = mean(Rec_rate_EPDF,2);
    empirical_rate_LLDF = mean(Rec_rate_LLDF,2);
    empirical_rate_RAC = mean(Rec_rate_RAC,2);
    empirical_rate_RAC_approx = mean(Rec_rate_RAC_approx,2);
    empirical_rate_RAC_approx2 = mean(Rec_rate_RAC_approx2,2);
    
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
    
    
    fprintf(fileID, '\n mean_delta_utility_RAC=%f \n mean_delta_utility_RAC_approx_old=%f \n mean_delta_utility_RAC_approx2=%f \n mean_delta_utility_RAC_approx=%f\n ', ...
        mean_delta_utility_RAC, mean_delta_utility_RAC_approx,mean_delta_utility_RAC_approx2, min(mean_delta_utility_RAC_approx, mean_delta_utility_RAC_approx2));
    fprintf(fileID, '\n mean_delta_RAC=%f  \n mean_delta_LLDF=%f \n mean_delta_LDF=%f \n mean_delta_RAC_approx_old=%f \n mean_delta_RAC_approx2=%f \n mean_delta_RAC_approx=%f \n mean_delta_EPDF=%f\n', ...
        mean_delta_RAC, mean_delta_LLDF, mean_delta_LDF, mean_delta_RAC_approx, mean_delta_RAC_approx2, min(mean_delta_RAC_approx, mean_delta_RAC_approx2), mean_delta_EPDF);
    
    fprintf(fileID, '\nFinish this instance with time %f seconds\n', toc(begin_time_stamp));
end

% %% finally output the mean result and write to the mat file
% mean_delta_utility_RAC = mean(Rec_delta_utility_RAC)
% mean_delta_utility_RAC_approx = mean(Rec_delta_utility_RAC_approx)
%
% mean_delta_LDF = mean(Rec_delta_LDF)
% mean_delta_EPDF = mean(Rec_delta_EPDF)
% mean_delta_LLDF = mean(Rec_delta_LLDF)
% mean_delta_RAC = mean(Rec_delta_RAC)
% mean_delta_RAC_approx = mean(Rec_delta_RAC_approx)
%
% fprintf(fileID, '\n mean_delta_utility_RAC=%f, \n mean_delta_utility_RAC_approx=%f', ...
%     mean_delta_utility_RAC, mean_delta_utility_RAC_approx);
% fprintf(fileID, '\n mean_delta_RAC=%f,  \n mean_delta_LLDF=%f, mean_delta_LDF=%f, \n mean_delta_RAC_approx=%f,\n mean_delta_EPDF=%f\n', ...
%     mean_delta_RAC, mean_delta_LLDF, mean_delta_LDF, mean_delta_RAC_approx, mean_delta_EPDF);

%save(sprintf('%s/comparision_scheduling_flow_%d.mat',filePath,next_conf));






