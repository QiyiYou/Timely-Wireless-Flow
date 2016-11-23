clear all; close all; 
n_flow = 2;
%n_flow = 3;

filePath = sprintf('fig/test_balance/n_flow=%d',n_flow);
fileID = fopen(sprintf('%s/test_balance.txt',filePath),'w');

flow1 = FlowInstance();
flow1.offset = 0;
flow1.period = 1;
flow1.delay = 2;
flow1.arrival_prob = 0.2;
flow1.success_prob = 0.4;
flow1.constructEverything();

flow2 = FlowInstance();
flow2.offset = 0;
flow2.period = 1;
flow2.delay = 2;
flow2.arrival_prob = 0.2;
flow2.success_prob = 0.6;
flow2.constructEverything();

flow3 = FlowInstance();
flow3.offset = 0;
flow3.period = 1;
flow3.delay = 2;
flow3.arrival_prob = 0.2;
flow3.success_prob = 0.7;
flow3.constructEverything();

flow_array = cell(n_flow,1);
flow_array{1} = flow1;
flow_array{2} = flow2;
%flow_array{3} = flow3;

for ii=1:n_flow
    fprintf(fileID, 'Flow %d: (offset, period, delay, arrival_prob, success_prob) = (%d, %d, %d, %f, %f)\n', ii, flow_array{ii}.offset, ...
        flow_array{ii}.period, flow_array{ii}.delay,flow_array{ii}.arrival_prob,flow_array{ii}.success_prob);
end

obj = DownlinkAPInstance();
obj.n_flow = n_flow;
obj.flow_array = flow_array;
obj.constructEverything();
obj.stateSanityCheck();

%utility_coeff = [1,1,1];
utility_coeff = [1,1];
%utility_coeff = utility_coeff./sum(utility_coeff);
utility_form = 'weighted_sum';
%utility_form = 'weighted_log_sum';

fprintf(fileID, '%s, w=utility_coeff=[', utility_form);
for ii=1:n_flow
    fprintf(fileID, '%f,',utility_coeff(ii));
end
fprintf(fileID, ']\n');

tic;
[optimal_policy_RAC, optimal_utility_RAC, optimal_throughput_per_flow_RAC] = ...
    getOptimalSolutionRAC(obj, utility_coeff, utility_form);
fprintf(fileID, '\nFinish getOptimalSolutionRAC with time %f seconds\n', toc);

tic;
[optimal_policy_RAC_approx, optimal_action_distribution_RAC_approx, optimal_utility_RAC_approx, optimal_throughput_per_flow_RAC_approx] = ...
    getApproximateSolutionRAC(obj, utility_coeff, utility_form);
fprintf(fileID, 'Finish getApproximateSolutionRAC with time %f seconds\n', toc);


tic;
[optimal_policy_balance_primal, optimal_utility_balance_primal, optimal_delta_balance_primal] = ...
    getBalancePrimalSolution(obj, utility_coeff, utility_form);
fprintf(fileID, 'Finish getBalancePrimalSolution with time %f seconds\n', toc);

tic;
[optimal_lambda_balancd_dual, optimal_h_balancd_dual, optimal_muu_balancd_dual, optimal_utility_balance_dual, optimal_potential_change_not_schedule, optimal_potential_change_schedule] ...
    = getBalanceDualSolution(obj, utility_coeff, utility_form);
fprintf(fileID, 'Finish getBalanceDualSolution with time %f seconds\n', toc);



optimal_utility_RAC
optimal_utility_RAC_approx
optimal_utility_balance_primal
optimal_delta_balance_primal
optimal_utility_balance_dual
optimal_lambda_balancd_dual
optimal_h_balancd_dual
optimal_muu_balancd_dual
optimal_potential_change_not_schedule
optimal_potential_change_schedule

%print information to the fildID

fprintf(fileID, '\noptimal_utility_RAC=%f\n', optimal_utility_RAC);
fprintf(fileID, 'optimal_utility_RAC_approx=%f\n', optimal_utility_RAC_approx);
fprintf(fileID, 'optimal_utility_balance_primal=%f\n', optimal_utility_balance_primal);
fprintf(fileID, 'optimal_utility_balance_dual=%f\n', optimal_utility_balance_dual);

fprintf(fileID, 'delta=%f\n', optimal_delta_balance_primal);
fprintf(fileID, 'lambda=%f\n', optimal_lambda_balancd_dual);
for kk=1:obj.n_flow
    fprintf(fileID, 'h(%d)=%f\n', kk, optimal_h_balancd_dual(kk));
end

%print individual flow information
for kk=1:obj.n_flow
    fprintf(fileID, '\n===============================================================================================================\n');
    fprintf(fileID,'Flow %d, lambda=%f, h%d=%f, lambda+h%d=%f\n', kk,optimal_lambda_balancd_dual,kk, ...
        optimal_h_balancd_dual(kk), kk,optimal_lambda_balancd_dual+optimal_h_balancd_dual(kk));
    %print transition matrix, aa=0 is not to schedule this flow, aa=1 is to
    %schedule this flow.
    for aa=0:1
        if(aa==0)
            fprintf(fileID, '\nTransition matrix for a%d=0, not schedule\n', kk);
        elseif (aa==1)
            fprintf(fileID, '\nTransition matrix for a%d=1, schedule\n', kk);
        end
            
        %print the first line
        fprintf(fileID, '%8s', '-');
        for ss=1:obj.flow_array{kk}.n_state
            ss_binary=obj.flow_array{kk}.getBinaryState(ss);
            state_combine=sprintf('%d=(%s)',ss, num2str(ss_binary));
            fprintf(fileID, ', %12s', state_combine);
        end
        fprintf(fileID, '\n');
        %print other lines
        for ss=1:obj.flow_array{kk}.n_state
            ss_binary=obj.flow_array{kk}.getBinaryState(ss);
            state_combine=sprintf('%d=(%s)',ss, num2str(ss_binary));
            fprintf(fileID, '%8s', state_combine);
            for next_ss=1:obj.flow_array{kk}.n_state
                if(aa==0)
                    prob_temp = obj.flow_array{kk}.transition_matrix(1,ss,2,next_ss);
                elseif(aa==1)
                    prob_temp = obj.flow_array{kk}.transition_matrix(1,ss,1,next_ss);
                end
                fprintf(fileID, ', %12f', prob_temp);
            end
            fprintf(fileID, '\n');
        end
    end
    
   
    
    %balance-primal, balance-dual information
    fprintf(fileID, '\nBalance-primal, Balance-dual information\n');
    state = sprintf('s%d', kk);
    reward = sprintf('w%d*r(%s,1)', kk, state);
    z0 = sprintf('z(%s,0)', state);
    z1 = sprintf('z(%s,1)', state);
    muu = sprintf('mu(%s)', state);
    DeltaPhi0 = sprintf('DeltaPhi(%s,0)', state);
    DeltaPhi1 = sprintf('DeltaPhi(%s,1)', state);
    reward_plus_DeltaPhi1 = sprintf('%s+%s', reward, DeltaPhi1);
    fprintf(fileID, '%8s, %12s, %12s, %12s, %12s, %15s, %15s, %25s\n', state, reward, z0, z1, muu, DeltaPhi0, DeltaPhi1,reward_plus_DeltaPhi1);
    for ss=1:obj.flow_array{kk}.n_state
        ss_binary=obj.flow_array{kk}.getBinaryState(ss);
        state_combine=sprintf('%d=(%s)',ss, num2str(ss_binary));
        ss_reward_schedule = obj.flow_array{kk}.reward_per_state_per_action(ss,1);
        fprintf(fileID, '%8s, %12f, %12f, %12f, %12f, %15f, %15f, %25f\n', state_combine, ss_reward_schedule, optimal_policy_balance_primal(kk,ss,2),  ...
                optimal_policy_balance_primal(kk,ss,1), optimal_muu_balancd_dual(kk,ss), optimal_potential_change_not_schedule(kk,ss), ...
                optimal_potential_change_schedule(kk,ss), ss_reward_schedule+optimal_potential_change_schedule(kk,ss));
    end
end

fclose(fileID);


