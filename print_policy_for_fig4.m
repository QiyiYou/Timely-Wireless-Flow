cvx_solver SeDuMi
cvx_save_prefs
cvx_precision best


n_flow = 3;


flow1 = NonOverlappedFlowInstance();
flow1.offset = 0;
flow1.period = 4;
flow1.delay = 4;
flow1.arrival_prob = 1;
flow1.success_prob = 0.5;
flow1.constructEverything();

flow2 = NonOverlappedFlowInstance();
flow2.offset = 2;
flow2.period = 4;
flow2.delay = 4;
flow2.arrival_prob = 1;
flow2.success_prob = 0.5;
flow2.constructEverything();

%iid traffic
flow3 = FlowInstance();
flow3.offset = 0;
flow3.period = 1;
flow3.delay = 3;
flow3.arrival_prob =  0.9;
flow3.success_prob = 0.7;
flow3.constructEverything();

flow_array = cell(n_flow,1);
flow_array{1} = flow1;
flow_array{2} = flow2;
flow_array{3} = flow3;


obj = DownlinkAPInstance();
obj.n_flow = n_flow;
obj.flow_array = flow_array;
obj.constructEverything();
obj.stateSanityCheck();

utility_coeff =  100*[1,1,1];
%utility_coeff = utility_coeff./sum(utility_coeff);
utility_form = 'weighted_log_sum';

tic;
[optimal_policy_RAC, optimal_utility_RAC, optimal_throughput_per_flow_RAC] = ...
    getOptimalSolutionRAC(obj, utility_coeff, utility_form);
fprintf('\nFinish getOptimalSolutionRAC with time %f seconds\n', toc);



tic;
[optimal_policy_RAC_approx, optimal_action_distribution_RAC_approx, optimal_utility_RAC_approx, optimal_throughput_per_flow_RAC_approx] = ...
    getApproximateSolutionRAC(obj, utility_coeff, utility_form);
fprintf('\nFinish getApproximateSolutionRAC with time %f seconds\n', toc);



%% compare RAC and RAC_approx
fileID = fopen('print_RAC_policy.tex','w');

fprintf(fileID, '& \\multicolumn{3}{c|}{$t=T_1=9$} & \\multicolumn{3}{c|}{$t=10$} & \\multicolumn{3}{c|}{$t = 11$} & \\multicolumn{3}{c|}{$t= T_2 = 12$} \\\\ \\hline \n');

fprintf(fileID, ' State $s = (s^1, s^2, s^3)$  & ');
for tt=1:obj.period_lcm
    for aa=1:obj.n_action
        if(tt == obj.period_lcm && aa == obj.n_action)
            fprintf(fileID, '$A=%d$ \\\\ \\hline \n ', aa);
        else
            fprintf(fileID, '$a=%d$ & ', aa);
        end
    end
end

for ss=1:obj.n_state
        ss_vec = obj.getVectorState(ss);
    for tt=1:obj.period_lcm    
        fprintf('tt=%d, ss=%d, ss_vec=[', tt, ss);
        for kk=1:obj.n_flow
            fprintf('%d, ', ss_vec(kk));
        end
        fprintf(']\n');
        
        action_prob_RAC = zeros(1, obj.n_flow);
        state_prob_temp = sum(squeeze(optimal_policy_RAC(tt,ss,:)));
        if(state_prob_temp < 1e-7)
            action_prob_RAC(:) = 0;
        else
            action_prob_RAC(:) = optimal_policy_RAC(tt,ss,:)./state_prob_temp;
        end
        
        action_prob_RAC_approx_per_flow = zeros(obj.n_flow, obj.n_action);
        for kk=1:obj.n_flow
            action_prob_RAC_approx_per_flow(kk, :) = squeeze(optimal_policy_RAC_approx(kk, tt, ss_vec(kk),:))'/sum(optimal_policy_RAC_approx(kk, tt, ss_vec(kk),:));
            fprintf('action_prob_RAC_approx_per_flow(%d,:)=[', kk);
            for aa=1:obj.n_action
                fprintf('%f, ', action_prob_RAC_approx_per_flow(kk, aa));
            end
            fprintf(']\n');
        end
        
        action_prob_RAC_approx = ones(1, obj.n_flow);
        for aa=1:obj.n_action
            for kk=1:obj.n_flow
                state_aa_prob_temp = optimal_policy_RAC_approx(kk, tt, ss_vec(kk),aa);
                state_prob_temp = sum(optimal_policy_RAC_approx(kk, tt, ss_vec(kk),:));
                if(state_prob_temp < 1e-7) %flow k will not be in this state
                    action_prob_RAC_approx(aa) = 0;
                    break;
                else
                    action_prob_RAC_approx(aa) = action_prob_RAC_approx(aa)*10*state_aa_prob_temp/state_prob_temp; %note that we scale it up by 10x to avoid small action_prob
                end
            end
        end
        
        
        %normalized to conditional distribution
        if(sum(action_prob_RAC_approx) < 1e-9) %this is empty state, and sum(action_prob) = 0
            %action_prob_RAC_approx = ones(1,obj.n_action)/obj.n_action;
            action_prob_RAC_approx(:) = 0;
        else
            action_prob_RAC_approx = action_prob_RAC_approx./sum(action_prob_RAC_approx);
        end
        
        
        fprintf('action_prob=           [');
        for kk=1:obj.n_flow
            fprintf('%f, ', action_prob_RAC(kk));
        end
        fprintf(']\n');
        
        fprintf('action_prob_RAC_approx=[');
        for kk=1:obj.n_flow
            fprintf('%f, ', action_prob_RAC_approx(kk));
        end
        fprintf(']\n');
        
        if(tt==1)
            fprintf(fileID, '(');
            if(ss_vec(1) == 1)
                fprintf(fileID, '0, ');
            else
                fprintf(fileID, '1, ');
            end
            
            if(ss_vec(2) == 1)
                fprintf(fileID, '0, ');
            else
                fprintf(fileID, '1, ');
            end
            
            state_bin_temp  = obj.flow_array{3}.getBinaryState(ss_vec(3));
            fprintf(fileID, '%d%d%d) & ', state_bin_temp(1), state_bin_temp(2), state_bin_temp(3));
        end

        
        for kk=1:obj.n_flow
            if(tt==obj.period_lcm &&  kk == obj.n_flow)
                if(sum(action_prob_RAC) < 1e-9)
                    fprintf(fileID, '- \\\\ \\hline \n');
                else
                    fprintf(fileID, '%.2f \\\\ \\hline \n', action_prob_RAC(kk));
                end
            else
                if(sum(action_prob_RAC) < 1e-9)
                    fprintf(fileID, '- & ');
                else
                    fprintf(fileID, '%.2f & ', action_prob_RAC(kk));
                end
            end
        end
    end 
end

fclose(fileID);