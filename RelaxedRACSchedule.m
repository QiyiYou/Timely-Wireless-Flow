function [successful_transmission, ...
    state_action_distribution, system_state, system_action, state_action_per_slot ] ...
    = RelaxedRACSchedule(obj, T, optimal_policy_RAC_relax, optimal_action_distribution_relax)
% the relaxed-RAC algorithm
% T is the number of slots to simulate
% optimal_policy_RAC_relax is the optimal relaxed-RAC policy obtained by solve
% the relaxed-RAC problem.
% optimal_action_distribution is the optimal action distribution over one
% large period

% optimal_policy_RAC_relax the joint distribution: z(obj.n_flow, obj.period_lcm, max_n_state, obj.n_action)
% optimal_action_distribution_relax is the marginal distribution z_action(obj.period_lcm, obj.n_action)

% the achieved state_action_distribution for each slot (in a period_lcm)
% this is used to compare with the global optimal RAC scheme, i.e., the
% joint distribution y in function getOptimalSolutionRAC.
state_action_distribution = zeros(obj.period_lcm, obj.n_state, obj.n_action);

state_action_per_slot = zeros(obj.n_state, obj.n_action,T);

% physical system state
system_state = zeros(1,T);
system_state(1) = obj.getInitialState();
system_action = zeros(1,T);
successful_transmission = zeros(obj.n_flow,T);

for tt=1:T
    %get the largest queue
    current_state = system_state(tt);
    current_state_vec = obj.getVectorState(current_state);
    
    action_prob = ones(1,obj.n_action);
    %we need to look up the individual marginal distribution tables to get
    %the conditional probability
    %     for aa=1:obj.n_action
    %         z_ta = optimal_action_distribution_relax(obj.getFirstPeriodSlot(tt),aa);
    %         if( z_ta == 0)
    %             action_prob(aa) = 0;
    %         else
    %             for kk=1:obj.n_flow
    %                 action_prob(aa) = action_prob(aa)*optimal_policy_RAC_relax(kk, obj.getFirstPeriodSlot(tt),current_state_vec(kk), aa)/(z_ta^(obj.n_flow-1));
    %             end
    %         end
    %     end
    for aa=1:obj.n_action
        first_period_slot = obj.getFirstPeriodSlot(tt);
        for kk=1:obj.n_flow
            state_aa_prob_temp = optimal_policy_RAC_relax(kk, first_period_slot, current_state_vec(kk),aa);
            state_prob_temp = sum(optimal_policy_RAC_relax(kk, first_period_slot, current_state_vec(kk),:));
            if(state_prob_temp < 1e-9) %flow k will not be in this state
                action_prob(aa) = 0;
                break;
            else
                action_prob(aa) = action_prob(aa)*state_aa_prob_temp/state_prob_temp;
             end
        end
    end
    %normalized to conditional distribution
    if(sum(action_prob) < 1e-9) %this is empty state, and sum(action_prob) = 0
        action_prob = ones(1,obj.n_action)/obj.n_action;
    else
        action_prob = action_prob./sum(action_prob);
    end
    
    
    optimal_action = -1;
    prob_temp = rand;
    for aa=1:obj.n_action
        if(aa == 1)
            if(prob_temp <= sum(action_prob(1)))
                optimal_action = 1;
                break;
            end
        else % aa >= 2
            if( prob_temp <= sum(action_prob(1:aa))  && ...
                prob_temp > sum(action_prob(1:aa-1)))
                optimal_action = aa;
                break;
            end
        end
    end
    
    %for debug purpose
    if(current_state_vec(optimal_action) == 1 && ~isequal(current_state_vec, ones(size(current_state_vec))))
        fprintf('schedule an empty flow, but has a non-empty flow, non-work-conserving\n');
        fprintf('sum(action_prob(current_state_vec == 1)=%f\n', sum(action_prob(current_state_vec == 1)));
    end
    
    if(optimal_action == -1)
        error('something wrong');
    end
    
    system_action(tt) = optimal_action;
    
    [next_state, isTransmitted_vec, isSuccessful_vec] = obj.oneSlotRealization(tt,current_state,optimal_action);
    
    %update the state_action_distribution, just count here, we will
    %normalize it to a distribtuion after all simulated slots
    state_action_distribution(obj.getFirstPeriodSlot(tt), current_state, optimal_action) = ...
        state_action_distribution(obj.getFirstPeriodSlot(tt), current_state, optimal_action) + 1;
    
    state_action_per_slot(current_state, optimal_action, tt) = 1;
    
    if(isSuccessful_vec(optimal_action) == 1)
        successful_transmission(optimal_action,tt) = 1;
    end
    
    %we have finished the last slot
    if(tt == T)
        break;
    end
    
    system_state(tt+1) = next_state;
end

%normalize the state_action_distribution
state_action_distribution = state_action_distribution/(T/obj.period_lcm);

end