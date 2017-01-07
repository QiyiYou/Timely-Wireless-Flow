function [successful_transmission, ...
      state_action_distribution, system_state, system_action, state_action_per_slot ] ...
    = RACSchedule_old(obj, T, optimal_policy_RAC)
% the largest-queue-first-scheduling algorithm
% T is the number of slots to simulate 
% optimal_policy_RAC is the optimal RAC policy obtained by solve RAC
% problem.

% optimal_policy_RAC the joint distribution: y(obj.period_lcm, obj.n_state, obj.n_action)
% let us convert it to the conditional distribution first

optimal_policy_RAC_cond = zeros(obj.period_lcm, obj.n_state, obj.n_action);
for tt=1:obj.period_lcm
    for ss=1:obj.n_state
        state_prob_temp = sum(squeeze(optimal_policy_RAC(tt,ss,:)));
        if(state_prob_temp == 0)
            optimal_policy_RAC_cond(tt, ss, :) = 0;
        else
            optimal_policy_RAC_cond(tt,ss, :) = optimal_policy_RAC(tt,ss,:)./state_prob_temp;
        end
    end
end


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
    
    action_prob = zeros(1,obj.n_action);
    for aa=1:obj.n_action
        action_prob(aa) = optimal_policy_RAC_cond(obj.getFirstPeriodSlot(tt),current_state, aa);
    end
    
    optimal_action = -1;
    prob_temp = rand;
    for aa=1:obj.n_action
        if(aa == 1)
            if(prob_temp <= sum(action_prob(1)))
                optimal_action = 1;
                break;
            end
        else
            % aa >= 2
            if( prob_temp <= sum(action_prob(1:aa))  && ...
                prob_temp > sum(action_prob(1:aa-1)))
                optimal_action = aa;
                break;
            end
        end
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