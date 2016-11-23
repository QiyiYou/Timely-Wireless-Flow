function [successful_transmission, ...
      state_action_distribution, system_state, system_action, state_action_per_slot ] ...
    = FiniteMDPSolutionSchedule(obj, T, optimal_policy_finite_MDP)
% the scheduling algorithm based on the solution of the finite_MDP, here we
% require that the utility_form is weighted-sum
% T is the number of slots to simulate 
% optimal_policy_finite_MDP is the optimal finite-MDP policy obtained by
% solve a large-period finite-horizon MDP

% optimal_policy_finite_MDP optimal_policy_finite_MDP(obj.period_lcm, obj.n_state)
% where optimal_policy_finite_MDP(t, ss) = a means to choose action a when
% the state is ss at time t.

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
    
    optimal_action = optimal_policy_finite_MDP(obj.getFirstPeriodSlot(tt), current_state);

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