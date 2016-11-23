function [successful_transmission, ...
    state_action_distribution, system_state, system_action, state_action_per_slot] ...
    = fullDualSchedule(obj, T, optimal_policy_full_dual)

%% 
% use the full-dual distributed algorithm to get the
% optimal utility, we can use either weighted sum or weithed log sum here
% obj: DownlinkAPInstance
% utility_coeff: the coefficient for each flow

% utility_form is a string
%     utility_form = 'weighted_sum' means the weighted sum utility with utility_coeff
%     utility_form = 'weighted_log_sum' means the weighted log sum utility with utility_coeff

%Note that we use the vectorized version

if( size(optimal_policy_full_dual,3) < T)
    error('wrong input');
end

%% system observatoins for half-dual scheduling
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


%% loop

for tt=1:T
    if(mod(tt,1000) == 0)
        fprintf('tt=%d\n', tt);
    end
    
    %get the current state
    current_state = system_state(tt);
    
    action_prob = zeros(1,obj.n_action);
    for aa=1:obj.n_action
        idx = obj.getIdxFromStateAction(current_state,aa);
        action_prob(aa) = optimal_policy_full_dual(idx, obj.getFirstPeriodSlot(tt), tt);
    end
    
    action_prob = action_prob./sum(action_prob);
    
    
    optimal_action = -1;
    prob_temp = rand;
    for aa=1:obj.n_action
        if(aa == 1)
            if(prob_temp <= sum(action_prob(1)))
                optimal_action = 1;
                break;
            end
        end
        % aa >= 2
        if( prob_temp <= sum(action_prob(1:aa))  && ...
                prob_temp > sum(action_prob(1:aa-1)))
            optimal_action = aa;
            break;
        end
    end
    if(optimal_action == -1)
       % error('something wrong');
       optimal_action = randi([1,obj.n_action]);
    end
    
    
    
    %real system evolution
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




end