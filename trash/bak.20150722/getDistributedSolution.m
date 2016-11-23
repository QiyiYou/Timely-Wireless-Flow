function [empirical_utility, empirical_throughput, state_action_distribution] = getDistributedSolution(obj, utility_coeff, utility_form, T)
% use the distributed algorithm to get the
% optimal utility, we can use either weighted sum or weithed log sum here
% obj: DownlinkAPInstance
% utility_coeff: the coefficient for each flow

% utility_form is a string
%     utility_form = 'weighted_sum' means the weighted sum utility with utility_coeff
%     utility_form = 'weighted_log_sum' means the weighted log sum utility with utility_coeff


% the achieved state_action_distribution for each slot (in a period_lcm)
% this is used to compare with the global optimal RAC scheme, i.e., the
% joint distribution y in function getOptimalSolutionRAC.
state_action_distribution = zeros(obj.period_lcm, obj.n_state, obj.n_action);

%the primal variable R_k for each flow k, primal vairable updates according to 
% R^*_k = \arg \max U_k(R_k) - q_k R_k;
primal_rate_user = zeros(obj.n_flow, T);

% the dual variable q_k  for each flow k 
% queue envolves as 
%     virtual_queue(t+1) = max( virtual_queue(t)-virtual_server_capacity(t), 0) + virtual_arrival(t+1);
dual_queue_user = zeros(obj.n_flow, T);


% physical system state
system_state = zeros(1,T);
system_state(1) = obj.getInitialState();
system_action = zeros(1,T);
transmitted_packet = zeros(obj.n_flow,T);

% empirical_throughput(i, t) is the empirical (normalized)
% timely throughput of flow i up to slot t (i.e., from slot 1 to slot t);
empirical_throughput = zeros(obj.n_flow,T);

% empirical_utility(t) is the total utility based on the empricial
% throughput of each flow up to slot t (i.e., from slot 1 to slot t);
empirical_utility = zeros(1,T);

for tt=1:T
    %primal  variable update
    
    %update primal variable R_k
    %suppose weighted log sum utility, U_k(R_k) = utility_coeff(kk) * log R_k
    %suppose weighted sum utility, U_k(R_k) = utility_coeff(kk) * R_k
    
    for kk=1:obj.n_flow
        if(isequal(utility_form, 'weighted_sum'))
            %weighted sum utility
            if(utility_coeff(kk)-dual_queue_user(kk,tt) >= 0)
                primal_rate_user(kk,tt) = 1;
            else
                primal_rate_user(kk,tt) = 0;
            end
        elseif(isequal(utility_form, 'weighted_log_sum'))
            %weighted log sum utility
            primal_rate_user(kk,tt) = min(1, utility_coeff(kk)/dual_queue_user(kk,tt));
        else
            error('wrong input utility_form, can only be ''weighted_sum'' or ''weighted_log_sum''');
        end
    end
    
    %construct the optimal action according to the current state 
    %get the largest queue
    current_state = system_state(tt);
    largest_queue_idx = -1;
    largest_queue_length = -1;
    for qq=1:obj.n_flow
        % b_packet=0 if no packet for flow qq, 1 if one packet for flow qq
        b_packet = hasPacket(obj, current_state, qq);
        % note that here we sholud consider normarlized timely throughput
        if(b_packet*dual_queue_user(qq,tt)*obj.success_prob(qq)/(obj.period_lcm/obj.period(qq)) > largest_queue_length)
            largest_queue_idx = qq;
            largest_queue_length = b_packet*dual_queue_user(qq,tt)*obj.success_prob(qq)/(obj.period_lcm/obj.period(qq));
        end
    end

    system_action(tt) = largest_queue_idx;
    
    %update the state_action_distribution, just count here, we will
    %normalize it to a distribtuion after all simulated slots
    state_action_distribution(getFirstPeriodSlot(obj,tt), current_state, largest_queue_idx) = ...
        state_action_distribution(getFirstPeriodSlot(obj,tt), current_state, largest_queue_idx) + 1;
    
    %realization of randomness. Note that randomeness only comes from TX and the virtual server 
    %capacity is also only related to TX. A&E is deterministic once TX is fixed. 
    %Therefore, we only need to consider realization of TX randomness. 
    
    next_state_tx_probability = squeeze(obj.transition_matrix_tx(current_state, largest_queue_idx,:));
    probability_temp = rand;
    next_state_tx = -1;
    for ss=1:obj.n_state
        if(ss == 1)
            if(probability_temp <= sum(next_state_tx_probability(1)))
                next_state_tx = 1;
                break;
            end
        end    
        % ss >= 2
        if( probability_temp <= sum(next_state_tx_probability(1:ss))  && ...
            probability_temp > sum(next_state_tx_probability(1:ss-1)))
                next_state_tx = ss;
                break;
        end
    end
    
    %Rembmer that transmitted packet is not related to
    %the A&E but only TX
    current_state_bin = getBinaryState(obj, current_state);
    next_state_tx_bin = getBinaryState(obj, next_state_tx);
    if(current_state_bin(largest_queue_idx) == 1 && next_state_tx_bin(largest_queue_idx) == 0)
        transmitted_packet(largest_queue_idx,tt) = 1;
    end
    
    %update the empirical throughput and the empirical utility
    for kk=1:obj.n_flow
        empirical_throughput(kk, tt) = sum(transmitted_packet(kk,1:tt))/(tt/obj.period(kk));
        if(isequal(utility_form, 'weighted_sum'))
            empirical_utility(tt) =  empirical_utility(tt) + utility_coeff(kk)*(empirical_throughput(kk,tt));
        elseif(isequal(utility_form, 'weighted_log_sum'))
            empirical_utility(tt) =  empirical_utility(tt) + utility_coeff(kk)*log(empirical_throughput(kk,tt));
        else
            error('wrong input utility_form, can only be ''weighted_sum'' or ''weighted_log_sum''');
        end
    end
    
    %we have finished the last slot
    if(tt == T)
        break;
    end
    
    %consider A&E here to get the actual next_state
    next_state = -1;
    for ss=1:obj.n_state
         if(obj.transition_matrix_ae(getFirstPeriodSlot(obj,tt), next_state_tx, ss) == 1)
             next_state = ss; 
             break;
         end
    end
    
    if(next_state == -1)
        error('something wrong');
    end
    
    system_state(tt+1) = next_state;
  
    
    %update dual variablel queue length
    step = 0.01;
    for kk=1:obj.n_flow
        % the following update works, but the convergence is not so good as the
        % second update
%         delta_change = step* max(0,  primal_rate_user(kk,tt) - empirical_throughput(kk, tt));
%         dual_queue_user(kk,tt+1) = dual_queue_user(kk,tt) + delta_change;
        % the following works
        dual_queue_user(kk,tt+1) = max(0, ...
                      dual_queue_user(kk,tt) - step*( empirical_throughput(kk, tt) - primal_rate_user(kk,tt)));
    end
end

%normalize the state_action_distribution
state_action_distribution = state_action_distribution/(T/obj.period_lcm);

end