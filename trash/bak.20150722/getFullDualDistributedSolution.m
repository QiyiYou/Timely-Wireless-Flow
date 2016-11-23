function [theoretical_utlity, theoretical_throughput, empirical_utility, empirical_throughput, primal_state_action_distribution] ...
    = getFullDualDistributedSolution(obj, utility_coeff, utility_form, T)
% use the full-dual distributed algorithm to get the
% optimal utility, we can use either weighted sum or weithed log sum here
% obj: DownlinkAPInstance
% utility_coeff: the coefficient for each flow

% utility_form is a string
%     utility_form = 'weighted_sum' means the weighted sum utility with utility_coeff
%     utility_form = 'weighted_log_sum' means the weighted log sum utility with utility_coeff


% the achieved state_action_distribution for each slot (in a period_lcm)
% this is used to compare with the global optimal RAC scheme, i.e., the
% joint distribution y in function getOptimalSolutionRAC.
% MOREOVER, in full-dual algorithm, we use this empirical distribution to
% make decions, so this is the primal variable which should be updated in
% every iteration
primal_state_action_distribution = zeros(obj.period_lcm, obj.n_state, obj.n_action, T);


%the primal variable R_k for each flow k, primal vairable updates according to 
% R^*_k = \arg \max U_k(R_k) - q_k R_k;
primal_rate_user = zeros(obj.n_flow, T);

% the dual variable q_k  for each flow k 
% queue envolves as 
%     virtual_queue(t+1) = max( virtual_queue(t)-virtual_server_capacity(t), 0) + virtual_arrival(t+1);
dual_queue_user = zeros(obj.n_flow, T);

%the dual variable for 3b) in the draft, i.e., the system randomness
%causaulity inequality from slot 1 to slot preiod_lcm - 1
dual_lambda = zeros(obj.n_state, obj.period_lcm-1, T);

%the dual variable for 3c) in the draft, i.e., the system randomness
%causaulity inequality for final slot preiod_lcm 
dual_mu = zeros(obj.n_state,T);

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
    
    %update primal variable y. Since here we use full-dual distribution
    %algorithm. We need to get the optimal state-action pair and then
    %update the emprical state-action discitrion, which is the udpated y
    primal_optimal_state = zeros(obj.period_lcm,1);
    primal_optimal_action = zeros(obj.period_lcm,1);
    %first update from slot 1 to slot obj.period_lcm
    for ii=1:obj.period_lcm
        max_temp_val = -inf;
        temp_val = 0;
        for ss=1:obj.n_state
            for aa=1:obj.n_action
                for next_ss=1:obj.n_state
                    %distingush slot obj.period_lcm
                    if(ii==obj.period_lcm)
                        temp_val = temp_val + dual_mu(next_ss, tt)*obj.transition_matrix(ii, ss,aa,next_ss);
                    else
                        temp_val = temp_val + dual_lambda(next_ss, ii, tt)*obj.transition_matrix(ii, ss,aa,next_ss);
                    end
                        
                end
                %distingush slot 1
                if(ii == 1)
                    temp_val = temp_val - dual_mu(ss, tt);
                else
                    temp_val = temp_val - dual_lambda(ss, ii-1, tt);
                end
                
                for kk=1:obj.n_flow
                    temp_val = temp_val + dual_queue_user(kk,tt)*obj.reward_per_state_per_action(ss,aa)/(obj.period_lcm/obj.period(kk));
                end
                if(temp_val > max_temp_val)
                    primal_optimal_state(ii) = ss;
                    primal_optimal_action(ii) = aa;
                    max_temp_val = temp_val;
                end
            end
        end
    end

    %update the emprical state-action discitrion, which is the udpated y
    if( tt == 1)
        % initial iteration
        for ii=1:obj.period_lcm
            ss = primal_optimal_state(ii);
            aa = primal_optimal_action(ii);
            primal_state_action_distribution(ii, ss, aa, tt) = 1;
        end
    else
        primal_state_action_distribution(:, :, :, tt) =  (tt-1)*primal_state_action_distribution(:, :, :, tt-1)/tt;
        for ii=1:obj.period_lcm
            ss = primal_optimal_state(ii);
            aa = primal_optimal_action(ii);
            primal_state_action_distribution(ii, ss, aa, tt) = primal_state_action_distribution(ii, ss, aa, tt) + 1/tt;
        end
    end
    
    
%     %construct the optimal action according to the current state 
%     %get the largest queue
%     current_state = system_state(tt);
%     largest_queue_idx = -1;
%     largest_queue_length = -1;
%     for qq=1:obj.n_flow
%         % b_packet=0 if no packet for flow qq, 1 if one packet for flow qq
%         b_packet = hasPacket(obj, current_state, qq);
%         % note that here we sholud consider normarlized timely throughput
%         if(b_packet*dual_queue_user(qq,tt)*obj.success_prob(qq)/(obj.period_lcm/obj.period(qq)) > largest_queue_length)
%             largest_queue_idx = qq;
%             largest_queue_length = b_packet*dual_queue_user(qq,tt)*obj.success_prob(qq)/(obj.period_lcm/obj.period(qq));
%         end
%     end
% 
%     system_action(tt) = largest_queue_idx;
    
%     %update the state_action_distribution, just count here, we will
%     %normalize it to a distribtuion after all simulated slots
%     state_action_distribution(getFirstPeriodSlot(obj,tt), current_state, largest_queue_idx) = ...
%         state_action_distribution(getFirstPeriodSlot(obj,tt), current_state, largest_queue_idx) + 1;
    
%     %real scheduling, base on on the conditional action distribution
%     %let us mute it first
    current_state = system_state(tt);
    randomized_distribution = squeeze(primal_state_action_distribution(getFirstPeriodSlot(obj,tt), current_state, :, tt));
    if (sum (randomized_distribution) > 0)
        randomized_distribution = randomized_distribution / sum (randomized_distribution);
    else
        randomized_distribution = (1/obj.n_action)*ones(obj.n_action,1);
    end
    
    probability_temp = rand;
    random_action = -1;
    for ss=1:obj.n_state
        if(ss == 1)
            if(probability_temp <= sum(randomized_distribution(1)))
                random_action = 1;
                break;
            end
        end
        % ss >= 2
        if( probability_temp <= sum(randomized_distribution(1:ss))  && ...
                probability_temp > sum(randomized_distribution(1:ss-1)))
            random_action = ss;
            break;
        end
    end
    system_action(tt) = random_action;

    %realization of randomness. Note that randomeness only comes from TX and the virtual server 
    %capacity is also only related to TX. A&E is deterministic once TX is fixed. 
    %Therefore, we only need to consider realization of TX randomness. 
    
    next_state_tx_probability = squeeze(obj.transition_matrix_tx(current_state, random_action,:));
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
    if(current_state_bin(random_action) == 1 && next_state_tx_bin(random_action) == 0)
        transmitted_packet(random_action,tt) = 1;
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
  
    
    %update dual variablel queue length. Note that we should not use the
    %emprical distribution, instead, we use the instantenous y, i.e.,
    %primal_optimal_state and primal_optimal_action
    step = 0.001;
    for kk=1:obj.n_flow
        % the following update works, but the convergence is not so good as the
        % second update
        %         delta_change = step* max(0,  primal_rate_user(kk,tt) - empirical_throughput(kk, tt));
        %         dual_queue_user(kk,tt+1) = dual_queue_user(kk,tt) + delta_change;
        % the following works
        change_temp = 0;
        for ii=1:obj.period_lcm
            primal_ss = primal_optimal_state(ii);
            primal_aa = primal_optimal_action(ii);
            change_temp = change_temp + obj.reward_per_state_per_action(primal_ss,primal_aa)/(obj.period_lcm/obj.period(kk));
        end
        change_temp = change_temp - primal_rate_user(kk,tt);
        dual_queue_user(kk,tt+1) = max(0, ...
            dual_queue_user(kk,tt) - step*(change_temp));
        
        %         dual_queue_user(kk,tt+1) = max(0, ...
        %                       dual_queue_user(kk,tt) - step*( empirical_throughput(kk, tt) - primal_rate_user(kk,tt)));
    end
    
    %update the dual varialbe dual_lambda and dual_mu
    % dual_lambda from slot 1 to slot obj.period_lcm-1
    for ii=1:obj.period_lcm-1
        for ss=1:obj.n_state
            primal_ss = primal_optimal_state(ii);
            primal_aa = primal_optimal_action(ii);
            change_temp = 0;
            change_temp = change_temp + obj.transition_matrix(ii, primal_ss,primal_aa,ss);
            primal_next_ss = primal_optimal_state(ii+1);
            if(primal_next_ss == ss)
                change_temp = change_temp - 1;
            end
            %%...
            dual_lambda(ss,ii, tt+1) = max(0, ...
                dual_lambda(ss, ii, tt) - step*(change_temp));
        end
    end
    
    % dual_mu for slot obj.period_lcm
    for ss=1:obj.n_state
        primal_ss = primal_optimal_state(obj.period_lcm);
        primal_aa = primal_optimal_action(obj.period_lcm);
        change_temp = 0;
        change_temp = change_temp + obj.transition_matrix(obj.period_lcm, primal_ss,primal_aa,ss);
        primal_next_ss = primal_optimal_state(1);
        if(primal_next_ss == ss)
            change_temp = change_temp - 1;
        end
        %%...
        dual_mu(ss, tt+1) = max(0, ...
            dual_mu(ss, tt) - step*(change_temp));
    end
    
    
    
end

%normalize the state_action_distribution
% state_action_distribution = state_action_distribution/(T/obj.period_lcm);

%calculate the theoretical_utlity, theoretical_throughput with the primal_state_action_distribution
theoretical_utlity = zeros(1,T);
theoretical_throughput = zeros(obj.n_flow,T);
for tt=1:T
    for kk=1:obj.n_flow
        for ii=1:obj.period_lcm
            for ss=1:obj.n_state
                %important! We only count when the action is to schdeule
                %flow kk. Because obj.reward_per_state_per_action(ss,aa) is
                %only for flow aa!!
                aa = kk;
                theoretical_throughput(kk,tt) = theoretical_throughput(kk,tt) ...
                    + primal_state_action_distribution(ii,ss,aa,tt)*obj.reward_per_state_per_action(ss,aa)/(obj.period_lcm/obj.period(kk));
            end
        end
        if(isequal(utility_form, 'weighted_sum'))
            theoretical_utlity(tt) =  theoretical_utlity(tt) + utility_coeff(kk)*(theoretical_throughput(kk,tt));
        elseif(isequal(utility_form, 'weighted_log_sum'))
            theoretical_utlity(tt) =  theoretical_utlity(tt) + utility_coeff(kk)*log(theoretical_throughput(kk,tt));
        else
            error('wrong input utility_form, can only be ''weighted_sum'' or ''weighted_log_sum''');
        end
    end
end

end