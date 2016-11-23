function [successful_transmission,...
    primal_rate_user, primal_state, primal_action, primal_state_action, primal_state_action_distribution, ...
    dual_queue_user, dual_lambda, dual_mu] ...
    = halfDualSchedule(obj, utility_coeff, utility_form, T)
% use the half-dual distributed algorithm to get the
% optimal utility, we can use either weighted sum or weithed log sum here
% obj: DownlinkAPInstance
% utility_coeff: the coefficient for each flow

% utility_form is a string
%     utility_form = 'weighted_sum' means the weighted sum utility with utility_coeff
%     utility_form = 'weighted_log_sum' means the weighted log sum utility with utility_coeff

%Note that the half dual combined both primal-dual variables update and the
%real system scheduling

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




primal_state = zeros(obj.period_lcm,T);
primal_action = zeros(obj.period_lcm,T);

% the primal variable x_h(s,a; t)
primal_state_action= zeros(obj.period_lcm, obj.n_state, obj.n_action, T);

% the achieved state_action_distribution for each slot (in a period_lcm)
% this is used to compare with the global optimal RAC scheme, i.e., the
% joint distribution y in function getOptimalSolutionRAC.
% MOREOVER, in full-dual algorithm, we use this empirical distribution to
% make decions, so this is the primal variable which should be updated in
% every iteration

%Note the different of primal_state_action, which is the  instantaneous
%state-action distribution, and the primal_state_action_distribution which
%is the cummlative state-action distribution
primal_state_action_distribution = zeros(obj.period_lcm, obj.n_state, obj.n_action, T);


%the primal variable R_k for each flow k, primal vairable updates according to
% R^*_k = \arg \max U_k(R_k) - q_k R_k;
primal_rate_user = zeros(obj.n_flow, T);

% the dual variable q_k  for each flow k
% queue envolves as
%     virtual_queue(t+1) = max( virtual_queue(t)-virtual_server_capacity(t), 0) + virtual_arrival(t+1);
dual_queue_user = zeros(obj.n_flow, T);

%the dual variable for 1b) in the draft, i.e., the system randomness
%causaulity inequality from slot 1 to slot preiod_lcm - 1
dual_lambda = zeros(obj.period_lcm-1, obj.n_state, T);

%the dual variable for 3c) in the draft, i.e., the system randomness
%causaulity inequality for final slot preiod_lcm
dual_mu = zeros(obj.n_state,T);



for tt=1:T
    fprintf('tt=%d\n', tt);
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
    
    %get the current state
    current_state = system_state(tt);
    %the corresponding slot in the first period
    ft = obj.getFirstPeriodSlot(tt); 
    
    tolerant = 1e-10;
    %update primal variable x. Since here we use full-dual distribution
    %algorithm. We need to get the optimal state-action pair and then
    %update the emprical state-action discitrion, which is the udpated y
    if(tt == 1) %initialize as state 1 and action 1
        primal_optimal_state = ones(obj.period_lcm,1);
        primal_optimal_action = ones(obj.period_lcm,1);
    else
        primal_optimal_state = primal_state(:,tt-1);
        primal_optimal_action = primal_action(:,tt-1);
    end
    
    %undate the primal varaible x for ft
    max_temp_val = -inf;
    optimal_action = -1;
    for aa=1:obj.n_action
        temp_val = 0;
        for next_ss=1:obj.n_state
            %distingush slot obj.period_lcm
            if(ft==obj.period_lcm)
                temp_val = temp_val + dual_mu(next_ss, tt)*obj.getTransitionProbability(ft+obj.period_lcm, current_state, aa, next_ss);
            else
                temp_val = temp_val + dual_lambda(ft, next_ss, tt)*obj.getTransitionProbability(ft+obj.period_lcm, current_state, aa, next_ss);
            end
        end
        %distingush slot 1
        if(ft == 1)
            temp_val = temp_val - dual_mu(current_state, tt);
        else
            temp_val = temp_val - dual_lambda(ft-1, current_state, tt);
        end
        
        for kk=1:obj.n_flow
            temp_val = temp_val + dual_queue_user(kk,tt)*obj.getRewardPerFlow(current_state,aa,kk)/(obj.period_lcm);
        end
        
        if(temp_val > max_temp_val + tolerant)
            primal_optimal_state(ft) = current_state;
            primal_optimal_action(ft) = aa;
            optimal_action = aa;
            max_temp_val = temp_val;
        end
    end

    
    %update primal_state and primal_action
    primal_state(:,tt) = primal_optimal_state;
    primal_action(:,tt) = primal_optimal_action;
    
    %update the primal_state_action
    for ii=1:obj.period_lcm
        ss = primal_optimal_state(ii);
        aa = primal_optimal_action(ii);
        primal_state_action(ii, ss, aa,tt) = 1;
    end
    
    %update the emprical state-action discitrion, which is the udpated x
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
    
    %update dual vriables. Note that we should not use the
    %emprical distribution, instead, we use the instantenous x, i.e.,
    %primal_optimal_state and primal_optimal_action or just
    %primal_state_action.
    
    step = 1/sqrt(tt);
    %step = 0.001;
    %update the dual_queue_user
    for kk=1:obj.n_flow
        change_temp = 0;
        for ii=1:obj.period_lcm
            primal_ss = primal_optimal_state(ii);
            primal_aa = primal_optimal_action(ii);
            change_temp = change_temp + obj.getRewardPerFlow(primal_ss,primal_aa,kk)/(obj.period_lcm);
        end
        change_temp = change_temp - primal_rate_user(kk,tt);
        dual_queue_user(kk,tt+1) = max(0, ...
            dual_queue_user(kk,tt) - step*(change_temp));
    end
    
    %update the dual varialbe dual_lambda and dual_mu
    % dual_lambda from slot 1 to slot obj.period_lcm-1
    for ii=1:obj.period_lcm-1
        for ss=1:obj.n_state
            primal_ss = primal_optimal_state(ii);
            primal_aa = primal_optimal_action(ii);
            change_temp = 0;
            change_temp = change_temp + obj.getTransitionProbability(ii+obj.period_lcm, primal_ss,primal_aa,ss);
            primal_next_ss = primal_optimal_state(ii+1);
            if(primal_next_ss == ss)
                change_temp = change_temp - 1;
            end
            %%...
            dual_lambda(ii, ss, tt+1) = max(0, ...
                dual_lambda(ii, ss, tt) - step*(change_temp));
        end
    end
    
    % dual_mu for slot obj.period_lcm
    for ss=1:obj.n_state
        primal_ss = primal_optimal_state(obj.period_lcm);
        primal_aa = primal_optimal_action(obj.period_lcm);
        change_temp = 0;
        change_temp = change_temp + obj.getTransitionProbability(obj.period_lcm, primal_ss,primal_aa,ss);
        primal_next_ss = primal_optimal_state(1);
        if(primal_next_ss == ss)
            change_temp = change_temp - 1;
        end
        %%...
        dual_mu(ss, tt+1) = max(0, ...
            dual_mu(ss, tt) - step*(change_temp));
    end
end




end