function [successful_transmission, ...
    state_action_distribution, system_state, system_action, state_action_per_slot, ...
    primal_rate_user, primal_state_action, primal_state_action_distribution, ...
    dual_queue_user, dual_lambda] ...
    = halfDualSchedule(obj, utility_coeff, utility_form, T)
% use the full-dual distributed algorithm to get the
% optimal utility, we can use either weighted sum or weithed log sum here
% obj: DownlinkAPInstance
% utility_coeff: the coefficient for each flow

% utility_form is a string
%     utility_form = 'weighted_sum' means the weighted sum utility with utility_coeff
%     utility_form = 'weighted_log_sum' means the weighted log sum utility with utility_coeff

%Note that we use the vectorized version

%% primal&dual variables for half-dual computation
% the primal variable w_h(idx; t), note that we use vectorized state-action
% pair
primal_state_action= zeros(obj.n_state_action, obj.period_lcm, T);

% the achieved state_action_distribution for each slot (in a period_lcm)
% this is used to compare with the global optimal RAC scheme, i.e., the
% joint distribution y in function getOptimalSolutionRAC.
% MOREOVER, in full-dual algorithm, we use this empirical distribution to
% make decions, so this is the primal variable which should be updated in
% every iteration

%Note the different of primal_state_action, which is the  instantaneous
%state-action distribution, and the primal_state_action_distribution which
%is the cummlative state-action distribution
primal_state_action_distribution = zeros(obj.n_state_action, obj.period_lcm, T);


%the primal variable R_k for each flow k, primal vairable updates according to
% R^*_k = \arg \max U_k(R_k) - q_k R_k;
primal_rate_user = zeros(obj.n_flow, T);

% the dual variable q_k  for each flow k
dual_queue_user = zeros(obj.n_flow, T);

%the dual variable for 1b) in the draft, i.e., the system randomness
%causaulity inequality from slot 1 to slot preiod_lcm 
dual_lambda = zeros(obj.n_state, obj.period_lcm, T);


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
    
    
    %update primal variable w. Since here we use full-dual distribution
    %algorithm. We need to get the optimal state-action pair and then
    %update the emprical state-action discitrion, which is the udpated y
    if(tt == 1) %initialize as state 1 and action 1
        primal_state_action(1,:,1) = 1;
    else
        primal_state_action(:,:,tt) = primal_state_action(:,:,tt-1);
    end
    
    ft = obj.getFirstPeriodSlot(tt);
    if(ft==1)
        last_ft = obj.period_lcm;
    else
        last_ft = ft -1;
    end
    
    delta = dual_lambda(:,ft,tt)'*obj.Pv(:,:,ft)'-dual_lambda(:,last_ft,tt)'*obj.Psi;
    for kk=1:obj.n_flow
        delta = delta + dual_queue_user(kk,tt)*obj.theta(:,kk)'/obj.period_lcm;
    end
    [max_val, max_idx] = max(delta(obj.Psi(current_state,:)==1));
    [state_temp, optimal_action] = obj.getStateActionFromIdx(max_idx);
    primal_state_action(:,ft,tt) = 0;
    primal_state_action(max_idx,ft,tt) = 1;
    
    %update the emprical state-action discitrion, which is the udpated x
    primal_state_action_distribution(:,:,tt) = mean(primal_state_action(:,:,max(1,tt-2000):tt),3);
    
    
    
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
    %step = 0.1;
    %update the dual_queue_user
    for kk=1:obj.n_flow
        change_temp = obj.theta(:,kk)'*(sum(primal_state_action(:,:,tt),2))/obj.period_lcm - primal_rate_user(kk,tt);
        dual_queue_user(kk,tt+1) = max(0, ...
            dual_queue_user(kk,tt) - step*(change_temp));
    end
    
    %update the dual varialbe dual_lambda and dual_mu
    % dual_lambda from slot 1 to slot obj.period_lcm
    for hh=1:obj.period_lcm
        next_hh = 1 + mod(hh,obj.period_lcm);
        change_temp = obj.Pv(:,:,hh)'*primal_state_action(:,hh,tt) - obj.Psi*primal_state_action(:,next_hh,tt);
        dual_lambda(:, hh, tt+1) = max(0, ...
            dual_lambda(:, hh, tt) - step*(change_temp));
    end
end




end