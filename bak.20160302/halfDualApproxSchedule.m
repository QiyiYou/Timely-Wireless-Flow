function [successful_transmission, ...
    state_action_distribution, system_state, system_action, state_action_per_slot, ...
    primal_rate_user, primal_state, primal_action, primal_state_action, ...
    dual_queue_user, dual_mu] ...
    = halfDualApproxSchedule(obj, utility_coeff, utility_form, T)
% use the full-dual-approx distributed algorithm to get the
% optimal utility, we can use either weighted sum or weithed log sum here
% obj: DownlinkAPInstance
% utility_coeff: the coefficient for each flow

% utility_form is a string
%     utility_form = 'weighted_sum' means the weighted sum utility with utility_coeff
%     utility_form = 'weighted_log_sum' means the weighted log sum utility with utility_coeff

primal_state = zeros(obj.period_lcm,T);
primal_action = zeros(obj.period_lcm,T);

% the primal variable x_h(s,a; t)
primal_state_action= zeros(obj.period_lcm, obj.n_state, obj.n_action, T);


% the induced primal variagls x_h^k(s^k,a;t), which is used to update the
% dual variables
max_n_state = 0;
for ii=1:obj.n_flow
    max_n_state = max(max_n_state, obj.flow_array{ii}.n_state);
end
primal_state_action_per_flow = zeros(obj.n_flow, obj.period_lcm, max_n_state, obj.n_action, T);


% the achieved state_action_distribution for each slot (in a period_lcm)
% this is used to compare with the global optimal RAC scheme, i.e., the
% joint distribution y in function getOptimalSolutionRAC.
% MOREOVER, in full-dual algorithm, we use this empirical distribution to
% make decions, so this is the primal variable which should be updated in
% every iteration



%the primal variable R_k for each flow k, primal vairable updates according to
% R^*_k = \arg \max U_k(R_k) - q_k R_k;
primal_rate_user = zeros(obj.n_flow, T);

% the dual variable q_k  for each flow k
% queue envolves as
%     virtual_queue(t+1) = max( virtual_queue(t)-virtual_server_capacity(t), 0) + virtual_arrival(t+1);
dual_queue_user = zeros(obj.n_flow, T);

%the dual variable for 1b) in the draft, i.e., the system randomness
%causaulity inequality from slot 1 to slot preiod_lcm
%dual_lambda = zeros(obj.period_lcm, obj.n_state, T);

%the induced dual variables
dual_mu = zeros(obj.n_flow, obj.period_lcm, max_n_state, T);


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



for tt=1:T
    if(mod(tt,100) == 0)
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
    
    tolerant = 1e-10;
    %update primal variable x. Since here we use full-dual distribution
    %algorithm. We need to get the optimal state-action pair and then
    %update the emprical state-action discitrion, which is the udpated y
    primal_optimal_state = zeros(obj.period_lcm,1);
    primal_optimal_state_per_flow = zeros(obj.n_flow, obj.period_lcm);
    primal_optimal_action = zeros(obj.period_lcm,1);
    
    if(tt == 1) %initialize as state 1 and action 1
        primal_state_action(:,1,1,1) = 1;
        primal_state_action_per_flow(:,:,1,1,1) = 1;
    else
        primal_state_action(:,:,:,tt) = primal_state_action(:, :,:,tt-1);
        primal_state_action_per_flow(:,:,:,:,tt) = primal_state_action_per_flow(:,:,:,:,tt-1);
    end
    
    
    ft = obj.getFirstPeriodSlot(tt); 
    if(ft==1)
        last_ft = obj.period_lcm;
    else
        last_ft = ft-1;
    end
    max_temp_val = -inf;
    for aa=1:obj.n_action
        primal_optimal_state_per_flow_hh_aa = zeros(obj.n_flow, 1);
        max_temp_val_aa = 0;
        for kk=1:obj.n_flow
            max_temp_val_aa_kk = -inf;
            for ss=1:obj.flow_array{kk}.n_state
                temp_val = 0;
                for next_ss=1:obj.flow_array{kk}.n_state
                    temp_val = temp_val + dual_mu(kk,ft,next_ss,tt)*obj.getTransitionProbabilityPerFlowState(ft+obj.period_lcm, kk, ss, aa, next_ss);
                end
                temp_val = temp_val - dual_mu(kk,last_ft, ss, tt);
                temp_val = temp_val + dual_queue_user(kk,tt)*obj.getRewardPerFlowState(ss,aa,kk)/(obj.period_lcm);
                if(temp_val > max_temp_val_aa_kk - tolerant)
                    max_temp_val_aa_kk = temp_val;
                    primal_optimal_state_per_flow_hh_aa(kk) = ss;
                end
            end
            max_temp_val_aa = max_temp_val_aa + max_temp_val_aa_kk;
        end
        if(max_temp_val_aa > max_temp_val - tolerant)
            max_temp_val = max_temp_val_aa;
            primal_optimal_action(ft) = aa;
            optimal_action = aa;
            primal_optimal_state_per_flow(:,ft) = primal_optimal_state_per_flow_hh_aa;
        end
    end
    state_vec = primal_optimal_state_per_flow(:,ft);
    primal_optimal_state(ft) = obj.getStateFromVector(state_vec);
    
    %update primal_state and primal_action
    primal_state(:,tt) = primal_optimal_state;
    primal_action(:,tt) = primal_optimal_action;
    
    %update the primal_state_action
    
    ss = primal_optimal_state(ft);
    aa = primal_optimal_action(ft);
    primal_state_action(ft, ss, aa,tt) = 1;
    
    for kk=1:obj.n_flow
        ss = primal_optimal_state_per_flow(kk,ft);
        aa = primal_optimal_action(ft);
        primal_state_action_per_flow(kk,ft,:,:,tt) = 0;
        primal_state_action_per_flow(kk, ft, ss, aa, tt) = 1;
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
    %step = 0.1;
    %update the dual_queue_user
    for kk=1:obj.n_flow
        change_temp = 0;
        for ii=1:obj.period_lcm
            [primal_ss, primal_aa] = find(squeeze(primal_state_action_per_flow(kk,ii,:,:,tt)) == 1);
            change_temp = change_temp + obj.getRewardPerFlowState(primal_ss,primal_aa,kk)/(obj.period_lcm);
        end
        change_temp = change_temp - primal_rate_user(kk,tt);
        dual_queue_user(kk,tt+1) = max(0, ...
            dual_queue_user(kk,tt) - step*(change_temp));
    end
    
    %update the dual varialbe  dual_mu
    for kk=1:obj.n_flow
        for ii=1:obj.period_lcm
            next_ii = 1 + mod(ii,obj.period_lcm);
            for ss=1:obj.flow_array{kk}.n_state
                [primal_ss, primal_aa] = find(squeeze(primal_state_action_per_flow(kk,ii,:,:,tt)) == 1);
                change_temp = 0;
                change_temp = change_temp + obj.getTransitionProbabilityPerFlowState(ii+obj.period_lcm, kk, primal_ss,primal_aa,ss);
                [primal_next_ss, primal_next_aa] = find(squeeze(primal_state_action_per_flow(kk,next_ii,:,:,tt)) == 1);
                if(primal_next_ss == ss)
                    change_temp = change_temp - 1;
                end
                %%...
                dual_mu(kk, ii, ss, tt+1) = max(0, ...
                    dual_mu(kk, ii, ss, tt) - step*(change_temp));
            end
        end
    end
end




end