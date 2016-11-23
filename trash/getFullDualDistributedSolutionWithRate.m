function [theoretical_utlity, theoretical_throughput,...
    primal_state, primal_action, primal_state_action, primal_state_action_distribution, ...
    dual_queue_user, dual_lambda, dual_mu] ...
    = getFullDualDistributedSolutionWithRate(obj, utility_coeff, utility_form, optimal_throughput_per_flow, T)
% use the full-dual distributed algorithm with given rate. Thus, this is
% the throughput optimal problem.
% obj: DownlinkAPInstance

% utility_coeff, utility_form are only used for calculation rather than
% algorithm desgin.

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


theoretical_throughput = zeros(obj.n_flow,T);
theoretical_utlity = zeros(T,1);

for tt=1:T
    fprintf('tt=%d\n', tt);
    %primal  variable update
    
    tolerant = 1e-10;
    %update primal variable x. Since here we use full-dual distribution
    %algorithm. We need to get the optimal state-action pair and then
    %update the emprical state-action discitrion, which is the udpated y
    primal_optimal_state = zeros(obj.period_lcm,1);
    primal_optimal_action = zeros(obj.period_lcm,1);
    %first update from slot 1 to slot obj.period_lcm
    for ii=1:obj.period_lcm
        max_temp_val = -inf;
        for ss=1:obj.n_state
            for aa=1:obj.n_action
                temp_val = 0;
                for next_ss=1:obj.n_state
                    %distingush slot obj.period_lcm
                    if(ii==obj.period_lcm)
                        temp_val = temp_val + dual_mu(next_ss, tt)*obj.getTransitionProbability(ii+obj.period_lcm, ss,aa,next_ss);
                    else
                        temp_val = temp_val + dual_lambda(ii, next_ss, tt)*obj.getTransitionProbability(ii+obj.period_lcm, ss, aa, next_ss);
                    end
                end
                %distingush slot 1
                if(ii == 1)
                    temp_val = temp_val - dual_mu(ss, tt);
                else
                    temp_val = temp_val - dual_lambda(ii-1, ss, tt);
                end
                
                for kk=1:obj.n_flow
                    temp_val = temp_val + dual_queue_user(kk,tt)*obj.getRewardPerFlow(ss,aa,kk)/(obj.period_lcm);
                end
                
                if(temp_val > max_temp_val + tolerant)
                    primal_optimal_state(ii) = ss;
                    primal_optimal_action(ii) = aa;
                    max_temp_val = temp_val;
                end
            end
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
    
    %calculate the theoretical_utlity, theoretical_throughput with the primal_state_action_distribution
    for kk=1:obj.n_flow
        for ii=1:obj.period_lcm
            for ss=1:obj.n_state
                for aa=1:obj.n_action
                    theoretical_throughput(kk,tt) = theoretical_throughput(kk,tt) ...
                        + primal_state_action_distribution(ii,ss,aa,tt)*obj.getRewardPerFlow(ss,aa,kk)/(obj.period_lcm);
                end
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
    
    %we have finished the last slot
    if(tt == T)
        break;
    end
    
    
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
            primal_ss = primal_optimal_state(ii);
            primal_aa = primal_optimal_action(ii);
            change_temp = change_temp + obj.getRewardPerFlow(primal_ss,primal_aa,kk)/(obj.period_lcm);
        end
        change_temp = change_temp - optimal_throughput_per_flow(kk);
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