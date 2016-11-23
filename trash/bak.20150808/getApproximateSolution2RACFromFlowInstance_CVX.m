function [optimal_policy, optimal_action_distribution, optimal_utility, optimal_throughput_per_flow] = getApproximateSolution2RACFromFlowInstance_CVX(obj, utility_coeff, utility_form)
% use the RAC approximate formulation to get the
% optimal utility, we can use either weighted sum or weithed log sum here
% here we specify that every flow can take only 2 actions: 1 means to
% transmit, 2 means not to transmit
% obj: DownlinkAPInstanceFromFlowInstance
% utility_coeff: the coefficient for each flow
% utility_form is a string
%     utility_form = 'weighted_sum' means the weighted sum utility with utility_coeff
%     utility_form = 'weighted_log_sum' means the weighted log sum utility with utility_coeff

%Now I will use cvx to solve this convex problem

max_n_state = 0;
for ii=1:obj.n_flow
    max_n_state = max(max_n_state, obj.flow_array{ii}.n_state);
end

cvx_begin

    fprintf('begin to construct optimization variables\n');
    
    % x is the marginal distribution of stae, x(t, i) is the praobability
    % that the system is in state i at slot t
    % Note that we use max_n_state here, but we can only use
    % obj.flow_array(ii).n_state for each flow ii, i.e., x(ii, :,:)
    variable x(obj.n_flow, obj.period_lcm, max_n_state);

    %y is the joint distribution of state and action, y(t, i, 2) is the joint
    %probability that the system is in state i and takes action j at slot t
    % action j=1: transmit this flow
    % action j=2: do not transmit this flow
    variable z(obj.n_flow, obj.period_lcm, max_n_state, 2);

    %z_action(k,t,1) is the prob. that flow k will take action 1 at slot t
    %z_action(k,t,2) is the prob. that flow k will take action 2 at slot t
    variable z_action(obj.n_flow, obj.period_lcm, 2);
    
    %r is the timely throughput vector. r(n) is the 
    %tiemly throughput for flow n
    variable r(obj.n_flow,1);
    
    fprintf('begin to set objective\n');
    %objective
    expression  Objective;
    Objective = 0;
    for nn=1:obj.n_flow
        if(isequal(utility_form, 'weighted_sum'))
            %weighted sum
            Objective = Objective + utility_coeff(nn)*r(nn);
        elseif (isequal(utility_form, 'weighted_log_sum'))
            %weighted log sum
            Objective = Objective + utility_coeff(nn)*log(r(nn));
        else
            error('wrong input utility_form, can only be ''weighted_sum'' or ''weighted_log_sum''');
        end
    end
    
    maximize( Objective );

   
    fprintf('begin to construct optimization constraints\n');
    subject to
    
    x >= 0;
    z >= 0;
    r >= 0;
    
    z_action >= 0;
    % x(t,:) and y(t,:,:) are probability distributions
    % Todo: can be expressed in martix way so as to reduce matlab running time
    %         for tt=1:obj.period_lcm
    %             sum(x(tt,:)) == 1;
    %             sum(sum(y(tt,:,:))) == 1;
    %         end
    for kk=1:obj.n_flow
        for tt=1:obj.period_lcm
            n_state = obj.flow_array{kk}.n_state;
            sum(squeeze(x(kk,tt,1:n_state))) == 1;
            sum(sum(squeeze(z(kk,tt,1:n_state,:)))) == 1;
            x(kk,tt,1:n_state) == sum(z(kk,tt,1:n_state,:),4);
        end
    end
    
    for tt=1:obj.period_lcm
        for kk=1:obj.n_flow
            n_state = obj.flow_array{kk}.n_state;
            for aa=1:2
                sum(squeeze(z(kk,tt,1:n_state,aa))) == z_action(kk,tt,aa);
            end
        end
        %the sum prob. of select action "transmit" is not greater than 1
        sum(squeeze(z_action(:, tt, 1))) <= 1;
    end

        % x(t+1,:) envolves according to  the transition probability and the last
        % joint distribition y(t,:,:)
        % Todo: can be expressed in martix way so as to reduce matlab running time
        for kk=1:obj.n_flow
            for tt=1:obj.period_lcm
                for ss=1:obj.flow_array{kk}.n_state
                    prob_temp = 0;
                    for last_ss=1:obj.flow_array{kk}.n_state
                        for last_action=1:2
                            %get the transition probabity in stationary phase,
                            %thus use tt+obj.period_lcm
                            if(last_action == 1) %1 means to transmit this flow
                                prob_temp = prob_temp + obj.flow_array{kk}.getTransitionProbability(tt+obj.period_lcm,last_ss,1, ss)*z(kk, tt,last_ss,last_action);
                            else
                                prob_temp = prob_temp + obj.flow_array{kk}.getTransitionProbability(tt+obj.period_lcm,last_ss,2, ss)*z(kk, tt,last_ss,last_action);
                            end
                        end
                    end
                    %  prob_temp = sum(sum(obj.transition_matrix(tt,:,:, ss).*y(tt,:,:)));
                    if(tt < obj.period_lcm)
                        x(kk, tt+1, ss) <= prob_temp;
                    else % tt = obj.period_lcm
                        x(kk, 1,ss) <= prob_temp;
                    end
                end
            end
        end

        %reward constraints
        for kk=1:obj.n_flow
            reward_temp = 0;
            for tt=1:obj.period_lcm
                for ss=1:obj.flow_array{kk}.n_state
                    for aa=1:2
                        if(aa == 1)
                            reward_temp = reward_temp+ obj.flow_array{kk}.reward_per_state_per_action(ss, 1)*z(kk,tt,ss,aa)/obj.period_lcm;
                        else
                            reward_temp = reward_temp + obj.flow_array{kk}.reward_per_state_per_action(ss, 2)*z(kk,tt,ss,aa)/obj.period_lcm;
                        end
                    end
                end
                %      reward_temp = reward_temp + sum((obj.reward_per_state_per_action(:,nn))'.*y(tt,:,nn)/(obj.period_lcm/obj.period(nn)));
            end
            r(kk) <= reward_temp;
        end
        
        fprintf('begin to solve the optimization problem\n');
cvx_end

optimal_policy = z;
optimal_action_distribution = z_action;
optimal_utility = Objective;
optimal_throughput_per_flow = r;

end