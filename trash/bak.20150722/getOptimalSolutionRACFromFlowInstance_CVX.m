function [optimal_policy, optimal_utility, optimal_throughput_per_flow] = getOptimalSolutionRACFromFlowInstance_CVX(obj, utility_coeff, utility_form)
% use the RAC formulation (Proposition 4 in Mobihoc 2015) to get the
% optimal utility, we can use either weighted sum or weithed log sum here
% obj: DownlinkAPInstanceFromFlowInstance
% utility_coeff: the coefficient for each flow
% utility_form is a string
%     utility_form = 'weighted_sum' means the weighted sum utility with utility_coeff
%     utility_form = 'weighted_log_sum' means the weighted log sum utility with utility_coeff

%Now I will use cvx to solve this convex problem

cvx_begin

    fprintf('begin to construct optimization variables\n');
    
    % x is the marginal distribution of stae, x(t, i) is the praobability
    % that the system is in state i at slot t
    variable x(obj.period_lcm, obj.n_state);

    %y is the joint distribution of state and action, y(t, i, j) is the joint
    %probability that the system is in state i and takes action j at slot t
    variable y(obj.period_lcm, obj.n_state, obj.n_action);

    %r is the (normalized) timely throughput vector. r(n) is the normalized
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
        y >= 0;

        % x(t,:) and y(t,:,:) are probability distributions
        % Todo: can be expressed in martix way so as to reduce matlab running time
%         for tt=1:obj.period_lcm
%             sum(x(tt,:)) == 1;
%             sum(sum(y(tt,:,:))) == 1;
%         end
        sum(x,2) == 1;
        sum(sum(y,2),3) == 1;

        % x(t,:) is the marginal distribution of y(t,:,:) in terms of the state
        % Todo: can be expressed in martix way so as to reduce matlab running time
%         for tt=1:obj.period_lcm
%             for ss=1:obj.n_state
%                 x(tt,ss) == sum(y(tt,ss,:));
%             end
%         end 
        x == sum(y,3);

        % x(t+1,:) envolves according to  the transition probability and the last
        % joint distribition y(t,:,:)
        % Todo: can be expressed in martix way so as to reduce matlab running time
        for tt=1:obj.period_lcm
            for ss=1:obj.n_state
                prob_temp = 0;
                for last_ss=1:obj.n_state
                    for last_action=1:obj.n_action
                        %get the transition probabity in stationary phase,
                        %thus use tt+obj.period_lcm
                        prob_temp = prob_temp + obj.getTransitionProbability(tt+obj.period_lcm,last_ss,last_action, ss)*y(tt,last_ss,last_action);
                    end
                end
              %  prob_temp = sum(sum(obj.transition_matrix(tt,:,:, ss).*y(tt,:,:)));
                if(tt < obj.period_lcm)
                    x(tt+1, ss) == prob_temp;
                else % tt = obj.period_lcm
                    x(1,ss) == prob_temp;
                end
            end
        end

        %reward constraints
        for nn=1:obj.n_flow
            reward_temp = 0;
            for tt=1:obj.period_lcm
                for ss=1:obj.n_state
                    for aa=1:obj.n_action
                        reward_temp = reward_temp + obj.getRewardPerFlow(ss,aa,nn)*y(tt,ss,aa)/obj.period_lcm;
                    end
                end
          %      reward_temp = reward_temp + sum((obj.reward_per_state_per_action(:,nn))'.*y(tt,:,nn)/(obj.period_lcm/obj.period(nn)));
            end
            r(nn) <= reward_temp;
        end
        
        fprintf('begin to solve the optimization problem\n');
cvx_end

optimal_policy = y;
optimal_utility = Objective;
optimal_throughput_per_flow = r;

end