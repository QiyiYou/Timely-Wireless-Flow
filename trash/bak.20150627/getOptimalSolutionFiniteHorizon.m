function [optimal_policy, optimal_reward, optimal_reward_per_flow] = getOptimalSolutionFiniteHorizon(obj, T, utility_coeff)
%consider T-slot MDP problem and use the weighted-sum utility with
%coefficient utility_coeff

optimal_policy = zeros(T, obj.n_state); %optimal_policy(t, i)=j means that at slot t, if the system state is i, then do action j
optimal_reward = zeros(T, obj.n_state); %reward under optiaml policy
% optimal_reward(t, i) is the total expected reward from slot t to T, if
% the system state at slot t is state i

optimal_reward_per_flow =  zeros(T, obj.n_state, obj.n_flow);%reward (aggregate normalized timely througput) for each flow under optiaml policy
% optimal_reward_per_flow(t, i, u) is the total expected reward for flow u from slot t to T, if
% the system state at slot t is state i



for tt=T:-1:1
    if(tt == T)
        for ss=1:obj.n_state
            a_opt = 1;
            reward_opt = 0;
            for aa=1:obj.n_action
                if(obj.reward_per_state_per_action(ss,aa) > reward_opt)
                    a_opt = aa;
                    %Note that we consider normerlized
                    %timely throughput metric (metric 2). We need to normalized it
                    %by the number of injected packets, i.e., T/period(a_opt);
                    %It is T/period(a_opt) rather than
                    %period_lcm/period(a_opt) in Mobihoc because we
                    %consider T-slot rather than period_lcm-slot.
                    %Also, we consider weighted-sum utility. So we need to
                    %normalize it with the coefficient.
                    reward_opt = utility_coeff(aa)*obj.reward_per_state_per_action(ss,aa)/(T/obj.period(aa));
                end
            end
            optimal_policy(tt, ss) = a_opt;
            optimal_reward(tt, ss) = reward_opt;
            
            %evaluate per flow reward. Still consider the normalized
            %timely throughput. But we do not need the utility coefficient
            %now.
            optimal_reward_per_flow(tt, ss, a_opt) = obj.reward_per_state_per_action(ss, a_opt)/(T/obj.period(a_opt));
        end
    else
        for ss=1:obj.n_state
            a_opt = 1;
            reward_opt = 0;
            for aa=1:obj.n_action
                reward_temp = utility_coeff(aa)*obj.reward_per_state_per_action(ss,aa)/(T/obj.period(aa));
                for ss_next = 1:obj.n_state
                    reward_temp = reward_temp + obj.transition_matrix(getFirstPeriodSlot(obj,tt), ss, aa, ss_next)*optimal_reward(tt+1,ss_next);
                end
                if(reward_temp > reward_opt)
                    a_opt = aa;
                    reward_opt = reward_temp;
                end
            end
            optimal_policy(tt, ss) = a_opt;
            optimal_reward(tt, ss) = reward_opt;
            
            %evaluate per flow reward
            for nn=1:obj.n_flow
                if( nn == a_opt)
                    reward_opt_per_flow_temp = obj.reward_per_state_per_action(ss,a_opt)/(T/obj.period(a_opt));
                else
                    reward_opt_per_flow_temp = 0;
                end
                for ss_next = 1:obj.n_state
                    reward_opt_per_flow_temp = reward_opt_per_flow_temp + obj.transition_matrix(getFirstPeriodSlot(obj,tt), ss, a_opt, ss_next).*optimal_reward_per_flow(tt+1,ss_next,nn);
                end
                optimal_reward_per_flow(tt, ss, nn) = reward_opt_per_flow_temp;
            end
     
        end
    end
end

end