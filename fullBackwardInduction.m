function [optimal_policy, optimal_reward, optimal_reward_per_flow] = fullBackwardInduction(obj, T, utility_coeff, fileID)
%consider T-slot MDP problem and use the weighted-sum utility with
%coefficient utility_coeff
%the method is the backward induction by considering all possible states
%fileID is the file to write the output 


%Note that we get the aggreate reward from slot 1 to T, the output is not
%normalized by T !!!!!

optimal_policy = zeros(T, obj.n_state); %optimal_policy(t, i)=j means that at slot t, if the system state is i, then do action j
optimal_reward = zeros(T, obj.n_state); %reward under optiaml policy
optimal_reward_per_flow =  zeros(T, obj.n_state, obj.n_flow);%reward (aggregate absolutely timely througput) 
% for each flow under optiaml policy
% optimal_reward_per_flow(t, i, u) is the total expected reward for flow u from slot t to T, if
% the system state at slot t is state i


for tt=T:-1:1
    fprintf('tt=%d\n',tt);
    fprintf(fileID, 'tt=%d\n',tt);
    if(tt == T)
        for ss=1:obj.n_state
            a_opt = 1;
            reward_opt = 0;
            for aa=1:obj.n_action
                %we consider weighted-sum utility. So we need to
                %normalize it with the coefficient.
                reward_temp = utility_coeff(aa)*obj.getRewardPerFlow(ss,aa, aa);
                if( reward_temp > reward_opt)
                    a_opt = aa;
                    reward_opt = reward_temp;
                end
            end
            optimal_policy(tt, ss) = a_opt;
            optimal_reward(tt, ss) = reward_opt;
            
            fprintf(fileID, 'optimal_policy(%d,%d)=%d, ', tt, ss, a_opt);
            fprintf(fileID, 'optimal_reward(%d,%d)=%f\n', tt, ss, reward_opt);
            fprintf('optimal_policy(%d,%d)=%d, ', tt, ss, a_opt);
            fprintf('optimal_reward(%d,%d)=%f\n', tt, ss, reward_opt);
            %evaluate per flow reward. we do not need the utility coefficient now.
            %optimal_reward_per_flow(tt, ss, a_opt) = obj.getRewardPerFlow(ss,a_opt, a_opt);
        end
    else % tt < T
        for ss=1:obj.n_state
            a_opt = 1;
            reward_opt = 0;
            for aa=1:obj.n_action
                reward_temp = utility_coeff(aa)*obj.getRewardPerFlow(ss,aa,aa);
                for ss_next = 1:obj.n_state
                    reward_temp = reward_temp + obj.getTransitionProbability(tt, ss, aa, ss_next)*optimal_reward(tt+1,ss_next);
                end
                if(reward_temp > reward_opt)
                    a_opt = aa;
                    reward_opt = reward_temp;
                end
            end
            optimal_policy(tt, ss) = a_opt;
            optimal_reward(tt, ss) = reward_opt;
            fprintf(fileID, 'optimal_policy(%d,%d)=%d, ', tt, ss, a_opt);
            fprintf(fileID, 'optimal_reward(%d,%d)=%f\n', tt, ss, reward_opt);
            fprintf('optimal_policy(%d,%d)=%d, ', tt, ss, a_opt);
            fprintf('optimal_reward(%d,%d)=%f\n', tt, ss, reward_opt);
            
            %evaluate per flow reward
%             for nn=1:obj.n_flow
%                 if( nn == a_opt)
%                     reward_opt_per_flow_temp = obj.getRewardPerFlow(ss,a_opt, a_opt);
%                 else
%                     reward_opt_per_flow_temp = 0;
%                 end
%                 for ss_next = 1:obj.n_state
%                     reward_opt_per_flow_temp = reward_opt_per_flow_temp + obj.getTransitionProbability(tt, ss, a_opt, ss_next).*optimal_reward_per_flow(tt+1,ss_next,nn);
%                 end
%                 optimal_reward_per_flow(tt, ss, nn) = reward_opt_per_flow_temp;
%             end
        end
    end
end
end