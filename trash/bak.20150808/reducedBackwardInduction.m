function [optimal_policy, optimal_reward, optimal_reward_per_flow] = reducedBackwardInduction(obj, T, utility_coeff)
%consider T-slot MDP problem and use the weighted-sum utility with
%coefficient utility_coeff
%the method is the reduced-state-space backward induction by incorpating
%the priority-based policy and reduced states

%divided into different groups and get the critical slots
critical_slots = [];
for kk=1:obj.n_flow
    delay = obj.flow_array{kk}.delay;
    isFound = -1;
    for ii=1:length(critical_slots)
        if(critical_slots(ii) == delay)
            isFound = 1;
            break;
        end
    end
    if(isFound == -1) %does not exist 
        critical_slots(end+1) = delay;
    end
end
%sort the critical_slots in descending order
critical_slots = sort(critical_slots, 'descend');
N = length(critical_slots);
groups = cell(N,1);
for ii=1:N
    for kk=1:obj.n_flow
        delay = obj.flow_array{kk}.delay;
        if(delay == critical_slots(ii))
            groups{ii}(end+1) = kk;
        end
    end 
end

priority_flow = cell(N,1); % the prioritized flows in descend order at each critical slots, only for the valid (non-expired) flows
priority_states = cell(N,1); % the prioritized states in descend order at each critical slots, only for valid states

%Note that we get the aggreate reward from slot 1 to T, the output is not
%normalized by T !!!!!

optimal_policy = zeros(T, obj.n_state); %optimal_policy(t, i)=j means that at slot t, if the system state is i, then do action j
optimal_reward = zeros(T, obj.n_state); %reward under optiaml policy
optimal_reward_per_flow =  zeros(T, obj.n_state, obj.n_flow);%reward (aggregate absolutely timely througput) 
% for each flow under optiaml policy
% optimal_reward_per_flow(t, i, u) is the total expected reward for flow u from slot t to T, if
% the system state at slot t is state i


for tt=T:-1:1
    if(tt > critical_slots(1))  % we do nothing until the first critical slots
        continue;
    end
    
    isCriticalSlot = -1;
    whichCriticalSlot = -1;
    for ii=1:N
        if(tt == critical_slots(ii)) %% tt is a critical slot, and it is the ii-th critical slots
            isCriticalSlot = 1;
            whichCriticalSlot = ii;
            break;
        end
    end
    
    if(isCriticalSlot == 1) %it is a critical slots, we need to update the priority and compute the reward at this stage 
        if(whichCriticalSlot == 1) %if it is the first critical slot, use p*k to sort
            this_group = groups{whichCriticalSlot}; % get the current group
            n_this_group = length(this_group);
            f_group = zeros(n_this_group,0); % the fucntion f of this group, we use index from 1 to n_this_group
                                             % where index i means real flow this_group(ii);
            for ii=1:n_this_group
                flow_temp = this_group(ii);
                f_group(ii) = obj.flow_array{flow_temp}.success_prob*utility_coeff(flow_temp);
            end
            priority_idx = sort(f_group, 'descend');
            priority_flow{1} = this_group(priority_idx);
            for jj=1:n_this_group % we only consider n_this_group states, \tilde{S}_jj
                state_vec = ones(obj.n_flow,1); % 1 means no packet
                for zz=jj:n_this_group
                    state_vec(priority_flow(zz)) = 2; % 2 means has pacekt
                end
                state = obj.getStateFromVector(state_vec);
                priority_states{1}(end+1) = state;
                optimal_policy(tt, state) = priority_flow(jj);
                optimal_reward(tt, state) = f_group(priority_flow(jj));
            end
        else % this is not the first critical slot
           %% todo          
        end
        
    else %it is not a critical slots, use backward induction by considering the reduced states and the priority-based policy
    end

%     if(tt == T)
%         for ss=1:obj.n_state
%             a_opt = 1;
%             reward_opt = 0;
%             for aa=1:obj.n_action
%                 %we consider weighted-sum utility. So we need to
%                 %normalize it with the coefficient.
%                 reward_temp = utility_coeff(aa)*obj.getRewardPerFlow(ss,aa, aa);
%                 if( reward_temp > reward_opt)
%                     a_opt = aa;
%                     reward_opt = reward_temp;
%                 end
%             end
%             optimal_policy(tt, ss) = a_opt;
%             optimal_reward(tt, ss) = reward_opt;
%             
%             %evaluate per flow reward. we do not need the utility coefficient now.
%             optimal_reward_per_flow(tt, ss, a_opt) = obj.getRewardPerFlow(ss,a_opt, a_opt);
%         end
%     else % tt < T
%         for ss=1:obj.n_state
%             a_opt = 1;
%             reward_opt = 0;
%             for aa=1:obj.n_action
%                 reward_temp = utility_coeff(aa)*obj.getRewardPerFlow(ss,aa,aa);
%                 for ss_next = 1:obj.n_state
%                     reward_temp = reward_temp + obj.getTransitionProbability(tt, ss, aa, ss_next)*optimal_reward(tt+1,ss_next);
%                 end
%                 if(reward_temp > reward_opt)
%                     a_opt = aa;
%                     reward_opt = reward_temp;
%                 end
%             end
%             optimal_policy(tt, ss) = a_opt;
%             optimal_reward(tt, ss) = reward_opt;
%             
%             %evaluate per flow reward
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
%         end
%     end
end
end