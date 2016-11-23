classdef DownlinkAPInstanceFromFlowInstance < handle
    %DownlinkAPInstance: a class for the downlink AP scenario where each
    %slot can only scheduling one user/flow
    % we consider the non-overlapped traffic pattern
    
    properties (SetAccess = public, GetAccess = public)
        
        %number of users/flows
        n_flow;
        
        %flow array
        flow_array;
        
        %large period
        period_lcm;
        
        % K = n_flow
        %number of states: from 1 to n_state(1)*n_state(2)*n_state(3)*...*n_state(K)
        % state 1:                             (S1, S2, ... , SK) = (1,1, ... ,1)
        % state 2:                             (S1, S2, ... , SK) = (1,1, ... ,2)
        % state n_state(K):                    (S1, S2, ... , SK) = (1,1, ... ,n_state(K))
        % state n_state(K)+1:                  (S1, S2, ... , SK) = (1,1, ... ,2,1)
        % state n_state(K)+2:                  (S1, S2, ... , SK) = (1,1, ... ,2,2)
        % state 2n_state(K):                   (S1, S2, ... , SK) = (1,1, ... ,2,n_state(K))
        % state (n_state(K-1)-1)*n_state(K)+1: (S1, S2, ... , SK) = (1,1, ... ,n_state(K-1),1)
        % state (n_state(K-1)-1)*n_state(K)+2: (S1, S2, ... , SK) = (1,1, ... ,n_state(K-1),2))
        % state n_state(K-1)*n_state(K)+2:     (S1, S2, ... , SK) = (1,1, ... ,n_state(K-1),n_state(K)))
        % state n_state(1)*...*n_state(K)      (S1, S2, ... , SK) = (n_state(1),n_state(2), ... ,n_state(K-1),n_state(K)))
        n_state;
        
        % number of actions
        % action i: transmit user i
        n_action;
        
        % transition probability due to both transmission and A&E (first TX
        % from the beginning of each slot to the end of the slot, then AE
        % from the end of the slot to the begnning of next slot)
        % Since TX is independnt of slots (or periodic with period 1) and A&E is periodic with period period_lcm, we will only
        % consider the first period, i.e., from slot 1 to slot period_lcm.
        % For any time slot t, not that transition_matrix(t, i, a, j) = transition_matrix(rem(t, period_lcm), i, a, j)
        % Dimension:  zeros(period_lcm, n_state, n_action, n_state)
        % transmition_matrix(t, i, a, j) is the probability from state i under action a at the beginning of slot t
        % to the state j at the begnning of slot t+1 if we consider both transmission and A&E
     %   transition_matrix;
        
        % the reward per slot for each state-action
        % Dimension: zeros(n_state, n_action)
     %   reward_per_state_per_action;
    end
    
    methods
        
        function  obj=DownlinkAPInstanceFromFlowInstance()
            %constructor function
        end
        
        
        function [ret] = isValid(obj)
            if(obj.n_flow ~= length(obj.flow_array))
                ret = -1;
                error('wrong input');
            end
            ret = 1;
        end
        
        function constructEverything(obj)
            if(isValid(obj) == -1)
                return;
            end
            
            obj.period_lcm = obj.flow_array(1).period;
            obj.n_state = obj.flow_array(1).n_state;
            for ii=1:obj.n_flow-1
                obj.period_lcm = lcm(obj.period_lcm, obj.flow_array(ii+1).period); %number of slots per (large) period
                obj.n_state = obj.n_state*obj.flow_array(ii+1).n_state;
            end
            obj.n_action = obj.n_flow;
        end
        
        function [state] = getStateFromVector(obj, state_vec)
            if(length(state_vec) ~= obj.n_flow)
                error('wrong input');
            end
            
            for ii=1:obj.n_flow
                if(state_vec(ii) <= 0 || state_vec(ii) > obj.flow_array(ii).n_state)
                    error('wrong input %d', ii);
                end
            end
            
            reverse_base = ones(obj.n_flow,1);
            for ii=obj.n_flow-1:-1:1
                reverse_base(ii) = reverse_base(ii+1)*obj.flow_array(ii+1).n_state;
            end
            
            state = 0;
            for ii=obj.n_flow:-1:1
                if(ii == obj.n_flow)
                    state = state + state_vec(ii);
                else
                    state = state + (state_vec(ii)-1)*reverse_base(ii);
                end
            end
        end
        
        function [state_vec] = getVectorState(obj, state)
            if(state <=  0 || state > obj.n_state)
                error('wrong input');
            end
            state_size = zeros(1,obj.n_flow);
            for ii=1:obj.n_flow
                state_size(ii) = obj.flow_array(ii).n_state;
            end
            
            reverse_base = ones(obj.n_flow,1);
            for ii=obj.n_flow-1:-1:1
                reverse_base(ii) = reverse_base(ii+1)*obj.flow_array(ii+1).n_state;
            end
            
            state_vec = zeros(1, obj.n_flow);
            state_keep_decreasing = state;
            for ii=1:obj.n_flow
                if(ii < obj.n_flow)
                    state_vec(ii) = ceil(state_keep_decreasing/reverse_base(ii));
                    state_keep_decreasing = state_keep_decreasing - (state_vec(ii)-1)*reverse_base(ii);
                else %ii == obj.n_flow
                    state_vec(ii) = state_keep_decreasing;
                    state_keep_decreasing = state_keep_decreasing - state_vec(ii);
                end
            end
            
            %sanity check
            if( abs(state_keep_decreasing) > 1e-10)
                error('state_keep_decreasing is not 0, something wrong');
            end
        end
        
        function [prob] = getTransitionProbability(obj, tt, state, action, next_state)
            if(tt <=0 || state <=0 || state > obj.n_state || action <=0 || action > obj.n_action ...
                      || next_state <=0 || next_state > obj.n_state)
                  error('wrong input');
            end
            state_vec = obj.getVectorState(state);
            next_state_vec = obj.getVectorState(next_state);
            prob = 1;
            for ii=1:obj.n_flow
                if(action == ii)
                    prob = prob*obj.flow_array(ii).getTransitionProbability(tt, state_vec(ii), 1, next_state_vec(ii));
                else
                    prob = prob*obj.flow_array(ii).getTransitionProbability(tt, state_vec(ii), 2, next_state_vec(ii));
                end
            end
        end
        
        function [reward] = getRewardPerFlow(obj, state, action, k)
            if( state <=0 || state > obj.n_state || action <=0 || action > obj.n_action ...
                          || k <=0 || k > obj.n_flow)
                      error('wrong input');
            end
            state_vec = obj.getVectorState(state);
            if(action == k)
                reward = obj.flow_array(k).reward_per_state_per_action(state_vec(k), 1);
            else
                reward = obj.flow_array(k).reward_per_state_per_action(state_vec(k), 2);
            end
        end
        
    end
    
end