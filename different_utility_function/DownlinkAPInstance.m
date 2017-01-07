classdef DownlinkAPInstance < handle
    %DownlinkAPInstance: a class for the downlink AP scenario where each
    %slot can only scheduling one user/flow
    
    properties (SetAccess = public, GetAccess = public)
        
        %number of users/flows
        n_flow;
        
        %flow array, this is a cell array, could include both FlowInstance
        %and NonOverlappedFlowInstance
        flow_array;
        
        %large period
        period_lcm;
        
        % number of states: from 1 to N;
        % Let K = n_flow, Nk = number of states of flow k, then N=N(1)*N(2)*...*N(K), then the mapping is as follows
        % state 1:                             (S1, S2, ... , SK) = (1,1, ... ,1,1)
        % state 2:                             (S1, S2, ... , SK) = (1,1, ... ,1,2)
        % state N(K):                          (S1, S2, ... , SK) = (1,1, ... ,1,N(K))
        % state N(K)+1:                        (S1, S2, ... , SK) = (1,1, ... ,2,1)
        % state N(K)+2:                        (S1, S2, ... , SK) = (1,1, ... ,2,2)
        % state 2*N(K):                        (S1, S2, ... , SK) = (1,1, ... ,2,N(K))
        % state (N(K-1)-1)*N(K)+1:             (S1, S2, ... , SK) = (1,1, ... ,N(K-1),1)
        % state (N(K-1)-1)*N(K)+2:             (S1, S2, ... , SK) = (1,1, ... ,N(K-1),2))
        % state N(K-1)*N(K):                   (S1, S2, ... , SK) = (1,1, ... ,N(K-1),N(K)))
        % state N(1)*...*N(K)                  (S1, S2, ... , SK) = (N(1),N(2), ... ,N(K-1),N(K)))
        n_state;
        
        % number of actions
        % action i: transmit user i
        n_action;
        
        
        % number of state-action paris, which is equal to n_state*n_action
        % Let N = n_state, K = n_action, then the mapping is as follows
        % state-action 1                       (state 1, action 1)
        % state-action 2                       (state 1, action 2)
        % state-action K                       (state 1, action K)
        % state-action K+1                     (state 2, action 1)
        % state-action K+2                     (state 2, action 2)
        % state-action 2K                      (state 2, action K)
        % state-action 2K+1                    (state 3, action 1)
        % state-action 2K+2                    (state 3, action 2)
        % state-action 3K                      (state 3, action K)
        % state-action (N-1)K+1                (state N, action 1)
        % state-action (N-1)K+2                (state N, action 2)
        % state-action NK                      (state N, action K)
        n_state_action;
       
        % Dimension: zeros(n_state, n_state_action)
        % Let N = n_state, K = n_action, then n_state_action=NK
        % Psi matrix specifies which state is the state_action_idx under
        % Psi(i,j) means that state_action_idx j is under state i
        Psi;
        
        % Dimension: zeros(n_state_action,n_state, period_lcm)
        % Pv is the vectorized transition matrix vectroized
        % Pv(i,j,t) is the transition probablity from state_action_idx i to
        % state j at slot t (in the steady state), i.e., we should treat t as  t + period_lcm
        Pv; 
        
        % Demension: zeros(n_state_action, n_flow)
        % theta is the vectorized reward matrix
        % theta(i,k) is the flow-k reward under state_action_idx i
        theta;
    end
    
    methods
        
        function  obj=DownlinkAPInstance()
            %constructor function
        end
        
        
        function [ret] = isValid(obj)
            if(obj.n_flow ~= length(obj.flow_array))
                ret = -1;
                error('wrong input');
            end
            ret = 1;
        end
        
        
        function [ret] = stateSanityCheck(obj)
            for ss=1:obj.n_state
                ss_vec = obj.getVectorState(ss);
                ss_back = obj.getStateFromVector(ss_vec);
                fprintf('ss=%d,ss_vec=[ ', ss);
                for ii=1:obj.n_flow
                    fprintf('%d, ', ss_vec(ii));
                end
                fprintf('], ss_back=%d\n', ss_back);
                if(ss ~= ss_back)
                    error('something wrong when ss=%d, ss_back=%d', ss, ss_back);
                end
            end
            ret = 1;
        end
        
        %write to the file
        function [ret] = stateSanityCheck_file(obj,fileID)
            for ss=1:obj.n_state
                ss_vec = obj.getVectorState(ss);
                ss_back = obj.getStateFromVector(ss_vec);
                fprintf(fileID, 'ss=%d,ss_vec=[ ', ss);
                for ii=1:obj.n_flow
                    fprintf(fileID, '%d, ', ss_vec(ii));
                end
                fprintf(fileID, '], ss_back=%d\n', ss_back);
                if(ss ~= ss_back)
                    error('something wrong when ss=%d, ss_back=%d', ss, ss_back);
                end
            end
            ret = 1;
        end
        
        function [ret] = stateActionSanityCheck(obj)
            for ii=1:obj.n_state_action
                [state, action] = obj.getStateActionFromIdx(ii);
                ii_back = obj.getIdxFromStateAction(state,action);
                fprintf('ii=%d, (state %d, action %d), ii_back=%d\n', ii, state, action, ii_back);
                if(ii ~= ii_back)
                    error('something wrong when ii=%d, ii_back=%d', ii, ii_back);
                end
            end
            ret = 1;
        end
        
        %write to the file
        function [ret] = stateActionSanityCheck_file(obj,fileID)
            for ii=1:obj.n_state_action
                [state, action] = obj.getStateActionFromIdx(ii);
                ii_back = obj.getIdxFromStateAction(state,action);
                fprintf(fileID,'ii=%d, (state %d, action %d), ii_back=%d\n', ii, state, action, ii_back);
                if(ii ~= ii_back)
                    error('something wrong when ii=%d, ii_back=%d', ii, ii_back);
                end
            end
            ret = 1;
        end
        
        
        
        function constructEverything(obj)
            if(isValid(obj) == -1)
                return;
            end
            
            obj.period_lcm = obj.flow_array{1}.period;
            obj.n_state = obj.flow_array{1}.n_state;
            for ii=1:obj.n_flow-1
                obj.period_lcm = lcm(obj.period_lcm, obj.flow_array{ii+1}.period); %number of slots per (large) period
                obj.n_state = obj.n_state*obj.flow_array{ii+1}.n_state;
            end
            obj.n_action = obj.n_flow;
            obj.n_state_action = obj.n_state*obj.n_action;
            tic;
            obj.Psi = obj.constructPsi();
            fprintf('obj.constructPsi()\n');
            toc;
            tic;
            obj.Pv = obj.constructPv();
            fprintf('obj.constructPv()\n');
            toc;
            tic;
            obj.theta = obj.constructTheta();
            fprintf('obj.constructTheta()\n');
            toc;
        end
        
        function [state] = getStateFromVector(obj, state_vec)
            if(length(state_vec) ~= obj.n_flow)
                error('wrong input');
            end
            
            for ii=1:obj.n_flow
                if(state_vec(ii) <= 0 || state_vec(ii) > obj.flow_array{ii}.n_state)
                    error('wrong input %d', ii);
                end
            end
            
            reverse_base = ones(obj.n_flow,1);
            for ii=obj.n_flow-1:-1:1
                reverse_base(ii) = reverse_base(ii+1)*obj.flow_array{ii+1}.n_state;
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
                state_size(ii) = obj.flow_array{ii}.n_state;
            end
            
            reverse_base = ones(obj.n_flow,1);
            for ii=obj.n_flow-1:-1:1
                reverse_base(ii) = reverse_base(ii+1)*obj.flow_array{ii+1}.n_state;
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
        
        function [state_action_idx] = getIdxFromStateAction(obj, state, action)
            if(state <=0 || state > obj.n_state ...
               || action <= 0 || action > obj.n_action)
                error('wrong input');
            end
            state_action_idx = (state-1)*obj.n_action + action;
        end
        
        function [state, action] = getStateActionFromIdx(obj, state_action_idx)
            if(state_action_idx <= 0 || state_action_idx > obj.n_state_action)
                error('wrong input');
            end
            state = ceil(state_action_idx/obj.n_action);
            action = state_action_idx - (state-1)*obj.n_action;
        end
        
        function [slot] = getFirstPeriodSlot(obj, t)
            if( t <= 0 )
                error('wrong input');
            end
            slot = rem(t, obj.period_lcm);
            
            %if t = k * obj.period_lcm, then the reminder is 0 and we
            %should map it to slot obj.period_lcm
            if(slot == 0)
                slot = obj.period_lcm;
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
                    prob = prob*obj.flow_array{ii}.getTransitionProbability(tt, state_vec(ii), 1, next_state_vec(ii));
                else
                    prob = prob*obj.flow_array{ii}.getTransitionProbability(tt, state_vec(ii), 2, next_state_vec(ii));
                end
            end
        end
        
        %get the transition prob for flow kk from flow-kk state to flow-kk
        %next_state under action (the whole system action, rather than
        %flow-kk action)
        function [prob] = getTransitionProbabilityPerFlowState(obj, tt, kk, state, action, next_state)
            if(tt <=0 || state <=0 || state > obj.flow_array{kk}.n_state || action <=0 || action > obj.n_action ...
                    || next_state <=0 || next_state > obj.flow_array{kk}.n_state)
                error('wrong input');
            end
            if(action == kk)
                prob = obj.flow_array{kk}.getTransitionProbability(tt, state, 1, next_state);
            else
                prob = obj.flow_array{kk}.getTransitionProbability(tt, state, 2, next_state);
            end
        end
        
        
        %get the full transition probablity matrix at some slot tt, which is useful to look
        %up the tabel
        function [prob_matrix] = getOneSlotTransitionProbabilityMatrix(obj,tt)
            if(tt <=0)
                error('wrong input');
            end
            prob_matrix = zeros(obj.n_state, obj.n_action, obj.n_state);
            for state=1:obj.n_state
                for action=1:obj.n_action
                    for next_state=1:obj.n_state
                       prob_matrix(state,action,next_state) = obj.getTransitionProbability(tt, state, action, next_state);
                    end
                end
            end
        end
        
        %Note that state is the global state
        function [reward] = getRewardPerFlow(obj, state, action, k)
            if( state <=0 || state > obj.n_state || action <=0 || action > obj.n_action ...
                          || k <=0 || k > obj.n_flow)
                      error('wrong input');
            end
            state_vec = obj.getVectorState(state);
            if(action == k)
                reward = obj.flow_array{k}.reward_per_state_per_action(state_vec(k), 1);
            else
                reward = obj.flow_array{k}.reward_per_state_per_action(state_vec(k), 2);
            end
        end
        
        
        %get the reward for flow k with flow-k state and gobal action 
        % Note state is flow-k state!!!
        function [reward] = getRewardPerFlowState(obj, state, action, k)
            if( state <=0 || state > obj.flow_array{k}.n_state || action <=0 || action > obj.n_action ...
                    || k <=0 || k > obj.n_flow)
                error('wrong input');
            end
            if(action == k)
                reward = obj.flow_array{k}.reward_per_state_per_action(state, 1);
            else
                reward = obj.flow_array{k}.reward_per_state_per_action(state, 2);
            end
        end
        
        
        %get the reward matrix for flow k
        function [reward_matrix] = getRewardMatrixPerFlow(obj, k)
            if(k <=0 || k > obj.n_flow)
                error('wrong input');
            end
            reward_matrix = zeros(obj.n_state, obj.n_action);
            for state=1:obj.n_state
                for action=1:obj.n_action
                    reward_matrix(state,action) = obj.getRewardPerFlow(state,action,k);
                end
            end
        end
        
        function [next_state, isTransmitted, isSuccessful] = oneSlotRealization(obj, t, state, action)
            isTransmitted = zeros(obj.n_flow,1);
            isSuccessful = zeros(obj.n_flow,1);
            state_vec = obj.getVectorState(state);
            next_state_vec = zeros(1,obj.n_flow);
            for kk=1:obj.n_flow
                if(action == kk)
                    [next_state_vec(kk), isTransmitted(kk), isSuccessful(kk)] = obj.flow_array{kk}.oneSlotRealization(t,state_vec(kk),1);
                else
                    [next_state_vec(kk), isTransmitted(kk), isSuccessful(kk)] = obj.flow_array{kk}.oneSlotRealization(t,state_vec(kk),2);
                end
            end
            next_state = obj.getStateFromVector(next_state_vec);
        end
        
        function [init_state] = getInitialState(obj)
            init_state_vec = zeros(1,obj.n_flow);
            for kk=1:obj.n_flow
                init_state_vec(kk) = obj.flow_array{kk}.getInitialState();
            end
            init_state = obj.getStateFromVector(init_state_vec);
        end
        
        function [Psi] = constructPsi(obj)      
            Psi = zeros(obj.n_state, obj.n_state_action);
            
            for idx=1:obj.n_state_action
                [state, action] = obj.getStateActionFromIdx(idx);
                Psi(state, idx) = 1;
            end
        
            
            % use Kronecker product
            %Psi = kron(eye(obj.n_state), ones(1, obj.n_action));
        end
        
        function [Pv] = constructPv(obj)
            
%             %use loop to contruct PV, consume a lot of time!!
%             Pv = zeros(obj.n_state_action, obj.n_state, obj.period_lcm);
%             for hh=1:obj.period_lcm
%                 for idx=1:obj.n_state_action
%                     [state,action]=obj.getStateActionFromIdx(idx);
%                     for next_state=1:obj.n_state
%                         Pv(idx,next_state,hh) = obj.getTransitionProbability(hh+obj.period_lcm,state,action,next_state);
%                     end
%                 end
%             end
            
            Pv = zeros(obj.n_state_action, obj.n_state, obj.period_lcm);
            for hh=1:obj.period_lcm
                %tranition matrix for flow 1-kk for each individual action
                current_Pv = cell(obj.n_action,1);
                
                for aa=1:obj.n_action
                    for state = 1:obj.flow_array{1}.n_state;
                        for next_state = 1:obj.flow_array{1}.n_state;
                            if(aa == 1)
                                current_Pv{aa}(state, next_state) = obj.flow_array{1}.getTransitionProbability(hh+obj.period_lcm, state, 1, next_state);
                            else
                                current_Pv{aa}(state, next_state) = obj.flow_array{1}.getTransitionProbability(hh+obj.period_lcm, state, 2, next_state);
                            end
                        end
                    end
                end
                
                for kk=1:obj.n_flow-1
                    %flow kk+1 transition matrix
                    next_flow_Pv = cell(obj.n_action,1);
                    for aa=1:obj.n_action
                        for state = 1:obj.flow_array{kk+1}.n_state;
                            for next_state = 1:obj.flow_array{kk+1}.n_state;
                                if(aa == kk+1)
                                    next_flow_Pv{aa}(state, next_state) = obj.flow_array{kk+1}.getTransitionProbability(hh+obj.period_lcm, state, 1, next_state);
                                else
                                    next_flow_Pv{aa}(state, next_state) = obj.flow_array{kk+1}.getTransitionProbability(hh+obj.period_lcm, state, 2, next_state);
                                end
                            end
                        end
                    end
                    
                    %use kron product to update current_Pv
                    for aa=1:obj.n_action
                        current_Pv{aa} = kron(current_Pv{aa}, next_flow_Pv{aa});
                    end
                end
                
                %"shuffle the row of current_Pv such that its indicies are in
                %line with the state_action representation
                for aa=1:obj.n_action
                    Pv(aa:obj.n_action:end, :, hh) = current_Pv{aa};
                end
            end
        end
        
        function [theta] = constructTheta(obj)
            theta = zeros(obj.n_state_action, obj.n_flow);
            for kk=1:obj.n_flow
                for idx=1:obj.n_state_action
                    [state, action] = obj.getStateActionFromIdx(idx);
                    theta(idx,kk) = obj.getRewardPerFlow(state,action,kk);
                end
            end
        end
        
        %get the intensity of an interval [t1,t2], where t1 < t2, both in
        %terms of at the beginning. (see YDS algorithm)
        function [intensity] = getIntervalIntensity(obj,t1,t2)
            if( t1 >= t2)
                error('wrong input');
            end
            intensity = 0;
            for kk=1:obj.n_flow
                intensity = intensity + obj.flow_array{kk}.getNumberOfPacketsWithinInterval(t1,t2);
            end
            intensity = intensity/(t2-t1);
        end
    end
    
end