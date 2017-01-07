classdef NonOverlappedFlowInstance < handle
    % A flow instance, which is offseted perodic iid traffic
    % Note that here it is non-overlapped flow instance and thus we can use
    % a binary 0 or 1 to indicate its state, in other words, the number of
    % states is greatly reduced from 2^D to 2.
    
    properties (SetAccess = public, GetAccess = public)
        
        % the offset of the first packet of the flow
        % Dimension: zeros(1,1)
        offset;
        
        % the period of the flow
        % Dimension: zeros(1,1)
        period;
        
        % the delay of each flow, delay(ii) <= period (ii) because we only consider non-overlapping traffic
        % Dimension:zeros(1,1)
        delay;
        
        % the arrival probability in the first slot of one period
        %Dimension: zeros(1,1);
        arrival_prob;
        
        % the probability to successfully deliver a packet of each flow if scheduled
        % Dimension: zeros(1,1)
        success_prob;
        
        
        % number of states, in terms of whether it has one packet of not
        % state 1 means no packet, i.e., 0 in binary
        % state 2 means has packet, i.e., 1 in binary
        n_state = 2;
        
        % number of actions
        % action 1: transmit this flow
        % action 2: do not transmit this flow
        n_action = 2;

        % transition probability only due to transmission, since TX is
        % idependent of time, i.e., transmition_matrix_tx is the same for
        % all slots, we will ignore the time dimension here.
        % Dimension:  zeros(n_state, n_action, n_state);
        % transmition_matrix_tx(i, a, j) is the probability from state i under action a at the beginnong of any slot t
        % to the state j at the end of the slot t if we only consider transmission but ignore A&E
        transition_matrix_tx;
       
        
        % transition probability only due to A&E. 
        % Note that this only happens at the END of each slot, i.e., after TX transition probability
        % Since A&E is periodic with period period, we will only
        % consider the first period, i.e., from slot 1 to slot period.
        % For any time slot t, not that transition_matrix_ae(t, i, j) = transition_matrix_ae(rem(t, period), i, j)
        % Dimension: zeros(period, n_state, n_state);
        % transition_matrix_ae(t, i, j) is the probability from state i at the end of slot t to
        % state j at the beginnig of slot t+1 if we only consider A&E but ignore transimission. 
        % This expiration is determinstic and the arrival is random
        transition_matrix_ae;
       
        
        % transition probability due to both transmission and A&E (first TX
        % from the beginning of each slot to the end of the slot, then AE
        % from the end of the slot to the begnning of next slot)
        % Since TX is independnt of slots (or periodic with period 1) and A&E is periodic with period period_lcm, we will only
        % consider the first period, i.e., from slot 1 to slot period.(BUT
        % remember here we ignore the offset! So it is actuall from slot offset+1 to offset+period)
        % For any time slot t, not that transition_matrix(t, i, a, j) = transition_matrix(rem(t, period), i, a, j)
        % Dimension:  zeros(period, n_state, n_action, n_state)
        % transmition_matrix(t, i, a, j) is the probability from state i under action a at the beginning of slot t
        % to the state j at the begnning of slot t+1 if we consider both transmission and A&E
        transition_matrix;
        
        % the reward per slot for each state-action
        % Dimension: zeros(n_state, n_action)
        reward_per_state_per_action;
    end
    
    methods
        
        function  obj=NonOverlappedFlowInstance()
            %constructor function
        end
        
        
        %judge whether the input parameters are valid
        function [ret] = isValid(obj)
            ret = 1;
            if(obj.period <= 0 || obj.delay <= 0 || obj.delay > obj.period || ...
                   obj.success_prob <= 0 || obj.success_prob > 1 || length(obj.arrival_prob) ~= 1 ...
                   || obj.arrival_prob <= 0 || obj.arrival_prob > 1)
                ret = -1;
                error('wrong input');
            end
        end
        
        
        %calculate some other parameters according to the input parameters
        function calculateParameters(obj)
            obj.n_state = 2;
            obj.n_action = 2;
        end
        
        function constructEverything(obj)
            if(isValid(obj) == -1)
                return;
            end
            
            calculateParameters(obj);
            constructTransitionMatrixTX(obj);
            constructTransitionMatrixAE(obj);
            constructTransitionMatrix(obj);
            constructRewardPerStatePerAction(obj);
        end
        
%         %convert state into binary representation
%         function [state_bin] = getBinaryState(obj, state)
%             if(state < 1 || state > obj.n_state)
%                 error('wrong input');
%             end
%             state_bin = de2bi(state-1, obj.delay, 'left-msb');
%         end
%         
%         %convert binary representation into state
%         function [state] = getStateFromBinary(obj, state_bin)
%             if(length(state_bin) ~= obj.delay)
%                 error('wrong input');
%             end
%             state = bi2de(state_bin, 'left-msb')+1;
%         end
        
        %judge if the flow has one packet at the input state
        % recall state 2 means has packet while state 1 means no packet
        function [ret] = hasPacket(obj, state)
            if(state == 2)
                ret = 1;
            elseif(state == 1)
                ret = -1;
            else
                error('wrong input');
            end
        end
        
        
        function [slot] = getFirstPeriodSlot(obj, t)
            if( t <= obj.offset) %we do not consider the slots before offset
                error('wrong input');
            end
            slot = rem(t-obj.offset, obj.period);
            
            %if t = k * obj.period, then the reminder is 0 and we
            %should map it to slot obj.period
            if(slot == 0)
                slot = obj.period;
            end
        end
        
%         %find the packet with least lead time under the input state
%         function [least_lead_time] = findLeastLeadTimePacket(obj, state)
%             if(state < 0 || state > obj.n_state)
%                 error('wrong input');
%             end
%             state_bin = obj.getBinaryState(state);
%             least_lead_time = 0;
%             for ii=1:obj.delay
%                 if(state_bin(ii) == 1)
%                     least_lead_time = ii;
%                     break;
%                 end
%             end
%         end
%         

        %get the next expiration slot if the current slot is t and current
        %state is state, 
        % return slot=t, means that the packet will expire in the BEGINNING
        % of slot t
        function [slot] = getNextExpirationSlot(obj, t, state)
            if( t <= 0 || state <=0 || state > obj.n_state)
                error('wrong input');
            end
            if(state == 2)
                %has packet
                % get the period index for current slot t
                period_ind = ceil((t-obj.offset)/obj.period);
                slot = obj.offset + obj.period*(period_ind-1) + obj.delay + 1;
            elseif(state == 1) %no packet
                slot = +inf; %never expire
            else
                error('wrong input');
            end        
        end
      
        
        %get the index of last arrived packet before current time slot t
        %that is, \argmax_{m} t_arr(m) <= t (note that it is not striclty less than)
        % return 0 if no last arrived packet
        function [last_packet] = getLastArrivePacket(obj, t)
            if ( t < obj.offset  + 1)
                last_packet = 0;
                return;
            end
            last_packet = floor((t-obj.offset-1)/obj.period + 1);
        end
        
        %get the index of last expired packet before current time slot t
        %that is, \argmax_{m} t_exp(m) <= t (note that it is not striclty less than)
        % return 0 if no last expired packet
        function [last_packet] = getLastExpirePacket(obj, t)
            if ( t < obj.offset  + 1 + obj.delay)
                last_packet = 0;
                return;
            end
            last_packet = floor((t-obj.offset-1-obj.delay)/obj.period + 1);
        end
        
        %get the index of next arriving packet after current time slot t
        %that is, \argmin_{m} t_arr(m) >= t (note that it is not striclty greater than)
        function [next_packet] = getNextArrivePacket(obj, t)
            if ( t < obj.offset  + 1)
                next_packet = 1;
                return;
            end
            next_packet = ceil((t-obj.offset-1)/obj.period + 1);
        end
        
        %get the index of next expiring packet after current time slot t
        %that is, \argmin_{m} t_exp(m) >= t (note that it is not striclty greater than)
        function [next_packet] = getNextExpirePacket(obj, t)
            if ( t < obj.offset  + 1 + obj.delay)
                next_packet = 1;
                return;
            end
            next_packet = ceil((t-obj.offset-1-obj.delay)/obj.period + 1);
        end
        
        
        %get the number of packets within interval [t1,t2], at the
        %beginning of t1 and at the beginning of t2
        % One packet is whitin [t1,t2] if both the arrival time and
        % expiration time are within [t1,t2]
        function [num_packet] = getNumberOfPacketsWithinInterval(obj,t1,t2)
            if(t1 >= t2)
                error('wrong input');
            end
            n1 = obj.getNextArrivePacket(t1);
            n2 = obj.getLastExpirePacket(t2);
            if(n2 == 0)
                num_packet = 0;
            else
                num_packet = max(n2-n1+1, 0);
            end
        end
        
        
        function [init_state] = getInitialState(obj)
            if(obj.offset == 0)
                if(rand < obj.arrival_prob)
                    init_state = 2;
                else
                    init_state = 1;
                end
            else
                init_state = 1;
            end
        end
        
        %construct the transition matrix due to TX
        function constructTransitionMatrixTX(obj)   
            %transmition_matrix_tx(i,a,j) is the probability from state i under action a at the beginning of any slot t
            %to the state j at the end slot t if we only consider
            %transmission and expiration but ignore arrival
            obj.transition_matrix_tx = zeros(obj.n_state, obj.n_action, obj.n_state);
            
            % if the state is 1 (no packet), then with probability 1 to go
            % to state 1 again
            obj.transition_matrix_tx(1,:,1) = 1;
            
            % if state is 2 (there exists 1 packet)
            
            % if the action is 1, i.e., transmit this flow
            obj.transition_matrix_tx(2, 1, 2) = 1- obj.success_prob;
            obj.transition_matrix_tx(2, 1, 1) = obj.success_prob;
            
            % if the action is 2, i.e., do not transmit this flow
            obj.transition_matrix_tx(2, 2, 2) = 1;
              
            
            %sanity check
            for ss=1:obj.n_state
                for aa=1:obj.n_action
                    if(abs(sum(obj.transition_matrix_tx(ss,aa,:)) - 1) > 1e-6)
                        dist = abs(sum(obj.transition_matrix_tx(ss,aa,:)) - 1)
                        error('wrong probability for state %d action %d', ss, aa);
                    end
                end
            end
        end
        
        %construct the transition matrix due to A&E
        function constructTransitionMatrixAE(obj)
            
            obj.transition_matrix_ae = zeros(obj.period, obj.n_state, obj.n_state);
            %let us consider two cases
            % case I: delay = period, the packet will expired and the new
            % packet will come with arrivl_prob
            if( obj.delay == obj.period)
                for tt=1:obj.period
                    if( tt == obj.delay)
                        obj.transition_matrix_ae(tt, :, 1) = 1-obj.arrival_prob;
                        obj.transition_matrix_ae(tt, :, 2) = obj.arrival_prob; 
                    else % not the expiration/arrival time
                        obj.transition_matrix_ae(tt, 1, 1) = 1;
                        obj.transition_matrix_ae(tt, 2, 2) = 1;
                    end
                end
            % case II: delay ~= period, i.e., delay < period
            else
                for tt=1:obj.period
                    if( tt == obj.delay) %only expiration 
                        obj.transition_matrix_ae(tt, :, 1) = 1;
                    elseif (tt == obj.period) % only arrival
                        obj.transition_matrix_ae(tt, :, 1) = 1-obj.arrival_prob;
                        obj.transition_matrix_ae(tt, :, 2) = obj.arrival_prob; 
                    else % not arrival not expiration
                         obj.transition_matrix_ae(tt, 1, 1) = 1;
                        obj.transition_matrix_ae(tt, 2, 2) = 1;
                    end
                end
            end
            
            %sanity check
            for tt=1:obj.period
                for ss=1:obj.n_state
                    if(abs(sum(obj.transition_matrix_ae(tt,ss,:)) - 1) > 1e-6)
                        dist = abs(sum(obj.transition_matrix_ae(tt,ss,:)) - 1)
                        error('wrong probability for time %d state %d', tt, ss);
                    end
                end
            end
        end
        
        %construct the transition matrix by considering both TXE and A
        function constructTransitionMatrix(obj)
            %transition probability due to both transmission and A&E
            obj.transition_matrix = zeros(obj.period, obj.n_state, obj.n_action, obj.n_state);
            for tt=1:obj.period
                for ss=1:obj.n_state
                    for aa=1:obj.n_action
                        for next_state=1:obj.n_state
                            tran_prob_temp = 0;
                            for txe_state=1:obj.n_state
                                tran_prob_temp = tran_prob_temp + (obj.transition_matrix_tx(ss,aa,txe_state))*(obj.transition_matrix_ae(tt,txe_state,next_state));
                            end
                            obj.transition_matrix(tt,ss, aa, next_state) = tran_prob_temp;
                        end
                    end
                end
            end
            
            %sanity check
            for tt=1:obj.period
                for ss=1:obj.n_state
                    for aa=1:obj.n_action
                        if(abs(sum(obj.transition_matrix(tt,ss,aa,:)) - 1) > 1e-6)
                            dist = abs(sum(obj.transition_matrix(tt,ss,:)) - 1)
                            error('wrong probability for tt %d state %d action %d', tt, ss, aa);
                        end
                    end
                end
            end
        end
        
        %construct the reward per reward per state. Equation 11 in Mobihoc 2015
        % Note that this reward is not related to any coefficient or the
        % period. To use them, we need to normalized by coefficient if weighted-sum utility is considered.
        function constructRewardPerStatePerAction(obj)   
            obj.reward_per_state_per_action = zeros(obj.n_state, obj.n_action);
            %state 1, no packets
            obj.reward_per_state_per_action(1,:) = 0;
            %state 2 has packet, action 1 is to transmit this flow
            obj.reward_per_state_per_action(2,1) = obj.success_prob;
            %state 2 has packet, action 2 is not to transmit this flow
            obj.reward_per_state_per_action(2,2) = 0;
        end
        
        function [prob] = getTransitionProbability(obj, t, state, action, next_state)
            if(t <=0 || state <=0 || state > obj.n_state || action <=0 || action > obj.n_action ...
                    || next_state <=0 || next_state > obj.n_state)
                error('wrong input');
            end
            if( t < obj.offset)
                if(next_state == 1) %it comes to state 1 no matter what state and what action is
                    prob = 1;
                else
                    prob = 0;
                end
            elseif(t == obj.offset)
                prob = obj.transition_matrix(obj.getFirstPeriodSlot(t+obj.period), state, action, next_state);
            else %% t > obj.offset
                prob = obj.transition_matrix(obj.getFirstPeriodSlot(t), state, action, next_state);
            end   
        end
        
        %the realization of one-slot transition, given the input slot,
        %input state and input action
        % output: next_state is the next state based on this realization
        %         isTransmitted 1 if one packet is transmitted
        %         isSuccessful 1 if one packet is transmitted and the transmission is successful
        function [next_state, isTransmitted, isSuccessful] = oneSlotRealization(obj, t, state, action)
            if(action == 1) %transmit this flow
                if(state == 1) %do not have any packet
                    isTransmitted = -1;
                    isSuccessful = -1;
                    next_state = obj.oneSlotRealizationOnlyForState(t, state, action);
                elseif(state == 2) % has one packet, i.e. state = 2
                    isTransmitted = 1;
                    transmission_probability_temp = rand;
                    if(transmission_probability_temp < obj.success_prob) % the tranmission is successful with prob success_prob
                        isSuccessful = 1;
                        %we need carefully get the next state, this is ugly
                        %here because we need to re-implement the
                        %realization similar to the construction of
                        %transition probability
                        
                        % the only packet has been transmitted succesfully,
                        % only need to consider arrival
                        next_state = 1;
                        
                        %consider the arrival, come to the beginning of the
                        %next slot, i.e., t+1 

                        if(t >= obj.offset && obj.getFirstPeriodSlot(t+obj.period) == obj.period ) % next slot arrival prob!! in the last slot of one period, thus will have arrival in the next slot (the new period)
                            arrival_prob_temp = rand;
                            if(arrival_prob_temp < obj.arrival_prob)
                                next_state = 2; %has one arrival
                            else
                                next_state = 1;
                            end
                        end
                    else % the transimission is failed
                        isSuccessful = -1;
                        %we need carefully get the next state, this is ugly
                        %here because we need to re-implement the
                        %realization similar to the construction of
                        %transition probability
                        if(obj.getNextExpirationSlot(t,state) ~= t+1) % the packet will not be expired
                            next_state = 2;
                        else % the packet will be expired
                             next_state = 1;
                            %consider the arrival, come to the beginning of the
                            %next slot, i.e., t+1
                            if(t >= obj.offset && obj.getFirstPeriodSlot(t+obj.period) == obj.period ) % next slot arrival prob!!
                                arrival_prob_temp = rand;
                                if(arrival_prob_temp < obj.arrival_prob)
                                    next_state = 2; %has one arrival
                                else
                                    next_state = 1;
                                end
                            end
                        end
                    end
                else
                    error('wrong input');
                end
            elseif(action == 2) %do not transmit this flow
                isTransmitted = -1;
                isSuccessful = -1;
                next_state = obj.oneSlotRealizationOnlyForState(t, state, action);
            else
                error('wrong input');
            end
        end
        
        %the realization of one-slot transition, given the input slot,
        %input state and input action. Here we only care about next state.
        % output: next_state is the next state based on this realization
        function [next_state] = oneSlotRealizationOnlyForState(obj, t, state, action)
            next_state_prob = zeros(1,obj.n_state);
            for ss=1:obj.n_state
                next_state_prob(ss) = obj.getTransitionProbability(t, state, action, ss);
            end
            
            probability_temp = rand;
            next_state = -1;
            for ss=1:obj.n_state
                if( probability_temp <= sum(next_state_prob(1:ss)))
                    next_state = ss;
                    break;
                end
            end
            if(next_state == -1)
                error('something wrong');
            end
        end
       
    end

end