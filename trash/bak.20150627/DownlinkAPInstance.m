classdef DownlinkAPInstance < handle
    %DownlinkAPInstance: a class for the downlink AP scenario where each
    %slot can only scheduling one user/flow
    % we consider the non-overlapped traffic pattern
    
    properties (SetAccess = public, GetAccess = public)
        
        %number of users/flows
        n_flow;
        
        % the offset of the first packet of each flow
        % Dimension: zeros(n_flow,1)
        offset;
        
        % the period of each flow
        % Diemsion: zeros(n_flow,1)
        period;
        
        % the delay of each flow, delay(ii) <= period (ii) because we only consider non-overlapping traffic
        % Diemsion:zeros(n_flow,1)
        delay;
        
        % the probability to successfully deliver a packet of each flow if scheduled
        % Dimension: zeros(n_flow,1)
        success_prob;
        
        period_lcm;
        
        % number of states, in terms of relative index, there are at most one packet for each flow because of non-overlapping traffic
        % state is from 1 to 2^n_flow, which represents from (00...0) to (11...1)
        % that is to say state i is the binary representation of decimal integer i-1
        % we use the left most bit to reprensent user 1 and the right most bit to repensent user n_flow
        
        % For example: n_flow =2  ('_b' means base 2, '_d' means base 10)
        % state 1 = (00)_b = (0)_d,  user 1 has 0 packet and user 2 has 0 packet, de2bi(1-1,2,'left-msb')=[0 0], bi2de([0 0], 'left-msb')+1=1
        % state 2 = (01)_b = (1)_d,  user 1 has 0 packet and user 2 has 1 packet  de2bi(2-1,2,'left-msb')=[0 1], bi2de([0 1], 'left-msb')+1=2
        % state 3 = (10)_b = (2)_d,  user 1 has 1 packet and user 2 has 0 packet, de2bi(3-1,2,'left-msb')=[1 0], bi2de([1 0], 'left-msb')+1=3
        % state 4 = (11)_b = (3)_d,  user 1 has 1 packet and user 2 has 1 packet, de2bi(4-1,2,'left-msb')=[1 1], bi2de([1 1], 'left-msb')+1=4
        n_state;
        
        % number of actions
        % action i: transmit user i
        n_action;

        % transition probability only due to transmission, since TX is
        % idependent of time, i.e., transmition_matrix_tx is the same for
        % all slots, we will ignore the time dimension here.
        % Dimension:  zeros(n_state, n_action, n_state);
        % transmition_matrix_tx(i, a, j) is the probability from state i under action a at the beginnong of any slot t
        % to the state j at the end of the slot t if we only consider transmission but ignore A&E
        transition_matrix_tx;
        
        
        % transition probability only due to A&E. 
        % Note that this only happens at the END of each slot, i.e., after TX transition probability
        % Since A&E is periodic with period period_lcm, we will only
        % consider the first period, i.e., from slot 1 to slot period_lcm.
        % For any time slot t, not that transition_matrix_ae(t, i, j) = transition_matrix_ae(rem(t, period_lcm), i, j)
        % Dimension: zeros(period_lcm, n_state, n_state);
        % transition_matrix_ae(t, i, j) is the probability from state i at the end of slot t to
        % state j at the beginnig of slot t+1 if we only consider A&E but ignore transimission. 
        % This matrix is determinstic in the sense that the probability is either 0 or 1
        transition_matrix_ae;
        
        % transition probability due to both transmission and A&E (first TX
        % from the beginning of each slot to the end of the slot, then AE
        % from the end of the slot to the begnning of next slot)
        % Since TX is independnt of slots (or periodic with period 1) and A&E is periodic with period period_lcm, we will only
        % consider the first period, i.e., from slot 1 to slot period_lcm.
        % For any time slot t, not that transition_matrix(t, i, a, j) = transition_matrix(rem(t, period_lcm), i, a, j)
        % Dimension:  zeros(period_lcm, n_state, n_action, n_state)
        % transmition_matrix(t, i, a, j) is the probability from state i under action a at the beginning of slot t
        % to the state j at the begnning of slot t+1 if we consider both transmission and A&E
        transition_matrix;
        
        % the reward per slot for each state-action
        % Dimension: zeros(n_state, n_action)
        reward_per_state_per_action;
    end
    
    methods
        
        function  obj=DownlinkAPInstance()
            %constructor function
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
        
        %get the initial state at slot 1
        function [init_state] = getInitialState(obj)
             init_state_bin = zeros(1,obj.n_flow);
             for nn=1:obj.n_flow
                 if(obj.offset(nn) == 0)
                     init_state_bin(nn) = 1;
                 end
             end
              init_state = bi2de(init_state_bin, 'left-msb')+1;
        end
        
        %conver state into binary representation
        function [state_bin] = getBinaryState(obj, state)
            if(state < 1 || state > obj.n_state)
                error('wrong input');
            end
            state_bin = de2bi(state-1, obj.n_flow, 'left-msb');
        end
        
        %conver binary representation into state
        function [state] = getStateFromBinary(obj, state_bin)
            if(length(state_bin) ~= obj.n_flow)
                error('wrong input');
            end
            state = bi2de(state_bin, 'left-msb')+1;
        end
        
        %judge if the input flow  has one packet at the input state
        function [ret] = hasPacket(obj, state, flow)
            state_bin = getBinaryState(obj,state);
            ret = state_bin(flow);
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
        
        %judge whether the input parameters are valid
        function [ret] = isValid(obj)
            ret = 1;
            for nn=1:obj.n_flow
                if(obj.period(nn) <= 0 || obj.delay(nn) <= 0 || obj.delay(nn) > obj.period(nn) ...
                   || obj.success_prob(nn) <=0 || obj.success_prob(nn) >=1)
                    ret = -1;
                    error('wrong input for flow %d', nn);
                end
            end
        end
        
        
        
        %calculate some other parameters according to the input parameters
        function calculateParameters(obj)
            obj.period_lcm  = obj.period(1);
            for ii=1:obj.n_flow-1
                obj.period_lcm = lcm(obj.period_lcm, obj.period(ii+1)); %number of slots per (large) period
            end
            
            obj.n_state = 2^obj.n_flow;
            obj.n_action = obj.n_flow;
        end
        
        %construct the transition matrix due to TX
        function constructTransitionMatrixTX(obj)   
            %transition probability only due to transmission
            obj.transition_matrix_tx = zeros(obj.n_state, obj.n_action, obj.n_state);
            %transmition_matrix_tx(i,a,j) is the probability from state i under action a at any slot t
            %to the state j at slot t+1 if we only consider transmission but ignore A&E
            for ss=1:obj.n_state
                ss_bin = de2bi(ss-1, obj.n_flow, 'left-msb');
                for aa=1:obj.n_action
                    if(ss_bin(aa) == 1) %there exists one packet at state ss for flow aa
                        next_state_success_bin = ss_bin; %the next state if flow aa has successfully delivered one packet
                        next_state_success_bin(aa) = 0;
                        next_state_success = bi2de(next_state_success_bin, 'left-msb')+1;
                        obj.transition_matrix_tx(ss, aa, ss) = 1- obj.success_prob(aa);
                        obj.transition_matrix_tx(ss, aa, next_state_success) = obj.success_prob(aa);
                    else %there does not exist any packet at state ss for flow aa
                        obj.transition_matrix_tx(ss, aa, ss)  = 1;
                    end
                end
            end
            
            %sanity check
            for ss=1:obj.n_state
                for aa=1:obj.n_action
                    if(sum(obj.transition_matrix_tx(ss,aa,:)) ~= 1)
                        error('wrong probability for state %d action %d', ss, aa);
                    end
                end
            end
        end
        
        %construct the transition matrix due to A&E
        function constructTransitionMatrixAE(obj) 
            %transition_matrix_ae is the transition probability only due to A&E
            obj.transition_matrix_ae = zeros(obj.period_lcm, obj.n_state, obj.n_state);
            for tt=1:obj.period_lcm
                for ss=1:obj.n_state
                    ss_bin = de2bi(ss-1, obj.n_flow, 'left-msb');
                    next_state_bin = ss_bin;
                    for nn=1:obj.n_flow
                        %first consider departure/expiration (E)
                        if(rem(tt-obj.offset(nn)-obj.delay(nn), obj.period(nn)) == 0)
                            next_state_bin(nn) = 0;
                        end
                        %then consider arrival (A)
                        if(rem(tt-obj.offset(nn), obj.period(nn)) == 0)
                            next_state_bin(nn) = 1;
                        end
                    end
                    next_state = bi2de(next_state_bin, 'left-msb')+1;
                    obj.transition_matrix_ae(tt, ss, next_state) = 1;
                end
            end
            
            %sanity check
            for tt=1:obj.period_lcm
                for ss=1:obj.n_state
                    if(sum(obj.transition_matrix_ae(tt,ss,:)) ~= 1)
                        error('wrong probability for time %d state %d', tt, ss);
                    end
                end
            end
        end
        
        %construct the transition matrix by considering both TX and A&E
        function constructTransitionMatrix(obj)
            %transition probability due to both transmission and A&E
            obj.transition_matrix = zeros(obj.period_lcm, obj.n_state, obj.n_action, obj.n_state);
            for tt=1:obj.period_lcm
                for ss=1:obj.n_state
                    for aa=1:obj.n_action
                        for next_state=1:obj.n_state
                            tran_prob_temp = 0;
                            for tx_state=1:obj.n_state
                                tran_prob_temp = tran_prob_temp + (obj.transition_matrix_tx(ss,aa,tx_state))*(obj.transition_matrix_ae(tt,tx_state,next_state));
                            end
                            obj.transition_matrix(tt,ss, aa, next_state) = tran_prob_temp;
                        end
                    end
                end
            end
            
            %sanity check
            for tt=1:obj.period_lcm
                for ss=1:obj.n_state
                    for aa=1:obj.n_action
                        if(sum(obj.transition_matrix(tt,ss,aa,:)) ~= 1)
                            error('wrong probability for state %d action %d', ss, aa);
                        end
                    end
                end
            end
        end
        
        %construct the reward per reward per state. Equation 11 in Mobihoc 2015
        % Note that this reward is not related to any coefficient or the
        % period. To use them, we need to normalized by period and/or
        % coefficient if weighted-sum utility is considered.
        function constructRewardPerStatePerAction(obj)   
            obj.reward_per_state_per_action = zeros(obj.n_state, obj.n_action);
            for ss=1:obj.n_state
                ss_bin = de2bi(ss-1, obj.n_flow, 'left-msb');
                for aa=1:obj.n_action
                    if(ss_bin(aa) == 1) %there exists one packet at state ss for flow aa
                        %this reward_per_slot is not normalized by the
                        %period, thus, it is metric 1: the absolute timely
                        %throughput per slot    
                        obj.reward_per_state_per_action(ss,aa) = obj.success_prob(aa);
                    end
                end
            end
        end
        
       
    end

end