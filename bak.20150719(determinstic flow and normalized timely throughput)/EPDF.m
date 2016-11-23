function [virtual_arrival, virtual_server_capacity, virtual_departure, virtual_queue, ...
      state_action_distribution, system_state, system_action, state_action_per_slot ] ...
    = EPDF(obj, T, absolute_throughput_per_flow)
% the EPDF algorithm, Earliest Postive-debt Deadline First Scheduling
% Algorithm.
% T is the number of slots to simulate 
% absolute_throughput_per_flow is the input absolute timely througput for each flow 

% the achieved state_action_distribution for each slot (in a period_lcm)
% this is used to compare with the global optimal RAC scheme, i.e., the
% joint distribution y in function getOptimalSolutionRAC.
state_action_distribution = zeros(obj.period_lcm, obj.n_state, obj.n_action);

state_action_per_slot = zeros(obj.n_state, obj.n_action,T);

% the virtual queue envolves as 
%     virtual_queue(t+1) = max( virtual_queue(t)-virtual_server_capacity(t), 0) + virtual_arrival(t+1);
virtual_queue = zeros(obj.n_flow, T);
virtual_arrival = zeros(obj.n_flow, T);
virtual_departure = zeros(obj.n_flow, T);
virtual_server_capacity = zeros(obj.n_flow, T); %u_c(t), this comes from the real physical system


% for nn=1:obj.n_flow
%     if(rand < absolute_throughput_per_flow(nn)/obj.success_prob(nn))
%         virtual_arrival(nn,1) = 1;
%     end
% end
% 
% %initialize the virtual queue
% virtual_queue(:,1) = virtual_arrival(:,1);

% physical system state
system_state = zeros(1,T);
system_state(1) = obj.getInitialState();
system_action = zeros(1,T);


for tt=1:T
    %construct the virtual server capacity (use LQFS)
    %get the largest queue
    current_state = system_state(tt);
    earliest_deadline_idx = -1;
    earliest_dealine_val = +inf;
    for qq=1:obj.n_flow
        % b_packet=0 if no packet for flow qq, 1 if one packet for flow qq
        b_packet = hasPacket(obj, current_state, qq);
        %I will change here in accordance to the normalized timely
        %throughput rather than absolutly timely throughput. There is a
        %factor different for the two both metrics.
        
%         % absolutly timely throughput
%         if(b_packet*virtual_queue(qq,tt)*obj.success_prob(qq) > largest_queue_length)
%             largest_queue_idx = qq;
%             largest_queue_length = b_packet*virtual_queue(qq,tt)*obj.success_prob(qq);
%         end
        
        %normarlized timely throughput
        if(b_packet == 1 && virtual_queue(qq,tt) > 0)
            if(obj.getNextExpirationSlot(qq,tt) < earliest_dealine_val)
                earliest_deadline_idx = qq;
                earliest_dealine_val = obj.getNextExpirationSlot(qq,tt);
            end
        end
    end
    
    if(earliest_deadline_idx == -1)
        for qq=1:obj.n_flow
            b_packet = hasPacket(obj, current_state, qq);
            %schedual a packet
            if(b_packet == 1)
                earliest_deadline_idx = qq;
                break;
            end
        end
    end
    
    if(earliest_deadline_idx == -1)
        earliest_deadline_idx = randi([1,obj.n_action]);
    end
    

    system_action(tt) = earliest_deadline_idx;
    
    %print out the state and action
%     b_packet = zeros(obj.n_flow,1);
%     queue_size = zeros(obj.n_flow,1);
%     for ii=1:obj.n_flow
%         b_packet(ii) =  hasPacket(obj, current_state, ii);
%         queue_size(ii) = virtual_queue(ii,tt);
%     end
%     display(b_packet');
%     display(queue_size');
%     display(earliest_deadline_idx);
    
    %update the state_action_distribution, just count here, we will
    %normalize it to a distribtuion after all simulated slots
    state_action_distribution(getFirstPeriodSlot(obj,tt), current_state, earliest_deadline_idx) = ...
        state_action_distribution(getFirstPeriodSlot(obj,tt), current_state, earliest_deadline_idx) + 1;
   
    state_action_per_slot(current_state, earliest_deadline_idx, tt) = 1;
    
    %realization of randomness. Note that randomeness only comes from TX and the virtual server 
    %capacity is also only related to TX. A&E is deterministic once TX is fixed. 
    %Therefore, we only need to consider realization of TX randomness. 
    
    next_state_tx_probability = squeeze(obj.transition_matrix_tx(current_state, earliest_deadline_idx,:));
    probability_temp = rand;
    next_state_tx = -1;
    for ss=1:obj.n_state
        if(ss == 1)
            if(probability_temp <= sum(next_state_tx_probability(1)))
                next_state_tx = 1;
                break;
            end
        end    
        % ss >= 2
        if( probability_temp <= sum(next_state_tx_probability(1:ss))  && ...
            probability_temp > sum(next_state_tx_probability(1:ss-1)))
                next_state_tx = ss;
                break;
        end
    end
    
    %Rembmer that virtual_server_capacity is not related to
    %the A&E nor TX but not related to whether there exiest a packet
    % because here virtual_server_capacity is u_k(t)
    if(hasPacket(obj, current_state, earliest_deadline_idx))
        virtual_server_capacity(earliest_deadline_idx,tt) = 1;
    end
    
    virtual_departure(:,tt) = min(virtual_queue(:,tt), virtual_server_capacity(:,tt));
    
    %we have finished the last slot
    if(tt == T)
        break;
    end
    
    %consider A&E here to get the actual next_state
    next_state = -1;
    for ss=1:obj.n_state
         if(obj.transition_matrix_ae(getFirstPeriodSlot(obj,tt), next_state_tx, ss) == 1)
             next_state = ss; 
             break;
         end
    end
    
    if(next_state == -1)
        error('something wrong');
    end
    
    system_state(tt+1) = next_state;

    %construct virtual arrival (Bernuouly generate an arrival) for next slot
    % REMEBER here if  it is the absolutely timely throughput divided by
    % probabiltiy, i.e., the workload
    for nn=1:obj.n_flow
        if(rand < absolute_throughput_per_flow(nn)/obj.success_prob(nn))
            virtual_arrival(nn,tt) = 1;
        end
    end
    
    
    %update queue length
    virtual_queue(:,tt+1) = max(virtual_queue(:,tt)-virtual_server_capacity(:,tt), 0) + virtual_arrival(:,tt);
end

%normalize the state_action_distribution
state_action_distribution = state_action_distribution/(T/obj.period_lcm);

end