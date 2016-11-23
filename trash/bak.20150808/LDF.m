function [successful_transmission, virtual_arrival, virtual_server_capacity, virtual_departure, virtual_queue, ...
      state_action_distribution, system_state, system_action, state_action_per_slot ] ...
    = LDF(obj, T, strict_throughput_per_flow)
% the largest-queue-first-scheduling algorithm
% T is the number of slots to simulate 
% strict_throughput_per_flow is the inputtimely througput for each flow 

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
successful_transmission = zeros(obj.n_flow, T);
virtual_server_capacity = zeros(obj.n_flow, T); %e_c(t), this comes from the real physical system

%construct virtual arrival (Bernoulli generate an arrival) for the first slot
% REMEBER here if we construct per-slot virtual arrival, we need divide
% the (normalized) throughput by the period, otherwise, it is the
% absolutely timely throughput
for nn=1:obj.n_flow
    if(rand < strict_throughput_per_flow(nn))
        virtual_arrival(nn,1) = 1;
    end
end

%initialize the virtual queue
virtual_queue(:,1) = virtual_arrival(:,1);

% physical system state
system_state = zeros(1,T);
system_state(1) = obj.getInitialState();
system_action = zeros(1,T);


for tt=1:T
    %construct the virtual server capacity (use LDF)
    %get the largest queue
    current_state = system_state(tt);
    current_state_vec = obj.getVectorState(current_state);
    largest_queue_idx = -1;
    largest_queue_length = -inf;
    for qq=1:obj.n_flow
        % b_packet=0 if no packet for flow qq, 1 if one packet for flow qq
        b_packet = obj.flow_array{qq}.hasPacket(current_state_vec(qq));
        %I will change here in accordance to the normalized timely
        %throughput rather than absolutly timely throughput. There is a
        %factor different for the two both metrics.
        
        % absolutly timely throughput
        if(b_packet*virtual_queue(qq,tt)*obj.flow_array{qq}.success_prob > largest_queue_length)
            largest_queue_idx = qq;
            largest_queue_length = b_packet*virtual_queue(qq,tt)*obj.flow_array{qq}.success_prob;
        end
    end
    if(largest_queue_idx == -1)
        error('something wrong');
    end
 
    system_action(tt) = largest_queue_idx;
    
    
    [next_state, isTransmitted_vec, isSuccessful_vec] = obj.oneSlotRealization(tt,current_state,largest_queue_idx);
    
    %update the state_action_distribution, just count here, we will
    %normalize it to a distribtuion after all simulated slots
    state_action_distribution(obj.getFirstPeriodSlot(tt), current_state, largest_queue_idx) = ...
        state_action_distribution(obj.getFirstPeriodSlot(tt), current_state, largest_queue_idx) + 1;
   
    state_action_per_slot(current_state, largest_queue_idx, tt) = 1;
    
  
    if(isSuccessful_vec(largest_queue_idx) == 1)
        virtual_server_capacity(largest_queue_idx,tt) = 1;
        successful_transmission(largest_queue_idx,tt) = 1;
    end
    
    virtual_departure(:,tt) = min(virtual_queue(:,tt), virtual_server_capacity(:,tt));
    
    %we have finished the last slot
    if(tt == T)
        break;
    end
    
    
    system_state(tt+1) = next_state;

    %construct virtual arrival (Bernuouly generate an arrival) for next slot
    for nn=1:obj.n_flow
        if(rand < strict_throughput_per_flow(nn))
            virtual_arrival(nn,tt+1) = 1;
        end
    end
    
    
    %update queue length
    virtual_queue(:,tt+1) = max(virtual_queue(:,tt)-virtual_server_capacity(:,tt), 0) + virtual_arrival(:,tt+1);
end

%normalize the state_action_distribution
state_action_distribution = state_action_distribution/(T/obj.period_lcm);

end