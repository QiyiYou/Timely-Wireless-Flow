clear all; close all; 
n_flow = 2;
% flow1 = FlowInstance();
% flow1.offset = 1;
% flow1.period = 3;
% flow1.delay = 3;
% flow1.arrival_prob = rand(1,flow1.period);
% flow1.success_prob = 0.3;
% flow1.constructEverything();

flow2 = NonOverlappedFlowInstance();
flow2.offset = 0;
flow2.period = 4;
flow2.delay = 4;
flow2.arrival_prob = 1;
flow2.success_prob = 0.5;
flow2.constructEverything();

% flow3 = FlowInstance();
% flow3.offset = 0;
% flow3.period = 2;
% flow3.delay = 1;
% flow3.arrival_prob = rand(1,flow3.period);
% flow3.success_prob = 0.1;
% flow3.constructEverything();

flow4 = NonOverlappedFlowInstance();
flow4.offset = 2;
flow4.period = 4;
flow4.delay = 4;
flow4.arrival_prob = 1;
flow4.success_prob = 0.5;
flow4.constructEverything();

flow_array = cell(n_flow,1);
flow_array{1} = flow2;
flow_array{2} = flow4;
% 
% flow_array = cell(n_flow,1);
% flow_array{1} = flow1;
% flow_array{2} = flow2;
% flow_array{3} = flow3;

obj = DownlinkAPInstanceFromFlowInstance();
obj.n_flow = n_flow;
obj.flow_array = flow_array;
obj.constructEverything();
obj.stateSanityCheck();


%utility_coeff = rand(obj.n_flow,1);
utility_coeff = [0.5, 0.5];
utility_form = 'weighted_log_sum';

[optimal_policy, optimal_utility, optimal_throughput_per_flow] = getOptimalSolutionRACFromFlowInstance_CVX(obj, utility_coeff, utility_form)


%[optimal_policy_approx, optimal_utility_approx, optimal_throughput_per_flow_approx] = getApproximateSolutionRACFromFlowInstance_CVX(obj, utility_coeff, utility_form);


 
epsilon = 0;
%strict_throughput_per_flow = max(throughput_per_flow - epsilon, 0);
% let us use optimal throughput from RAC optmization problem
strict_throughput_per_flow = max(optimal_throughput_per_flow - epsilon, 0)
%we can simulate longer time than MDP
n_period_lcm_lqfs = 4000;
T_lqfs = n_period_lcm_lqfs*obj.period_lcm;

tic;
[ successful_transmission, virtual_arrival, virtual_server_capacity, virtual_departure, virtual_queue, state_action_distribution, ...
 Rec_system_state, Rec_system_action, Rec_state_action_per_slot] = ...
    LDF(obj, T_lqfs, strict_throughput_per_flow);
fprintf('Finish LDF with time %f seconds\n', toc);

avg_virtual_arrival = mean(virtual_arrival,2)
avg_virtual_server_capacity = mean(virtual_server_capacity,2)
avg_virtual_departure = mean(virtual_departure,2)
avg_empirical_throughput = avg_virtual_departure
final_virtual_queue = virtual_queue(:,end)
ratio_final_virtual_queue_to_time = virtual_queue(:,end)/(T_lqfs)

cum_virtual_arrival = cumsum(virtual_arrival,2);
cum_virtual_server_capacity = cumsum(virtual_server_capacity,2);
cum_virtual_departure = cumsum(virtual_departure,2);

for nn=1:obj.n_flow
    figure;
    set(gca,'FontSize',20);
    hold on;
    plot(1:T_lqfs,cum_virtual_arrival(nn,:), '-b', 'Linewidth', 3);
    plot(1:T_lqfs,cum_virtual_server_capacity(nn,:), '-g', 'Linewidth', 3);
    plot(1:T_lqfs,cum_virtual_departure(nn,:), '--r', 'Linewidth', 2);
    plot(1:T_lqfs,virtual_queue(nn,:), '-k', 'Linewidth', 3);
    hold off;
    xlabel('Slot','FontSize', 20, 'FontName', 'Arial');
    legend('Cumulative Arrival', 'Cumulative Server Capacity', 'Cumulative Departure', 'Virtual Queue', 'Location', 'NorthWest');
    title(sprintf('%d flows, epsilon=%f',obj.n_flow, epsilon));
    box on;
    grid on;
    print(sprintf('stability_LDF_result_flow_%d',nn),'-dpdf');
    print(sprintf('stability_LDF_result_flow_%d',nn),'-depsc');
end




