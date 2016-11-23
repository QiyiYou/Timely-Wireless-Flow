%use MDP to formulate the timely throughput problem (Mobihoc 2015)

close all; clear all;

obj = DownlinkAPInstance();

%mannually set DownlinkAPInstance
obj.n_flow = 3; %number of users/flows
obj.offset = zeros(obj.n_flow,1); %the offset of the first packet of each flow
obj.period = zeros(obj.n_flow,1); %the period of each flow
obj.delay = zeros(obj.n_flow,1); %the delay of each flow, delay(ii) <= period (ii) because we only consider non-overlapping traffic
obj.success_prob = zeros(obj.n_flow,1); %the probability to successfully deliver a packet of each flow if scheduled
obj.offset(1) = 0;
obj.offset(2) = 1;
obj.offset(3) = 2;
obj.period(1) = 2;
obj.period(2) = 4;
obj.period(3) = 3;
obj.delay(1) = 1;
obj.delay(2) = 2;
obj.delay(3) = 3;
obj.success_prob(1) = 0.3;
obj.success_prob(2) = 0.4;
obj.success_prob(3) = 0.8;


tic;
obj.constructEverything();
fprintf('Finish construction with time %d seconds\n', toc);



utility_coeff = zeros(3,1);
utility_coeff(1) = 1/3;
utility_coeff(2) = 2/3;
utility_coeff(3) = 1/3;



tic;
[optimal_policy_RAC, optimal_utility_RAC, optimal_throughput_per_flow_RAC] = getOptimalSolutionRAC_CVX(obj, utility_coeff,'weighted_log_sum');
fprintf('Finish RAC optimization using CVX with time %f seconds\n', toc);



epsilon = 0;
%strict_throughput_per_flow = max(throughput_per_flow - epsilon, 0);
% let us use optimal throughput from RAC optmization problem
strict_throughput_per_flow = max(optimal_throughput_per_flow_RAC - epsilon, 0);
%we can simulate longer time than MDP
n_period_lcm_lqfs = 2000;
T_lqfs = n_period_lcm_lqfs*obj.period_lcm;

tic;
[virtual_arrival, virtual_server_capacity, virtual_departure, virtual_queue, state_action_distribution] = largestQueueFirstScheduling(obj, T_lqfs, strict_throughput_per_flow);
fprintf('Finish LQFS with time %f seconds\n', toc);

avg_virtual_arrival = mean(virtual_arrival,2)
avg_virtual_server_capacity = mean(virtual_server_capacity,2)
avg_virtual_departure = mean(virtual_departure,2);
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
    print(sprintf('stability_LQFS_result_flow_%d',nn),'-dpdf');
end