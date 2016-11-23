%use MDP to formulate the timely throughput problem (Mobihoc 2015)

%this special case will show the LQFS is NOT throughput optimal, it is
%actually throughput suboptimal.

%Two flows with (0, 4, 4)  and (2, 4, 4), success_prob = 0.5 for both.
%The RAC optimal solution shows the throughput is: (0.8750, 0.8750)
%But note that at slot 1&2 (and repeated in period 4, i.e., 5&6, 9%10, etc),
%if the system state is 11, then RAC opitmal policy will always choose
%flow 2 due to the more urgent deadline for flow 2.
%Similary, at slot 3&4 (and repeated in period 4, i.e., 7&8, 11&12, etc),
%if the system sate is 11, then RAC optimal policy will always choose flow
%1 due to more urgent deadline for flow 1.

%However, the LQFS will not consider the coming deadline (future), but only
%greedily maximizes the current "utility". Simulation shows the LQFS can
%support the rate, roughly (0.83, 0.83). Therefore we can show that LQFS is
%not throughput optimal. 

close all; clear all;

obj = DownlinkAPInstance();

%mannually set DownlinkAPInstance
obj.n_flow = 2; %number of users/flows
obj.offset = zeros(obj.n_flow,1); %the offset of the first packet of each flow
obj.period = zeros(obj.n_flow,1); %the period of each flow
obj.delay = zeros(obj.n_flow,1); %the delay of each flow, delay(ii) <= period (ii) because we only consider non-overlapping traffic
obj.success_prob = zeros(obj.n_flow,1); %the probability to successfully deliver a packet of each flow if scheduled
obj.offset(1) = 0;
obj.offset(2) = 2;
obj.period(1) = 4;
obj.period(2) = 4;
obj.delay(1) = 4;
obj.delay(2) = 4;
obj.success_prob(1) = 0.5;
obj.success_prob(2) = 0.5;


tic;
obj.constructEverything();
fprintf('Finish construction with time %d seconds\n', toc);



utility_coeff = zeros(3,1);
utility_coeff(1) = 1/2;
utility_coeff(2) = 1/2;



tic;
[optimal_policy_RAC, optimal_utility_RAC, optimal_throughput_per_flow_RAC] = getOptimalSolutionRAC_CVX(obj, utility_coeff,'weighted_log_sum');
fprintf('Finish RAC optimization using CVX with time %f seconds\n', toc);



epsilon = 0;
%strict_throughput_per_flow = max(throughput_per_flow - epsilon, 0);
% let us use optimal throughput from RAC optmization problem
strict_throughput_per_flow = max(optimal_throughput_per_flow_RAC - epsilon, 0)
%we can simulate longer time than MDP
n_period_lcm_lqfs = 10000;
T_lqfs = n_period_lcm_lqfs*obj.period_lcm;

tic;
[virtual_arrival, virtual_server_capacity, virtual_departure, virtual_queue, state_action_distribution, ...
 Rec_system_state, Rec_system_action, Rec_state_action_per_slot] = ...
    largestQueueFirstScheduling(obj, T_lqfs, strict_throughput_per_flow);
    %largestQueueFirstScheduling(obj, T_lqfs, strict_throughput_per_flow);
fprintf('Finish LQFS with time %f seconds\n', toc);

avg_virtual_arrival = mean(virtual_arrival,2)
avg_virtual_server_capacity = mean(virtual_server_capacity,2)
avg_virtual_departure = mean(virtual_departure,2)
avg_empirical_throughput = avg_virtual_departure.*obj.period
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
    title(sprintf('%d flows, flow %d, epsilon=%f',obj.n_flow, nn, epsilon));
    box on;
    grid on;
    print(sprintf('stability_LQFS_result_flow_%d',nn),'-dpdf');
end


%analyze the state and/or action evolution of LQFS
Rec_system_state_distribution = zeros(T_lqfs, obj.n_state);
Rec_state_action_distribution = cummean(Rec_state_action_per_slot,3);

for ss=1:obj.n_state
    Rec_system_state_distribution(:, ss) = cummean(Rec_system_state == ss);
end



x_avg_RAC = squeeze(mean(sum(optimal_policy_RAC,3),1))
x_avg_LQFS = squeeze(Rec_system_state_distribution(end,:))
y_avg_RAC = squeeze(mean(optimal_policy_RAC,1))
y_avg_LQFS = squeeze(Rec_state_action_distribution(:,:,end))

diff_x_avg_RAC_LQFS = sum(abs(Rec_system_state_distribution - ones(T_lqfs, 1)*x_avg_RAC),2);
diff_y_avg_RAC_LQFS = sum(abs(reshape(y_avg_RAC, [], 1)*ones(1,T_lqfs) - reshape(Rec_state_action_distribution, [], T_lqfs)),1);

figure;
set(gca,'FontSize',20);
hold on;
plot(1:T_lqfs,diff_x_avg_RAC_LQFS, '-b', 'Linewidth', 3);
hold off;
xlabel('Slot','FontSize', 20, 'FontName', 'Arial');
ylabel('Diff x','FontSize', 20, 'FontName', 'Arial');
title(sprintf('%d flows, epsilon=%f',obj.n_flow, epsilon));
box on;
grid on;
print(sprintf('stability_LQFS_result_diff_x'),'-dpdf');

figure;
set(gca,'FontSize',20);
hold on;
plot(1:T_lqfs,diff_y_avg_RAC_LQFS, '-b', 'Linewidth', 3);
hold off;
xlabel('Slot','FontSize', 20, 'FontName', 'Arial');
ylabel('Diff y','FontSize', 20, 'FontName', 'Arial');
title(sprintf('%d flows, epsilon=%f',obj.n_flow, epsilon));
box on;
grid on;
print(sprintf('stability_LQFS_result_diff_y'),'-dpdf');
