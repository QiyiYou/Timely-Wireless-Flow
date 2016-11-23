%I will compare the performance of RAC (global solution) and distributed
%algorithm (distributed solution)


%this special case will show the the distributed algorithm cannot converge to the 
%optimal solution

%Two flows with (0, 4, 4)  and (2, 4, 4), success_prob = 0.5 for both.
%The RAC optimal solution shows the throughput is: (0.8750, 0.8750)
%But note that at slot 1&2 (and repeated in period 4, i.e., 5&6, 9%10, etc),
%if the system state is 11, then RAC opitmal policy will always choose
%flow 2 due to the more urgent deadline for flow 2.
%Similary, at slot 3&4 (and repeated in period 4, i.e., 7&8, 11&12, etc),
%if the system sate is 11, then RAC optimal policy will always choose flow
%1 due to more urgent deadline for flow 1.

%However, the LQFS will not consider the coming deadline (future), but only
%greedily maximizes the current "utility". Simulation shows the distributed algorithm can
%only converge to the the rate, roughly (0.83, 0.83). Therefore we can show that dual algorithm
%cannot converge to the global optimal soulution here. 

close all; clear all;

obj = DownlinkAPInstance();

% %mannually set DownlinkAPInstance
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

utility_coeff = zeros(3,1);
utility_coeff(1) = 1/2;
utility_coeff(2) = 1/2;

tic;
obj.constructEverything();
fprintf('Finish construction with time %d seconds\n', toc);


utility_form = 'weighted_log_sum';

tic;
[optimal_policy_RAC, optimal_utility_RAC, optimal_throughput_per_flow_RAC] = getOptimalSolutionRAC_CVX(obj, utility_coeff, utility_form);
fprintf('Finish RAC optimization using CVX with time %f seconds\n', toc);
optimal_throughput_per_flow_RAC



n_period_lcm = 10000;
T = n_period_lcm*obj.period_lcm;

tic;
[Rec_empirical_utility_distributed, Rec_empirical_throughput_per_flow_distributed, state_action_distribution_distributed] ...
    = getDistributedSolution(obj, utility_coeff, utility_form, T);
fprintf('Finish distributed algorithm with time %f seconds\n', toc);


step = obj.period_lcm;
figure;
set(gca,'FontSize',20);
hold on;
plot(1:step:T, optimal_utility_RAC.*ones(size(1:step:T)), '-r', 'Linewidth', 3);
plot(1:step:T,Rec_empirical_utility_distributed(1:step:T), '-b', 'Linewidth', 3);
hold off;
%xlabel('Number of Period','FontSize', 20, 'FontName', 'Arial');
xlabel('Slot','FontSize', 20, 'FontName', 'Arial');

if(isequal(utility_form, 'weighted_sum'))
    ylabel('Weighted Sum Utility','FontSize', 20, 'FontName', 'Arial');
elseif(isequal(utility_form, 'weighted_log_sum'))
    ylabel('Weighted Log Sum Utility','FontSize', 20, 'FontName', 'Arial');
else
    error('wrong input utility_form, can only be ''weighted_sum'' or ''weighted_log_sum''');
end
%ylim([-1.5,-0.5]);
legend('Optimal RAC', 'Distributed', 'location', 'NorthEast');
box on;
grid on;
print(sprintf('diff_RAC_distributed_%d_flows_sum_utility',obj.n_flow),'-dpdf');

for kk=1:obj.n_flow
    figure;
    set(gca,'FontSize',20);
    hold on;
    plot(1:step:T, optimal_throughput_per_flow_RAC(kk)*ones(size(1:step:T)), '-r', 'Linewidth', 3);
    plot(1:step:T, Rec_empirical_throughput_per_flow_distributed(kk,1:step:T), '-b', 'Linewidth', 3);
    hold off;
    xlabel('Slot','FontSize', 20, 'FontName', 'Arial');
    ylabel(sprintf('Timely Throughput of Flow %d',kk),'FontSize', 20, 'FontName', 'Arial');
    if(optimal_throughput_per_flow_RAC(kk) <= 0.5)
        legend('Optimal RAC', 'Distributed', 'location', 'NorthEast');
    else
        legend('Optimal RAC', 'Distributed', 'location', 'SouthEast');
    end
    ylim([0,1]);
    box on;
    grid on;
    print(sprintf('diff_RAC_distributed_%d_flows_throughput_flow_%d',obj.n_flow,kk),'-dpdf');
end


