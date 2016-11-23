%I will compare the performance of RAC (global solution) and distributed
%algorithm (distributed solution)

close all; clear all;

obj = DownlinkAPInstance();

% %mannually set DownlinkAPInstance
obj.n_flow = 3; %number of users/flows
obj.offset = zeros(obj.n_flow,1); %the offset of the first packet of each flow
obj.period = zeros(obj.n_flow,1); %the period of each flow
obj.delay = zeros(obj.n_flow,1); %the delay of each flow, delay(ii) <= period (ii) because we only consider non-overlapping traffic
obj.success_prob = zeros(obj.n_flow,1); %the probability to successfully deliver a packet of each flow if scheduled
obj.offset(1) = 0;
obj.offset(2) = 2;
obj.offset(3) = 3;
obj.period(1) = 2;
obj.period(2) = 3;
obj.period(3) = 4;
obj.delay(1) = 2;
obj.delay(2) = 3;
obj.delay(3) = 3;
obj.success_prob(1) = 0.3;
obj.success_prob(2) = 0.4;
obj.success_prob(3) = 0.8;
utility_coeff = zeros(3,1);
utility_coeff(1) = 1/4;
utility_coeff(2) = 2/4;
utility_coeff(3) = 1/4;


%randomly generate DownlinkAPInstance
% obj.n_flow = 3; %number of users/flows
% obj.offset = randi([1,5], obj.n_flow,1); %the offset of the first packet of each flow
% obj.period = randi([1,5], obj.n_flow,1); %the period of each flow
% obj.delay = zeros(obj.n_flow,1); %the delay of each flow, delay(ii) <= period (ii) because we only consider non-overlapping traffic
% for ii=1:obj.n_flow
%     obj.delay(ii) = randi([1,obj.period(ii)]);
% end
% obj.success_prob = rand(obj.n_flow,1); %the probability to successfully deliver a packet of each flow if scheduled
% utility_coeff = rand(obj.n_flow,1);%the coefficient for each flow, we consider the weighted-sum utility
% utility_coeff = utility_coeff/sum(utility_coeff);

tic;
obj.constructEverything();
fprintf('Finish construction with time %d seconds\n', toc);


utility_form = 'weighted_log_sum';

tic;
[optimal_policy_RAC, optimal_utility_RAC, optimal_throughput_per_flow_RAC] = getOptimalSolutionRAC_CVX(obj, utility_coeff, utility_form);
fprintf('Finish RAC optimization using CVX with time %f seconds\n', toc);



n_period_lcm = 1000;
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
ylim([-1.5,-0.5]);
legend('Optimal RAC', 'Distributed', 'location', 'SouthEast');
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
    if(kk==1)
        legend('Optimal RAC', 'Distributed', 'location', 'NorthEast');
    else
        legend('Optimal RAC', 'Distributed', 'location', 'SouthEast');
    end
    ylim([0,1]);
    box on;
    grid on;
    print(sprintf('diff_RAC_distributed_%d_flows_throughput_flow_%d',obj.n_flow,kk),'-dpdf');
end


