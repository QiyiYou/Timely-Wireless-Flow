%I will compare the performance of finite-MDP (suboptimial) and the RAC
%(optimal) for weighted sum utility

close all; clear all;

obj = DownlinkAPInstance();

% %mannually set DownlinkAPInstance
% obj.n_flow = 3; %number of users/flows
% obj.offset = zeros(obj.n_flow,1); %the offset of the first packet of each flow
% obj.period = zeros(obj.n_flow,1); %the period of each flow
% obj.delay = zeros(obj.n_flow,1); %the delay of each flow, delay(ii) <= period (ii) because we only consider non-overlapping traffic
% obj.success_prob = zeros(obj.n_flow,1); %the probability to successfully deliver a packet of each flow if scheduled
% obj.offset(1) = 0;
% obj.offset(2) = 2;
% obj.offset(3) = 3;
% obj.period(1) = 2;
% obj.period(2) = 3;
% obj.period(3) = 4;
% obj.delay(1) = 2;
% obj.delay(2) = 3;
% obj.delay(3) = 3;
% obj.success_prob(1) = 0.3;
% obj.success_prob(2) = 0.4;
% obj.success_prob(3) = 0.8;


%randomly generate DownlinkAPInstance
obj.n_flow = 2; %number of users/flows
obj.offset = randi([1,5], obj.n_flow,1); %the offset of the first packet of each flow
obj.period = randi([1,5], obj.n_flow,1); %the period of each flow
obj.delay = zeros(obj.n_flow,1); %the delay of each flow, delay(ii) <= period (ii) because we only consider non-overlapping traffic
for ii=1:obj.n_flow
    obj.delay(ii) = randi([1,obj.period(ii)]);
end
obj.success_prob = rand(obj.n_flow,1); %the probability to successfully deliver a packet of each flow if scheduled

tic;
obj.constructEverything();
fprintf('Finish construction with time %d seconds\n', toc);

pp=obj.period
oo=obj.offset
dd=obj.delay
ss=obj.success_prob


utility_coeff = rand(obj.n_flow,1);%the coefficient for each flow, we consider the weighted-sum utility
utility_coeff = utility_coeff/sum(utility_coeff);

% utility_coeff = zeros(3,1);
% utility_coeff(1) = 1/3;
% utility_coeff(2) = 1/3;
% utility_coeff(3) = 1/3;


tic;
[optimal_policy_RAC, optimal_utility_RAC, optimal_throughput_per_flow_RAC] = getOptimalSolutionRAC_CVX(obj, utility_coeff, 'weighted_sum');
fprintf('Finish RAC optimization using CVX with time %f seconds\n', toc);


n_period_lcm_mdp = 20;
Rec_optimal_utility_MDP = zeros(n_period_lcm_mdp,1);
Rec_optimal_throughput_per_flow_MDP = zeros(n_period_lcm_mdp, obj.n_flow);
for nn=1:n_period_lcm_mdp
    T_mdp = nn*obj.period_lcm;
    
    tic;
    [optimal_policy_MDP, optimal_utility_MDP, optimal_throughput_per_flow_MDP] = getOptimalSolutionFiniteHorizon(obj, T_mdp, utility_coeff);
    fprintf('nn=%d, out of n_period_lcm_mdp=%d, Finish backward induction with time %f seconds\n', nn, n_period_lcm_mdp, toc);
    
    init_state = obj.getInitialState();
    optimal_utility_MDP = optimal_utility_MDP(1,init_state);
    optimal_throughput_per_flow_MDP = squeeze(optimal_throughput_per_flow_MDP(1,init_state,:));
    Rec_optimal_utility_MDP(nn) = optimal_utility_MDP;
    Rec_optimal_throughput_per_flow_MDP(nn,:) = optimal_throughput_per_flow_MDP;
end

figure;
set(gca,'FontSize',20);
hold on;
plot(1:n_period_lcm_mdp, optimal_utility_RAC*ones(1,n_period_lcm_mdp), '-r', 'Linewidth', 3);
plot(1:n_period_lcm_mdp,Rec_optimal_utility_MDP, '-b', 'Linewidth', 3);
hold off;
xlabel('Number of Period','FontSize', 20, 'FontName', 'Arial');
ylabel('Weighted Sum Utility','FontSize', 20, 'FontName', 'Arial');
legend('Optimal RAC', 'Sub-optimal MDP', 'location', 'SouthEast');
box on;
grid on;
print(sprintf('diff_MDP_RAC_%d_flows',obj.n_flow),'-dpdf');