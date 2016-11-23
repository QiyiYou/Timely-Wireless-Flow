%use MDP to formulate the timely throughput problem (Mobihoc 2015)

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
obj.n_flow = 3; %number of users/flows
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


utility_coeff = rand(obj.n_flow,1);%the coefficient for each flow, we consider the weighted-sum utility
utility_coeff = utility_coeff/sum(utility_coeff);

% utility_coeff = zeros(3,1);
% utility_coeff(1) = 1/3;
% utility_coeff(2) = 1/3;
% utility_coeff(3) = 1/3;


utility_form = 'weighted_log_sum';

tic;
[optimal_policy_RAC, optimal_utility_RAC, optimal_throughput_per_flow_RAC] = getOptimalSolutionRAC_CVX(obj, utility_coeff, utility_form);
fprintf('Finish RAC optimization using CVX with time %f seconds\n', toc);

% tic;
% [optimal_policy_RAC2, optimal_utility_RAC2, optimal_throughput_per_flow_RAC2] = getOptimalSolutionRAC_Yalmip(obj, utility_coeff,utility_form);
% fprintf('Finish RAC optimization using Yalmip with time %f seconds\n', toc);


tic;
n_period_lcm_distributed = 10000;
T_distributed = n_period_lcm_distributed*obj.period_lcm;
[optimal_utility_distributed, optimal_throughput_per_flow_distributed, state_action_distribution_distributed] ...
    = getDistributedSolution(obj, utility_coeff, utility_form, T_distributed);
fprintf('Finish distributed algorithm with time %f seconds\n', toc);


% 
% 
% n_period_lcm_mdp = 10;
% T_mdp = n_period_lcm_mdp*obj.period_lcm;
% 
% tic;
% [optimal_policy_MDP, optimal_utility_MDP, optimal_throughput_per_flow_MDP] = getOptimalSolutionFiniteHorizon(obj, T_mdp, utility_coeff);
% fprintf('Finish backward induction with time %f seconds\n', toc);
% 
% init_state = obj.getInitialState();
% optimal_utility_MDP = optimal_utility_MDP(1,init_state);
% optimal_throughput_per_flow_MDP = squeeze(optimal_throughput_per_flow_MDP(1,init_state,:));
