%I will compare the performance of finite-MDP (suboptimial) and the RAC
%(optimal) for weighted sum utility

close all; clear all;

obj = DownlinkAPInstance();

% %mannually set DownlinkAPInstance
obj.n_flow = 8; %number of users/flows
obj.offset = zeros(obj.n_flow,1); %the offset of the first packet of each flow
obj.period = 10*ones(obj.n_flow,1); %the period of each flow
obj.delay = zeros(obj.n_flow,1); %the delay of each flow, delay(ii) <= period (ii) because we only consider non-overlapping traffic
obj.success_prob = zeros(obj.n_flow,1); %the probability to successfully deliver a packet of each flow if scheduled
obj.delay([1,2]) = 3;
obj.delay([3,4,5]) = 6;
obj.delay([6,7,8]) = 10;
obj.success_prob(1) = 0.8;
obj.success_prob(2) = 0.4;
obj.success_prob(3) = 0.3;
obj.success_prob(4) = 0.8;
obj.success_prob(5) = 0.4;
obj.success_prob(6) = 0.3;
obj.success_prob(7) = 0.8;
obj.success_prob(8) = 0.4;

tic;
obj.constructEverything();
fprintf('Finish construction with time %d seconds\n', toc);


utility_coeff = randi([1,20],obj.n_flow,1);
% utility_coeff(1) = 2;
% utility_coeff(2) = 5;
% utility_coeff(3) = 6;

% 
% tic;
% [optimal_policy_RAC, optimal_utility_RAC, optimal_throughput_per_flow_RAC] = getOptimalSolutionRAC_CVX(obj, utility_coeff, 'weighted_sum');
% fprintf('Finish RAC optimization using CVX with time %f seconds\n', toc);


tic;
[optimal_policy_MDP, optimal_utility_MDP, optimal_throughput_per_flow_MDP] = getOptimalSolutionFiniteHorizon(obj, obj.period(1), utility_coeff);
fprintf('Finish backward induction with time %f seconds\n', toc);
