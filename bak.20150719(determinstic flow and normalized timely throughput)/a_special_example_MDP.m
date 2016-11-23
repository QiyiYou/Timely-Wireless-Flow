%I will compare the performance of finite-MDP (suboptimial) and the RAC
%(optimal) for weighted sum utility

close all; clear all;

obj = DownlinkAPInstance();

% %mannually set DownlinkAPInstance
obj.n_flow = 3; %number of users/flows
obj.offset = zeros(obj.n_flow,1); %the offset of the first packet of each flow
obj.period = zeros(obj.n_flow,1); %the period of each flow
obj.delay = zeros(obj.n_flow,1); %the delay of each flow, delay(ii) <= period (ii) because we only consider non-overlapping traffic
obj.success_prob = zeros(obj.n_flow,1); %the probability to successfully deliver a packet of each flow if scheduled
obj.offset(1) = 0;
obj.offset(2) = 0;
obj.offset(3) = 0;
obj.period(1) = 3;
obj.period(2) = 3;
obj.period(3) = 3;
obj.delay(1) = 2;
obj.delay(2) = 3;
obj.delay(3) = 3;
obj.success_prob(1) = 0.8;
obj.success_prob(2) = 0.4;
obj.success_prob(3) = 0.3;

tic;
obj.constructEverything();
fprintf('Finish construction with time %d seconds\n', toc);


utility_coeff = zeros(3,1);
utility_coeff(1) = 2;
utility_coeff(2) = 5;
utility_coeff(3) = 6;


tic;
[optimal_policy_RAC, optimal_utility_RAC, optimal_throughput_per_flow_RAC] = getOptimalSolutionRAC_CVX(obj, utility_coeff, 'weighted_sum');
fprintf('Finish RAC optimization using CVX with time %f seconds\n', toc);


tic;
[optimal_policy_MDP, optimal_utility_MDP, optimal_throughput_per_flow_MDP] = getOptimalSolutionFiniteHorizon(obj, 3, utility_coeff);
fprintf('Finish backward induction with time %f seconds\n', toc);
