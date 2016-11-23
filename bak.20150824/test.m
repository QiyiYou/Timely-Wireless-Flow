
clear all; close all;

n_flow = 3;

flow1 = NonOverlappedFlowInstance();
flow1.offset = 0;
flow1.period = 2;
flow1.delay = 2;
flow1.arrival_prob = 1;
flow1.success_prob = 0.8;
flow1.constructEverything();

flow2 = NonOverlappedFlowInstance();
flow2.offset = 0;
flow2.period = 2;
flow2.delay = 2;
flow2.arrival_prob = 1;
flow2.success_prob = 0.5;
flow2.constructEverything();

flow3 = FlowInstance();
flow3.offset = 4;
flow3.period = 2;
flow3.delay = 3;
flow3.arrival_prob = 1;
flow3.success_prob = 0.3;
flow3.constructEverything();


flow_array = cell(n_flow,1);
flow_array{1} = flow1;
flow_array{2} = flow2;
flow_array{3} = flow3;


obj = DownlinkAPInstance();
obj.n_flow = n_flow;
obj.flow_array = flow_array;
obj.constructEverything();
obj.stateSanityCheck();
obj.stateActionSanityCheck();

utility_coeff = [0.3, 0.7, 0.8];
%utility_coeff = [0.3, 0.7];
utility_coeff = utility_coeff./sum(utility_coeff);
utility_form = 'weighted_log_sum';

tic;
[optimal_policy_RAC, optimal_utility_RAC, optimal_throughput_per_flow_RAC] = ...
    getOptimalSolutionRAC(obj, utility_coeff, utility_form);
fprintf('\nFinish getOptimalSolutionRAC with time %f seconds\n', toc);

tic;
[optimal_policy_RAC_v, optimal_utility_RAC_v, optimal_throughput_per_flow_RAC_v] = ...
    getOptimalSolutionRAC_v(obj, utility_coeff, utility_form);
fprintf('\nFinish getOptimalSolutionRAC_v with time %f seconds\n', toc);