% a counter example to show that the approxmiate dual (lambda = sum (mu))
% is sub-optimal
clear all; close all; 
n_flow = 2;

flow1 = NonOverlappedFlowInstance();
flow1.offset = 0;
flow1.period = 2;
flow1.delay = 2;
flow1.arrival_prob = 0.4;
flow1.success_prob = 0.5;
flow1.constructEverything();

flow2 = NonOverlappedFlowInstance();
flow2.offset = 0;
flow2.period = 2;
flow2.delay = 2;
flow2.arrival_prob = 1;
flow2.success_prob = 0.5;
flow2.constructEverything();

flow_array = cell(n_flow,1);
flow_array{1} = flow1;
flow_array{2} = flow2;



obj = DownlinkAPInstance();
obj.n_flow = n_flow;
obj.flow_array = flow_array;
obj.constructEverything();
obj.stateSanityCheck();

utility_coeff = [0.5, 0.5];
utility_coeff = utility_coeff./sum(utility_coeff);
utility_form = 'weighted_sum';


tic;
[optimal_policy_RAC, optimal_utility_RAC, optimal_throughput_per_flow_RAC] = ...
    getOptimalSolutionRAC(obj, utility_coeff, utility_form);
fprintf('\nFinish getOptimalSolutionRAC with time %f seconds\n', toc);

tic;
[optimal_policy_RAC_approx, optimal_action_distribution_RAC_approx, optimal_utility_RAC_approx, optimal_throughput_per_flow_RAC_approx] ...
= getApproximateSolutionRAC(obj, utility_coeff, utility_form);
fprintf('\nFinish getApproximateSolutionRAC with time %f seconds\n', toc);


tic;
[optimal_nu_dual, optimal_lambda_dual, optimal_utility_dual] = ...
    getOptimalSolutionRACDual(obj, utility_coeff, utility_form);
fprintf('\nFinish getOptimalSolutionRACDual with time %f seconds\n', toc);


tic;
[optimal_nu_dual_relax, optimal_lambda_dual_relax, optimal_mur_dual_relax, optimal_utility_dual_relax] = ...
    getOptimalSolutionRACDualRelax(obj, utility_coeff, utility_form);
fprintf('\nFinish getOptimalSolutionRACDual with time %f seconds\n', toc);



optimal_utility_RAC
optimal_utility_dual
optimal_utility_RAC_approx
optimal_utility_dual_relax
x1 = squeeze(optimal_policy_RAC(1,:,:))
x2 = squeeze(optimal_policy_RAC(2,:,:))
z11 = squeeze(optimal_policy_RAC_approx(1,1,:,:))
z12 = squeeze(optimal_policy_RAC_approx(1,2,:,:))
z21 = squeeze(optimal_policy_RAC_approx(2,1,:,:))
z22 = squeeze(optimal_policy_RAC_approx(2,2,:,:))
optimal_nu_dual
optimal_nu_dual_relax
optimal_lambda_dual
optimal_lambda_dual_relax
mur1 = squeeze(optimal_mur_dual_relax(1,:,:))
mur2 = squeeze(optimal_mur_dual_relax(2,:,:))

