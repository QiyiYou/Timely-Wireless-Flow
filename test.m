clear all; close all;


%select gurobi to solve the LP, much faster than the default SDP3
%cvx_solver gurobi_2
%cvx_solver SDPT3
cvx_solver SeDuMi
cvx_save_prefs
cvx_precision best

n_flow = 2;


flow1 = NonOverlappedFlowInstance();
flow1.offset = 0;
% flow1.period = randi([2,5]);
% flow1.delay = randi([1, flow1.period]);
% flow1.arrival_prob = randi([50,100])/100;
% flow1.success_prob = randi([50,100])/100;
flow1.period = 2;
flow1.delay = 2;
flow1.arrival_prob = 1;
flow1.success_prob = 0.5;
flow1.constructEverything();

flow2 = NonOverlappedFlowInstance();
flow2.offset = 0;
% flow2.period = randi([2,5]);
% flow2.delay = randi([1, flow1.period]);
% flow2.arrival_prob = randi([50,100])/100;
% flow2.success_prob = randi([50,100])/100;
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

%utility_coeff =  [1,1,1];
utility_coeff =  [1,1];
utility_coeff = utility_coeff./sum(utility_coeff);
utility_form = 'weighted_log_sum';


tic;
[optimal_policy_RAC, optimal_utility_RAC, optimal_throughput_per_flow_RAC, status] = ...
    getOptimalSolutionRAC_v(obj, utility_coeff, utility_form);
fprintf('\nFinish getOptimalSolutionRAC with time %f seconds\n', toc);

optimal_throughput_per_flow_RAC

tic;
[optimal_policy_RAC_approx, optimal_action_distribution_RAC_approx, optimal_utility_RAC_approx, optimal_throughput_per_flow_RAC_approx, status] = ...
    getApproximateSolutionRAC(obj, utility_coeff, utility_form);
fprintf('\nFinish getApproximateSolutionRAC with time %f seconds\n', toc);


epsilon = 0;
%strict_throughput_per_flow = max(throughput_per_flow - epsilon, 0);
% let us use optimal throughput from RAC optmization problem
strict_throughput_per_flow = max(optimal_throughput_per_flow_RAC - epsilon, 0);

N_instance = 100;
throughput_per_flow_RAC = zeros(2,N_instance);
throughput_per_flow_RAC_approx = zeros(2, N_instance);

for instance=1:N_instance
    instance
    T = 10000;
    
    tic;
    [successful_transmission_RAC, state_action_distribution_RAC, system_state_RAC, system_action_RAC, state_action_per_slot_RAC ] ...
        = RACSchedule(obj, T, optimal_policy_RAC);
    fprintf( '\nFinish RACSchedule with time %f seconds\n', toc);
    
    
    
%     tic;
%     [successful_transmission_RAC_approx, state_action_distribution_RAC_approx, system_state_RAC_approx, system_action_RAC_approx, state_action_per_slot_RAC_approx] ...
%         = RelaxedRACSchedule_old(obj, T, optimal_policy_RAC_approx, optimal_action_distribution_RAC_approx);
%     fprintf( '\nFinish RelaxedRACSchedule with time %f seconds\n', toc);
%     
%    
    
    throughput_per_flow_RAC(:, instance) = mean(successful_transmission_RAC,2);
%    throughput_per_flow_RAC_approx(:, instance) = mean(successful_transmission_RAC_approx,2);    
end

throughput_per_flow_RAC_avg = mean(throughput_per_flow_RAC, 2);
%throughput_per_flow_RAC_approx_avg = mean(throughput_per_flow_RAC_approx, 2);

gap = sum(abs(throughput_per_flow_RAC_avg - optimal_throughput_per_flow_RAC))/sum(optimal_throughput_per_flow_RAC)

