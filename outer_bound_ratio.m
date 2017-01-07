%% show the efficiency ratio of our RAC-Approx solution for frame-synchronized traffic pattern
clear all; close all;

%select gurobi to solve the LP, much faster than the default SDP3
cvx_solver gurobi_2
cvx_save_prefs 
cvx_precision best

N_flow = 10

filePath = sprintf('fig/outer_bound_ratio/N_flow=%d',N_flow);
fileID = fopen(sprintf('%s/conf_outer_bound_ratio.txt',filePath),'w');

flow1 = NonOverlappedFlowInstance();
%flow1 = FlowInstance();
flow1.offset = 0;
flow1.period = 3;
flow1.delay = 3;
flow1.arrival_prob = 1;
flow1.success_prob = 0.8;
flow1.constructEverything();

flow2 = NonOverlappedFlowInstance();
flow2.offset = 0;
flow2.period = 3;
flow2.delay = 3;
flow2.arrival_prob = 1;
flow2.success_prob = 0.8;
flow2.constructEverything();


ratio = zeros(N_flow,1);

N_instance = 1

fprintf(fileID, 'N_flow=%d, N_instance=%d\n', N_flow, N_instance);

for n_flow=N_flow:N_flow
    n_flow 
    
    flow_array = cell(n_flow,1);
    for ii=1:n_flow
        if(mod(ii,2) == 1)
            flow_array{ii} = flow1;
        else
            flow_array{ii} = flow2;
        end
    end
    
    fprintf(fileID, '\nn_flow=%d\n', n_flow);
    for ii=1:n_flow
        fprintf(fileID, 'Flow %d: (offset, period, delay, success_prob, arrival_prob) = (%d, %d, %d, %f, %f)\n', ii, flow_array{ii}.offset, ...
            flow_array{ii}.period, flow_array{ii}.delay, flow_array{ii}.success_prob, flow_array{ii}.arrival_prob(1));
    end

    obj = DownlinkAPInstance();
    obj.n_flow = n_flow;
    obj.flow_array = flow_array;
    obj.constructEverything();
    %obj.stateSanityCheck();
    
    for ii=1:N_instance %ten realization of the coefficient
        ii
        utility_coeff = 100*rand(obj.n_flow,1);
        %utility_coeff = ones(obj.n_flow,1);      
        %utility_coeff = [0.1135, 0.5373, 0.3492];
       % utility_coeff = utility_coeff./sum(utility_coeff);
        utility_form = 'weighted_sum';
        tic;
        [optimal_policy, optimal_utility, optimal_throughput_per_flow] = ...
            getOptimalSolutionRAC_v(obj, utility_coeff, utility_form);
        toc
        
        [optimal_policy_approx, optimal_action_distribution_approx, optimal_utility_approx, optimal_throughput_per_flow_approx] = ...
            getApproximateSolutionRAC(obj, utility_coeff, utility_form);
        
        ratio(n_flow) = max(ratio(n_flow), optimal_utility_approx/optimal_utility);
        
        ratio
    end
    
    fprintf(fileID, 'ratio=%f\n', ratio(n_flow));
    
end

ratio

fclose(fileID);

save(sprintf('%s/outer_bound_ratio.mat',filePath));



figure;
font_size = 25;
line_width = 3;
set(gca,'FontSize',font_size);
plot(2:N_flow,  ratio(2:N_flow), '-bo', 'Linewidth', line_width);
xlabel('Number of flows');
ylabel('Ratio');
xlim([2,N_flow]);
%ylim([1,1.08]);
%ylim([0,1.5]);
grid on;
box on;
export_fig(sprintf('%s/ratio', filePath), '-pdf','-transparent','-nocrop');

