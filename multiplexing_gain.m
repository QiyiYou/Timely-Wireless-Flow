clear all; close all; 

N_flow = 5;
sum_rate = zeros(1, N_flow);

for nn=2:N_flow
    n_flow = nn
    
    
    flow1 = NonOverlappedFlowInstance();
    flow1.offset = 0;
    flow1.period = 3;
    flow1.delay = 3;
    flow1.arrival_prob = 1/n_flow;  %ensure total arrival rate = 1
    flow1.success_prob = 0.5;
    flow1.constructEverything();

    
    flow_array = cell(n_flow,1);
    for ii=1:n_flow
        flow_array{ii} = flow1;
    end
    
    obj = DownlinkAPInstance();
    obj.n_flow = n_flow;
    obj.flow_array = flow_array;
    obj.constructEverything();
    obj.stateSanityCheck();
    
    utility_coeff =  ones(1,n_flow);
    utility_form = 'weighted_sum';
    
    tic;
    [optimal_policy_RAC, optimal_utility_RAC, optimal_throughput_per_flow_RAC] = ...
        getOptimalSolutionRAC(obj, utility_coeff, utility_form);
    toc;
    
    sum_rate(n_flow) = sum(optimal_throughput_per_flow_RAC);
end


figure;
font_size = 25;
line_width = 3;
set(gca,'FontSize',font_size);
plot(2:N_flow,sum_rate(1,2:N_flow), '-b', 'Linewidth', line_width);
xlabel('# of flows','FontSize', font_size, 'FontName', 'Arial');
ylabel('Sum Timely Throu.','FontSize', font_size, 'FontName', 'Arial');
box on;
grid off;
export_fig('multiplexing_gain.fig');





