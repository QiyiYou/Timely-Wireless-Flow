clear all; close all; 
n_flow = 2;

filePath = sprintf('fig/counter_example_LDF/n_flow=%d',n_flow);
fileID = fopen(sprintf('%s/conf_counter_example_LDF.txt',filePath),'w');

flow1 = NonOverlappedFlowInstance();
flow1.offset = 0;
flow1.period = 4;
flow1.delay = 4;
flow1.arrival_prob = 1;
flow1.success_prob = 0.5;
flow1.constructEverything();

flow2 = NonOverlappedFlowInstance();
flow2.offset = 2;
flow2.period = 4;
flow2.delay = 4;
flow2.arrival_prob = 1;
flow2.success_prob = 0.5;
flow2.constructEverything();

flow_array = cell(n_flow,1);
flow_array{1} = flow1;
flow_array{2} = flow2;


for ii=1:n_flow
    fprintf(fileID, '\nFlow %d: (offset, period, delay, success_prob) = (%d, %d, %d, %f), ', ii, flow_array{ii}.offset, ...
        flow_array{ii}.period, flow_array{ii}.delay, flow_array{ii}.success_prob);
    fprintf(fileID, 'arrival_prob = (');
    for jj=1:length(flow_array{ii}.arrival_prob)
        fprintf(fileID, '%f, ', flow_array{ii}.arrival_prob(jj));
    end
    fprintf(fileID, ')\n');
end

obj = DownlinkAPInstance();
obj.n_flow = n_flow;
obj.flow_array = flow_array;
obj.constructEverything();
obj.stateSanityCheck();

utility_coeff = [0.5, 0.5];
utility_coeff = utility_coeff./sum(utility_coeff);
utility_form = 'weighted_sum';

fprintf(fileID, '\n%s, utility_coeff=[', utility_form);
for ii=1:n_flow
    fprintf(fileID, '%f,',utility_coeff(ii));
end
fprintf(fileID, ']\n');

tic;
[optimal_policy_RAC, optimal_utility_RAC, optimal_throughput_per_flow_RAC] = ...
    getOptimalSolutionRAC(obj, utility_coeff, utility_form);
fprintf(fileID, '\nFinish getOptimalSolutionRAC with time %f seconds\n', toc);




tic;
[optimal_policy_RAC_approx, optimal_action_distribution_RAC_approx, optimal_utility_RAC_approx, optimal_throughput_per_flow_RAC_approx] = ...
    getApproximateSolutionRAC(obj, utility_coeff, utility_form);
fprintf(fileID, '\nFinish getApproximateSolutionRAC with time %f seconds\n', toc);




T = 6000;

tic;
[successful_transmission_RAC, state_action_distribution_RAC, system_state_RAC, system_action_RAC, state_action_per_slot_RAC ] ...
    = RACSchedule(obj, T, optimal_policy_RAC);
fprintf(fileID, '\nFinish RACSchedule with time %f seconds\n', toc);



tic;
[successful_transmission_RAC_approx, state_action_distribution_RAC_approx, system_state_RAC_approx, system_action_RAC_approx, state_action_per_slot_RAC_approx] ...
 = RelaxedRACSchedule(obj, T, optimal_policy_RAC_approx, optimal_action_distribution_RAC_approx);
fprintf(fileID, '\nFinish RelaxedRACSchedule with time %f seconds\n', toc);



epsilon = 0;
%strict_throughput_per_flow = max(throughput_per_flow - epsilon, 0);
% let us use optimal throughput from RAC optmization problem
strict_throughput_per_flow = max(optimal_throughput_per_flow_RAC - epsilon, 0);



tic;
[successful_transmission_LDF, virtual_arrival_LDF, virtual_server_capacity_LDF, virtual_departure_LDF,  virtual_queue_LDF, state_action_distribution_LDF, ...
 Rec_system_state_LDF, Rec_system_action_LDF, Rec_state_action_per_slot_LDF] = ...
    LDF(obj, T, strict_throughput_per_flow);
fprintf(fileID, '\nFinish LDF with time %f seconds\n', toc);

tic;
[successful_transmission_LLDF, virtual_arrival_LLDF, virtual_server_capacity_LLDF, virtual_departure_LLDF,  virtual_queue_LLDF, state_action_distribution_LLDF, ...
 Rec_system_state_LLDF, Rec_system_action_LLDF, Rec_state_action_per_slot_LLDF] = ...
    LLDF(obj, T, strict_throughput_per_flow);
fprintf(fileID, '\nFinish LLDF with time %f seconds\n', toc);


cummean_successful_transmission_LDF = cummean(successful_transmission_LDF,2);
cummean_successful_transmission_LLDF = cummean(successful_transmission_LLDF,2);
cummean_successful_transmission_RAC = cummean(successful_transmission_RAC,2);
cummean_successful_transmission_RAC_approx = cummean(successful_transmission_RAC_approx,2);

save(sprintf('%s/counter_example_LDF.mat',filePath));
%load('fig/counter_example_LDF/n_flow=2/counter_example_LDF.mat');

for nn=1:obj.n_flow
    figure;
    font_size = 25;
    line_width = 3;
    set(gca,'FontSize',font_size);
    hold on;
     plot(1:T,cummean_successful_transmission_LDF(nn,:), '-', 'color', [0,0,0], 'Linewidth', line_width);
     plot(1:T,cummean_successful_transmission_LLDF(nn,:), '-', 'color', [0,0.78,0], 'Linewidth', line_width);
 %   plot(1:T,cummean_successful_transmission_RAC_approx(nn,:), '-m', 'Linewidth', line_width);
    plot(1:T,cummean_successful_transmission_RAC(nn,:), '-b', 'Linewidth', line_width);
    plot(1:T,optimal_throughput_per_flow_RAC(nn).*ones(1,T), '-.r', 'Linewidth', line_width);
    hold off;
    xlabel('Slot','FontSize', font_size, 'FontName', 'Arial');
    ylabel('Running Timely Throu.','FontSize', font_size, 'FontName', 'Arial');
    %legend('EPDF', 'LDF', 'L-LDF', 'RAC-Approx', 'Finite-MDP', 'RAC', 'Optimal', 'Location', 'Northeast');
    legend('LDF', 'L-LDF',  'RAC', 'Optimal', 'Location', 'Southeast');
    ylim([0.1,0.25]);
%    xlim([1,T]);
    %title(sprintf('%d flows, ',obj.n_flow));
    box on;
    grid off;
%     print(sprintf('%s/comparision_scheduling_flow_%d',filePath, nn),'-dpdf');
%     print(sprintf('%s/comparision_scheduling_flow_%d',filePath, nn),'-depsc');
    export_fig(sprintf('%s/counter_example_LDF_flow_%d',filePath, nn), '-pdf','-transparent','-nocrop');
    export_fig(sprintf('%s/counter_example_LDF_flow_%d',filePath, nn), '-eps','-transparent','-nocrop');
end




