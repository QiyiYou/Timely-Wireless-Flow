clear all; close all; 
n_flow = 2;

filePath = sprintf('fig/dual_example/n_flow=%d',n_flow);
fileID = fopen(sprintf('%s/conf_dual_example.txt',filePath),'w');

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

% flow3 = NonOverlappedFlowInstance();
% flow3.offset = 4;
% flow3.period = 6;
% flow3.delay = 5;
% flow3.arrival_prob = 1;
% flow3.success_prob = 0.3;
% flow3.constructEverything();

flow_array = cell(n_flow,1);
flow_array{1} = flow1;
flow_array{2} = flow2;
%flow_array{3} = flow3;


for ii=1:n_flow
    fprintf(fileID, '\nFlow %d: (offset, period, delay, arrival_prob, success_prob) = (%d, %d, %d, %f, %f), ', ii, flow_array{ii}.offset, ...
        flow_array{ii}.period, flow_array{ii}.delay, flow_array{ii}.arrival_prob, flow_array{ii}.success_prob);
    fprintf(fileID, ')\n');
end

obj = DownlinkAPInstance();
obj.n_flow = n_flow;
obj.flow_array = flow_array;
obj.constructEverything();
obj.stateSanityCheck();

%utility_coeff = [0.3, 0.7, 0.8];
utility_coeff = [0.3, 0.7];
utility_coeff = utility_coeff./sum(utility_coeff);
utility_form = 'weighted_log_sum';

fprintf(fileID, '\n%s, utility_coeff=[', utility_form);
for ii=1:n_flow
    fprintf(fileID, '%f,',utility_coeff(ii));
end
fprintf(fileID, ']\n');

tic;
[optimal_policy_RAC, optimal_utility_RAC, optimal_throughput_per_flow_RAC] = ...
    getOptimalSolutionRAC(obj, utility_coeff, utility_form);
fprintf(fileID, '\nFinish getOptimalSolutionRAC with time %f seconds\n', toc);

T = 1000;

tic;
[theoretical_utlity_full_dual, theoretical_throughput_full_dual,...
    primal_rate_user_full_dual, primal_state_full_dual, primal_action_full_dual, primal_state_action_full_dual, primal_state_action_distribution_full_dual, ...
    dual_queue_user_full_dual, dual_lambda_full_dual, dual_mu_full_dual] ...
    = getFullDualDistributedSolution(obj, utility_coeff, utility_form, T);
fprintf(fileID, '\nFinish getFullDualDistributedSolution with time %f seconds\n', toc);


tic;
[theoretical_utlity_full_dual2, theoretical_throughput_full_dual2,...
    primal_state_full_dual2, primal_action_full_dual2, primal_state_action_full_dual2, primal_state_action_distribution_full_dual2, ...
    dual_queue_user_full_dual2, dual_lambda_full_dual2, dual_mu_full_dual2] ...
    = getFullDualDistributedSolutionWithRate(obj, utility_coeff, utility_form, optimal_throughput_per_flow_RAC, T);
fprintf(fileID, '\nFinish getFullDualDistributedSolutionWithRate with time %f seconds\n', toc);


tic;
[successful_transmission_full_dual, ...
    state_action_distribution_full_dual, system_state_full_dual, system_action_full_dual, state_action_per_slot_full_dual ] ...
    = fullDualSchedule(obj, T, primal_state_action_full_dual);
fprintf(fileID, '\nFinish fullDualSchedule with time %f seconds\n', toc);

tic;
[successful_transmission_full_dual2, ...
    state_action_distribution_full_dual2, system_state_full_dual2, system_action_full_dual2, state_action_per_slot_full_dual2 ] ...
    = fullDualSchedule(obj, T, primal_state_action_full_dual2);
fprintf(fileID, '\nFinish fullDualSchedule (with rate) with time %f seconds\n', toc);

tic;
[successful_transmission_half_dual,...
    primal_rate_user_half_dual, primal_state_half_dual, primal_action_half_dual, primal_state_action_half_dual, primal_state_action_distribution_half_dual, ...
    dual_queue_user_half_dual, dual_lambda_half_dual, dual_mu_half_dual] ...
    = halfDualSchedule(obj, utility_coeff, utility_form, T);
fprintf(fileID, '\nFinish halfDualSchedule with time %f seconds\n', toc);

cummean_successful_transmission_full_dual = cummean(successful_transmission_full_dual,2);
cummean_successful_transmission_full_dual2 = cummean(successful_transmission_full_dual2,2);
cummean_successful_transmission_half_dual = cummean(successful_transmission_half_dual,2);

cummean_utlity_full_dual = zeros(T,1);
cummean_utlity_full_dual2 = zeros(T,1);
cummean_utlity_half_dual = zeros(T,1);
for tt=1:T
    for kk=1:obj.n_flow
        if(isequal(utility_form, 'weighted_sum'))
            cummean_utlity_full_dual(tt) =  cummean_utlity_full_dual(tt) + utility_coeff(kk)*(cummean_successful_transmission_full_dual(kk,tt));
            cummean_utlity_full_dual2(tt) =  cummean_utlity_full_dual2(tt) + utility_coeff(kk)*(cummean_successful_transmission_full_dual2(kk,tt));
            cummean_utlity_half_dual(tt) =  cummean_utlity_half_dual(tt) + utility_coeff(kk)*(cummean_successful_transmission_half_dual(kk,tt));
        elseif(isequal(utility_form, 'weighted_log_sum'))
            cummean_utlity_full_dual(tt) =  cummean_utlity_full_dual(tt) + utility_coeff(kk)*log(cummean_successful_transmission_full_dual(kk,tt));
            cummean_utlity_full_dual2(tt) =  cummean_utlity_full_dual2(tt) + utility_coeff(kk)*log(cummean_successful_transmission_full_dual2(kk,tt));
            cummean_utlity_half_dual(tt) =  cummean_utlity_half_dual(tt) + utility_coeff(kk)*log(cummean_successful_transmission_half_dual(kk,tt));
        else
            error('wrong input utility_form, can only be ''weighted_sum'' or ''weighted_log_sum''');
        end
    end
end

fclose(fileID);

save(sprintf('%s/dual_example.mat',filePath));
%load('fig/dual_example/n_flow=2/dual_example.mat');

for nn=1:obj.n_flow
    figure;
    font_size = 12;
    line_width = 3;
    set(gca,'FontSize',font_size);
    hold on;
    plot(1:T,theoretical_throughput_full_dual(nn,:), '-g', 'Linewidth', line_width);
    plot(1:T,cummean_successful_transmission_full_dual(nn,:), '-b', 'Linewidth', line_width);
    plot(1:T,cummean_successful_transmission_full_dual2(nn,:), '-m', 'Linewidth', line_width);
    plot(1:T,cummean_successful_transmission_half_dual(nn,:), '-k', 'Linewidth', line_width);
    plot(1:T,optimal_throughput_per_flow_RAC(nn).*ones(1,T), '-.r', 'Linewidth', line_width);
    hold off;
    xlabel('Slot','FontSize', font_size, 'FontName', 'Arial');
    ylabel('Running Timely Throu.','FontSize', font_size, 'FontName', 'Arial');
    legend('Full-Dual-Theory', 'Full-Dual-Schedule',  'Full-Dual2-Schedule', 'Half-Dual-Schedule', 'Optimal', 'Location', 'Southeast');
%    legend('Half-Dual-Schedule', 'Optimal', 'Location', 'Southeast');
 %   ylim([0.1,0.3]);
%    xlim([1,T]);
    %title(sprintf('%d flows, ',obj.n_flow));
    box on;
    grid off;
%     print(sprintf('%s/comparision_scheduling_flow_%d',filePath, nn),'-dpdf');
%     print(sprintf('%s/comparision_scheduling_flow_%d',filePath, nn),'-depsc');
    export_fig(sprintf('%s/dual_example_flow_%d',filePath, nn), '-pdf','-transparent','-nocrop');
    export_fig(sprintf('%s/dual_example_flow_%d',filePath, nn), '-eps','-transparent','-nocrop');
end


figure;
font_size = 12;
line_width = 3;
set(gca,'FontSize',font_size);
hold on;
plot(1:T,theoretical_utlity_full_dual, '-g', 'Linewidth', line_width);
plot(1:T,cummean_utlity_full_dual, '-b', 'Linewidth', line_width);
plot(1:T,cummean_utlity_full_dual2, '-m', 'Linewidth', line_width);
plot(1:T,cummean_utlity_half_dual, '-k', 'Linewidth', line_width);
plot(1:T,optimal_utility_RAC.*ones(1,T), '-.r', 'Linewidth', line_width);
hold off;
xlabel('Slot','FontSize', font_size, 'FontName', 'Arial');
ylabel('Running Utility','FontSize', font_size, 'FontName', 'Arial');
legend('Full-Dual-Theory', 'Full-Dual-Schedule', 'Full-Dual2-Schedule', 'Half-Dual-Schedule', 'Optimal', 'Location', 'Southeast');
%legend('Half-Dual-Schedule', 'Optimal', 'Location', 'Southeast');
%    ylim([0.1,0.25]);
%    xlim([1,T]);
%title(sprintf('%d flows, ',obj.n_flow));
box on;
grid off;
%     print(sprintf('%s/comparision_scheduling_flow_%d',filePath, nn),'-dpdf');
%     print(sprintf('%s/comparision_scheduling_flow_%d',filePath, nn),'-depsc');
export_fig(sprintf('%s/dual_example_utility',filePath), '-pdf','-transparent','-nocrop');
export_fig(sprintf('%s/dual_example_utility',filePath), '-eps','-transparent','-nocrop');

