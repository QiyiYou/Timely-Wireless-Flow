clear all; close all; 


n_flow = 2;

filePath = sprintf('fig/capacity_region/syn/n_flow=%d',n_flow);
fileID = fopen(sprintf('%s/conf_capacity_region_syn.txt',filePath),'w');

flow1 = NonOverlappedFlowInstance();
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
flow2.success_prob = 0.6;
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


utility_coeff = zeros(obj.n_flow,1);
coeff_vec= [0:0.05:1];
n_coeff_vec = length(coeff_vec);
Rec_optimal_utility = zeros(n_coeff_vec,1);
Rec_optimal_throughput_per_flow = zeros(n_coeff_vec,2);
Rec_optimal_utility_approx = zeros(n_coeff_vec,1);
Rec_optimal_throughput_per_flow_approx = zeros(n_coeff_vec,2);
Rec_elapsed_time_optimal = zeros(n_coeff_vec,1);
Rec_elapsed_time_approx = zeros(n_coeff_vec,1);

for nn=1:n_coeff_vec
    uu = coeff_vec(nn);
    utility_coeff(1) = uu;
    utility_coeff(2) = 1-uu;
    utility_form = 'weighted_sum';
    tic;
    [optimal_policy, optimal_utility, optimal_throughput_per_flow] = ...
        getOptimalSolutionRAC_v(obj, utility_coeff, utility_form);
    Rec_elapsed_time_optimal(nn) = toc;
    fprintf('utility_coeff(1)=%d, Finish RAC optimization using CVX with time %f seconds\n', utility_coeff(1), Rec_elapsed_time_optimal(nn));
    
    tic;
    [optimal_policy_approx, optimal_action_distribution_approx, optimal_utility_approx, optimal_throughput_per_flow_approx] = ...
        getApproximateSolutionRAC(obj, utility_coeff, utility_form);
    Rec_elapsed_time_approx(nn) = toc;
    fprintf('utility_coeff(1)=%d, Finish RAC approxmiate optimization using CVX with time %f seconds\n', utility_coeff(1), Rec_elapsed_time_approx(nn));
    
     Rec_optimal_utility(nn) = optimal_utility;
     Rec_optimal_throughput_per_flow(nn,:) = optimal_throughput_per_flow;
    
    Rec_optimal_utility_approx(nn) = optimal_utility_approx;
    Rec_optimal_throughput_per_flow_approx(nn,:) = optimal_throughput_per_flow_approx;
end

%make the capacity region is complete 
Rec_optimal_throughput_per_flow(1,1) = 0;
Rec_optimal_throughput_per_flow_approx(1,1) = 0;
Rec_optimal_throughput_per_flow(end,2) = 0;
Rec_optimal_throughput_per_flow_approx(end,2) = 0;

fprintf(fileID, '\nRec_optimal_throughput_per_flow=\n');
for nn=1:n_coeff_vec
    fprintf(fileID, '(%f, %f)\n', Rec_optimal_throughput_per_flow(nn,1), Rec_optimal_throughput_per_flow(nn,2));
end

fprintf(fileID, '\nRec_optimal_throughput_per_flow_approx=\n');
for nn=1:n_coeff_vec
    fprintf(fileID, '(%f, %f)\n', Rec_optimal_throughput_per_flow_approx(nn,1), Rec_optimal_throughput_per_flow_approx(nn,2));
end


fclose(fileID);

save(sprintf('%s/capacity_region_syn.mat',filePath));

load('fig/capacity_region/syn/n_flow=2/capacity_region_syn.mat');

figure;
font_size = 22.4;
line_width = 5;
set(gca,'FontSize',font_size);
hold on;
plot(Rec_optimal_throughput_per_flow(:,1),  Rec_optimal_throughput_per_flow(:,2), '-r', 'Linewidth', line_width);
plot(Rec_optimal_throughput_per_flow(:,1),  Rec_optimal_throughput_per_flow(:,2), '-.g', 'Linewidth', line_width-2);
plot(Rec_optimal_throughput_per_flow_approx(:,1),  Rec_optimal_throughput_per_flow_approx(:,2), ':b', 'Linewidth', line_width);
hold off;
%xlabel('$R_1$','FontSize', font_size-2, 'FontName', 'Arial', 'Interpreter', 'latex');
%ylabel('$R_2$','FontSize', font_size-2, 'FontName', 'Arial', 'Interpreter', 'latex');
xlabel('R1','FontSize', font_size, 'FontName', 'Arial');
ylabel('R2','FontSize', font_size, 'FontName', 'Arial');
%max_rate = max(max(max(Rec_optimal_throughput_per_flow)),max(max(Rec_optimal_throughput_per_flow_approx)));
xlim([0.2,0.4]);
ylim([0.2,0.4]);
%set(gca, 'XTickLabel', num2str(get(gca,'XTick')','%f'));
legend('RAC','Hou[4]', 'RAC-Approx','Location','Northeast');
box on;
grid on;
% print(sprintf('%s/capacity_regin_syn', filePath),'-dpdf');
% print(sprintf('%s/capacity_regin_syn', filePath),'-depsc');
export_fig(sprintf('%s/capacity_regin_syn', filePath), '-pdf','-transparent','-nocrop');
export_fig(sprintf('%s/capacity_regin_syn', filePath), '-eps','-transparent','-nocrop');




