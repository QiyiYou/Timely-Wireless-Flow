clear all; close all; 
n_flow = 2;
flow1 = FlowInstance();
flow1.offset = 0;
flow1.period = 4;
flow1.delay = 4;
flow1.arrival_prob = [1,0,0,0];
flow1.success_prob = 0.8;
flow1.constructEverything();

flow2 = FlowInstance();
flow2.offset = 2;
flow2.period = 4;
flow2.delay = 4;
flow2.arrival_prob = [1, 0, 0, 0];
flow2.success_prob = 0.6;
flow2.constructEverything();


flow_array(1) = flow1;
flow_array(2) = flow2;

obj = DownlinkAPInstanceFromFlowInstance();
obj.n_flow = n_flow;
obj.flow_array = flow_array;
obj.constructEverything();

%sanity check
for ss=1:obj.n_state
    ss_vec = obj.getVectorState(ss);
    ss_back = obj.getStateFromVector(ss_vec);
    fprintf('ss=%d,ss_vec=[ ', ss);
    for ii=1:obj.n_flow
        fprintf('%d, ', ss_vec(ii));
    end
    fprintf('], ss_back=%d\n', ss_back);
    if(ss ~= ss_back)
        error('something wrong when ss=%d, ss_back=%d', ss, ss_back);
    end
end


utility_coeff = zeros(obj.n_flow,1);
coeff_vec= [0:0.1:1];
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
    utility_form = 'weighted_log_sum';
    tic;
 %   [optimal_policy, optimal_utility, optimal_throughput_per_flow] = getOptimalSolutionRACFromFlowInstance_CVX(obj, utility_coeff, utility_form);
    Rec_elapsed_time_optimal(nn) = toc;
    fprintf('utility_coeff(1)=%d, Finish RAC optimization using CVX with time %f seconds\n', utility_coeff(1), Rec_elapsed_time_optimal(nn));
    
    tic;
    [optimal_policy_approx, optimal_utility_approx, optimal_throughput_per_flow_approx] = getApproximateSolutionRACFromFlowInstance_CVX(obj, utility_coeff, utility_form);
    Rec_elapsed_time_approx(nn) = toc;
    fprintf('utility_coeff(1)=%d, Finish RAC approxmiate optimization using CVX with time %f seconds\n', utility_coeff(1), Rec_elapsed_time_approx(nn));
    
%     Rec_optimal_utility(nn) = optimal_utility;
%     Rec_optimal_throughput_per_flow(nn,:) = optimal_throughput_per_flow;
    
    Rec_optimal_utility_approx(nn) = optimal_utility_approx;
    Rec_optimal_throughput_per_flow_approx(nn,:) = optimal_throughput_per_flow_approx;
end

save('capacity_region_two_flow_paper_example.mat');

load('capacity_region_two_flow_paper_example.mat');

figure;
set(gca,'FontSize',20);
hold on;
%plot(Rec_optimal_throughput_per_flow(:,1),  Rec_optimal_throughput_per_flow(:,2), '-r', 'Linewidth', 3);
plot(Rec_optimal_throughput_per_flow_approx(:,1),  Rec_optimal_throughput_per_flow_approx(:,2), '--b', 'Linewidth', 3);
hold off;
% xlabel('$R_1$','FontSize', 20, 'FontName', 'Arial', 'Interpreter', 'latex');
% ylabel('$R_2$','FontSize', 20, 'FontName', 'Arial', 'Interpreter', 'latex');
xlabel('R1','FontSize', 20, 'FontName', 'Arial');
ylabel('R2','FontSize', 20, 'FontName', 'Arial');
xlim([0,1]);
ylim([0,1]);
box on;
grid on;
print(sprintf('capacity_regin_two_flow'),'-dpdf');
print(sprintf('capacity_regin_two_flow'),'-depsc');




