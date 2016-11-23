%use MDP to formulate the timely throughput problem (Mobihoc 2015)

close all; clear all;

obj = DownlinkAPInstance();

%mannually set DownlinkAPInstance
obj.n_flow = 2; %number of users/flows
obj.offset = zeros(obj.n_flow,1); %the offset of the first packet of each flow
obj.period = zeros(obj.n_flow,1); %the period of each flow
obj.delay = zeros(obj.n_flow,1); %the delay of each flow, delay(ii) <= period (ii) because we only consider non-overlapping traffic
obj.success_prob = zeros(obj.n_flow,1); %the probability to successfully deliver a packet of each flow if scheduled
obj.offset(1) = 0;
obj.offset(2) = 0;
obj.period(1) = 3;
obj.period(2) = 3;
obj.delay(1) = 3;
obj.delay(2) = 3;
obj.success_prob(1) = 0.3;
obj.success_prob(2) = 0.6;

tic;
obj.constructEverything();
fprintf('Finish construction with time %d seconds\n', toc);


utility_coeff = zeros(obj.n_flow,1);
coeff_vec= [0:0.01:1];
n_coeff_vec = length(coeff_vec);
Rec_optimal_utility = zeros(n_coeff_vec,1);
Rec_optimal_throughput_per_flow = zeros(n_coeff_vec,2);

for cc=1:n_coeff_vec
    utility_coeff(1) = coeff_vec(cc);
    utility_coeff(2) = 1-coeff_vec(cc);
    
    n_period_lcm_mdp = 1;
    T_mdp = n_period_lcm_mdp*obj.period_lcm;
    
    tic;
    [optimal_policy, optimal_utility, optimal_throughput_per_flow] = getOptimalSolutionRAC_CVX(obj, utility_coeff, 'weighted_log_sum');
    fprintf('utility_coeff(1)=%d, Finish RAC optimization using CVX with time %f seconds\n', utility_coeff(1), toc);
    
    Rec_optimal_utility(cc) = optimal_utility;
    Rec_optimal_throughput_per_flow(cc,:) = optimal_throughput_per_flow;
end


figure;
set(gca,'FontSize',20);
hold on;
plot( Rec_optimal_throughput_per_flow(:,1),  Rec_optimal_throughput_per_flow(:,2), 'ro', 'Linewidth', 1);
hold off;
xlabel('R1','FontSize', 20, 'FontName', 'Arial');
ylabel('R2','FontSize', 20, 'FontName', 'Arial');
xlim([0,1]);
ylim([0,1]);
box on;
grid on;
print(sprintf('capacity_regin_two_flow'),'-dpdf');
