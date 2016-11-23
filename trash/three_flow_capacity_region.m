%use MDP to formulate the timely throughput problem (Mobihoc 2015)

close all; clear all;

obj = DownlinkAPInstance();

%mannually set DownlinkAPInstance
obj.n_flow = 3; %number of users/flows
obj.offset = zeros(obj.n_flow,1); %the offset of the first packet of each flow
obj.period = zeros(obj.n_flow,1); %the period of each flow
obj.delay = zeros(obj.n_flow,1); %the delay of each flow, delay(ii) <= period (ii) because we only consider non-overlapping traffic
obj.success_prob = zeros(obj.n_flow,1); %the probability to successfully deliver a packet of each flow if scheduled
obj.offset(1) = 0;
obj.offset(2) = 0;
obj.offset(3) = 0;
obj.period(1) = 5;
obj.period(2) = 5;
obj.period(3) = 5;
obj.delay(1) = 5;
obj.delay(2) = 5;
obj.delay(3) = 5;
obj.success_prob(1) = 0.3;
obj.success_prob(2) = 0.7;
obj.success_prob(3) = 0.4;

tic;
obj.constructEverything();
fprintf('Finish construction with time %d seconds\n', toc);


utility_coeff = zeros(obj.n_flow,1);
coeff1_vec= [0:0.01:1];
coeff2_vec= [0:0.01:1];
n_coeff1_vec = length(coeff1_vec);
n_coeff2_vec = length(coeff2_vec);
Rec_optimal_utility = zeros(n_coeff1_vec,n_coeff2_vec);
Rec_optimal_throughput_per_flow = zeros(n_coeff1_vec,n_coeff2_vec,obj.n_flow);

for cc1=1:n_coeff1_vec
    utility_coeff(1) = coeff1_vec(cc1);
    for cc2=1:n_coeff2_vec
        
        utility_coeff(2) = coeff2_vec(cc2);
        
        if(utility_coeff(1) + utility_coeff(2) > 1)
            continue;
        end
        
        utility_coeff(3) = max(1 - utility_coeff(1) - utility_coeff(2), 0);
        
        tic;
        [optimal_policy, optimal_utility, optimal_throughput_per_flow] = getOptimalSolutionRAC_CVX(obj, utility_coeff,'weighted_log_sum');
        fprintf('utility_coeff(1)=%d, utility_coeff(2)=%d, Finish RAC optimization using CVX with time %f seconds\n', utility_coeff(1), utility_coeff(2),toc);
       
        Rec_optimal_utility(cc1,cc2) = optimal_utility;
        Rec_optimal_throughput_per_flow(cc1,cc2,:) = optimal_throughput_per_flow;
    end
end

Rec_optimal_throughput_per_flow_2D = reshape(Rec_optimal_throughput_per_flow, [], obj.n_flow);

figure;
set(gca,'FontSize',20);
hold on;
plot( Rec_optimal_throughput_per_flow_2D(:,1),  Rec_optimal_throughput_per_flow_2D(:,2), 'ro', 'Linewidth', 1);
hold off;
xlabel('R1','FontSize', 20, 'FontName', 'Arial');
ylabel('R2','FontSize', 20, 'FontName', 'Arial');
xlim([0,1]);
ylim([0,1]);
box on;
grid on;
print(sprintf('capacity_regin_three_flow_R1_R2'),'-dpdf');

figure;
set(gca,'FontSize',20);
hold on;
plot( Rec_optimal_throughput_per_flow_2D(:,1),  Rec_optimal_throughput_per_flow_2D(:,3), 'ro', 'Linewidth', 1);
hold off;
xlabel('R1','FontSize', 20, 'FontName', 'Arial');
ylabel('R3','FontSize', 20, 'FontName', 'Arial');
xlim([0,1]);
ylim([0,1]);
box on;
grid on;
print(sprintf('capacity_regin_three_flow_R1_R3'),'-dpdf');

figure;
set(gca,'FontSize',20);
hold on;
plot( Rec_optimal_throughput_per_flow_2D(:,2),  Rec_optimal_throughput_per_flow_2D(:,3), 'ro', 'Linewidth', 1);
hold off;
xlabel('R2','FontSize', 20, 'FontName', 'Arial');
ylabel('R3','FontSize', 20, 'FontName', 'Arial');
xlim([0,1]);
ylim([0,1]);
box on;
grid on;
print(sprintf('capacity_regin_three_flow_R2_R3'),'-dpdf');


figure;
set(gca,'FontSize',20);
plot3( Rec_optimal_throughput_per_flow_2D(:,1),  Rec_optimal_throughput_per_flow_2D(:,2), Rec_optimal_throughput_per_flow_2D(:,3), 'ro', 'Linewidth', 1);
xlabel('R1','FontSize', 20, 'FontName', 'Arial');
ylabel('R2','FontSize', 20, 'FontName', 'Arial');
zlabel('R3','FontSize', 20, 'FontName', 'Arial');
xlim([0,1]);
ylim([0,1]);
zlim([0,1]);
box on;
grid on;
print(sprintf('capacity_regin_three_flow'),'-dpdf');
