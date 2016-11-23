clear all; close all; 


n_flow = 3;

filePath = sprintf('fig/capacity_region_tight_outer_bound/n_flow=%d',n_flow);
fileID = fopen(sprintf('%s/conf_capacity_region_tight_outer_bound.txt',filePath),'w');

flow1 = NonOverlappedFlowInstance();
flow1.offset = 0;
flow1.period = 3;
flow1.delay = 3;
flow1.arrival_prob = 1;
flow1.success_prob = 0.5;
flow1.constructEverything();

flow2 = NonOverlappedFlowInstance();
%flow2 = FlowInstance();
flow2.offset = 0;
flow2.period = 3;
flow2.delay = 3;
flow2.arrival_prob = 1;
flow2.success_prob = 0.5;
flow2.constructEverything();

flow3 = NonOverlappedFlowInstance();
%flow3 = FlowInstance();
flow3.offset = 0;
flow3.period = 3;
flow3.delay = 3;
flow3.arrival_prob = 1;
flow3.success_prob = 0.5;
flow3.constructEverything();


flow_array = cell(n_flow,1);
flow_array{1} = flow1;
flow_array{2} = flow2;
flow_array{3} = flow3;

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


%n_instance = 100;
Rec_optimal_utility = zeros(1,1);
Rec_optimal_throughput_per_flow = zeros(1,n_flow);
Rec_optimal_utility_approx = zeros(1,1);
Rec_optimal_throughput_per_flow_approx = zeros(1,n_flow);

ratio = 1;
tic;
for u1=0:0.1:1 %ten realization of the coefficient
    for u2=0:0.1:1-u1
        utility_coeff = [u1,u2,1-u1-u2];
        %utility_coeff = ones(obj.n_flow,1);
        %utility_coeff = [0.1135, 0.5373, 0.3492];
        utility_coeff = utility_coeff./sum(utility_coeff);
        utility_form = 'weighted_sum';
        
        
        [optimal_policy, optimal_utility, optimal_throughput_per_flow] = ...
            getOptimalSolutionRAC_v(obj, utility_coeff, utility_form);
        
        
        [optimal_policy_approx, optimal_action_distribution_approx, optimal_utility_approx, optimal_throughput_per_flow_approx] = ...
            getApproximateSolutionRAC(obj, utility_coeff, utility_form);
        
        Rec_optimal_utility(end+1) = optimal_utility;
        Rec_optimal_throughput_per_flow(end+1,:) = optimal_throughput_per_flow;
        
        Rec_optimal_utility_approx(end+1) = optimal_utility_approx;
        Rec_optimal_throughput_per_flow_approx(end+1,:) = optimal_throughput_per_flow_approx;
        
        ratio = max(ratio, optimal_utility_approx/optimal_utility);
    end
end
toc;

ratio
%make the capacity region is complete 
% Rec_optimal_throughput_per_flow(1,1) = 0;
% Rec_optimal_throughput_per_flow_approx(1,1) = 0;
% Rec_optimal_throughput_per_flow(end,2) = 0;
% Rec_optimal_throughput_per_flow_approx(end,2) = 0;

max_r = max(Rec_optimal_throughput_per_flow);
Rec_optimal_throughput_per_flow(end+1,:) = [0, 0, 0];
Rec_optimal_throughput_per_flow(end+1,:) = [max_r(1), 0, 0];
Rec_optimal_throughput_per_flow(end+1,:) = [0, max_r(2), 0];
Rec_optimal_throughput_per_flow(end+1,:) = [0, 0, max_r(3)];

max_r_approx = max(Rec_optimal_throughput_per_flow_approx);
Rec_optimal_throughput_per_flow_approx(end+1,:) = [0, 0, 0];
Rec_optimal_throughput_per_flow_approx(end+1,:) = [max_r_approx(1), 0, 0];
Rec_optimal_throughput_per_flow_approx(end+1,:) = [0, max_r_approx(2), 0];
Rec_optimal_throughput_per_flow_approx(end+1,:) = [0, 0, max_r_approx(3)];




% fprintf(fileID, '\nRec_optimal_throughput_per_flow=\n');
% for nn=1:n_instance
%     fprintf(fileID, '(%f, %f)\n', Rec_optimal_throughput_per_flow(nn,1), Rec_optimal_throughput_per_flow(nn,2));
% end
% 
% fprintf(fileID, '\nRec_optimal_throughput_per_flow_approx=\n');
% for nn=1:n_instance
%     fprintf(fileID, '(%f, %f)\n', Rec_optimal_throughput_per_flow_approx(nn,1), Rec_optimal_throughput_per_flow_approx(nn,2));
% end


fclose(fileID);

save(sprintf('%s/capacity_region_tight_outer_bound.mat',filePath));

%load('fig/capacity_region_tight_outer_bound/n_flow=2/capacity_region_tight_outer_bound.mat');

flow_conf = '';
for ii=1:n_flow
    flow_conf = sprintf('%sFlow %d: (offset, period, delay, success-prob, arrival-prob) = (%d, %d, %d, %.2f, %.2f)\n', flow_conf, ii, flow_array{ii}.offset, ...
        flow_array{ii}.period, flow_array{ii}.delay, flow_array{ii}.success_prob, flow_array{ii}.arrival_prob(1));
end
%remove the last '\n'
flow_conf = flow_conf(1:length(flow_conf)-1)


figure;
font_size = 22.4;
line_width = 5;
set(gca,'FontSize',font_size);

K = convhull(Rec_optimal_throughput_per_flow(:,1),Rec_optimal_throughput_per_flow(:,2),Rec_optimal_throughput_per_flow(:,3));
trisurf(K,Rec_optimal_throughput_per_flow(:,1),Rec_optimal_throughput_per_flow(:,2),Rec_optimal_throughput_per_flow(:,3), ...
    'FaceColor','w', 'EdgeColor', 'r', 'linewidth', line_width );
hold on;
K_approx = convhull(Rec_optimal_throughput_per_flow_approx(:,1),Rec_optimal_throughput_per_flow_approx(:,2),Rec_optimal_throughput_per_flow_approx(:,3));
trisurf(K_approx,Rec_optimal_throughput_per_flow_approx(:,1),Rec_optimal_throughput_per_flow_approx(:,2),Rec_optimal_throughput_per_flow_approx(:,3), ...
    'FaceColor','w', 'EdgeColor', 'b', 'linewidth', line_width);
hold off;
xlabel('R1','FontSize', font_size, 'FontName', 'Arial');
ylabel('R2','FontSize', font_size, 'FontName', 'Arial');
zlabel('R3','FontSize', font_size, 'FontName', 'Arial');
hl=legend('$$\mathcal{R}$$ in (10)', '$$\mathcal{R}^{\mathrm{outer}}$$ in (13)','Location','Northeast');
set(hl, 'Interpreter','latex');
box on;
grid on;
title(flow_conf, 'Fontsize', 10);

figure;
font_size = 22.4;
line_width = 5;
set(gca,'FontSize',font_size);

K = convhull(Rec_optimal_throughput_per_flow(:,1),Rec_optimal_throughput_per_flow(:,2),Rec_optimal_throughput_per_flow(:,3));
trisurf(K,Rec_optimal_throughput_per_flow(:,1),Rec_optimal_throughput_per_flow(:,2),Rec_optimal_throughput_per_flow(:,3), ...
    'FaceColor','w', 'EdgeColor', 'r', 'linewidth', line_width );
% hold on;
% K_approx = convhull(Rec_optimal_throughput_per_flow_approx(:,1),Rec_optimal_throughput_per_flow_approx(:,2),Rec_optimal_throughput_per_flow_approx(:,3));
% trisurf(K_approx,Rec_optimal_throughput_per_flow_approx(:,1),Rec_optimal_throughput_per_flow_approx(:,2),Rec_optimal_throughput_per_flow_approx(:,3), ...
%     'FaceColor','w', 'EdgeColor', 'b', 'linewidth', line_width);
% hold off;
xlabel('R1','FontSize', font_size, 'FontName', 'Arial');
ylabel('R2','FontSize', font_size, 'FontName', 'Arial');
zlabel('R3','FontSize', font_size, 'FontName', 'Arial');
%hl=legend('$$\mathcal{R}$$ in (10)', '$$\mathcal{R}^{\mathrm{outer}}$$ in (13)','Location','Northeast');
set(hl, 'Interpreter','latex');
box on;
grid on;
title(flow_conf, 'Fontsize', 10);

figure;
font_size = 22.4;
line_width = 5;
set(gca,'FontSize',font_size);

%K = convhull(Rec_optimal_throughput_per_flow(:,1),Rec_optimal_throughput_per_flow(:,2),Rec_optimal_throughput_per_flow(:,3));
%trisurf(K,Rec_optimal_throughput_per_flow(:,1),Rec_optimal_throughput_per_flow(:,2),Rec_optimal_throughput_per_flow(:,3), ...
%    'FaceColor','w', 'EdgeColor', 'r', 'linewidth', line_width );
% hold on;
 K_approx = convhull(Rec_optimal_throughput_per_flow_approx(:,1),Rec_optimal_throughput_per_flow_approx(:,2),Rec_optimal_throughput_per_flow_approx(:,3));
 trisurf(K_approx,Rec_optimal_throughput_per_flow_approx(:,1),Rec_optimal_throughput_per_flow_approx(:,2),Rec_optimal_throughput_per_flow_approx(:,3), ...
     'FaceColor','w', 'EdgeColor', 'b', 'linewidth', line_width);
% hold off;
xlabel('R1','FontSize', font_size, 'FontName', 'Arial');
ylabel('R2','FontSize', font_size, 'FontName', 'Arial');
zlabel('R3','FontSize', font_size, 'FontName', 'Arial');
%hl=legend('$$\mathcal{R}$$ in (10)', '$$\mathcal{R}^{\mathrm{outer}}$$ in (13)','Location','Northeast');
set(hl, 'Interpreter','latex');
box on;
grid on;
title(flow_conf, 'Fontsize', 10);

figure;
font_size = 22.4;
line_width = 5;
set(gca,'FontSize',font_size);
scatter3(Rec_optimal_throughput_per_flow(:,1),  Rec_optimal_throughput_per_flow(:,2), Rec_optimal_throughput_per_flow(:,3), 'r', 'Linewidth', line_width);
hold on;
scatter3(Rec_optimal_throughput_per_flow_approx(:,1),  Rec_optimal_throughput_per_flow_approx(:,2), Rec_optimal_throughput_per_flow_approx(:,3), 'b', 'Linewidth', line_width);
hold off;
%xlabel('$R_1$','FontSize', font_size-2, 'FontName', 'Arial', 'Interpreter', 'latex');
%ylabel('$R_2$','FontSize', font_size-2, 'FontName', 'Arial', 'Interpreter', 'latex');
xlabel('R1','FontSize', font_size, 'FontName', 'Arial');
ylabel('R2','FontSize', font_size, 'FontName', 'Arial');
zlabel('R3','FontSize', font_size, 'FontName', 'Arial');
%max_rate = max(max(max(Rec_optimal_throughput_per_flow)),max(max(Rec_optimal_throughput_per_flow_approx)));
% xlim([0.2,0.5]);
% ylim([0.2,0.5]);
hl=legend('$$\mathcal{R}$$ in (10)', '$$\mathcal{R}^{\mathrm{outer}}$$ in (13)','Location','Northeast');
set(hl, 'Interpreter','latex');
box on;
grid on;
title(flow_conf, 'Fontsize', 10);
%export_fig(sprintf('%s/capacity_region_tight_outer_bound', filePath), '-pdf','-transparent','-nocrop');
%export_fig(sprintf('%s/capacity_region_tight_outer_bound', filePath), '-eps','-transparent','-nocrop');







