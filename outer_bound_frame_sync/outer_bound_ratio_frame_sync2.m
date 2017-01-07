%% show the efficiency ratio of our RAC-Approx solution for frame-synchronized traffic pattern
clear all; close all;

rng shuffle

%select gurobi to solve the LP, much faster than the default SDP3
%cvx_solver gurobi_2
cvx_solver gurobi_3
cvx_save_prefs
cvx_precision best

N_flow=2


filePath = sprintf('../fig/outer_bound_ratio_frame_sync/N_flow=%d',N_flow);
%find the next number for the configuration file
next_conf = 1;
while(1)
    if(exist(sprintf('%s/conf_outer_bound_ratio_%d.txt',filePath, next_conf), 'file') == 2)
        next_conf = next_conf + 1;
    else
        break;
    end
end
fileID = fopen(sprintf('%s/conf_outer_bound_ratio_%d.txt',filePath, next_conf),'w');


N_lambda_instance = 1;
ratio = zeros(N_flow,1);

fprintf(fileID, 'N_flow=%d, N_lambda_instance=%d\n', N_flow, N_lambda_instance);

for n_flow = 2:N_flow
    
    tic;
    
    n_flow
    
    success_prob = 0.13;
    %T = floor(n_flow/success_prob)+1;
    
    T = 4;
    
    flow = NonOverlappedFlowInstance();
    %flow1 = FlowInstance();
    flow.offset = 0;
    flow.period = T;
    flow.delay = T;
    flow.arrival_prob = 1;
    flow.success_prob = success_prob;
    flow.constructEverything();
    
    flow_array = cell(n_flow, 1);
    for ii=1:n_flow
        flow_array{ii} = flow;
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
    
    
    lambda = sort(ones(obj.n_flow,1), 'descend');
    
    
    fprintf(fileID, 'lambda=[');
    for ii=1:obj.n_flow
        fprintf(fileID, '%f, ', lambda(ii));
    end
    fprintf(fileID, ']\n');
    
    cvx_begin
    
    variable x;
    variable w(obj.n_flow,1);
    maximize (x)
    
    subject to
    
    for cc=1:obj.n_flow
        w(cc) == lambda(cc)*x*T/obj.flow_array{cc}.success_prob;
    end
    
    for kk=1:obj.n_flow
        idle_time = getIdleTime(obj, kk);
        sum(w(1:kk)) + idle_time <= T;
    end
    
    cvx_end
    
    [r0_opt, status] = getApproximateSolutionRAC_given_direction(obj, lambda);
    x
    r0_opt
    ratio(n_flow) = max(ratio(n_flow), r0_opt/x);
    ratio
    fprintf(fileID, 'x=%f, r0_opt=%f, ', x, r0_opt);
    fprintf(fileID, 'ratio=%f\n', ratio(n_flow));
    
    fprintf(fileID, 'elapsed time  %f seconds', toc);
end



ratio
fprintf(fileID, '\nration=[');
for ii=1:N_flow
    fprintf(fileID, '%f,', ratio(n_flow));
end
fprintf(fileID, ']\n');

save(sprintf('%s/outer_bound_ratio_%d.mat',filePath, next_conf));


figure;
font_size = 22.4;
line_width = 5;
set(gca,'FontSize',font_size);
plot(2:1:N_flow,  ratio(2:1:N_flow), '-bo', 'Linewidth', line_width);
xlabel('Number of flows');
ylabel('Ratio');
title(sprintf('p=%.2f', success_prob));
%ylim([0,1.5]);
%xlim([2,N_flow]);
grid on;
box on;
export_fig(sprintf('%s/ratio_p=%.2f_%d', filePath, success_prob, next_conf), '-pdf','-transparent','-nocrop');

p = success_prob
D = T
rate_exact = 2*(1-(1-p)^D) - p*D*(1-p)^(D-1)
rate_exact_per_flow = rate_exact/D/N_flow

%
k0 = ceil(1/p)+1
zk0 = 1 - (k0-1)*0.5*p
%2*(0.5*p + (1-0.5*p)*p/(2-p) + (1-p)*min(0.5/(1-p), 1)*p)
if(T < k0)
    rate_relax = 2*0.5*D*p
else
%    rate_relax = 2*0.5*(k0-1)*p +  2*zk0*(1-(1-p)^(D-k0+1));
    rate_relax = 2 - (2- ceil(1/p)*p)*(1-p)^(D-ceil(1/p));
end
rate_relax_per_flow = rate_relax/D/N_flow

ratio_calu = rate_relax/rate_exact

if(abs(ratio(2) - ratio_calu) < 1e-10)
    fprintf('matched\n');
else
    fprintf('abs(ratio(2) - ratio_calu)=%e\n', abs(ratio(2) - ratio_calu));
    fprintf('not matched\n');
end
