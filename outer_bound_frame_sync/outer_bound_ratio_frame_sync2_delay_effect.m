%% show the efficiency ratio of our RAC-Approx solution for frame-synchronized traffic pattern
clear all; close all;

rng shuffle

%select gurobi to solve the LP, much faster than the default SDP3
cvx_solver gurobi_2
cvx_save_prefs
cvx_precision best


N_flow=2

filePath = sprintf('../fig/outer_bound_ratio_frame_sync/N_flow=%d',N_flow);
%find the next number for the configuration file
next_conf = 1;
while(1)
    if(exist(sprintf('%s/conf_outer_bound_ratio_delay_effect_%d.txt',filePath, next_conf), 'file') == 2)
        next_conf = next_conf + 1;
    else
        break;
    end
end
fileID = fopen(sprintf('%s/conf_outer_bound_ratio_delay_effect_%d.txt',filePath, next_conf),'w');

success_prob_vec = 0.01:0.01:1

for pp=1:length(success_prob_vec)
    
%success_prob = 0.25;
success_prob = success_prob_vec(pp);

D_vec = 1:1:20
Rec_ratio = zeros(size(D_vec));


fprintf(fileID, 'N_flow=%d, success_prob=%.2f\n', N_flow, success_prob);

for dd = 1:length(D_vec)
    n_flow = N_flow;
    T = D_vec(dd)
    
   
 %   T = floor(n_flow/0.8)+1;
    
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
    
    
    lambda = sort(ones(obj.n_flow,1)*100, 'descend');
    
    
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
    Rec_ratio(dd) = max(Rec_ratio(dd), r0_opt/x);
    
    fprintf(fileID, 'x=%f, r0_opt=%f, ', x, r0_opt);
    fprintf(fileID, 'ratio=%f\n', Rec_ratio(dd));
    
    Rec_ratio
end



Rec_ratio
fprintf(fileID, '\nRec_ratio=[');
for dd=1:length(D_vec)
    fprintf(fileID, '%f,', Rec_ratio(dd));
end
fprintf(fileID, ']\n');

Rec_ratio_theoretic = zeros(size(D_vec));
p = success_prob;
T0 = ceil(1/p)+1;
aT0 = 1 - (T0-1)*0.5*p;
%N_exact: the expected number of delivered packets in a frame in the exact system
%N_relax: the expected number of delivered packets in a frame in the relax system
fprintf(fileID, '\nRec_ratio_theoretic=[');
for dd=1:length(D_vec)
    D = D_vec(dd);
    N_exact = 2*(1-(1-p)^D) - p*D*(1-p)^(D-1);
    
    if(D < T0)
        N_relax = D*p;
    else
        N_relax = (T0-1)*p +  2*aT0*(1-(1-p)^(D-T0+1));
    end
    Rec_ratio_theoretic(dd) = N_relax/N_exact;
    fprintf(fileID, '%f,', Rec_ratio_theoretic(dd));
end
fprintf(fileID, ']\n');

[max_ratio, max_ratio_idx ] = max(Rec_ratio);

fprintf(fileID, 'p=%f,max_ratio=%f, max_ratio_idx=%d\n', success_prob, max_ratio,max_ratio_idx );

save(sprintf('%s/outer_bound_ratio_delay_effect_%d.mat',filePath, next_conf));

end

figure;
font_size = 22.4;
line_width = 2;
set(gca,'FontSize',font_size);
hold on;
plot(D_vec,  Rec_ratio, '-bo', 'Linewidth', line_width, 'Markersize', 10);
plot(D_vec,  Rec_ratio, '-rx', 'Linewidth', line_width, 'Markersize', 10);
hold off;
xlabel('D (=Period)');
ylabel('Ratio');
legend('Opt. Prob.', 'Calculation', 'location', 'best');
title(sprintf('Frame-Synchronized: K=%d, p=%.2f', N_flow, success_prob));
%ylim([0,1.5]);
grid on;
box on;
export_fig(sprintf('%s/ratio_delay_effect_K=%d_p=%.2f_%d', filePath, N_flow, success_prob, next_conf), '-pdf','-transparent','-nocrop');
