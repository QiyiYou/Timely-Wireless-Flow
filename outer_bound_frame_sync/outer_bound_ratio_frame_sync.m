%% show the efficiency ratio of our RAC-Approx solution for frame-synchronized traffic pattern
clear all; close all;

rng shuffle 

%select gurobi to solve the LP, much faster than the default SDP3
cvx_solver gurobi_2
cvx_save_prefs
cvx_precision best

N_flow=10


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


T = 3;
N_lambda_instance = 1000;
ratio = zeros(N_flow,1);

fprintf(fileID, 'N_flow=%d, N_lambda_instance=%d\n', N_flow, N_lambda_instance);

for n_flow = N_flow:N_flow
    n_flow
    
    %success_prob_vec = zeros(n_flow, 1);
    %success_prob_vec(1:2:end) = 0.6;
    %success_prob_vec(2:2:end) = 0.8;
    
    success_prob_vec = 0.8*ones(n_flow,1);
%    success_prob_vec = rand(n_flow, 1);
    
    for permutation_instance=1:1 %n_flow^3
        
        permutation_instance
        
        flow_array = cell(n_flow,1);
        
        idx_perm = randperm(n_flow);
        
        for ii=1:n_flow
            flow = NonOverlappedFlowInstance();
            %flow1 = FlowInstance();
            flow.offset = 0;
            flow.period = T;
            flow.delay = T;
            flow.arrival_prob = 1;
            flow.success_prob = success_prob_vec(idx_perm(ii));
            flow.constructEverything();
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
        
        for lambda_instance = 1:N_lambda_instance
            lambda = sort(rand(obj.n_flow,1)*100, 'descend')
            
           
            
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
        end
    end
    
    
    
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
plot(2:N_flow,  ratio(2:N_flow), '-bo', 'Linewidth', line_width);
xlabel('Number of flows');
ylabel('Ratio');
%ylim([0,1.5]);
grid on;
box on;
export_fig(sprintf('%s/ratio_%d', filePath, next_conf), '-pdf','-transparent','-nocrop');
