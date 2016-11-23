clear all; close all; 


n_flow = 1;

filePath = sprintf('fig/capacity_region/iid/n_flow=%d',n_flow);
fileID = fopen(sprintf('%s/conf_capacity_region_iid.txt',filePath),'w');

flow1 = FlowInstanceWithSubsidy();
flow1.offset = 0;
flow1.period = 1;
flow1.delay = 3;
flow1.arrival_prob = 0.7;
flow1.success_prob = 0.8;
flow1.W = 0.8;
flow1.constructEverything();



flow_array = cell(n_flow,1);
flow_array{1} = flow1;

for ii=1:n_flow
    fprintf(fileID, '\nFlow %d: (offset, period, delay, success_prob) = (%d, %d, %d, %f), ', ii, flow_array{ii}.offset, ...
        flow_array{ii}.period, flow_array{ii}.delay, flow_array{ii}.success_prob);
    fprintf(fileID, 'arrival_prob = ');
    for jj=1:length(flow_array{ii}.arrival_prob)
        fprintf(fileID, '%f, ', flow_array{ii}.arrival_prob(jj));
    end
    fprintf(fileID, ', W=%f', flow_array{ii}.W);
    fprintf(fileID, '\n');
end

obj = DownlinkAPInstance();
obj.n_flow = n_flow;
obj.flow_array = flow_array;
obj.constructEverything();
obj.stateSanityCheck();


utility_coeff = zeros(obj.n_flow,1);
coeff_vec= [1:0.1:1];
n_coeff_vec = length(coeff_vec);
Rec_optimal_utility = zeros(n_coeff_vec,1);
Rec_optimal_throughput_per_flow = zeros(n_coeff_vec,1);
Rec_optimal_utility_approx = zeros(n_coeff_vec,1);
Rec_optimal_throughput_per_flow_approx = zeros(n_coeff_vec,1);
Rec_elapsed_time_optimal = zeros(n_coeff_vec,1);
Rec_elapsed_time_approx = zeros(n_coeff_vec,1);

for nn=1:n_coeff_vec
    uu = coeff_vec(nn);
    utility_coeff(1) = uu;
    utility_coeff(2) = 1-uu;
    utility_form = 'weighted_sum';
    tic;
    [optimal_policy, optimal_utility, optimal_throughput_per_flow] = ...
        getOptimalSolutionRAC(obj, utility_coeff, utility_form);
    Rec_elapsed_time_optimal(nn) = toc;
    fprintf('utility_coeff(1)=%d, Finish RAC optimization using CVX with time %f seconds\n', utility_coeff(1), Rec_elapsed_time_optimal(nn));
    
     Rec_optimal_utility(nn) = optimal_utility;
     Rec_optimal_throughput_per_flow(nn,:) = optimal_throughput_per_flow;
    
end

%make the capacity region is complete 
Rec_optimal_throughput_per_flow(1,1) = 0;
Rec_optimal_throughput_per_flow_approx(1,1) = 0;
Rec_optimal_throughput_per_flow(end,2) = 0;
Rec_optimal_throughput_per_flow_approx(end,2) = 0;


fprintf(fileID, '\nRec_optimal_throughput_per_flow=\n');
for nn=1:n_coeff_vec
    fprintf(fileID, '%f\n', Rec_optimal_throughput_per_flow(nn,1));
end


fclose(fileID);






