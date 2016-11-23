%I will compare the performance of finite-MDP (suboptimial) and the RAC
%(optimal) for weighted sum utility

close all; clear all;

n_flow = 7; %number of users/flows

filePath = sprintf('fig/heter_deadline_MDP/n_flow=%d',n_flow);
fileID = fopen(sprintf('%s/conf_heter_deadline_MDP.txt',filePath),'w');


%mannually set DownlinkAPInstance

offset_v = zeros(n_flow,1); %the offset of the first packet of each flow
period_v = 10*ones(n_flow,1); %the period of each flow
delay_v = zeros(n_flow,1); %the delay of each flow, delay(ii) <= period (ii) because we only consider non-overlapping traffic
delay_v([1,2]) = 2;
delay_v([3,4]) = 6;
delay_v([5,6,7]) = 10;
success_prob_v = randi([10,100],n_flow,1)/100; %the probability to successfully deliver a packet of each flow if scheduled
%success_prob_v = [0.8400, 0.9200, 0.2100, 0.9300, 0.6700, 0.1800, 0.3500, 0.5900];
utility_coeff = randi([1,20],n_flow,1);
%utility_coeff = [20,7,12,5,16,6,11,14];
utility_form = 'weighted_sum';

flow_array = cell(n_flow,1);
for kk=1:n_flow
    flow_array{kk} = NonOverlappedFlowInstance();
    flow_array{kk}.offset = offset_v(kk);
    flow_array{kk}.period = period_v(kk);
    flow_array{kk}.delay = delay_v(kk);
    flow_array{kk}.arrival_prob = 1;
    flow_array{kk}.success_prob = success_prob_v(kk);
    flow_array{kk}.constructEverything();
end

obj = DownlinkAPInstanceFromFlowInstance();
obj.n_flow = n_flow;
obj.flow_array = flow_array;
obj.constructEverything();
obj.stateSanityCheck_file(fileID);

for ii=1:n_flow
    fprintf(fileID, '\nFlow %d: (offset, period, delay, success_prob) = (%d, %d, %d, %f), ', ii, flow_array{ii}.offset, ...
        flow_array{ii}.period, flow_array{ii}.delay, flow_array{ii}.success_prob);
    fprintf(fileID, 'arrival_prob = (');
    for jj=1:length(flow_array{ii}.arrival_prob)
        fprintf(fileID, '%f, ', flow_array{ii}.arrival_prob(jj));
    end
    fprintf(fileID, ')\n');
end

fprintf(fileID, '\n%s, utility_coeff=[', utility_form);
for ii=1:n_flow
    fprintf(fileID, '%f,',utility_coeff(ii));
end
fprintf(fileID, ']\n');


T = period_v(1);
tic;
fprintf('begin to fullBackwardInduction\n');
[optimal_policy, optimal_reward, optimal_reward_per_flow] = fullBackwardInduction(obj, T, utility_coeff, fileID);
elapsed_time = toc;
fprintf('finish fullBackwardInduction with elapsed time %f \n', elapsed_time);
fprintf(fileID, 'finish fullBackwardInduction with elapsed time %f\n', elapsed_time);

fclose(fileID);

save(sprintf('%s/heter_deadline_MDP.mat',filePath));

%load('fig/heter_deadline_MDP/n_flow=7/heter_deadline_MDP.mat');
