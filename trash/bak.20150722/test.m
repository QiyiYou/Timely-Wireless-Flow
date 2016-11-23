clear all; close all; 
n_flow = 3;
flow1 = FlowInstance();
flow1.offset = 0;
flow1.period = 3;
flow1.delay = 4;
flow1.arrival_prob = rand(1,flow1.period);
flow1.success_prob = 0.6;
flow1.constructEverything();

flow2 = FlowInstance();
flow2.offset = 0;
flow2.period = 4;
flow2.delay = 3;
flow2.arrival_prob = rand(1,flow2.period);
flow2.success_prob = 0.6;
flow2.constructEverything();

flow3 = FlowInstance();
flow3.offset = 0;
flow3.period = 2;
flow3.delay = 2;
flow3.arrival_prob = rand(1,flow3.period);
flow3.success_prob = 0.6;
flow3.constructEverything();

flow_array(1) = flow1;
flow_array(2) = flow2;
flow_array(3) = flow3;

obj = DownlinkAPInstanceFromFlowInstance();
obj.n_flow = n_flow;
obj.flow_array = flow_array;
obj.constructEverything();

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

utility_coeff = rand(obj.n_flow,1);
utility_form = 'weighted_sum'
%[optimal_policy, optimal_utility, optimal_throughput_per_flow] = getOptimalSolutionRACFromFlowInstance_CVX(obj, utility_coeff, utility_form)


[optimal_policy, optimal_utility, optimal_throughput_per_flow] = getApproximateSolutionRACFromFlowInstance_CVX(obj, utility_coeff, utility_form);


