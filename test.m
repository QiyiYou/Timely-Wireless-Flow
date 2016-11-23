%% show the efficiency ratio of our RAC-Approx solution for frame-synchronized traffic pattern
clear all; close all;

n_flow = 4;

flow1 = NonOverlappedFlowInstance();
%flow1 = FlowInstance();
flow1.offset = 0;
flow1.period = 3;
flow1.delay = 3;
flow1.arrival_prob = 0.5;
flow1.success_prob = 0.8;
flow1.constructEverything();

%flow2 = NonOverlappedFlowInstance();
flow2 = FlowInstance();
flow2.offset = 1;
flow2.period = 2;
flow2.delay = 3;
flow2.arrival_prob = 0.5;
flow2.success_prob = 0.6;
flow2.constructEverything();


flow_array = cell(n_flow,1);
for ii=1:n_flow
    if(ii == 1)
        flow_array{ii} = flow1;
    else
        flow_array{ii} = flow2;
    end
end
            
obj = DownlinkAPInstance();
obj.n_flow = n_flow;
obj.flow_array = flow_array;
obj.constructEverything();
%obj.stateSanityCheck();

