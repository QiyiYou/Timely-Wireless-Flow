clear all; close all;

%consider determinstic model and find the most intensive interval

n_flow = 3;

%flow1 = FlowInstance();
flow1 = NonOverlappedFlowInstance();
flow1.offset = 0;
flow1.period = 4;
flow1.delay = 2;
flow1.arrival_prob = 1;
flow1.success_prob = 1;
flow1.constructEverything();

flow2 = NonOverlappedFlowInstance();
%flow2 = FlowInstance();
flow2.offset = 0;
flow2.period = 3;
flow2.delay = 3;
flow2.arrival_prob = 1;
flow2.success_prob = 1;
flow2.constructEverything();

flow3 = FlowInstance();
flow3.offset = 0;
flow3.period = 3;
flow3.delay = 2;
flow3.arrival_prob = 1;
flow3.success_prob = 1;
flow3.constructEverything();


flow_array = cell(n_flow,1);
flow_array{1} = flow1;
flow_array{2} = flow2;
flow_array{3} = flow3;


obj = DownlinkAPInstance();
obj.n_flow = n_flow;
obj.flow_array = flow_array;
obj.constructEverything();
obj.stateSanityCheck();
obj.stateActionSanityCheck();

T1 = 2*obj.period_lcm;
T2 = 10*obj.period_lcm;
intensity = zeros(T1,T2);
for t1=1:T1
    for t2=t1+1:T2
        intensity(t1,t2) = obj.getIntervalIntensity(t1,t2);
    end
end

figure;
legendInfo = cell(T1,1);
color = rand(T1,3);
hold on;
for t1=1:T1
    plot(1:T2, intensity(t1,1:T2), 'color', color(t1,:), 'linewidth', 2);
    legendInfo{t1} = ['t1 = ' num2str(t1)];
end
legend(legendInfo);
hold off;
xlabel('t2');
ylabel('intensity');