#n_flow=2, n_instance=10
sed -i 's/n_flow=[0-9].*/n_flow=2/' comparision_scheduling_algorithm_many_flows.m
sed -i 's/n_instance=[0-9].*/n_instance=10/' comparision_scheduling_algorithm_many_flows.m
for (( i=1; i<=8; i++ ))
do
	matlab < comparision_scheduling_algorithm_many_flows.m &> /dev/null &
	sleep 10
done