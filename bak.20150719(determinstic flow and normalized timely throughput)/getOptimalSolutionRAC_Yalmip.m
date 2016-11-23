function [optimal_policy, optimal_utility, optimal_throughput_per_flow] = getOptimalSolutionRAC_Yalmip(obj, utility_coeff, utility_form)
% use the RAC formulation (Proposition 4 in Mobihoc 2015) to get the
% optimal utility, we can use either weighted sum or weithed log sum here
% obj: DownlinkAPInstance
% utility_coeff: the coefficient for each flow
% utility_form is a string
%     utility_form = 'weighted_sum' means the weighted sum utility with utility_coeff
%     utility_form = 'weighted_log_sum' means the weighted log sum utility with utility_coeff

fprintf('begin to construct optimization variables\n');
 
% x is the marginal distribution of stae, x(t, i) is the praobability
% that the system is in state i at slot t
x = sdpvar(obj.period_lcm, obj.n_state, 'full');

%y is the joint distribution of state and action, y(t, i, j) is the joint
%probability that the system is in state i and takes action j at slot t
y = sdpvar(obj.period_lcm, obj.n_state, obj.n_action, 'full');

%r is the (normalized) timely throughput vector. r(n) is the normalized
%tiemly throughput for flow n
r = sdpvar(obj.n_flow,1, 'full');

fprintf('begin to construct optimization constraints\n');

Constraints = [];
Constraints = Constraints + [x >=0, y >= 0];

% x(t,:) and y(t,:,:) are probability distributions
% Todo: can be expressed in martix way so as to reduce matlab running time
for tt=1:obj.period_lcm
    Constraints = Constraints + [sum(x(tt,:)) == 1];
    Constraints = Constraints + [sum(sum(y(tt,:,:))) == 1];
end

% x(t,:) is the marginal distribution of y(t,:,:) in terms of the state
% Todo: can be expressed in martix way so as to reduce matlab running time
for tt=1:obj.period_lcm
    for ss=1:obj.n_state
        Constraints = Constraints + [x(tt,ss) == sum(y(tt,ss,:))];
    end
end

% x(t+1,:) envolves according to  the transition probability and the last
% joint distribition y(t,:,:)
% Todo: can be expressed in martix way so as to reduce matlab running time
for tt=1:obj.period_lcm
    for ss=1:obj.n_state
        prob_temp = 0;
        for last_ss=1:obj.n_state
            for last_action=1:obj.n_action
                prob_temp = prob_temp + obj.transition_matrix(tt,last_ss,last_action, ss)*y(tt,last_ss,last_action);
            end
        end
        if(tt < obj.period_lcm)
            Constraints = Constraints + [x(tt+1, ss) == prob_temp];
        else % tt = obj.period_lcm
            Constraints = Constraints + [x(1,ss) == prob_temp];
        end
    end
end

%reward constraints
for nn=1:obj.n_flow
    reward_temp = 0;
    for tt=1:obj.period_lcm
        %we need to specficy the action to be nn
        for ss=1:obj.n_state
            reward_temp = reward_temp + obj.reward_per_state_per_action(ss,nn)*y(tt,ss,nn)/(obj.period_lcm/obj.period(nn));
        end
    end
    Constraints = Constraints + [ r(nn) <= reward_temp];
end

fprintf('begin to set objective\n');
%objective
Objective = 0;
for nn=1:obj.n_flow
    if(isequal(utility_form, 'weighted_sum'))
        %weighted sum
        Objective = Objective + utility_coeff(nn)*r(nn);
    elseif (isequal(utility_form, 'weighted_log_sum'))
        %weighted log sum
        Objective = Objective + utility_coeff(nn)*log(r(nn));
    else
        error('wrong input utility_form, can only be ''weighted_sum'' or ''weighted_log_sum''');
    end
end

Options = sdpsettings;
%Options.solver = 'bmibnb';
%Options.solver = 'fmincon';
%Options.solver = 'ipopt';

fprintf('begin to solve the optimization problem\n');

Diag = optimize(Constraints,-Objective,Options);

if Diag.problem == 0
    % Extract and display value
    optimal_policy = value(y);
    optimal_utility = value(Objective);
    optimal_throughput_per_flow = value(r);
else
    display('Hmm, something went wrong!');
    Diag.info
    yalmiperror(Diag.problem)
end

end