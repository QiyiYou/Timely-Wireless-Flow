function [optimal_lambda, optimal_h, optimal_muu, optimal_obj, optimal_potential_change_not_schedule, optimal_potential_change_schedule] = ...
    getBalanceDualSolution(obj, utility_coeff, utility_form)
% use the balance-dual relaxed formulation to get the
% optimal utility, we can use either weighted sum or weithed log sum here
% obj: DownlinkAPInstanceFromFlowInstance
% utility_coeff: the coefficient for each flow
% utility_form is a string, can only be 'weighted_sum'
%     utility_form = 'weighted_sum' means the weighted sum utility with utility_coeff
%     utility_form = 'weighted_log_sum' means the weighted log sum utility with utility_coeff

%Now I will use cvx to solve this convex problem

if(isequal(utility_form, 'weighted_sum'))
    %weighted sum
else
    error('wrong input utility_form, can only be ''weighted_sum''');
end
        
max_n_state = 0;
for ii=1:obj.n_flow
    %we can only sovle the iid case
    if(obj.flow_array{ii}.period ~= 1)
        error('we can only solve the iid case by using balance formulation');
    end
    max_n_state = max(max_n_state, obj.flow_array{ii}.n_state);
end


cvx_begin

    fprintf('begin to construct optimization variables\n');
    
    %Variable name "mu" is the name of an existing MATLAB function or directory:
    %D:\Program Files\MATLAB\R2014a\toolbox\robust\rctobsolete\mutools\commands\mu.m
    % we use muu to replace mu
    %muu(k,sk) is the dual varialbe (potential) of  state sk for flow k
    variable muu(obj.n_flow, max_n_state);
    
    variable lambda;
    variable h(obj.n_flow,1);
    
    fprintf('begin to set objective\n');
    
    %objective    
    minimize( lambda + sum(h) );

   
    fprintf('begin to construct optimization constraints\n');
    subject to
    
    lambda >= 0;
    h >= 0;
    
    %we set a reference potential for each flow, i.e, muu(k,1) = 0, where
    %state 1 means no flow-k packet
    muu(:,1) == 0;
    
    lambda == sum(h);
    
    % reward + potential change inequaltiy 
    for kk=1:obj.n_flow
        for ss=1:obj.flow_array{kk}.n_state
            potential_change_not_schedule = 0;
            potential_change_schedule = 0;
            for next_ss=1:obj.flow_array{kk}.n_state
                potential_change_not_schedule = potential_change_not_schedule + obj.flow_array{kk}.getTransitionProbability(1,ss,2, next_ss)*muu(kk, next_ss);
                potential_change_schedule = potential_change_schedule + obj.flow_array{kk}.getTransitionProbability(1,ss,1, next_ss)*muu(kk, next_ss);
            end
            potential_change_not_schedule = potential_change_not_schedule - muu(kk,ss);
            potential_change_schedule = potential_change_schedule - muu(kk,ss);
            
            potential_change_not_schedule <= h(kk);
            utility_coeff(kk)*obj.flow_array{kk}.reward_per_state_per_action(ss, 1) + potential_change_schedule <= h(kk) + lambda;
        end
    end  
    
    
    fprintf('begin to solve the optimization problem\n');
cvx_end

optimal_lambda = lambda;
optimal_obj = lambda + sum(h);
optimal_h = h;
optimal_muu = muu;


% check the potential change
optimal_potential_change_not_schedule = zeros(obj.n_flow, max_n_state);
optimal_potential_change_schedule = zeros(obj.n_flow, max_n_state);
for kk=1:obj.n_flow
    for ss=1:obj.flow_array{kk}.n_state
        for next_ss=1:obj.flow_array{kk}.n_state
            optimal_potential_change_not_schedule(kk, ss) = optimal_potential_change_not_schedule(kk, ss) + obj.flow_array{kk}.getTransitionProbability(1,ss,2, next_ss)*muu(kk, next_ss);
            optimal_potential_change_schedule(kk, ss) = optimal_potential_change_schedule(kk, ss) + obj.flow_array{kk}.getTransitionProbability(1,ss,1, next_ss)*muu(kk, next_ss);
        end
        optimal_potential_change_not_schedule(kk, ss) = optimal_potential_change_not_schedule(kk, ss) - muu(kk,ss);
        optimal_potential_change_schedule(kk, ss) = optimal_potential_change_schedule(kk, ss) - muu(kk,ss);
    end
end

end