function [optimal_policy, optimal_utility, optimal_delta] = getBalancePrimalSolution(obj, utility_coeff, utility_form)
% use the balance-primal relaxed formulation to get the
% optimal utility, we can use either weighted sum or weithed log sum here
% obj: DownlinkAPInstanceFromFlowInstance
% utility_coeff: the coefficient for each flow
% utility_form is a string
%     utility_form = 'weighted_sum' means the weighted sum utility with utility_coeff
%     utility_form = 'weighted_log_sum' means the weighted log sum utility with utility_coeff

%Now I will use cvx to solve this convex problem

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
    
    %z is the joint distribution of state and action, y(k, sk, ak) is the joint
    %probability that the flow-k is in state sk and the action is ak (1 or 2)
    % action ak=1: transmit this flow k
    % action ak=2: do not transmit this flow k
    variable z(obj.n_flow, max_n_state, 2);
    
    % delta is aribitrary
    variable delta;
    
    %delta == 0;
    
    fprintf('begin to set objective\n');
    %objective
    expression  Objective;
    Objective = 0;
    for kk=1:obj.n_flow
        expression rkk;
        rkk = 0;
        for ss=1:obj.flow_array{kk}.n_state
            rkk = rkk+ obj.flow_array{kk}.reward_per_state_per_action(ss, 1)*z(kk,ss,1);
        end
        
        if(isequal(utility_form, 'weighted_sum'))
            %weighted sum
            Objective = Objective + utility_coeff(kk)*rkk;
        elseif (isequal(utility_form, 'weighted_log_sum'))
            %weighted log sum
            Objective = Objective + utility_coeff(kk)*log(rkk);
        else
            error('wrong input utility_form, can only be ''weighted_sum'' or ''weighted_log_sum''');
        end
    end
    
    maximize( Objective );

   
    fprintf('begin to construct optimization constraints\n');
    subject to
    
    z >= 0;

    
    % z(k,:,:) are probability distributions
    for kk=1:obj.n_flow
        n_state = obj.flow_array{kk}.n_state;
        %sum(sum(squeeze(z(kk,1:n_state,:)))) == 1;
        sum(sum(squeeze(z(kk,1:n_state,:)))) <= 1 + delta;
    end
    
    % summation of z(k,:,1) should be less than 1 
    %sum(sum(squeeze(z(:,:,1)))) <= 1;
    sum(sum(squeeze(z(:,:,1)))) <= 1 - delta;
    
    
    % probablity flow conservation
    for kk=1:obj.n_flow
        for ss=1:obj.flow_array{kk}.n_state
            prob_temp = 0;
            for last_ss=1:obj.flow_array{kk}.n_state
                prob_temp = prob_temp + obj.flow_array{kk}.getTransitionProbability(1,last_ss,1, ss)*z(kk, last_ss,1);
                prob_temp = prob_temp + obj.flow_array{kk}.getTransitionProbability(1,last_ss,2, ss)*z(kk, last_ss,2);
            end
            sum(squeeze(z(kk,ss,:))) == prob_temp;
        end
    end  
    
    
    fprintf('begin to solve the optimization problem\n');
cvx_end

optimal_policy = z;
optimal_utility = Objective;
optimal_delta = delta;
%optimal_throughput_per_flow = r;

end