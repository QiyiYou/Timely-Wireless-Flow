function [optimal_nu, optimal_lambda, optimal_mur, optimal_utility] = getOptimalSolutionRACDualRelax(obj, utility_coeff, utility_form)
% use relaxed version of the dual of RAC formulation (Proposition 4 in Mobihoc 2015) to get the
% optimal utility, we can only use weighted sum 
% obj: DownlinkAPInstanceFromFlowInstance
% utility_coeff: the coefficient for each flow
% utility_form is a string
%     utility_form = 'weighted_sum' means the weighted sum utility with utility_coeff

%Now I will use cvx to solve this convex problem

if(isequal(utility_form, 'weighted_sum'))
    %weighted sum
    fprintf('correct for weighted-sum-utility!\n');
else
    error('wrong input utility_form, can only be ''weighted_sum''');
end

max_n_state = 0;
for ii=1:obj.n_flow
    max_n_state = max(max_n_state, obj.flow_array{ii}.n_state);
end

cvx_begin

    fprintf('begin to construct optimization variables\n');
    
    %nu(h) is the gain of slot h \in [1,T]
    variable nu(obj.period_lcm, 1);

    %lambda(h,s) is the relative value of state s at slot h
    variable lambda(obj.period_lcm, obj.n_state);

    %mu is reserved by some MATLAB function, use mur instaed (r means
    %relax)
    variable mur(obj.n_flow, obj.period_lcm, max_n_state);
    
    fprintf('begin to set objective\n');
    %objective
    expression  Objective;
    Objective = sum(nu);
    
    minimize( Objective );

   
    fprintf('begin to construct optimization constraints\n');
    subject to
        
        for hh=1:obj.period_lcm
            for ss=1:obj.n_state
                for aa=1:obj.n_action
                    
                    prob_temp = 0;
                    for next_ss = 1:obj.n_state
                        prob_temp = prob_temp + lambda(hh,next_ss)*obj.getTransitionProbability(hh+obj.period_lcm,ss,aa, next_ss);
                    end
                    
                    reward_temp = 0;
                    for kk=1:obj.n_flow
                        reward_temp = reward_temp + utility_coeff(kk)*obj.getRewardPerFlow(ss,aa,kk)/obj.period_lcm;
                    end
                    
                    last_hh = 0;
                    if(hh == 1)
                        last_hh = obj.period_lcm;
                    else
                        last_hh = hh - 1;
                    end
                    
                    %the constraints
                    nu(hh) + lambda(last_hh,ss) - prob_temp >= reward_temp;
                end
            end
        end
        
        for hh=1:obj.period_lcm
            for ss=1:obj.n_state
                 ss_vec = obj.getVectorState(ss);
                 temp = 0;
                 for kk=1:obj.n_flow
                     temp = temp + mur(kk,hh,ss_vec(kk));
                 end
                 lambda(hh,ss) == temp;
            end
        end
        
        fprintf('begin to solve the optimization problem\n');
cvx_end

optimal_nu = nu;
optimal_lambda = lambda;
optimal_mur = mur;
optimal_utility = Objective;

end