function [optimal_policy, optimal_utility, optimal_throughput_per_flow, status] = getOptimalSolutionRAC_v(obj, utility_coeff, utility_form)
% use the RAC formulation (Proposition 4 in Mobihoc 2015) to get the
% optimal utility, we can use either weighted sum or weithed log sum here
% obj: DownlinkAPInstanceFromFlowInstance
% utility_coeff: the coefficient for each flow
% utility_form is a string
%     utility_form = 'weighted_sum' means the weighted sum utility with utility_coeff
%     utility_form = 'weighted_log_sum' means the weighted log sum utility with utility_coeff

%Note that we use vectorized version of the linear constraints

%Now I will use cvx to solve this convex problem

cvx_begin

    fprintf('begin to construct optimization variables\n');
    
    
    %w is the vectorized joint state-action distribution
    variable w(obj.n_state_action, obj.period_lcm);

    %r is the timely throughput vector. r(n) is the
    %tiemly throughput for flow n
    variable r(obj.n_flow,1);
    
    fprintf('begin to set objective\n');
    %objective
    expression  Objective;
    Objective = 0;
    for nn=1:obj.n_flow
        if(isequal(utility_form, 'weighted_sum'))
            %weighted sum
            Objective = Objective + utility_coeff(nn)*r(nn);
        elseif (isequal(utility_form, 'weighted_log_sum'))
            %weighted log sum
            Objective = Objective + utility_coeff(nn)*log(r(nn));
        else
            Objective = Objective + utility_coeff(nn)*sqrt(r(nn));
            %error('wrong input utility_form, can only be ''weighted_sum'' or ''weighted_log_sum''');
        end
    end
    
    maximize( Objective );

   
    fprintf('begin to construct optimization constraints\n');
    subject to
    
       w >= 0;
       r >= 0;
       r >= 0;
       r <= 1;
       sum(w,1) == 1;
  
       for hh=1:obj.period_lcm
           next_hh = 1 + mod(hh, obj.period_lcm);
           obj.Psi*w(:,next_hh) <= obj.Pv(:,:,hh)'*w(:,hh);
       end
        
       for kk=1:obj.n_flow
           temp = 0;
           for hh=1:obj.period_lcm
               temp = temp + obj.theta(:,kk)'*w(:,hh)/obj.period_lcm;
           end
           r(kk) <= temp;
       end
        
        fprintf('begin to solve the optimization problem\n');
cvx_end


%y is the joint distribution of state and action, y(t, i, j) is the joint
%probability that the system is in state i and takes action j at slot t
% see variable y(obj.period_lcm, obj.n_state, obj.n_action) in getOptimalSolutionRAC
optimal_policy = zeros(obj.period_lcm, obj.n_state, obj.n_action); 
for hh=1:obj.period_lcm
    for ss=1:obj.n_state
        for aa=1:obj.n_action
            idx = obj.getIdxFromStateAction(ss,aa);
            optimal_policy(hh, ss, aa) = w(idx, hh);
        end
    end
end


status = cvx_status;
optimal_utility = Objective;
optimal_throughput_per_flow = r;

end