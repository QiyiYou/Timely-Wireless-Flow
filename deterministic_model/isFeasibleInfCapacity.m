function [ isFeasible, x_opt ] = isFeasibleInfCapacity(K, D)
%isFeasibleInfCapacity Check the input K flows with delay-array D is
%feasible or not if we assume the channel capacity is +inf.
%   Detailed explanation goes here
%   Input: 
%         K: # of flows
%         D: size(D) = 1xK, D(k) is the delay of flow k
%
%   Output:
%         isFeasible:  isFeasible=1 means that the input K flows can be
%                      supported, isFeasible=0 otherwise
%         x_opt: Only useful when isFeasible=1. Describe the feasible scheduling
%                policy. x_opt(k,t)=1 means that at slot t, flow k should be scheduled

% the total nubmer of slots to be considered
T = 1;
for kk=1:K
    T = lcm(T,D(kk));
end
% Define variables, x(k,t)=1 means that flow k is scheduled at slot t
x = binvar(K,T);

% Define constraints and objective

Cons = [];
% constraints to meet the deadline

% the first T-D+1 terms
for kk=1:K
    for tt=1:T-D(kk)+1
        Cons = Cons + [sum(x(kk, tt:tt+D(kk)-1)) >= 1];
    end
end


% the last D-1 terms (tail + head)
% concatenate the tail and the head of x
y = [x,x];
for kk=1:K
    for tt=T-D(kk)+2:T
        Cons = Cons + [sum(y(kk, tt:tt+D(kk)-1)) >= 1];
    end
end


% constraints to only schedule one flow
for tt=1:T
    Cons = Cons + [sum(x(:,tt)) <= 1];
end

% 
Obj = sum(sum(x));

% Set some options for YALMIP and solver
opt = sdpsettings('verbose',0,'solver','gurobi');
% Solve the problem
sol = optimize(Cons,Obj,opt);
% Analyze error flags
if sol.problem == 0
 % Extract and display value
    isFeasible = 1;
    x_opt = value(x);
else
    isFeasible = 0;
    x_opt = [];
%     display('Hmm, something went wrong!');
%     sol.info
%     yalmiperror(sol.problem)
end



end

