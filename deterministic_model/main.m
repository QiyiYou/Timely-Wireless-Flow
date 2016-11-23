% Define variables
sdpvar L f1 f2 f3 f4 f5;
sdpvar x1 x2 x3 x4;

% Define constraints and objective

% nonnegative constraints
Cons = [f1 >=0; f2 >= 0; f3 >= 0; f4 >= 0; f5 >= 0];
Cons = Cons + [x1 >= 0; x2 >= 0; x3 >= 0; x4 >= 0];

% flow constraints
Cons = Cons + [L == f1+f4];
Cons = Cons + [f1 == f2; f2 == f3; f4 == f5];
Cons = Cons + [f3 + f5 == L];

% volume-capacity constraints
Cons = Cons + [f1 <= x1];
Cons = Cons + [f2 <= x2 + x3];
Cons = Cons + [f3 <= x4];
Cons = Cons + [f4 <= x2 + x4];
Cons = Cons + [f5 <= x1 + x3];

% frequence sum equals to 1
Cons = Cons + [x1 + x2 + x3 + x4 == 1];

% maximize the total flow L
Obj = -L;

% Set some options for YALMIP and solver
opt = sdpsettings('verbose',1,'solver','gurobi');
% Solve the problem
sol = optimize(Cons,Obj,opt);
% Analyze error flags
if sol.problem == 0
 % Extract and display value
    L_opt = value(L)
else
    display('Hmm, something went wrong!');
    sol.info
    yalmiperror(sol.problem)
end