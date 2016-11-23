function [X, Fval]=compute_weighted_r(aepf1, aepf2, aepf3, pvec, w_vec)

% A&E profiles
o1=aepf1(1);
prd1=aepf1(2);
dl1=aepf1(3);

o2=aepf2(1);
prd2=aepf2(2);
dl2=aepf2(3);

o3=aepf3(1);
prd3=aepf3(2);
dl3=aepf3(3);

bigprd=lcm(lcm(prd1,prd2),prd3);

% initialization
f1a=zeros(1,bigprd);
f1e=f1a;
f2a=f1a;
f2e=f1a;
f3a=f1a;
f3e=f1a;


% the channel success probability
p1=pvec(1);
p2=pvec(2);
p3=pvec(3);




simm1=ceil(dl1/prd1);
simm2=ceil(dl2/prd2);
simm3=ceil(dl3/prd3);
for(m=1:(bigprd/prd1))
    f1a(1,mod(o1,prd1)+(m-1)*prd1+1)=1;
    f1e(1,mod(o1+dl1,prd1)+(m-1)*prd1+1)=1;
end
for(m=1:(bigprd/prd2))
    f2a(1,mod(o2,prd2)+(m-1)*prd2+1)=1;
    f2e(1,mod(o2+dl2,prd2)+(m-1)*prd2+1)=1;
end
for(m=1:(bigprd/prd3))
    f3a(1,mod(o3,prd3)+(m-1)*prd3+1)=1;
    f3e(1,mod(o3+dl3,prd3)+(m-1)*prd3+1)=1;
end

% flow-1 states: emptyset, 1. 
% flow-2 states: emptyset, 1. 
% flow-3 states: emptyset, 1.

% Create the transition matrix for arrival and expiration
Psi1a=zeros(simm1+1);
Psi1a(2:(simm1+1), 1:simm1)=eye(simm1);
Psi1a(simm1+1, simm1+1)=1;
Psi1a=sparse(Psi1a);
Psi1e=zeros(simm1+1);
Psi1e(1:simm1, 1:simm1)=eye(simm1);
Psi1e(simm1, simm1+1)=1;
Psi1e=sparse(Psi1e);

Psi2a=zeros(simm2+1);
Psi2a(2:(simm2+1), 1:simm2)=eye(simm2);
Psi2a(simm2+1, simm2+1)=1;
Psi2a=sparse(Psi2a);
Psi2e=zeros(simm2+1);
Psi2e(1:simm2, 1:simm2)=eye(simm2);
Psi2e(simm2, simm2+1)=1;
Psi2e=sparse(Psi2e);

Psi3a=zeros(simm3+1);
Psi3a(2:(simm3+1), 1:simm3)=eye(simm3);
Psi3a(simm3+1, simm3+1)=1;
Psi3a=sparse(Psi3a);
Psi3e=zeros(simm3+1);
Psi3e(1:simm3, 1:simm3)=eye(simm3);
Psi3e(simm3, simm3+1)=1;
Psi3e=sparse(Psi3e);


% prepare the LP 
% total number of x variables: bigprd*12
% total number of y variables: bigprd*12
% total number of w variables: bigprd*21

max_q1=simm1;
fs1=[0:max_q1];
max_q2=simm2;
fs2=[0:max_q2];
max_q3=simm3;
fs3=[0:max_q3];

% construct the big T set
bigT=zeros(simm1+1,simm2+1,simm3+1,4); % for the fourth coordinate, the first entry is the length and the rest is the flow. 
total_state_act_num=1;
for(i1=1:(max_q1+1))
    for(i2=1:(max_q2+1))
        for(i3=1:(max_q3+1))
           if(fs1(1,i1)>=1)
                total_state_act_num=total_state_act_num+1;               
               f_count=bigT(i1,i2,i3,1)+1;
               bigT(i1,i2,i3,1)=f_count;
               bigT(i1,i2,i3,1+f_count)=1;
           end
           if(fs2(1,i2)>=1)
                total_state_act_num=total_state_act_num+1;               
               f_count=bigT(i1,i2,i3,1)+1;
               bigT(i1,i2,i3,1)=f_count;
               bigT(i1,i2,i3,1+f_count)=2;
           end
           if(fs3(1,i3)>=1)
                total_state_act_num=total_state_act_num+1;               
               f_count=bigT(i1,i2,i3,1)+1;
               bigT(i1,i2,i3,1)=f_count;
               bigT(i1,i2,i3,1+f_count)=3;
           end
        end
    end
end
% total number of rate variables: 3

total_n_var=3+bigprd*((max_q1+1)*(max_q2+1)*(max_q3+1)*2+total_state_act_num);
ini_offset=3;
period=(max_q1+1)*(max_q2+1)*(max_q3+1)*2+total_state_act_num;
x_offset=0;
y_offset=(max_q1+1)*(max_q2+1)*(max_q3+1);
w_offset=(max_q1+1)*(max_q2+1)*(max_q3+1)*2;



% set the upper and lower bounds. 
LB=sparse(zeros(total_n_var,1));
UB=ones(total_n_var,1);



% Group 0: Prob inequalities: 
A0=sparse(zeros(1,total_n_var));
A0(1,ini_offset+x_offset+(1:(max_q1+1)*(max_q2+1)*(max_q3+1)))=ones(1,(max_q1+1)*(max_q2+1)*(max_q3+1));
b0=1;

% Group 1: Tx equalities: 
A1=sparse(zeros(bigprd*(w_offset-y_offset), total_n_var));
b1=sparse(zeros(bigprd*(w_offset-y_offset),1));
for(h=0:bigprd-1)
% insert the y variable entries
    A1(h*(w_offset-y_offset)+(1:(w_offset-y_offset)), ini_offset+period*h+y_offset+(1:(w_offset-y_offset)))=sparse(-eye((w_offset-y_offset)));
    col_count=0;
    for(i1=1:(max_q1+1))
        for(i2=1:(max_q2+1))
            for(i3=1:(max_q3+1))
                if(bigT(i1,i2,i3,1)==0)
                    col_count=col_count+1;
                    % the w variable % When not transmitting at all.
                    A1(h*(w_offset-y_offset)+(i1-1)*(max_q2+1)*(max_q3+1)+(i2-1)*(max_q3+1)+i3, ini_offset+period*h+w_offset+col_count)=1;
                else
                    for(sch=1:bigT(i1,i2,i3,1))
                        col_count=col_count+1;
                        chosen_flow=bigT(i1,i2,i3,sch+1);
                        % when fails
                        A1(h*(w_offset-y_offset)+(i1-1)*(max_q2+1)*(max_q3+1)+(i2-1)*(max_q3+1)+i3, ini_offset+period*h+w_offset+col_count)=1-pvec(1,chosen_flow);
                        % when succeeds
                        if(chosen_flow==1)
                            A1(h*(w_offset-y_offset)+(i1-2)*(max_q2+1)*(max_q3+1)+(i2-1)*(max_q3+1)+i3, ini_offset+period*h+w_offset+col_count)=pvec(1,chosen_flow);
                        elseif(chosen_flow==2)
                            A1(h*(w_offset-y_offset)+(i1-1)*(max_q2+1)*(max_q3+1)+(i2-2)*(max_q3+1)+i3, ini_offset+period*h+w_offset+col_count)=pvec(1,chosen_flow);
                        elseif(chosen_flow==3)
                            A1(h*(w_offset-y_offset)+(i1-1)*(max_q2+1)*(max_q3+1)+(i2-1)*(max_q3+1)+i3-1, ini_offset+period*h+w_offset+col_count)=pvec(1,chosen_flow);
                        end
                    end
                end
            end
        end
    end
end

% Group 2 equalities: the expiration/arrival one. 
A2=sparse(zeros(bigprd*y_offset, total_n_var));
b2=sparse(zeros(bigprd*y_offset,1));
for(h=0:bigprd-1)
% insert the x variable entries
    hnext=mod(h+1,bigprd);
    A2(hnext*y_offset+(1:y_offset), ini_offset+period*hnext+x_offset+(1:y_offset))=sparse(-eye(y_offset));
    
    temp1=sparse(eye(max_q1+1));
    if(f1e(1,hnext+1)==1)
        temp1=Psi1e*temp1;
    end
    if(f1a(1,hnext+1)==1)
        temp1=Psi1a*temp1;
    end
    temp2=sparse(eye(max_q2+1));
    if(f2e(1,hnext+1)==1)
        temp2=Psi2e*temp2;
    end
    if(f2a(1,hnext+1)==1)
        temp2=Psi2a*temp2;
    end
    temp3=sparse(eye(max_q3+1));
    if(f3e(1,hnext+1)==1)
        temp3=Psi3e*temp3;
    end
    if(f3a(1,hnext+1)==1)
        temp3=Psi3a*temp3;
    end
    
    actual_evo=kron(kron(temp1,temp2),temp3);
    A2(hnext*y_offset+(1:y_offset), ini_offset+period*h+y_offset+(1:(w_offset-y_offset)))=sparse(actual_evo);
   
end

% Group 3, the scheduling equalities
A3=sparse(zeros(bigprd*y_offset, total_n_var));
b3=sparse(zeros(bigprd*y_offset,1));
for(h=0:bigprd-1)
% insert the x variable entries
    A3(h*y_offset+(1:y_offset), ini_offset+period*h+x_offset+(1:y_offset))=sparse(-eye(y_offset));
    
    col_count=0;
    for(i1=1:max_q1+1)
        for(i2=1:max_q2+1)
            for(i3=1:max_q3+1)
                if(bigT(i1,i2,i3,1)==0)
                    col_count=col_count+1;
                    % the w variable % When not transmitting at all.
                    A3(h*y_offset+(i1-1)*(max_q2+1)*(max_q3+1)+(i2-1)*(max_q3+1)+i3, ini_offset+period*h+w_offset+col_count)=1;
                else
                    for(sch=1:bigT(i1,i2,i3,1))
                        col_count=col_count+1;
                        A3(h*y_offset+(i1-1)*(max_q2+1)*(max_q3+1)+(i2-1)*(max_q3+1)+i3, ini_offset+period*h+w_offset+col_count)=1;
                    end
                end
            end
        end
    end
end
% Group 4, the rate-computation equalities
A4=sparse(zeros(3, total_n_var));
b4=sparse(zeros(3,1));
% insert the rate variable entries
A4(1, 1)=-sum(f1a)/p1;
A4(2, 2)=-sum(f2a)/p2;
A4(3, 3)=-sum(f3a)/p3;

for(h=0:bigprd-1)
    col_count=0;
    for(i1=1:(max_q1+1))
        for(i2=1:(max_q2+1))
            for(i3=1:(max_q3+1))
                if(bigT(i1,i2,i3,1)==0)
                    col_count=col_count+1;
                else
                    for(sch=1:bigT(i1,i2,i3,1))
                        col_count=col_count+1;
                        chosen_flow=bigT(i1,i2,i3,sch+1);
                        % the beneficial contribution
                        if(chosen_flow==1)
                            A4(1, ini_offset+period*h+w_offset+col_count)=1;
                        elseif(chosen_flow==2)
                            A4(2, ini_offset+period*h+w_offset+col_count)=1;
                        elseif(chosen_flow==3)
                            A4(3, ini_offset+period*h+w_offset+col_count)=1;
                        end
                    end
                end
            end
        end
    end
end


f=zeros(total_n_var,1);

f(1:3,1)=-w_vec';
% turn off the display
options = optimset('Display', 'off');
[X Fval]=linprog(f,[],[],[A0;A1;A2;A3;A4],[b0;b1;b2;b3;b4],LB,UB,[],options);
    
 

     


end
