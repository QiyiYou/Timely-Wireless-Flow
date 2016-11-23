close all
clc
clear

to_record=0;

% A&E profiles
test_pattern=3;
if(test_pattern==1)
o1=0;
prd1=3;
dl1=2;

o2=0;
prd2=3;
dl2=3;

o3=0;
prd3=3;
dl3=3;

% the channel success probability
p1=0.8;
p2=0.4;
p3=0.3;

elseif(test_pattern==2)
    % A&E profiles
o1=0;
prd1=4;
dl1=3;

o2=2;
prd2=4;
dl2=4;

o3=0;
prd3=4;
dl3=4;

% the channel success probability
p1=0.8;
p2=0.4;
p3=0.3;

elseif(test_pattern==3)
    % A&E profiles
o1=0;
prd1=5;
dl1=3;

o2=1;
prd2=5;
dl2=5;

o3=0;
prd3=5;
dl3=4;

% the channel success probability
p1=0.6;
p2=0.4;
p3=0.25;

end


bigprd=lcm(lcm(prd1,prd2),prd3);

% initialization
f1a=zeros(1,bigprd);
f1e=f1a;
f2a=f1a;
f2e=f1a;
f3a=f1a;
f3e=f1a;



pvec=[p1,p2,p3];



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

% the utility function set it to be a_k\log(r_k+1)
uak=[1.5 2 4];
queue_rate=ones(1,3);
queue_states=zeros((max_q1+1)*(max_q2+1)*(max_q3+1), bigprd);

max_iter_num=10000;
beta=0.01;
% Some fixed computation
% compute the transmission matrix
Phi_TX=zeros((max_q1+1)*(max_q2+1)*(max_q3+1), total_state_act_num);
Phi_SCH=Phi_TX;
Phi_reward=zeros(3, total_state_act_num);
col_count=0;
for(i1=1:(max_q1+1))
    for(i2=1:(max_q2+1))
        for(i3=1:(max_q3+1))
            if(bigT(i1,i2,i3,1)==0)
                col_count=col_count+1;
                % the w variable % When not transmitting at all.
                Phi_TX((i1-1)*(max_q2+1)*(max_q3+1)+(i2-1)*(max_q3+1)+i3, col_count)=1;
                Phi_SCH((i1-1)*(max_q2+1)*(max_q3+1)+(i2-1)*(max_q3+1)+i3, col_count)=1;
            else
                for(sch=1:bigT(i1,i2,i3,1))
                    col_count=col_count+1;
                    chosen_flow=bigT(i1,i2,i3,sch+1);
                    Phi_SCH((i1-1)*(max_q2+1)*(max_q3+1)+(i2-1)*(max_q3+1)+i3, col_count)=1;
                    Phi_reward(chosen_flow,col_count)=pvec(1,chosen_flow);
                    % when fails
                    Phi_TX((i1-1)*(max_q2+1)*(max_q3+1)+(i2-1)*(max_q3+1)+i3, col_count)=1-pvec(1,chosen_flow);
                    % when succeeds
                    if(chosen_flow==1)
                        Phi_TX((i1-2)*(max_q2+1)*(max_q3+1)+(i2-1)*(max_q3+1)+i3, col_count)=pvec(1,chosen_flow);
                    elseif(chosen_flow==2)
                        Phi_TX((i1-1)*(max_q2+1)*(max_q3+1)+(i2-2)*(max_q3+1)+i3, col_count)=pvec(1,chosen_flow);
                    elseif(chosen_flow==3)
                        Phi_TX((i1-1)*(max_q2+1)*(max_q3+1)+(i2-1)*(max_q3+1)+i3-1, col_count)=pvec(1,chosen_flow);
                    end
                end
            end
        end
    end
end
% Phi_EVO
Phi_EVO=zeros((max_q1+1)*(max_q2+1)*(max_q3+1),(max_q1+1)*(max_q2+1)*(max_q3+1),bigprd);
for(h=0:bigprd-1)
        %Phiae(h) is for the end of time slot h. 
        % compute the hnext index
        hnext=mod(h+1,bigprd);
        %
        temp1=(eye(max_q1+1));
        if(f1e(1,hnext+1)==1)
            temp1=Psi1e*temp1;
        end
        if(f1a(1,hnext+1)==1)
            temp1=Psi1a*temp1;
        end
        temp2=(eye(max_q2+1));
        if(f2e(1,hnext+1)==1)
            temp2=Psi2e*temp2;
        end
        if(f2a(1,hnext+1)==1)
            temp2=Psi2a*temp2;
        end
        temp3=(eye(max_q3+1));
        if(f3e(1,hnext+1)==1)
            temp3=Psi3e*temp3;
        end
        if(f3a(1,hnext+1)==1)
            temp3=Psi3a*temp3;
        end
   Phi_EVO(:,:,h+1)=kron(kron(temp1,temp2),temp3);
end

     

r_record=zeros(3,max_iter_num);
q_record=zeros(3,max_iter_num);
qs_record=zeros(8,max_iter_num,bigprd);
w_record=zeros(13,max_iter_num,bigprd);

for(iter=1:max_iter_num)
   % beta = 1/sqrt(iter);
    if(mod(iter,100000)==0)
        iter
    end
    r_star=uak./queue_rate-ones(1,3);
    w_star=zeros(total_state_act_num,bigprd);
    for(h=0:bigprd-1)
        % compute the hprev index
        hprev=mod(h-1,bigprd);
        vec_interest=queue_states(:,h+1)'*Phi_EVO(:,:,h+1)*Phi_TX-queue_states(:,hprev+1)'*Phi_SCH+queue_rate*Phi_reward;
        [Y,II]=max(vec_interest);
        w_star(II,h+1)=1;
    end
    
    queue_states_old=queue_states;
    for(h=0:bigprd-1)
        % compute the hnext index
        hnext=mod(h+1,bigprd);
        temp_vec=queue_states_old(:,h+1)-beta*(Phi_EVO(:,:,h+1)*Phi_TX*w_star(:,h+1)-Phi_SCH*w_star(:,hnext+1));
        queue_states(:,h+1)=max(0,temp_vec);
    end
    temp_vec=queue_rate+beta*r_star;
    for(h=0:bigprd-1)
        temp_vec=temp_vec-beta*(Phi_reward*w_star(:,h+1))';
    end
    queue_rate=max(0,temp_vec);

    r_record(:,iter)=r_star';
    q_record(:,iter)=queue_rate';
    
    for(h=0:bigprd-1)
        qs_record(:,iter,h+1)=queue_states(:,h+1);
        w_record(:,iter,h+1)=w_star(:,h+1);
    end
    
end
range=max_iter_num;

plot(1:range, r_record(1,1:range), 1:range, r_record(2,1:range), 1:range, r_record(3,1:range));
legend('flow 1', 'flow 2', 'flow 3');
title('rates');
figure
plot(1:range, q_record(1,1:range), 1:range, q_record(2,1:range), 1:range, q_record(3,1:range));
legend('flow 1', 'flow 2', 'flow 3');
title('q for rates');
for(h=0:bigprd-1)
figure
plot(1:range, qs_record(1,1:range,h+1), 1:range, qs_record(2,1:range,h+1), 1:range, qs_record(3,1:range,h+1), 1:range, qs_record(4,1:range,h+1),...
    1:range, qs_record(5,1:range,h+1), 1:range, qs_record(6,1:range,h+1), 1:range, qs_record(7,1:range,h+1), 1:range, qs_record(8,1:range,h+1));
legend('state 1', 'state 2', 'state 3', 'state 4', 'state 5', 'state 6', 'state 7', 'state 8');
title(['qs rates at time ' int2str(h+1)]);
end

satu_start=0.9*max_iter_num;
r_avg=sum(r_record(:, satu_start:max_iter_num),2)/(max_iter_num-satu_start+1);

w_avg=zeros(13, bigprd);
for(h=0:bigprd-1)
    
    w_avg(:,h+1)=sum(w_record(:, satu_start:max_iter_num,h+1),2)/(max_iter_num-satu_start+1);

%qs1_avg=sum(qs1_record(:, satu_start:max_iter_num),2)/(max_iter_num-satu_start+1);
%qs2_avg=sum(qs2_record(:, satu_start:max_iter_num),2)/(max_iter_num-satu_start+1);
%qs3_avg=sum(qs3_record(:, satu_start:max_iter_num),2)/(max_iter_num-satu_start+1);
end

%sim_reward=Phi_reward*(w1avg+w2avg+w3avg);


% verify it with a centralized LP solver
aepf1=[o1, prd1, dl1];
aepf2=[o2, prd2, dl2];
aepf3=[o3, prd3, dl3];
w_vec=uak./(1+r_avg');
[X, Fval]=compute_weighted_r(aepf1, aepf2, aepf3, pvec, w_vec );
LP_weighted_sum=-Fval
LP_rates=X(1:3,1)

LP_utility=uak*log(LP_rates+1)

dist_weighted_sum=w_vec*r_avg
r_avg
dist_utility=uak*log(r_avg+1)

for(h=0:bigprd-1)
    h
    hnext=mod(h+1,bigprd);
    LHS=Phi_SCH*w_avg(:,hnext+1)
    RHS=Phi_EVO(:,:,h+1)*Phi_TX*w_avg(:,h+1)
end


 

     



