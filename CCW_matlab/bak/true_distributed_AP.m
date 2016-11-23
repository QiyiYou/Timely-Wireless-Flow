clc
clear
close all

% A&E profiles
o1=0;
prd1=3;
dl1=2;

o2=0;
prd2=3;
dl2=3;

o3=0;
prd3=3;
dl3=3;


bigprd=lcm(lcm(prd1,prd2),prd3);

% initialization
f1a=zeros(1,bigprd);
f1e=f1a;
f2a=f1a;
f2e=f1a;
f3a=f1a;
f3e=f1a;


% the channel success probability
p1=0.8;
p2=0.4;
p3=0.3;

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
Psi1e=zeros(simm1+1);
Psi1e(1:simm1, 1:simm1)=eye(simm1);
Psi1e(simm1, simm1+1)=1;

Psi2a=zeros(simm2+1);
Psi2a(2:(simm2+1), 1:simm2)=eye(simm2);
Psi2a(simm2+1, simm2+1)=1;
Psi2e=zeros(simm2+1);
Psi2e(1:simm2, 1:simm2)=eye(simm2);
Psi2e(simm2, simm2+1)=1;

Psi3a=zeros(simm3+1);
Psi3a(2:(simm3+1), 1:simm3)=eye(simm3);
Psi3a(simm3+1, simm3+1)=1;
Psi3e=zeros(simm3+1);
Psi3e(1:simm3, 1:simm3)=eye(simm3);
Psi3e(simm3, simm3+1)=1;


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
% Prepare the filter mapping
filtering=zeros((max_q1+1)*(max_q2+1)*(max_q3+1),  total_state_act_num);
for(temp_state=1:(max_q1+1)*(max_q2+1)*(max_q3+1))
    col_count=0;
    for(i1=1:(max_q1+1))
        for(i2=1:(max_q2+1))
            for(i3=1:(max_q3+1))
                if(bigT(i1,i2,i3,1)==0)
                    col_count=col_count+1;
                    % the w variable % When not transmitting at all.
                    if(temp_state~=((i1-1)*(max_q2+1)*(max_q3+1)+(i2-1)*(max_q3+1)+i3))
                        filtering(temp_state, col_count)=-100;
                    else
                        filtering(temp_state, col_count)=100;
                    end
                else
                    for(sch=1:bigT(i1,i2,i3,1))
                        col_count=col_count+1;
                        if(temp_state~=((i1-1)*(max_q2+1)*(max_q3+1)+(i2-1)*(max_q3+1)+i3))
                            filtering(temp_state, col_count)=-100;
                        else
                            filtering(temp_state, col_count)=100;
                        end
                    end
                end
            end
        end
    end
end

%Prepare some variables for later use.     
max_iter_num=750000;
beta=0.0001
% the utility function set it to be a_k\log(r_k+1)
uak=[1.5 2 4];

r_record=zeros(3*bigprd,max_iter_num);
q_record=zeros(3*bigprd,max_iter_num);
w_record=zeros(3,max_iter_num);

r_star=zeros(3,bigprd);
w_star=zeros(total_state_act_num,bigprd);
queue_rate=ones(3,bigprd);
queue_states=zeros((max_q1+1)*(max_q2+1)*(max_q3+1), bigprd);

% current state is evolving between 1 and (max_q1+1)*(max_q2+1)*(max_q3+1)
current_state=1; 


rng('shuffle');
% The main body of the iteration. 
for(iter=1:max_iter_num)
    if(mod(iter,100000)==0)
        iter
    end
    % the inner loop
    for(h=0:bigprd-1)
        %Find the r_star value for the given h
        r_star(:,h+1)=max(0,uak'./queue_rate(:,h+1)-ones(3,1)-sum(r_star,2)+r_star(:,h+1));
        % compute the hprev index
        hprev=mod(h-1,bigprd);
        vec_interest=queue_states(:,h+1)'*Phi_EVO(:,:,h+1)*Phi_TX-queue_states(:,hprev+1)'*Phi_SCH+queue_rate(:,h+1)'*Phi_reward;
        % narrow down to the current state
        vec_interest=min(vec_interest,filtering(current_state,:));
%         mute=[];
%         col_count=0;
%         for(i1=1:(max_q1+1))
%             for(i2=1:(max_q2+1))
%                 for(i3=1:(max_q3+1))
%                     if(bigT(i1,i2,i3,1)==0)
%                         col_count=col_count+1;
%                         % the w variable % When not transmitting at all.
%                         if(current_state~=((i1-1)*(max_q2+1)*(max_q3+1)+(i2-1)*(max_q3+1)+i3))
%                             mute=[mute col_count];
%                         end
%                     else
%                         for(sch=1:bigT(i1,i2,i3,1))
%                             col_count=col_count+1;
%                             if(current_state~=((i1-1)*(max_q2+1)*(max_q3+1)+(i2-1)*(max_q3+1)+i3))
%                                 mute=[mute col_count];
%                             end
%                         end
%                     end
%                 end
%             end
%         end
%         vec_interest(1,mute)=-100*ones(1,length(mute));
        
        [Y,II]=max(vec_interest);
        % choose the action
        w_star(:,h+1)=zeros(length(w_star(:,h+1)),1);
        w_star(II,h+1)=1;
        w_record(:,iter)=w_record(:,iter)+Phi_reward*w_star(:,h+1);
        % The state will evolve accordingly.
        prob_dist=Phi_EVO(:,:,h+1)*Phi_TX(:,II);
        
        real=rand(1);
        temp_sum=0;
        temp_state=0;
        while(real>temp_sum)
            temp_state=temp_state+1;
            temp_sum=temp_sum +prob_dist(temp_state);
        end
        current_state=temp_state;

        % compute the new queue of the states
        % compute the hnext index
%         hnext=mod(h+1,bigprd);
%         temp_vec=queue_states(:,h+1)-beta*(Phi_EVO(:,:,h+1)*Phi_TX*w_star(:,h+1)-Phi_SCH*w_star(:,hnext+1));
%         queue_states(:,h+1)=max(0,temp_vec);
        temp_vec=queue_states(:,h+1)-beta*Phi_EVO(:,:,h+1)*Phi_TX*w_star(:,h+1);
        queue_states(:,h+1)=max(0,temp_vec);
        temp_vec=queue_states(:,hprev+1)+beta*Phi_SCH*w_star(:,h+1);
        queue_states(:,hprev+1)=max(0,temp_vec);
        
        % compute the new queue of the rates
        temp_vec=queue_rate(:,h+1)+beta*(r_star(:,h+1)-Phi_reward*w_star(:,h+1));
        queue_rate(:,h+1)=max(0,temp_vec);
        
        r_record(h*3+(1:3),iter)=r_star(:,h+1);
        q_record(h*3+(1:3),iter)=queue_rate(:,h+1);
       
    end
end

range=length(r_record(1,:));
close all;
plot(1:range, r_record(1,1:range), 1:range, r_record(2,1:range), 1:range, r_record(3,1:range), ...
    1:range, r_record(4,1:range), 1:range, r_record(5,1:range), 1:range, r_record(6,1:range), ...
    1:range, r_record(7,1:range), 1:range, r_record(8,1:range), 1:range, r_record(9,1:range));
legend('flow 1, h=0', 'flow 2, h=0', 'flow 3, h=0', 'flow 1, h=1', 'flow 2, h=1', 'flow 3, h=1', 'flow 1, h=2', 'flow 2, h=2', 'flow 3, h=2');
title('rates');
figure
plot(1:range, q_record(1,1:range), 1:range, q_record(2,1:range), 1:range, q_record(3,1:range), ...
    1:range, q_record(4,1:range), 1:range, q_record(5,1:range), 1:range, q_record(6,1:range), ...
    1:range, q_record(7,1:range), 1:range, q_record(8,1:range), 1:range, q_record(9,1:range));
legend('flow 1, h=0', 'flow 2, h=0', 'flow 3, h=0', 'flow 1, h=1', 'flow 2, h=1', 'flow 3, h=1', 'flow 1, h=2', 'flow 2, h=2', 'flow 3, h=2');
title('q for rates');

% figure;
% plot(1:range, q_record(1,1:range), 1:range, q_record(2,1:range), 1:range, q_record(3,1:range));
% legend('flow 1, h=0', 'flow 2, h=0', 'flow 3, h=0');
% title('q for rates');
% 
% figure;
% plot(1:range, q_record(4,1:range), 1:range, q_record(5,1:range), 1:range, q_record(6,1:range));
% legend('flow 1, h=1', 'flow 2, h=1', 'flow 3, h=1');
% title('q for rates');
% 
% figure;
% plot( 1:range, q_record(7,1:range), 1:range, q_record(8,1:range), 1:range, q_record(9,1:range));
% legend( 'flow 1, h=2', 'flow 2, h=2', 'flow 3, h=2');
% title('q for rates');

satu_start=0.9*range;
r_avg=sum(r_record(:, satu_start:range),2)/(range-satu_start+1);
rr_avg=zeros(3,1);
for(h=0:bigprd-1)
    rr_avg=rr_avg+r_avg((h*3+(1:3)),1);
end

w_avg=sum(w_record(:, satu_start:range),2)/(range-satu_start+1);

aepf1=[o1, prd1, dl1];
aepf2=[o2, prd2, dl2];
aepf3=[o3, prd3, dl3];
pvec=[p1, p2, p3];
w_vec=uak./(1+rr_avg');
[X, Fval]=compute_weighted_r(aepf1, aepf2, aepf3, pvec, w_vec );
LP_weighted_sum=-Fval
LP_rates=X(1:3,1)

LP_utility=uak*log(LP_rates+1)

dist_weighted_sum=w_vec*rr_avg
rr_avg
dist_utility=uak*log(rr_avg+1)

w_weighted_sum=w_vec*w_avg
w_avg
dist_w_utility=uak*log(w_avg+1)

 

     



