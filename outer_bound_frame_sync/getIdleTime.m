function [idle_time] = getIdleTime(obj, k)
%getIdleTime   Get the expected idel time for frame-synchronized traffic pattern if only flows 1,2,...,k can be scheuded under any work-conserving policy.
%  
%  see paper http://cesg.tamu.edu/wp-content//uploads/2012/08/questa12.pdf

%check if the traffic pattern is frame-synchronized
T = obj.flow_array{1}.period;
p = zeros(obj.n_flow,1);
for cc=1:obj.n_flow
    if(obj.flow_array{cc}.delay ~= T || obj.flow_array{cc}.period ~= T || obj.flow_array{cc}.arrival_prob ~= 1 || obj.flow_array{cc}.offset ~=0)
        error('not frame-synchronized traffic pattern');
    end
    p(cc) = obj.flow_array{cc}.success_prob;
end


if(k >= T || T==1)
    idle_time = 0;
    return;
end

%k < T, we calculate it recursively, see Algorithm 1 in paper http://cesg.tamu.edu/wp-content//uploads/2012/08/questa12.pdf

pi = zeros(k, T);
for tt=1:T
    pi(1, tt) = p(1)*(1-p(1))^(tt-1);
end

for cc=2:k
    for tt=1:T
        for tau=1:tt-1
            pi(cc, tt) = pi(cc, tt) + pi(cc-1, tau)*p(cc)*(1-p(cc))^(tt-tau-1); 
        end
    end
end

idle_time = 0;

for tt=1:T-1
    idle_time = idle_time + (T-tt)*pi(k,tt);
end



