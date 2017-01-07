clear all; close all;
p = 1/23.5
T0 = ceil(1/p)+1
D=1:0.01:T0-1
ratio_reverse = (2*(1-(1-p).^D)-p*D.*(1-p).^(D-1))./(D*p);
ratio = 1./ratio_reverse;
diff_d1 = -(1-p).^(D-1).*(log(1-p)*(2+(D-2)*p) + p) - p;
diff_d1_approx = (1-p).^(D-1).*( p*(2+(D-2)*p)/(1-p) - p) -p;

A = (2+2*D*p).*(1-p).^D + D.*(D+1).*p^2.*(1-p).^(D-1) - 2;

B = 2*(1+(D+1)*p)*(1-p)^2 + p^2*(1-p)*(D+1).*(D+2) - 2*(1+D*p)*(1-p) - p^2*D.*(D+1);
figure;
hold on
%plot(D, ratio, '-ro');
plot(D, B, '-b');
hold off