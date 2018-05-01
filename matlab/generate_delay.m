% Suppose that currently the delay is d(i), the required acceleration is
% a=5. Based on this situation, the next required delay and be computed by
% the formula:
%       d(i)=(1000*d(i-1)*k)/(500*k + (k*(a*d(i-1)^2 + 250000*k))^(1/2))
% 
% Here we use this formula to compute 10 delays in order, then we test
% those delay.

% step 1: generate delay based on first delay and acceleration
d(1) = 1000;
a = 20;
k = 2500/8;

for i=2:10
    d(i)=(1000*d(i-1)*k)/(500*k + (k*(a*d(i-1)^2 + 250000*k))^(1/2));
end

d

% step 2: test the result
for i=1:10
    v(i)= k/d(i);
end

for i=2:10
    acc(i)=1000000*(v(i)-v(i-1))/d(i);
end

acc
