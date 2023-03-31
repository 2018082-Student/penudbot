clear all
close all
clc

syms a4 a5 q2 m1 m2 l1 l2 g d1 d2 q1 a1 a2 a3
m1 = 1;
m2 = 1;
l1 = 0.5;
l2 = 0.5;
d1 = 0.25;
d2 = 0.25;
I_1 = 0.021458;
I_2 = 0.021458;

a1 = I_1 + m1 * d1^2 + m2 * l1^2;
a2 = I_2 + m2 * d2^2;
a3 = m2 * l1 * d2;
a4 = m2 * l1 + m1 * d1;
a5 = m1 * d1;

Er = a4 + a5;

n = 10000;
q2_lin = linspace(0, 2*pi, n);
y= zeros(1,n);
for i = 1:length(y)
    q2 = q2_lin(i);
    y( i ) = ( 1 / a3 ) * ( Er + sqrt( a4^2 + a5^2 + 2 * a4 * a5 * cos(q2) ) ) * ( a1 * a3 - a2^2 * cos( q2 )^2 );
end
plot(q2_lin,y);