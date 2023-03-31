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

a1 = I_1 + m1 * d1^2 + m2 * l1^2
a2 = I_2 + m2 * d2^2
a3 = m2 * l1 * d2
a4 = m2 * l1 + m1 * d1
a5 = m1 * d1

n = 10000;
w_lin = linspace(pi, 2*pi, n);
y= zeros(1,n);
for i = 1:length(y)
    w = w_lin(i);
    y(i) = ( -a4 * sin( w ) * ( a4 * ( 1 - cos(w) ) + 2 * a5 ) ) / w;
end
plot(w_lin,y);