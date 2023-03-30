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
g = 9.81;
a1 =0.3333333333333333;
a2= 0.08333333333333333;
a3 = 0.125;
a4 = 7.3575;
a5 = 2.4525;
Er = (l1+d2)*m2*g+1*m1*g;


%f = (1 / a2) * ((l1+d2)*m2*g+1*m1*g + sqrt( a4^2 + a5^2 + 2 * a4 * a5 * cos(q2) ) ) * (a1 * a2 - a3^2 * cos(q2).^2);

%f = ( -a4 * (a4 * (1 - cos(q2)) + 2 * a5) * sin(q2) ) / q2

%f2 = diff(f,q2) == 0;

%extreme_points = solve(f2,q2)
n = 1000;
q2_lin = linspace(0, 2*pi, n);
y= zeros(1,n);
for i = 1:length(y)
    q2 = q2_lin(i);
    y(i) = (1 / a2) * ((l1+d2)*m2*g+1*m1*g + sqrt( a4^2 + a5^2 + 2 * a4 * a5 * cos(q2) ) ) * (a1 * a2 - a3^2 * cos(q2)^2);
end

plot(q2_lin,y);