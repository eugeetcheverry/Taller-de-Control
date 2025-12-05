function [t, y] = euler(f, t0, tf, h)

N = floor((tf - t0)/h); %Number of step

t =t0:h:tf; %Step time
y = zeros(0, N + 1);


end