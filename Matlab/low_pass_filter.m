clear all
close all

fc = 10;
num = 2*pi*fc;
den = [1 2*pi*fc];
H = tf(num, den);

Hd = c2d(H, 0.0085)
