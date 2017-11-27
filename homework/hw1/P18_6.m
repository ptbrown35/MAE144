clear all
close all
clc

a5 = .05, a4 = 32.361, a3 = .76881, a2 = 237.95, a1 = 1.0706, a0 = 201.40,
b4 = .001, b3 = .00004, b2 = .024320, b1 = .00036480, b0 = .10706,
bd3 = .000001, bd2 = .0018240, bd1 = 1.0706, bd0 = 201.40;

A = [1 a5 a4 a3 a2 a1 a0]
B = [b4 b3 b2 b1 b0]
BD = [bd3 bd2 bd1 bd0]
numG=B; denG=A;

damp(tf(numG,denG))
tf(numG,denG)

g.K=logspace(-3.5,4,400); g.axes=[-3 1 -2 2];
g.omega=logspace(-2,3,100); g.line=1; g.style='k-';

figure(1), bode(numG,denG)                                                                            
figure(2), Bode(numG,denG,g)