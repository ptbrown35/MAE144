%%

den = rand(1,5) * 10;
num = rand(1,3) * 10;

disp('zeros')
roots(num)
disp('poles')
roots(den)

%%

G = tf(num,den)
figure(1)
rlocus(G)
figure(2)
bode(G)
