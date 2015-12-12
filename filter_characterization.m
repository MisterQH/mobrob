close all
clc

dt = 1/250;
tau = 0.1;
tau2 = 100.0;
tau3 = 0.1;
alpha = tau / (tau + dt);
alpha2 = tau2 / (tau2 + dt);
alpha3 = tau3 / (tau3 + dt);

alpha_lp = dt / (tau + dt);

hp = tf([alpha, -alpha],[1, -alpha], dt, 'variable','z^-1');
hp2 = tf([alpha2, -alpha2],[1, -alpha2], dt, 'variable','z^-1');
hp3 = tf([alpha3, -alpha3],[1, -alpha3], dt, 'variable','z^-1');

lp = tf([alpha_lp],[1, alpha_lp-1], dt, 'variable','z^-1');

inte = tf([dt],[1, -1], dt, 'variable','z^-1');
diff = tf([1/dt, -1/dt],[1], dt, 'variable','z^-1');

figure(1)
bode(lp^2); grid on; hold on;
bode(hp^2);
legend('Low pass', 'High pass');

figure(2)
bode(lp^2 * diff); grid on;

figure(3)
step(hp * inte); hold on; grid on;
step(hp^2 * inte);
legend('1st order high pass * integral', '2nd order high pass * integral')
