% pkg load control

%% Dise�o de PID a partir de los par�metros ts y Mp
% Definici�n de variables
ts = 3;
Mp = 0.2;

% C�lculo de los valores del polo deseado
syms sig w_d;
wd = pi^2 / (log(1/Mp)*ts);
eqn1 = exp(-sig*ts) / sqrt(1 - (sig / sqrt(sig^2 + wd^2))^2) == exp(-pi);
% eqn2  = exp(-(sig*pi)/w_d) == Mp;

% res = solve([eqn1 eqn2], [sig w_d]);
% sigma = double(res.sig);
% wd = double(res.w_d);
% sigma = double(solve(eqn1, sig));

% sigma = 2.2;
% w_d = 1.71;

sigma = pi / ts;
wd = pi^2 / (log(1/Mp)*ts);

% C�lculo de la posci�n del polo que hay que a�adir
a = tan(pi + 2*(atan(wd/sigma))) * wd + sigma;

% C�lculo del valor de la ganacia
K = sqrt(wd^2 + sigma^2) * sqrt(wd^2 + sigma^2) / sqrt(wd^2 + (sigma - a)^2);

% C�lculo de los valores del PD
Kd = K / 7;
Kp = Kd * abs(a);


% Simulaci�n del resultado
s = tf('s');
sys_pid = pid(Kp, 0, Kd);
sys = 7 / s^2;

figure(1);
step(feedback(sys*sys_pid,1))
title('Respuesta ante entrada escal�n unitario')
xlabel('Tiempo')
ylabel('Amplitud')

figure(2);
rlocus(feedback(sys*sys_pid,1))
title('Lugar de las ra�ces')
xlabel('Eje Real')
ylabel('Eje Imaginario')