clear;clc;
% load sistem_ident;

%% Diseño del controlador LQR

% Se obtiene el espacio de estados del sistema identificado
% sys = tf(Plant_Ident_3P_Step_10m);
% sys = tf([7],[1 0 0]);
T = 0.02
amplitud = 1/100;
% val_init = -0.08;
val_init = 0;

% [A,B,C,D] = tf2ss(sys.Numerator{1}, sys.Denominator{1});
A = [0 1; 0 0]; B = [0; 7]; C = [1 0]; D = 0;

sys_ss = ss(A,B,C,D);

% Se muestra la controlabilidad
co = ctrb(sys_ss);
controlability = rank(co)

% Se cacucla la matriz de retroalimentación
Q = C'*C;
% Q(1,1) = 20 * 10^11; 
% Q = Q*8*10^5;   % Ajustar Q hasta obtener la respuesta deseada
R = 1;
K = lqr(A,B,Q,R)

% Se realimenta el sistema y se obtiene el sistema controlado
Ac = [(A-B*K)];
sys_cl = ss(Ac,B,C,D);

% Se simula el sistema y se muestran los resultados
t = 0:T:3;
r = amplitud*ones(size(t));
[y,t,x]=lsim(sys_cl,r,t, [val_init, 0]);
figure(1)
plot(t,y*100);
title('Respuesta al escalón con LQR')
xlabel('Tiempo (segundos)');
ylabel('Distancia (cm)');

% Se muestra el valor de la cte por la que hay que multipicar
% la consigna para obtener el valor en el estacionario deseado
Kr = 1 / dcgain(sys_cl)

% Cálcula de la sobreoscilación
Mp = (max(y) - y(end)) / y(end) * 100

%% Simulación de la acción de control
% El objetivo es obtener a lacción de control a través de Y
% Y = Cx + Du => Y = U = R - Kx => D = 0; C = K;
% Y = Kx; U = R' - Y;
C_c = [0 1/7];
D_c = 0;

% Se realimenta el sistema y se obtiene el sistema controlado
sys_cl = ss(Ac,B,C_c,D_c);

% Se simula el sistema y se muestran los resultados
[y,t,x]=lsim(sys_cl,r,t, [-0.1, 0]);
u = r' - y;
figure(2);
plot(t, rad2deg(gradient(u,T)));
title('Señal de control con LQR')
xlabel('Tiempo (segundos)');
ylabel('Ángulo (º)');




