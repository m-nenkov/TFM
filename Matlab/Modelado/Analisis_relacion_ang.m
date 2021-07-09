clc;

%% Análisis de la relación entre el ángulo del servo y la base
% Datos de ángulo de la base
alpha = -(out.ang_superior.Data + pi/2);
alpha = rad2deg(alpha);
figure(1);
plot(alpha);

% Datos del ángulo del servomotor
gamma = (out.ang_inferior.Data - deg2rad(90));
gamma = rad2deg(gamma);
figure(2);
plot(gamma);

% Aproximación por una recta de N orden
figure(3);
N = 3;
% Y = gamma; X = alpha;
Y = alpha; X = gamma;

f = polyfit(X, Y, N)

plot(Y, X, 'r', polyval(f,X), X, 'g');
title('Relación entre ángulo del servo-motor y de la plataforma');
xlabel('Ángulo de la plataforma (º)') 
ylabel('Ángulo del servo-motor (º)')
legend({'Simulación','Estimación polinómica'},'Location','northwest')

% Cálculo del error
RMSE = sqrt(mean((Y - polyval(f,X)).^2))