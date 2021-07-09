%% Simulación de la respuesta del sistema ante LQR
s = tf('s');
sys = 7 / s^2;

Q = C'*C;
R = 1;

X = 0;
filename = '../../Resultados/Camara/LQR_1.txt'; % Archivo de la respuesta del sistema que se quiere leer

% [A,B,C,D] = tf2ss(sys.Numerator{1}, sys.Denominator{1});
A = [0 1; 0 0]; B = [0; 7]; C = [1 0]; D = 0;
sys_ss = ss(A,B,C,D);

K = lqr(A,B,Q,R)

% Se realimenta el sistema y se obtiene el sistema controlado
Ac = [(A-B*K)];
sys_cl = ss(Ac,B,C,D);

% Se abre el fichero con los datos

T = 0.005;
SIZE_DATA = 2; % Número de datos que se envían en un solo paquete (una mitad para cada tipo de dato)

% Se lee el archivo como un vector de char
full_data = fileread(filename);

% Calcula el numero total de elementos en el array y redondea al menor
% multiplo de 4 letras
R = length(full_data);
R = R - mod(R,4);

% Recorre el vector dividiendolo en arrays de 4 letras (2 bytes)
data=[];
for i = 1:4:R
    data(end+1) = sscanf(cat(2, full_data(i+2:i+3), full_data(i:i+1)),'%4x', [1, Inf]);
end

% Se eliminan los X primeros paquetes de datos, para mejorar la respuesta
% inicial

data = data(X*SIZE_DATA*2+1:end);

R = length(data); 
% Divide en datos de entrada y de salida
inputs = [];
datos = [];

for i = 1:R
    if mod(i,2) % Dato de salida
    	inputs(end+1) = ((data(i) / 500 - 50));
    else    % Dato de entrada
        datos(end+1) = (data(i) / 10000 - 0.3);
    end
end

% Se igualan las longitudes de los dos vectores
% R = length(inputs);
% datos = datos(1:R);

% Correción de los datos
datos = datos(abs(datos) < 0.23);

% Definición del vector de tiempos
R = length(datos);
t_datos = (0:T:(R*T-T));

R = length(inputs);
t_inputs = (0:T:(R*T-T));

amp = abs(datos(1))*100;

% Se simula el sistema y se muestran los resultados
r = 0*ones(size(t_datos));
[y,t,x]=lsim(sys_cl,r,t_datos, [-amp, 0]);

% Se dibujan las señales
figure(1);
hold on
plot(t_datos, datos*100);
plot(t_datos,y);
title("Respuesta del sistema")
xlabel('Tiempo (s)')
ylabel('Distancia (cm)')
legend 'Respuesta real' 'Respuesta ideal'

clear data fft_control f P1 P2 filename full_data i j k R X SIZE_DATA
clear s sys Kp Ki Kd pid sys_f