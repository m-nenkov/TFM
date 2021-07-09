%% Simulación de la respuesta del sistema ante PID
s = tf('s');
sys = 7 / s^2;

Kp = 2.6;
Ki = 0.0;
Kd = 0.4;

X = 0;

filename = '../../Resultados/Pantalla/PID_4.txt'; % Archivo de la respuesta del sistema que se quiere leer

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
t_t = (R*T-T);

pid = pid(Kp, Ki, Kd);
sys_f = feedback(pid*sys,1);
[y t] = step(amp*sys_f,t_t);

% Se dibujan las señales
figure(1);
hold on
plot(t_datos, datos*100);
plot(t, y-amp);
title("Respuesta del sistema")
xlabel('Tiempo (s)')
ylabel('Distancia (cm)')
legend 'Respuesta real' 'Respuesta ideal'

clear data fft_control f P1 P2 filename full_data i j k R X SIZE_DATA
clear s sys Kp Ki Kd pid sys_f