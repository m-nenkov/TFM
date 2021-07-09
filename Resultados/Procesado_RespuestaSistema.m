clc;
% Se abre el fichero con los datos
% filename = 'Capturas del eje X/capture_PIDCam_prueba3.txt';
filename = '../../Resultados/Pantalla/PID_2.txt';


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

X = 170;
data = data(X*SIZE_DATA*2+1:end);

R = length(data); 
% Divide en datos de entrada y de salida
inputs = [];
datos = [];
% j = 1;
% for i = 1:R/SIZE_DATA
%     if mod(i,2) % Dato de salida
%         for k = (i-1)*SIZE_DATA+1:i*SIZE_DATA-1
%             % inputs(end+1) = rad2deg((data(k) / 500 - 50));
%             inputs(end+1) = ((data(k) / 500 - 50));
%         end
%     else    % Dato de entrada
%         for k = (i-1)*SIZE_DATA-1:i*SIZE_DATA
%             datos(end+1) = (data(k) / 10000 - 0.3);
%         end
%     end
%     j = j+1;
% end

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

% Espectro de frecuencia de la señal de control
fft_control = fft(inputs);

P2 = abs(fft_control/R);
P1 = P2(1:R/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = 1/T*(0:(R/2))/R;

% Densidad Espectral de la señal
fft_control = fft_control(1:R/2+1); 
psdx = (T/R) * abs(fft_control).^2; 
psdx(2:end-1) = 2*psdx(2:end-1); 
freq = 0:(1/T)/R:(1/T)/2;

% Se dibujan las señales
figure(1);
plot(t_inputs, inputs);
title("Acción de control")
xlabel('Tiempo (s)')
ylabel('Ángulo (º)')

figure(2);
plot(t_datos, datos*100);
title("Respuesta del sistema")
xlabel('Tiempo (s)')
ylabel('Distancia (cm)')

figure(3);
plot(f, 10*log10(P1));
title("Espectro de frecuencias de la señal de control")
xlabel('f (Hz)')
ylabel('P1 (dB)')

figure(4);
plot(freq,10*log10(psdx))
title('Periodogram Using FFT')
xlabel('Frequency (Hz)')
ylabel('Power/Frequency (dB/Hz)')

clear data fft_control f P1 P2 filename full_data i j k R X SIZE_DATA