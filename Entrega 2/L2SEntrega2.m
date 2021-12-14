clc, clear, close all;
%============ Laboratorio II de Sistemas de Telecomunicaciones ============
% Entrega 2 
%
% PARA ESTA ENTREGA SE REQUIERE AÑADIR EL CANAL DISTORSIVO Y COREGIR
% ERRORES PREVIOS DE SIMULACIÓN
%
% Presentado por Grupo 8: 
% Elkin Burbano Molano y Elmer Jose Muñoz Zuñiga 
% Presentado a: 
% Ing. Manuela Silva
% 14 Dic 2021
% ======================= DEFINICIONES ====================================

M = 8;                      % Orden del esquema de modulación
L = log2(M);                % Número de bits por símbolo 
CantidadSimbolos = 10000;     % Cantidad de bits del mensaje

% ======================= CONSTELACIÓN ====================================
constelacion = [ -1.5 + 1.5j ; 0 + 2j ; 1.5 + 1.5j ; -0.75 + 0j; 0.75 + 0j ; -1.5 - 1.5j ; 0 - 2j ; 1.5 - 1.5j ];

%%
%========================= TRANSMISOR (TX) ================================
% ======================== MENSAJE BINARIO ================================

% Se genera un mensaje aleatorio
b = randsrc(1, CantidadSimbolos*L, [0 1; 0.5 0.5]); 

% ============================ MAPEO ======================================

% Agrupamos los bits del mensaje en una matriz de 'L' columnas (se divide en grupos de 3)
AgrupacionBits = reshape(b, L, [])'; 

%Convertimos cada fila de la matriz en un número decimal
BitsADecimal = bi2de(AgrupacionBits, 'left-msb');

%Se crean los simbolos que representan el mensaje
SYMBOL = Mapeo(BitsADecimal, constelacion);

% CONSTELACIÓN A TRANSMITIR
figure('name','Constelación en Tx'),
plot(SYMBOL,'o','MarkerFaceColor','m','MarkerSize',6)
title('Constelación Asignada')
axis([-3 3 -3 3]);
xlabel('Eje Real')
ylabel('Eje Imaginario')
grid on;

% ====================== FILTRO CONFORMADOR ===============================

Roff = 0.5;     % Roll-off
Span = 8;       % Cantidad de ceros que toman los filtros//ransiente: Es un vector de longitud dos que indica el número de periodos de símbolo antes y después de la respuesta máxima 
Fs = 16;        % Factor de Sobremuestreo NOTA: FS*Span debe dar un número par 
T = 1;          % Duración de cada bit en segundos

% Creamos el filtro conformador, longitud es 2*Fs*Span+1
SRRC = rcosfir(Roff, Span, Fs, T, 'sqrt');    

% Adaptamos los simbolos a la frecuencia del filtro y añadimos ceros para compensar la transiente del filtro
simbolosAdaptados = AdaptarSimbolos(SYMBOL, Fs, Span);

% Se filtra la señal
formaOnda = filter(SRRC, 1, simbolosAdaptados);

% ============== MODULACIÓN Y TRASLACIÓN EN FRECUENCIA ====================

R=1; % Tasa de simbolo
Bw = (R*(1+Roff)/2); % Ancho de banda de un pulso SRRC (ancho de banda de la señal modulada en banda base)
Fportadora= 3*Bw;  %Fportadora = R*Fs; 
n = numel(SYMBOL);

% Se obtiene la señal a transmitir
Modulada = Modulacion(formaOnda, Fportadora, Fs, Span, n);
%%
figure('Name','Comparación antes y despues de la distorsión'),
subplot(311)
FFF =Transformada(Modulada,Fs);
f2=linspace(-Fs/2,Fs/2,length(FFF));
plot(f2,abs(FFF/length(FFF)),'m','LineWidth',1), title('Espectro de la señal Modulada'), grid on;


%========================== CANAL AWGN/ MULTITRAYECTO =====================
%==========================================================================

ES = 0; % Inicialización de Energía de simbolo 
EbNo = 15; % Valor en veces
distanciasSim = DistanciaSimbolos(constelacion, M); 

for i=1:M
    ES = ES +((distanciasSim(i))^2)/M;
end

% MULTITRAYECTO------------------------------------------------------------
%  NOTA: El valor asignado de TAU ES 1s por requerimiento, pero en este
%  caso utilizaremos un valor más alto, para poder observar mejor el
%  comportamiento tanto de los graficos espectrales, como de BER vs EbNo 

tau=2; % En segundos
a=0.2; % Amplitud
SenalMulti=[zeros(1,tau*Fs) Modulada(1:end-tau*Fs)];
Multi=Modulada+(a*SenalMulti);

%------------------------- GRAFICOS ---------------------------------------
% Gráfico de h(f) teorico:
subplot(312)
FFT =Transformada(Multi,Fs);
f3=linspace(-Fs/2,Fs/2,length(FFT));
f=linspace(-Fs/2,Fs/2,length(FFT));

h=sqrt( ((1+0.2.*cos(2*pi*tau*f)).^2) + (0.2.*sin(2*pi*tau*f)).^2);
plot(f,abs(h),'b'),title('Espectro del Modelo de canal distorsivo'),grid on;

% Superposición de Espectro Multitrayecto y Canal Distorsivo.
% NOTA: Se realiza de esta manera, con el fin de observar mejor el efecto
% del canal distorsivo sobre la onda modulada. 

subplot(313)
h=80.*sqrt( ((1+0.2.*cos(2*pi*tau*f)).^2) + (0.2.*sin(2*pi*tau*f)).^2);
plot(f3,abs(FFT/length(FFT)),'m',f,abs(h/length(h)),'b'),title('Espectro de la señal con Multitrayecto y del Modelo de canal distorsivo'),grid on;

%------------------------- RUIDO AWGN--------------------------------------

% Varianza ruido
SIGMA = sqrt(ES/(2*log2(M)*EbNo)); %Es/SNR

% Introducimos ruido AWGN a la señal con multitrayecto
AWGN= SIGMA*randn(1, length(Modulada));
noiseSignal = Multi + AWGN;

%-------------------
figure('Name','Espectro de la señal en recepción (AWGN + Multitrayecto)'),
% Espectro de la señal con ruido AWGN que llega al receptor
noiseSignal_F = Transformada(noiseSignal,Fs);
fn=linspace(-Fs/2,Fs/2,length(noiseSignal_F));
plot(fn, abs(noiseSignal_F),'g'), title('Espectro de la señal en Rx'), grid on;

%%
%=============================== RECEPTOR =================================
% ============================ DEMODULACION ===============================

demodulada = Demodulacion(noiseSignal, Fportadora, Fs,Span, n);

%================================ FILTRAJE ================================
recuperados = filter(SRRC, 1, demodulada); % Recuperar los simolos de la forma de onda 
eyediagram(recuperados,2*Fs)

%=============================== MUESTREO =================================
%Se cortan los ceros que se agregaron a la forma de onda
recuperados = recuperados(2*Span*Fs + 1: end); 
recuperados = downsample(recuperados,Fs);

figure('Name','Simbolos Recibidos'),
plot(recuperados,'o','MarkerEdgeColor','black','MarkerFaceColor','blue','MarkerSize',6), title('Simbolos recibidos'), axis([-3 3 -3 3]);
grid on;

%============================= PROCESO DE DECISIÓN ========================

simbolosEstimados = DistanciaMinima(recuperados, constelacion);

figure('Name','Simbolos Estimados por criterio decisión'),
plot(simbolosEstimados,'o','MarkerEdgeColor','black','MarkerFaceColor','blue','MarkerSize',6), title('Simbolos estimados por criterio decisión'), axis([-3 3 -3 3]);
grid on;

%=============================== DE MAPEO ================================= 

b_D = Demapeo(simbolosEstimados, constelacion);

%==========================================================================
