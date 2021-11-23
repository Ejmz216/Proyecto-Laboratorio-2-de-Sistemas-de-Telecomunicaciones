clc, clear, close all;

%============ Laboratorio II de Sistemas de Telecomunicaciones ============
% Entrega 1 
% Presentado por Grupo 8: 
% Elkin Burbano Molano y Elmer Jose Muñoz Zuñiga 
% Presentado a: 
% Ing. Manuela Silva
% 23 Nov 2021
% ======================= DEFINICIONES ====================================

M = 8;                      % Orden del esquema de modulación
L = log2(M);                % Número de bits por símbolo 
CantidadSimbolos = 100;     % Cantidad de bits del mensaje

% ======================= CONSTELACIÓN ====================================
constelacion = [ -1.5 + 1.5j ; 0 + 2j ; 1.5 + 1.5j ; -0.75 + 0j; 0.75 + 0j ; -1.5 - 1.5j ; 0 - 2j ; 1.5 - 1.5j ];

%%
%========================= TRANSMISOR (TX) ================================
% ======================== MENSAJE BINARIO ================================

b = randsrc(1, CantidadSimbolos*L, [0 1; 0.5 0.5]); % Se genera un mensaje aleatorio

figure(2),
subplot(2,1,1)
% Muestra la primera mitad de los bits
stem(b(1:CantidadSimbolos/2),'filled');
title('Bits (in)');
xlabel('Bit Index');
ylabel('Binary Value');

% ============================ MAPEO ======================================

% Agrupamos los bits del mensaje en una matriz de 'L' columnas (se divide en grupos de 3)
AgrupacionBits = reshape(b, L, [])'; 

%Convertimos cada fila de la matriz en un número decimal
BitsADecimal = bi2de(AgrupacionBits, 'left-msb');

%Se crean los simbolos que representan el mensaje
SYMBOL = Mapeo(BitsADecimal, constelacion);

%---------
% CONSTELACIÓN A TRANSMITIR (SCATTERPLOT)
scatterplot(SYMBOL);
grid on;
%---------

% CONSTELACIÓN A TRANSMITIR
figure('name','Constelación en Tx'),
plot(SYMBOL,'o','MarkerFaceColor','m','MarkerSize',6)
title('Constelación Asignada')
axis([-3 3 -3 3]);
xlabel('Eje Real')
ylabel('Eje Imaginario')
grid on;

% Simbolos transmitidos 
figure(2) 
subplot(2,1,2)
stem(SYMBOL);
title('Simbolos (Tx)');
xlabel('Symbol Index');
ylabel('Integer Value');

% ====================== FILTRO CONFORMADOR ===============================

Roff = 0.5;     % Roll-off
Span = 8;       % Transiente: Es un vector de longitud dos que indica el número de periodos de símbolo antes y después de la respuesta máxima 
Fs = 16;        % Factor de Sobremuestreo NOTA: FS*Span debe dar un número par 
T = 1;          % Duración de cada bit en segundos

% Creamos el filtro conformador, longitud es 2*Fs*Span+1
SRRC = rcosfir(Roff, Span, Fs, T, 'sqrt');    

% Adaptamos los simbolos a la frecuencia del filtro y añadimos ceros para compensar la transiente del filtro
simbolosAdaptados = AdaptarSimbolos(SYMBOL, Fs, Span);

% Se filtra la señal
formaOnda = filter(SRRC, 1, simbolosAdaptados);


figure,
subplot(2,1,1), plot(real(formaOnda),'m','LineWidth',1), title('Forma De Onda Real'), grid on;
subplot(2,1,2), plot(imag(formaOnda),'g','LineWidth',1), title('Forma De Onda Imaginaria'), grid on;

% Espectro de la señal filtrada
figure(),
FFTR =fftshift(fft(real(formaOnda)));
FFFI =fftshift(fft(imag(formaOnda)));

fr2=linspace(-Fs/2,Fs/2,length(FFTR));
fi2=linspace(-Fs/2,Fs/2,length(FFFI));

subplot(2,1,1), plot(fr2,abs(FFTR/length(FFTR)),'m','LineWidth',1), title('Espectro de Onda Filtrada (REAL)'), grid on;
subplot(2,1,2), plot(fi2,abs(FFFI/length(FFFI)),'g','LineWidth',1), title('Espectro de Onda Filtrada (IMAGINARIA)'), grid on;


% ============== MODULACIÓN Y TRASLACIÓN EN FRECUENCIA ====================

R=1; % Tasa de simbolo
Bw = (R*(1+Roff)/2); 
Fportadora= 3*Bw;  %Fportadora = 2; 
n = numel(SYMBOL);

% Se obtiene la señal a transmitir
Modulada = Modulacion(formaOnda, Fportadora, Fs, Span, n);

figure,
plot(Modulada,'m','LineWidth',1) 
title('Modulada')
grid on;

%%
%========================== CANAL AWGN ====================================
ES = 0; % Inicialización de Energía de simbolo 
EbNo = 10; % Valor en veces
distanciasSim = DistanciaSimbolos(constelacion, M); 

for i=1:M
    ES = ES +((distanciasSim(i))^2)/M; %Sumatoria de las distancias de cada simbolo al centro de la constelacion al cuadrado, multiplicado por la probabilidad de símbolo 
end

% Varianza ruido
SIGMA = sqrt(ES/(2*log2(M)*EbNo));

% Introducimos ruido AWGN a la señal
AWGN= SIGMA*randn(1, length(Modulada));
noiseSignal = Modulada + AWGN;

figure,
plot(noiseSignal,'g','LineWidth',1), title('Señal con Ruido AWGN en Rx'), grid on;

% Espectro de la señal con ruido AWGN
noiseSignal_F = Transformada(noiseSignal,Fs);
fn=linspace(-Fs/2,Fs/2,length(noiseSignal_F));

figure,
plot(fn, abs(noiseSignal_F),'g'), title('Espectro de la señal en Rx'), grid on;

%%
%=============================== RECEPTOR =================================
% ============================ DEMODULACION ===============================

demodulada = Demodulacion(noiseSignal, Fportadora, Fs,Span, n);

%================================ FILTRAJE ================================
recuperados = filter(SRRC, 1, demodulada);
eyediagram(recuperados,2*Fs)

figure(),
subplot(211), plot(real(recuperados),'-m'), title('Forma De Onda Filtrada REAL en Rx'), grid on;
subplot(212), plot(imag(recuperados),'-g'), title('Forma De Onda Filtrada IMAG en Rx'), grid on;


%=============================== MUESTREO =================================
%Se cortan los ceros que se agregaron a la forma de onda
recuperados = recuperados(2*Span*Fs + 1: end); 
recuperados = downsample(recuperados,Fs);

%Verificamos qe los simbolos que se reciben son iguales a los enviados
iguales = isequal(SYMBOL, round(recuperados));   

figure,
subplot(2,1,1), stem(real(recuperados),'o','LineWidth',1,'MarkerEdgeColor','black','MarkerFaceColor','blue','MarkerSize',3), title('Simbolos Real'), axis([0 numel(recuperados) -4 4]), grid on;
subplot(2,1,2), stem(imag(recuperados),'o','LineWidth',1,'MarkerEdgeColor','black','MarkerFaceColor','blue','MarkerSize',3), title('Simbolos Imag'), axis([0 numel(recuperados) -4 4]), grid on;

figure,
plot(recuperados,'o','MarkerEdgeColor','black','MarkerFaceColor','blue','MarkerSize',6), title('Simbolos recibidos'), axis([-3 3 -3 3]);
grid on;

%============================= PROCESO DE DECISIÓN ========================

simbolosEstimados = DistanciaMinima(recuperados, constelacion);

figure,
plot(simbolosEstimados,'o','MarkerEdgeColor','black','MarkerFaceColor','blue','MarkerSize',6), title('Simbolos estimados por criterio decisión'), axis([-3 3 -3 3]);
grid on;

%=============================== DE MAPEO ================================= 

b_D = Demapeo(simbolosEstimados, constelacion);

figure();
subplot(2,1,1)
stairs(b,'m'),legend('Mensaje enviado'), axis([0 CantidadSimbolos*3 -1 2]);
grid on;
subplot(2,1,2)
stairs(b_D), legend('Mensaje Recibido'), axis([0 CantidadSimbolos*3 -1 2]);
grid on;

figure();
stairs(b), axis([0 CantidadSimbolos*3 -1 2]);
hold on;
stairs(b_D)
lgd = legend({'Mensaje Enviado','Mensaje Recibido'});

%==========================================================================

