clc, clear, close all;
%============ Laboratorio II de Sistemas de Telecomunicaciones ============
% Entrega 4
%
% PARA ESTA ENTREGA SE REQUIERE AÑADIR ECUALIACIÓN POR MEDIO DEL FACTOR DE
% ROTACIÓN QUE INTRODUCE h(t).
%
% Presentado por Grupo 8:
% Elkin Burbano Molano y Elmer Jose Muñoz Zuñiga
% Presentado a:
% Ing. Manuela Silva
% 1 Mar 2022
% ======================= DEFINICIONES ====================================

M = 8;                      % Orden del esquema de modulación
L = log2(M);                % Número de bits por símbolo
CantidadSimbolos = 8000;    % Cantidad de bits del mensaje

while true
    opcion = input(['Seleccione la cantidad de tonos para el sistema:\n ' ...
        '[ 1 ] 8 Tonos\n [ 2 ] 16 Tonos\n\n Ingrese una opcion [1] o [2]:  ']);
    if ((~isempty(opcion))&&((opcion==1)||(opcion==2)))
        break
    end
    clc
    warndlg('Ingresar una opcion valida')
end

% Se definen los parametros para cada caso
switch opcion
    case 1
        NTon=8;    % # Número de tonos
        Fs = 32;   % Factor de Sobremuestreo NOTA: FS*Span debe dar un número par

    case 2
        NTon=16;   % # Número de tonos
        Fs = 128;  % Factor de Sobremuestreo NOTA: FS*Span debe dar un número par

end

% ======================= CONSTELACIÓN ====================================
constelacion = [ -1.5 + 1.5j ; 0 + 2j ; 1.5 + 1.5j ; -0.75 + 0j; 0.75 + 0j ; -1.5 - 1.5j ; 0 - 2j ; 1.5 - 1.5j ];
%========================= TRANSMISOR (TX1) ================================
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

%================ CONVERSOR SERIE - PARALELO ==============================
Paralelo = reshape(SYMBOL, NTon,[])';

[numFilas,numColumnas] = size(Paralelo);

%==========================================================================

% CONSTELACIÓN A TRANSMITIR
figure('name','Constelación en Tx'), plot(SYMBOL,'o','MarkerFaceColor','m','MarkerSize',6), title('Constelación Asignada'), axis([-3 3 -3 3]); xlabel('Eje Real'), ylabel('Eje Imaginario')
grid on;

% ====================== FILTRO CONFORMADOR ===============================

Roff = 0.5;     % Roll-off
Span = 4;       % Cantidad de ceros que toman los filtros//ransiente: Es un vector de longitud dos que indica el número de periodos de símbolo antes y después de la respuesta máxima
T = NTon;       % Duración de cada bit en segundos

% Creamos el filtro conformador, longitud es 2*Fs*Span+1
SRRC = rcosfir(Roff, Span, Fs, T, 'sqrt');
figure()
for i=1:numColumnas

    % Adaptamos los simbolos a la frecuencia del filtro y añadimos ceros para compensar la transiente del filtro
    simbolosAdaptados{i} = AdaptarSimbolos(Paralelo(:,i), Fs, Span);

    % Se filtra la seña
    formaOnda{i} = filter(SRRC, 1, simbolosAdaptados{i});
    FS2=Fs/NTon;
    %Graficamos de momento para conocer si se están haciendo bien las cosas
    FFF =Transformada(formaOnda{1, i},FS2);
    f2=linspace(-FS2/2,FS2/2,length(FFF));
    plot(f2,abs(FFF/length(FFF)),'LineWidth',1), title('FILTRAJE'), grid on, hold on;
    legend();
end
%%
% ============== MODULACIÓN Y TRASLACIÓN EN FRECUENCIA ====================

R=1/NTon;               % Tasa de Baudios ->  1/T
Bw = (R*(1+Roff));      % Ancho de banda de un pulso SRRC (ancho de banda de la señal modulada en banda base)

fs=Fs*R;                %Frecuancia de muestreo
Ts = 1/fs;
t = 0 : Ts : (numFilas+2*Span)*NTon-Ts; % La resta de n-Ts es porque se comienza con el vector de tiempo en cero
fc=linspace(0.2,0.2*NTon,NTon);
SumModulada=zeros(1,length(t));

figure()
for i=1:numColumnas

    Modulada{i} = sqrt(2) * (real(formaOnda{i}).*cos(2*pi*fc(i).*t) - imag(formaOnda{i}).*sin(2*pi*fc(i).*t));
    SumModulada=SumModulada+Modulada{i};
   

    % Graficamos para observar cada portadora
    FFH =Transformada(Modulada{i},FS2);
    f3=linspace(-FS2/2,FS2/2,length(FFH));
    
    plot(f3,abs(FFH/length(FFH)),'LineWidth',1), title('MODULADA'), grid on, hold on;
    legend();
end

figure('Name','Comparación antes y despues de la distorsión'),
subplot(311)
FFQ =Transformada(SumModulada,FS2);
f5=linspace(-FS2/2,FS2/2,length(FFQ));
plot(f5,abs(FFQ/length(FFQ)),'m','LineWidth',1), title('Espectro de la señal Modulada'), grid on;

% ========================== CANAL AWGN/ MULTITRAYECTO =====================
% ==========================================================================

ES = 0; % Inicialización de Energía de simbolo

% NOTA: SE PROPONE UN VALOR EXTREMADAMENTE ALTO DE EbNo, SOLO CON EL FIN DE 
% PODER OBSERVAR LA MEJORA DEL ECUALIZADOR EN RECEPCIÓN, SIENDO EL VALOR
% ORGINAL PROPUESTO DE EbNo = 5 ó EbNo = 10.

EbNo = 80; %Valor en veces

distanciasSim = DistanciaSimbolos(constelacion, M);

for i=1:M
    ES = ES +((distanciasSim(i))^2)/M;
end

%  MULTITRAYECTO------------------------------------------------------------
%  NOTA: El valor asignado de TAU ES 1s por requerimiento, pero en este
%  caso utilizaremos un valor más alto, para poder observar mejor el
%  comportamiento tanto de los graficos espectrales, como de BER vs EbNo

tau=1; % En segundos
a=0.2; % Amplitud
SenalMulti=[zeros(1,tau*FS2) SumModulada(1:end-tau*FS2)];
Multi=SumModulada+(a*SenalMulti);

% ------------------------- GRAFICOS ---------------------------------------
% Gráfico de h(f) teorico:
subplot(312)
FFT =Transformada(Multi,FS2);
f3=linspace(-FS2/2,FS2/2,length(FFT));
f=linspace(-FS2/2,FS2/2,length(FFT));

h=sqrt( ((1+0.2.*cos(2*pi*tau*f)).^2) + (0.2.*sin(2*pi*tau*f)).^2);
plot(f,abs(h),'b'),title('Espectro del Modelo de canal distorsivo'),grid on;


% Superposición de Espectro Multitrayecto y Canal Distorsivo.
% NOTA: Se realiza de esta manera, con el fin de observar mejor el efecto
% del canal distorsivo sobre la onda modulada.

subplot(313)
h=140.*sqrt( ((1+0.2.*cos(2*pi*tau*f)).^2) + (0.2.*sin(2*pi*tau*f)).^2);
plot(f3,abs(FFT/length(FFT)),'m',f,abs(h/length(h)),'b'),title('Espectro de la señal con Multitrayecto y del Modelo de canal distorsivo'),grid on;

% ------------------------- RUIDO AWGN--------------------------------------

% Varianza ruido
SIGMA = sqrt(ES/(2*log2(M)*EbNo)); %Es/SNR

% Introducimos ruido AWGN a la señal con multitrayecto
AWGN= SIGMA*randn(1, length(SumModulada));
noiseSignal = Multi + AWGN;

%-------------------
figure('Name','Espectro de la señal en recepción (AWGN + Multitrayecto)'),
% Espectro de la señal con ruido AWGN que llega al receptor
noiseSignal_F = Transformada(noiseSignal,FS2);
fn=linspace(-FS2/2,FS2/2,length(noiseSignal_F));
plot(fn, abs(noiseSignal_F/length(noiseSignal_F)),'g'), title('Espectro de la señal en Rx'), grid on;

%=============================== RECEPTOR =================================
% ============================ DEMODULACION ===============================
for i=1:numColumnas
    
    Demodulada_real{i} = sqrt(2) * noiseSignal.*cos(2*pi*fc(i).*t);
    Demodulada_imag{i} = sqrt(2) * noiseSignal.*sin(2*pi*fc(i)*t);

    Demodulada{i} = (Demodulada_real{i} - 1j*Demodulada_imag{i})';

end

%============================ FILTRAJE  ===================================

for i=1:numColumnas

    % Se filtra la seña
    recuperados{i} = filter(SRRC, 1, Demodulada{i}); % Recuperar los simolos de la forma de onda

    %Se cortan los ceros que se agregaron a la forma de onda
    recuperados{i} = recuperados{i}(2*Span*Fs + 1: end);
    recuperados{i} = downsample(recuperados{i},Fs);

    %eyediagram(recuperados{i},2*FS2)

end
%========================= ECUALIZADOR ====================================

rec=Ecualizador(fc, a, tau, numColumnas, recuperados);

%=================== CONVERSOR PARALELO - SERIE ===========================

Prev = cell2mat(rec);
Serie = reshape(Prev',[],1)';

%=============================== MUESTREO =================================
figure('Name','Simbolos Recibidos'),
plot(Serie,'o','MarkerEdgeColor','black','MarkerFaceColor','blue','MarkerSize',6), title('Simbolos recibidos'), axis([-3 3 -3 3]);
grid on;

%============================= PROCESO DE DECISIÓN ========================

simbolosEstimados = DistanciaMinima(Serie, constelacion);
% figure('Name','Simbolos Estimados por criterio decisión'),
% plot(simbolosEstimados,'o','MarkerEdgeColor','black','MarkerFaceColor','blue','MarkerSize',6), title('Simbolos estimados por criterio decisión'), axis([-3 3 -3 3]);
% grid on;
%=============================== DE MAPEO =================================
 b_D = Demapeo(simbolosEstimados, constelacion);

 BER= sum(abs(b-b_D))/length(b);
%==========================================================================
