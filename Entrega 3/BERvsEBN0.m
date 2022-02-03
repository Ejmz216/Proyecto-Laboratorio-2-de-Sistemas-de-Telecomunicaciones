clc,clear, close all;

%============ Laboratorio II de Sistemas de Telecomunicaciones ============
% Entrega 3
%
% PARA ESTA ENTREGA SE REQUIERE AÑADIR UN SISTEMA MULTITONO Y COREGIR
% ERRORES PREVIOS DE SIMULACIÓN
%
% Presentado por Grupo 8:
% Elkin Burbano Molano y Elmer Jose Muñoz Zuñiga
% Presentado a:
% Ing. Manuela Silva
% 3 FEB 2022

% NOTA: EL SIGUIENTE CODIGO ES EL MISMO  EMPLEADO EN "LS2Entrega3",
% PERO SE OPTA POR HACER ESTE MECANISMO POR LA ROBUSTEZ DE SU SIMULACION
%==========================================================================

% Definiciones-------------------------------------------------------------
M = 8;                      % Orden del esquema de modulación
L = log2(M);                % Número de bits por símbolo
CantidadSimbolos = 8000;     % Cantidad de bits del mensaje
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

% Constelación-------------------------------------------------------------
constelacion = [ -1.5 + 1.5j ; 0 + 2j ; 1.5 + 1.5j ; -0.75 + 0j; 0.75 + 0j ; -1.5 - 1.5j ; 0 - 2j ; 1.5 - 1.5j ];

ES = 0;
distanciasSim = DistanciaSimbolos(constelacion, M);

for i=1:M
    ES = ES +((distanciasSim(i))^2)/M; %Sumatoria de las distancias de cada simbolo al centro de la constelacion al cuadrado, multiplicado por la probabilidad de símbolo
end

EbNoDB= 0 : 13;
BER = zeros(1, numel(EbNoDB));

for i = 1 : 1 : numel(EbNoDB)

    clc;
    %================================ TRANSMISOR (TX) =========================
    % =============================== MENSAJE BINARIO =========================

    b = randsrc(1, CantidadSimbolos*L, [0 1; 0.5 0.5]); % Se genera un mensaje aleatorio

    % =============================== MAPEO ===================================

    % Agrupamos los bits del mensaje en una matriz de 'L' columnas (se divide en grupos de 3)
    bitsAgrupados = reshape(b, L, [])';

    %Convertimos cada fila de la matriz en un número decimal
    bitsADecimal = bi2de(bitsAgrupados, 'left-msb');

    %Se crean los simbolos que representan el mensaje
    SYMBOL = Mapeo(bitsADecimal, constelacion);

    %================ CONVERSOR SERIE - PARALELO ==============================
    Paralelo = reshape(SYMBOL, NTon, [])';

    [numFilas,numColumnas] = size(Paralelo);

    % =============================== FILTRO CONFORMADOR ======================


    Roff = 0.5;     % Roll-off
    Span = 4;     % Cantidad de ceros que toman los filtros//ransiente: Es un vector de longitud dos que indica el número de periodos de símbolo antes y después de la respuesta máxima
    T = NTon;        % Duración de cada bit en segundos

    % Creamos el filtro conformador, longitud es 2*Fs*Span+1
    SRRC = rcosfir(Roff, Span, Fs, T, 'sqrt');
    for o=1:numColumnas

        % Adaptamos los simbolos a la frecuencia del filtro y añadimos ceros para compensar la transiente del filtro
        simbolosAdaptados{o} = AdaptarSimbolos(Paralelo(:,o), Fs, Span);

        % Se filtra la seña
        formaOnda{o} = filter(SRRC, 1, simbolosAdaptados{o});
        FS2=Fs/NTon;

    end
    % ============== MODULACIÓN Y TRASLACIÓN EN FRECUENCIA ====================

    R=1/NTon;               % Tasa de Baudios ->  1/T
    Bw = (R*(1+Roff));      % Ancho de banda de un pulso SRRC (ancho de banda de la señal modulada en banda base)

    SumModulada= Modulacion(formaOnda, Fs,R, Span, numFilas, numColumnas, NTon,FS2);


    %     %========================== CANAL =====================================
    %     %Iteramos varias veces el código variando EbNodB
    %
    %     EbNo = 10.^(EbNoDB(i)./10);
    %
    %
    %     % MULTITRAYECTO---------------------
    %     %  NOTA: El valor asignado de TAU ES 1s por requerimiento, pero en este
    %     %  caso utilizaremos un valor más alto, para poder observar mejor el
    %     %  comportamiento tanto de los graficos espectrales, como de BER vs EbNo
    %
    %     tau=2; % En segundos
    %     a=0.2; % Amplitud
    %     SenalMulti=[zeros(1,tau*FS2) SumModulada(1:end-tau*FS2)];
    %     Multi=SumModulada+(a*SenalMulti);
    %     %---------------------
    %
    %     % Varianza ruido
    %     SIGMA = sqrt(ES/(2*log2(M)*EbNo));
    %
    %     % Introducimos ruido AWGN a la señal
    %     AWGN= SIGMA*randn(1, length(SumModulada));
    %     noiseSignal = Multi+ AWGN;
    %

    %=============================== RECEPTOR =================================
    % ============================ DEMODULACION ===============================

    Demodulada = Demodulacion(SumModulada,Fs,R, Span, numFilas, numColumnas, NTon,FS2);

    %================================ FILTRAJE ================================
    for o=1:numColumnas

        % Se filtra la seña
        recuperados{o} = filter(SRRC, 1, Demodulada{o}); % Recuperar los simolos de la forma de onda

        %Se cortan los ceros que se agregaron a la forma de onda
        recuperados{o} = recuperados{o}(2*Span*Fs + 1: end);
        recuperados{o} = downsample(recuperados{o},Fs);

    end
    %================ CONVERSOR PARALELO - SERIE ==============================
    Prev = cell2mat(recuperados);
    Serie = reshape(Prev,[],1)';

    [SFilas,SColumnas] = size(Serie);

    %============================= PROCESO DE DECISIÓN ========================

    [simbolosEstimados, DMinima(i)]  = DistanciaMinima(Serie, constelacion);

    %=============================== DE MAPEO =================================

    b_D = Demapeo(simbolosEstimados, constelacion);
    BER(i) = sum(abs(b-b_D))/length(b);

end
%==========================================================================
%============================== BER VS EBNo ===============================
EbNo = 10.^(EbNoDB./10);

Eb=ES/log2(M);
No= 1./(EbNo./Eb);
Ps=(6.75)*qfunc(sqrt(((1.5)^2)./(2*No)));
BER_TEORICA=Ps/log2(M);

semilogy(EbNoDB, BER,'bx'),grid on, hold on;
semilogy(EbNoDB, BER_TEORICA),grid on
legend('BER SIMULADA','BER TEORICA')
xlabel('EbNo[dB]');
ylabel('BER');
grid on;
