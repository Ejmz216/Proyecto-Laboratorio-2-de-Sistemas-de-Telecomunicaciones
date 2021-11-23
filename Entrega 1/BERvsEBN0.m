clc, clear, close all;

%============ Laboratorio II de Sistemas de Telecomunicaciones ============
% Entrega 1 - BER VS EB/N0
% Presentado por Grupo 8: 
% Elkin Burbano Molano y Elmer Jose Muñoz Zuñiga 
% Presentado a: 
% Ing. Manuela Silva
% 23 Nov 2021

% NOTA: EL SIGUIENTE CODIGO ES EL MISMO  EMPLEADO EN "LS2Entrega1", 
% PERO SE OPTA POR HACER ESTE MECANISMO POR LA ROBUSTEZ DE SU SIMULACION 
%==========================================================================

% Definiciones-------------------------------------------------------------
M = 8;                      % Orden del esquema de modulación
L = log2(M);                % Número de bits por símbolo 
CantidadSimbolos = 1000000;     % Cantidad de bits del mensaje

% Constelación-------------------------------------------------------------
constelacion = [ -1.5 + 1.5j ; 0 + 2j ; 1.5 + 1.5j ; -0.75 + 0j; 0.75 + 0j ; -1.5 - 1.5j ; 0 - 2j ; 1.5 - 1.5j ];

ES = 0;     
distanciasSim = DistanciaSimbolos(constelacion, M);
    
for i=1:M
    ES = ES +((distanciasSim(i))^2)/M; %Sumatoria de las distancias de cada simbolo al centro de la constelacion al cuadrado, multiplicado por la probabilidad de símbolo 
end

EbNoDB= 0 : 14;
BER = zeros(1, numel(EbNoDB));

for i = 1 : 1 : numel(EbNoDB)

    
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
    
    % =============================== FILTRO CONFORMADOR ======================
    
    Roff = 0.5;     % Roll-off
    Span = 8;       % Transiente: Es un vector de longitud dos que indica el número de periodos de símbolo antes y después de la respuesta máxima 
    Fs = 16;        % Factor de Sobremuestreo NOTA: FS*Span debe dar un número par 
    T = 1;          % Duración de cada bit en segundos
    
    % Creamos el filtro conformador, longitud es 2*L*U+1
    SRRC = rcosfir(Roff, Span, Fs, T, 'sqrt');    
    
    % Añadimos ceros para compensar la transiente del filtro
    simbolosAdaptados = AdaptarSimbolos(SYMBOL, Fs, Span);
    
    % Se filtra la señal
    formaOnda = filter(SRRC, 1, simbolosAdaptados);
    
    % ============== MODULACIÓN Y TRASLACIÓN EN FRECUENCIA ====================
    
    % Frecuencia de portadora Fportadora*R=2 ya que R=1
    R=1;
    Bw = (R*(1+Roff)/2); 
    Fportadora= 4*Bw;  %Fportadora = 2; 
    n = numel(SYMBOL);
    
    % Se obtiene la señal a transmitir
    modulada = Modulacion(formaOnda, Fportadora, Fs, Span, n);


    %========================== CANAL AWGN ====================================
    %Iteramos varias veces el código variando EbNodB

    EbNo = 10.^(EbNoDB(i)./10); 
    % Varianza ruido
    SIGMA = sqrt(ES/(2*log2(M)*EbNo));
    
    % Introducimos ruido AWGN a la señal
    AWGN= SIGMA*randn(1, length(modulada));
    noiseSignal = modulada + AWGN;
    
 
    %=============================== RECEPTOR =================================
    % ============================ DEMODULACION ===============================
    
    demodulada = Demodulacion(noiseSignal, Fportadora, Fs,Span, n);
    
    %================================ FILTRAJE ================================
    recuperados = filter(SRRC, 1, demodulada);
    
    %=============================== MUESTREO =================================
    %Se cortan los ceros que se agregaron a la forma de onda
    recuperados = recuperados(2*Span*Fs + 1: end); 
    recuperados = downsample(recuperados,Fs);
    
    %Verificamos qe los simbolos que se reciben son iguales a los enviados
    iguales = isequal(SYMBOL, round(recuperados));   
    
    %============================= PROCESO DE DECISIÓN ========================
    
    simbolosEstimados = DistanciaMinima(recuperados, constelacion);
    
    %=============================== DE MAPEO ================================= 
    
    b_D = Demapeo(simbolosEstimados, constelacion);

BER(i) = sum(abs(b-b_D))/length(b);

end
%==========================================================================
%============================== BER VS EBNo ===============================
BANDERA=('done1')

EbNo = 10.^(EbNoDB./10);
BER_TEORICA = (4/log2(M))*qfunc(sqrt(3*EbNo*log2(M)/(M-1)));

semilogy(EbNoDB, BER,'bx'),grid on, hold on;
semilogy(EbNoDB, BER_TEORICA),grid on
legend('BER SIMULADA','BER TEORICA')
xlabel('EbNo[dB]');
ylabel('BER');
grid on;
