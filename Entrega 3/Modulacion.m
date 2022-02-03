function [SumModulada] = Modulacion(formaOnda, Fs,R, Span, numFilas, numColumnas, NTon, FS2)

%_____________________________________FUNCION PARA UNA PORTADORA________________________________________________________________
%     function [Modulada] = Modulacion(formaOnda, Fportadora, sobreMuestreo, Span, n)
%     % Importante tener en cuenta que para esto se asume que la Tasa de símbolo es R=1
%     Ts = 1/sobreMuestreo; 
%     t = 0 : Ts : (n+2*Span)-Ts; % La resta de n-Ts es porque se comienza con el vector de tiempo en cero  
%     Modulada = sqrt(2) * (real(formaOnda).*cos(2*pi*Fportadora.*t) - imag(formaOnda).*sin(2*pi*Fportadora.*t));
%_______________________________________________________________________________________________________________________________
%_____________________________________ FUNCION MULTITONO _______________________________________________________________________

fs=Fs*R;                %Frecuancia de muestreo
Ts = 1/fs;
t = 0 : Ts : (numFilas+2*Span)*NTon-Ts; % La resta de n-Ts es porque se comienza con el vector de tiempo en cero
fc=linspace(0.2,0.2*NTon,NTon);
SumModulada=zeros(1,length(t));

for i=1:numColumnas

    %Fportadora(i)= ((i-1)*Bw)+(Bw/2);
    % Fportadora(i)=((2*(i-1))*R/(2*NTon))*(1+Roff);

    Modulada{i} = sqrt(2) * (real(formaOnda{i}).*cos(2*pi*fc(i).*t) - imag(formaOnda{i}).*sin(2*pi*fc(i).*t));

    SumModulada=SumModulada+Modulada{i};

end
 
end

