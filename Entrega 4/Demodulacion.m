function [Demodulada] = Demodulacion(noiseSignal,Fs,R, Span, numFilas, numColumnas, NTon,FS2)

%_____________________________________FUNCION PARA UNA PORTADORA________________________________________________________________
%     Ts = 1/Fs;
%     t = 0 : Ts : (n+2*Span)-Ts; 
% 
%     Demodulada_real = sqrt(2) * Modulada.*cos(2*pi*Fportadora.*t);
%     
%     Demodulada_imag = sqrt(2) * Modulada.*sin(2*pi*Fportadora.*t);  
%     
%     Demodulada = (Demodulada_real + 1j*Demodulada_imag)';
%_______________________________________________________________________________________________________________________________
%_____________________________________ FUNCION MULTITONO _______________________________________________________________________

fs=Fs*R;                %Frecuancia de muestreo
Ts = 1/fs;
t = 0 : Ts : (numFilas+2*Span)*NTon-Ts; % La resta de n-Ts es porque se comienza con el vector de tiempo en cero
fc=linspace(0.2,0.2*NTon,NTon);
for i=1:numColumnas

    Demodulada_real{i} = sqrt(2) * noiseSignal.*cos(2*pi*fc(i).*t);
    Demodulada_imag{i} = sqrt(2) * noiseSignal.*sin(2*pi*fc(i)*t);
    
    Demodulada{i} = (Demodulada_real{i} - 1j*Demodulada_imag{i})';

end  
     
end
