function [Demodulada] = Demodulacion(Modulada, Fportadora, Fs, Span, n)

    Ts = 1/Fs;
    t = 0 : Ts : (n+2*Span)-Ts; 

    Demodulada_real = sqrt(2) * Modulada.*cos(2*pi*Fportadora.*t);
    
    Demodulada_imag = sqrt(2) * Modulada.*sin(2*pi*Fportadora.*t);  
    
    Demodulada = (Demodulada_real + 1j*Demodulada_imag)';
    
end
