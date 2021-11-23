function [Modulada] = Modulacion(formaOnda, Fportadora, sobreMuestreo, Span, n)

    % Importante tener en cuenta que para esto se asume que la Tasa de símbolo es R=1
    Ts = 1/sobreMuestreo; 
    t = 0 : Ts : (n+2*Span)-Ts; 

    Modulada = sqrt(2) * (real(formaOnda).*cos(2*pi*Fportadora.*t) - imag(formaOnda).*sin(2*pi*Fportadora.*t));

end

