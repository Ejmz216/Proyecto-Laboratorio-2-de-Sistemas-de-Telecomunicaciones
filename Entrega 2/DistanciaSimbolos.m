function [DistanciaSim] = DistanciaSimbolos(constelacion, M)

    for i=1:M
        DistanciaSim(i) = sqrt( ((real(constelacion(i)))^2 + (imag(constelacion(i)))^2) );
    end
end

