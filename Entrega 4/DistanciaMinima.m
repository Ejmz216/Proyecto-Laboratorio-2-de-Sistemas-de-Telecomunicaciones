
function [SimbolosEstimados, DistanciaMinima] = DistanciaMinima(simbolos, constelacion)

    %Hallamos la distancia desde cada simbolo recibido a cada posible símbolo de la constelación
    for i=1: numel(simbolos)
        for j=1: numel(constelacion)
           
            distancia(j) = sqrt((real(simbolos(i)) - real(constelacion(j)))^2 + (imag(simbolos(i)) - imag(constelacion(j)))^2);
            
        end
        
        %De todas las distancias calculadas, nos quedamos con la mas pequeña
        DistanciaMinima = min(distancia); 
        
        %Determinamos en que posición se dió esa distancia minima
        PosicionDM = find(distancia==DistanciaMinima);  
        
        %%Igualamos el símbolos que estamos analizando, con el símbolo mas cercano de la constelación
        simbolos(i) = constelacion(PosicionDM); 
        
        %Guardamos todos los símbolos estimados mediante la distancia minima en un nuevo vector
        SimbolosEstimados(i) = simbolos(i); 
    end

end
