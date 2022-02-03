function [bitsOUT] = Demapeo(simbolos, constelacion)

    decimales(simbolos == constelacion(1)) = 0;
    decimales(simbolos == constelacion(2)) = 1;
    decimales(simbolos == constelacion(3)) = 2;
    decimales(simbolos == constelacion(4)) = 3;
    
    decimales(simbolos == constelacion(5)) = 4;
    decimales(simbolos == constelacion(6)) = 5;
    decimales(simbolos == constelacion(7)) = 6;
    decimales(simbolos == constelacion(8)) = 7;
    
    
    bitsAgrupadosDEMAP = de2bi(decimales, 'left-msb');
    bitsOUT = reshape(bitsAgrupadosDEMAP', 1, []);
end

