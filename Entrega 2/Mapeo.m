function [simbolos] = Mapeo(bitDecimal, constelacion)

    simbolos(bitDecimal==0) = constelacion(1);
    simbolos(bitDecimal==1) = constelacion(2);
    simbolos(bitDecimal==2) = constelacion(3);
    simbolos(bitDecimal==3) = constelacion(4);
    
    simbolos(bitDecimal==4) = constelacion(5);
    simbolos(bitDecimal==5) = constelacion(6);
    simbolos(bitDecimal==6) = constelacion(7);
    simbolos(bitDecimal==7) = constelacion(8);
    
end

