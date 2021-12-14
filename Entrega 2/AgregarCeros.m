function [VectorN] = AgregarCeros(NSimbolos,VectorA)
VectorN = zeros(1,NSimbolos+2);
    for i=1:1:NSimbolos
        VectorN(i+1)=VectorA(i);
    end
end