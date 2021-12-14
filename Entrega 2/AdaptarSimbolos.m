function [simbolosAdaptados] = AdaptarSimbolos(simbolos, factorSobremuestreo, span)

    %Representar los simbolos del mensaje mediante impulsos (deltas de Dirac) antes de entrar al filtro
    %Acoplamos los tiempos de la secuencia mensaje que deseamos enviar con los timepos del filtro conformador   
    simbolosAdaptados = upsample(simbolos, factorSobremuestreo);            
    
    %Agregamos los ceros para compensar la transiente del filtro conformador
    simbolosAdaptados = [simbolosAdaptados, zeros(1, 2*span*factorSobremuestreo)];
end
