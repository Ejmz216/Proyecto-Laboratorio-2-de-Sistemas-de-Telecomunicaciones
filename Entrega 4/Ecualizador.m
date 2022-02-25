function [rec] = complejos(fc, a, tau, numColumnas, recuperados)
  
 for u=1:numColumnas
        evaluacion(u)=(1+(a*exp(1i*2*pi*tau*fc(u))));
        rec{u}=(1/evaluacion(u)).*recuperados{u};
    end
    
end