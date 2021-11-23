function [SenalTrasformada] = Transformada(senal,Fs)
    L=2^nextpow2(length(senal)*10);
    SenalTrasformada = fft(senal,L)/Fs;
    SenalTrasformada = fftshift(SenalTrasformada);
end