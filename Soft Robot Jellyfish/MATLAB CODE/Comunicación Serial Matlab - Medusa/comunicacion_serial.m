function [bit_parada]= comunicacion_serial(cadena)

delete(instrfind({'Port'},{'COM3'}));
serialPort=serial('COM3','BaudRate',9600);
fopen(serialPort); 
pause(1)
%cadena=3100001;
cadena=num2str(cadena);
fprintf(serialPort,'%s',cadena);
bit_parada = fscanf(serialPort,'%d');
% aviso de que se culminó el proceso en la tarjeta de control
if bit_parada==1
    disp(bit_parada)
    fclose(serialPort);
end


end