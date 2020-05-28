% Estructura de los tipos de datos 
% comunicacin_serial(a, pin, pwm, tipo_control, num_pulsos)
% tipo control --1 Automatico --2 Manual

function [bit_parada]= automatico(b, numero_pin, pwm, tipo_control, num_pulsos)
global a
%cadena=3100001;
a.analogWrite(numero_pin,pwm);
pause(0.2)
a.analogWrite(numero_pin,0);
bit_parada = 1;
% aviso de que se culminó el proceso en la tarjeta de control

end