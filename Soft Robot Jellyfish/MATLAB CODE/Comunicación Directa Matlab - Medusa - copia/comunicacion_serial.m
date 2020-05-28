% Estructura de los tipos de datos 
% comunicacin_serial(a, pin, pwm, tipo_control, num_pulsos)
% tipo control --1 Automatico --2 Manual

function [bit_parada]= comunicacion_serial(b, numero_pin, pwm, tipo_control, num_pulsos)
global a
%cadena=3100001;
for i=1:num_pulsos
a.analogWrite(numero_pin,pwm);
pause(0.25)
a.analogWrite(numero_pin,0);

end
bit_parada = 1;
% aviso de que se culminó el proceso en la tarjeta de control

end