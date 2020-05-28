%vid1 = VideoReader('video7-1.wmv')
vid1 = VideoReader('ascenso.wmv')
se = strel('line',14,70);
radio=30;
% contador para almacenar el desplazamiento
cont=1;
cont2=1;
tiempo=[0];
i=[0];
j=[0];
areas1=[0];
while hasFrame(vid1)
video = readFrame(vid1);
%im1=imread('Captura.jpg');
I = rgb2gray(video);
BW = im2bw(I,0.97); % bien con 0.785 el vieo 8 con 0.97
% aplicación de operaciones de erosión dilatación
BW=imdilate(BW,se);
BW=imfill(BW,'holes');
%imshow(BW)
areas = bwlabel(BW);

%centroide de areas
stat = regionprops(areas,'centroid');
% dimension de areas
dim_area = regionprops(areas,'area')
area_mayor(1,cont)=dim_area(1,1);
% obtenemos la posición del centroide 
center=[stat(1).Centroid(1) stat(1).Centroid(2)];
% almacenamiento de puntos para graficar 
b = mod(cont,1);
%if b==0
    i(1,cont2)=center(1,1);
    j(1,cont2)=center(1,2);
    tiempo(1,cont2)=cont2;
    cont2=cont2+1;
%end
%bordes
bordes=edge(areas,'canny');
% mostrar el frame capturado del video 
imshow(video);
viscircles(center,radio,'EdgeColor','b');
hold on 
%imshow(areas)
hold off

cont=cont+1;
pause(0.00001)

areas1=areas1+areas;

end

% desplazamiento de arriba a abajo
for m=1:length(j)
    jj(1,m)=j(1,length(j)-m+1)
end

% grafica de trayectoria recorrida
subplot(3,2,1),plot(i,jj,'-'),title('Trayectoria Recorrida')
hold on
plot(i,jj,'o')
grid on
xlim([0 1280])
ylim([0 720])
desplazamiento_x=i;
desplazamiento_y=j;

% grafica de desplazamiento en "y" en base al tiempo 
%figure
subplot(3,2,4),plot(tiempo,jj,'-'),title('Desplazamiento en "y" vs tiempo')
hold on
plot(tiempo,jj,'.')
grid on
xlim([0 620])

% grafica de desplazamiento en "x" en base al tiempo 
%figure
subplot(3,2,3),plot(tiempo,desplazamiento_x,'-'),title('Desplazamiento en "x" vs tiempo')
hold on
plot(tiempo,i,'.')
ylim([100 900])
xlim([0 620])
grid on

% grafica del area segun los ciclos/tiempo
%figure
C = struct2cell(area_mayor);
subplot(3,2,2),plot(tiempo,cat(1,C{:}),'-'),title('Area detectada vs tiempo')
grid on
xlim([0 620])
H=I;

% desplazamiento de la meduza en la imagen escala de grises
%%  SOLAPAMIENTO EN LA IMAGEN RGB

for dim=1:3
for fil=1:720
    for col=1:1280
        if areas1(fil,col)>=1
            video(fil,col,dim)=248; 
        end
    end
end
end
% 

subplot(3,2,6),imshow(video),title('Trayectoria Recorrida')
hold on

% gráfica de trayectoria sobrepuesta
for i=1:length(desplazamiento_x)
viscircles([desplazamiento_x(1,i) desplazamiento_y(1,i)],5,'EdgeColor','r')
end

%% graficas de velocidad

% Calculo de la velocidad
for k=1:length(tiempo)/1-1
    veloc(1,k)=[jj(1,k*1+1)-jj(1,k*1)]/1;
    contador(1,k)=k;
end

subplot(3,2,5),plot(contador,veloc),title('Velocidad en y [pix/s] vs tiempo')
hold on
xlim([0 620])
plot(contador,veloc,'.')


