function varargout = interfaz(varargin)
% INTERFAZ MATLAB code for interfaz.fig
%      INTERFAZ, by itself, creates a new INTERFAZ or raises the existing
%      singleton*.

% Edit the above text to modify the response to help interfaz

% Last Modified by GUIDE v2.5 03-Oct-2019 23:12:39

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @interfaz_OpeningFcn, ...
                   'gui_OutputFcn',  @interfaz_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before interfaz is made visible.
function interfaz_OpeningFcn(hObject, eventdata, handles, varargin)

global a paro control_fuzzy cont_temp
paro=0;
cont_temp=0;
control_fuzzy=readfis('control_borroso');
%a=arduino('COM3');
 axes(handles.axes2);
    im2=imread('upm.png');
    image(im2)
    
    axis off
    
    axes(handles.axes3);
    im2=imread('car.png');
    image(im2)
    
    axis off
    
    axes(handles.axes1);
    im2=imread('medusa_final.png');
    image(im2)
    
    axis off
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes interfaz wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = interfaz_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
global a pin_numero pwm tipo_control num_pulsos 

    %axes(handles.axes1);
    %im2=imread('upm.png');
    %image(im2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% vid=videoinput('winvideo',1,'MJPG_640x480');
% set(vid,'FramesPerTrigger',inf)
% set(vid,'ReturnedColorspace','rgb')
% se = strel('line',14,70);
% start(vid);
% fondo1 = getsnapshot(vid);
% axes(handles.axes1);
% imshow(fondo1)
% stop(vid);
% axis off
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if tipo_control==2

        [bit_parada]=comunicacion_serial(a, 5, pwm, 1, 1);

end
if tipo_control==1
    for i=1:num_pulsos 
        [bit_parada]=comunicacion_serial(a, 5, pwm, 1, 1);
    end
end


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% lectura del valor pwm del slider
global a pwm bit_pwm umbral
pwm = get(handles.slider1,'Value')
pwm=round(pwm);
% escritura del valor leido en pantalla
set(handles.text4,'String',pwm);


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
global a pin_numero pwm tipo_control num_pulsos 

% Estructura de los tipos de datos 
% comunicacin_serial(a, pin, pwm, tipo_control, num_pulsos)
% tipo control --1 Automatico --2 Manual

[bit_parada]=comunicacion_serial(a, 5, 200, 1, 1);




% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
global a pin_numero pwm tipo_control num_pulsos 

% Estructura de los tipos de datos 
% comunicacin_serial(a, pin, pwm, tipo_control, num_pulsos)
% tipo control --1 Automatico --2 Manual

[bit_parada]=comunicacion_serial(a, 6, 200, 1, 1);

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
global a pin_numero pwm tipo_control num_pulsos 

% Estructura de los tipos de datos 
% comunicacin_serial(a, pin, pwm, tipo_control, num_pulsos)
% tipo control --1 Automatico --2 Manual

[bit_parada]=comunicacion_serial(a, 7, 200, 1, 1);

% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
global a pin_numero pwm tipo_control num_pulsos 

% Estructura de los tipos de datos 
% comunicacin_serial(a, pin, pwm, tipo_control, num_pulsos)
% tipo control --1 Automatico --2 Manual

[bit_parada]=comunicacion_serial(a, 8, 200, 1, 1);

% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
global a pin_numero pwm tipo_control num_pulsos 

% Estructura de los tipos de datos 
% comunicacin_serial(a, pin, pwm, tipo_control, num_pulsos)
% tipo control --1 Automatico --2 Manual

[bit_parada]=comunicacion_serial(a, 9, 200, 1, 1);

function pulsos_Callback(hObject, eventdata, handles)
% hObject    handle to pulsos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pulsos as text
%        str2double(get(hObject,'String')) returns contents of pulsos as a double


% --- Executes during object creation, after setting all properties.
function pulsos_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pulsos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Lectura del numero de pulsos
function pushbutton7_Callback(hObject, eventdata, handles)
global a pwm num_pulsos pin_numero

num_pulsos=get(handles.pulsos,'String');
num_pulsos = str2double(num_pulsos);
%set(handles.text5,'String',num_pulsos);

% BOTTON ATRAS
function pushbutton8_Callback(hObject, eventdata, handles)
global a pin_numero pwm tipo_control num_pulsos 
if tipo_control==2

        [bit_parada]=comunicacion_serial(a, 6, pwm, 1, 1);

end
if tipo_control==1
    for i=1:num_pulsos
        [bit_parada]=comunicacion_serial(a, 6, pwm, 1, 1);
    end
end


% BOTTON ADELANTE
function pushbutton9_Callback(hObject, eventdata, handles)
global a pin_numero pwm tipo_control num_pulsos 
if tipo_control==2
 
        [bit_parada]=comunicacion_serial(a, 7, pwm, 1, 1);

end
if tipo_control==1 
    for i=1:num_pulsos
        [bit_parada]=comunicacion_serial(a, 7, pwm, 1, 1);
    end
end



% BOTTON IZQUIERDA
function pushbutton10_Callback(hObject, eventdata, handles)
global a pin_numero pwm tipo_control num_pulsos 
if tipo_control==2

        [bit_parada]=comunicacion_serial(a, 8, pwm, 1, 1);

end
if tipo_control==1
    for i=1:num_pulsos
        [bit_parada]=comunicacion_serial(a, 8, pwm, 1, 1);
    end
end



% BOTTTON DERECHA
function pushbutton11_Callback(hObject, eventdata, handles)
global a pin_numero pwm tipo_control num_pulsos 
if tipo_control==2
    
        [bit_parada]=comunicacion_serial(a, 9, pwm, 1, 1);
    
end
if tipo_control==1
    for i=1:num_pulsos 
        [bit_parada]=comunicacion_serial(a, 9, pwm, 1, 1);
    end
end


% --- Executes on selection change in listbox1.
function listbox1_Callback(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox1


% --- Executes during object creation, after setting all properties.
function listbox1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
global tipo_control
tipo_control=get(hObject,'Value');


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% BOTON DE AUTOMATICO
% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
global a umbral xInicial yInicial vid url paro pwm control_fuzzy

cont=2;
paro=0;
cont2=1;
radio=15;
se = strel('diamond',1);
control_fuzzy=readfis('control_borroso');
center2=[xInicial yInicial];
figure
while paro==0
 cont=cont+1   
%
% fondo1 = getsnapshot(vid);
% imshow(fondo1)
%cla;

if mod(cont,200)==0
    
temperatura=a.analogRead(0);
temperatura=temperatura*500/1023
set(handles.text12,'String',temperatura);

e1=xInicial-450;
e2=yInicial-350;

% plot(xInicial, yInicial, 'ko', 'MarkerSize',10, 'MarkerFaceColor','w');
% hold on
% plot(450, 350, 'ko', 'MarkerSize',10, 'MarkerFaceColor','b');
error_X=xInicial-450;
error_Y=yInicial-350;
control_fuzzy=readfis('control_borroso');
pwm_fuzzy=round(evalfis([error_X error_Y],control_fuzzy))

if pwm_fuzzy(1)>=255
    pwm_fuzzy(1)=250;
elseif pwm_fuzzy(2)>=255
    pwm_fuzzy(2)=250;
elseif pwm_fuzzy(3)>=255
    pwm_fuzzy(3)=250;
end

[bit_parada]=automatico(a, 5, pwm_fuzzy(2), 1, 1);
end

% lectura de una imagen
 img = snapshot(vid);
 I = rgb2gray(img); 
 area = im2bw(I,umbral);
 
% %%
areas = bwlabel(area);
areas = logical(areas);
areas = bwpropfilt(areas,'perimeter',1);
%imshow(areas);
imshow(img);
stat = regionprops(areas,'centroid');
center=[stat(1).Centroid(1) stat(1).Centroid(2)];
viscircles(center2,10,'EdgeColor','b');
viscircles(center,radio,'EdgeColor','r');
%flushinput(img)
clear vars img  I area 
end
%axes(handles.axes1);
%imhist(I)
%stop(vid);
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
global a pwm bit_pwm umbral vid paro
umbral = get(handles.slider2,'Value')
% escritura del valor leido en pantalla
set(handles.text6,'String',umbral);
img=snapshot(vid);

 I = rgb2gray(img); 
 area = im2bw(I,umbral);
 cla
 axes(handles.axes1);
 imshow(area);

% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in pushbutton13.
function pushbutton19_Callback(hObject, eventdata, handles)
 global xInicial yInicial vid url pwm_fuzzy umbral
%url='http://192.168.43.1:8080video';
%vid=ipcam('http://192.168.43.1:8080/video');

%img=imread('medusa_test.jpeg');
img=imread('trayectoria_2.JPG');
%img=snapshot(vid);
axes(handles.axes1);
imshow(img)
hold on
hold on
Lectura1=ginput(1)
xInicial=Lectura1(1,1)
yInicial=Lectura1(1,2)
Lectura2=ginput(1)
xInicial2=Lectura2(1,1)
yInicial2=Lectura2(1,2)
Lectura3=ginput(1)
xInicial3=Lectura3(1,1)
yInicial3=Lectura3(1,2)
Lectura4=ginput(1)
xInicial4=Lectura4(1,1)
yInicial4=Lectura4(1,2)


I = rgb2gray(img); 
area = im2bw(I,0.75);
areas = bwlabel(area);
areas = bwlabel(area);
areas = logical(areas);
areas = bwpropfilt(areas,'perimeter',1);
imshow(img);
stat = regionprops(areas,'centroid');
center=[stat(1).Centroid(1) stat(1).Centroid(2)];
%viscircles(center2,10,'EdgeColor','b');
%viscircles(center,radio,'EdgeColor','r');

plot(xInicial, yInicial, 'ko', 'MarkerSize',10, 'MarkerFaceColor','w');
hold on
plot(xInicial2, yInicial2, 'ko', 'MarkerSize',10, 'MarkerFaceColor','w');
hold on 
plot(xInicial3, yInicial3, 'ko', 'MarkerSize',10, 'MarkerFaceColor','w');
hold on 
plot(xInicial4, yInicial4, 'ko', 'MarkerSize',10, 'MarkerFaceColor','w');
hold on 
plot(center(1), center(2), 'ko', 'MarkerSize',10, 'MarkerFaceColor','b');


% --- Executes on button press in pushbutton14.
function pushbutton14_Callback(hObject, eventdata, handles)
%clear all
global paro
paro=1;


% --- Executes on button press in pushbutton15.
function pushbutton15_Callback(hObject, eventdata, handles)
clear all;


% --- Executes on button press in pushbutton16.
function pushbutton16_Callback(hObject, eventdata, handles)

global xInicial yInicial vid url pwm_fuzzy a num_pulsos paro
url='http://192.168.43.1:8080video';
vid=ipcam('http://192.168.43.1:8080/video');
paro=0;
cla
axes(handles.axes1);
while paro==0
 img=snapshot(vid);
 imshow(img);
 clear vars img
end
% e1=xInicial-450;
% e2=yInicial-350;
% 
% % plot(xInicial, yInicial, 'ko', 'MarkerSize',10, 'MarkerFaceColor','w');
% % hold on
% % plot(450, 350, 'ko', 'MarkerSize',10, 'MarkerFaceColor','b');
% error_X=xInicial-450
% error_Y=yInicial-350
% control_fuzzy=readfis('control_borroso');
% pwm_fuzzy=round(evalfis([error_X error_Y],control_fuzzy))
% 
% if pwm_fuzzy(1)>=255
%     pwm_fuzzy(1)=250;
% elseif pwm_fuzzy(2)>=255
%     pwm_fuzzy(2)=250;
% elseif pwm_fuzzy(3)>=255
%     pwm_fuzzy(3)=250;
% end
% 
% error_X=abs(error_X);
% error_Y=abs(error_Y);
% 
%     num_pulsos_horizontal=round(error_X/100)
%     num_pulsos_vertical=round(error_Y/100)
% 
% 
% 
% if pwm_fuzzy(1)~=0
%     [bit_parada]=comunicacion_serial(a, 5, pwm_fuzzy(1), 1, num_pulsos_horizontal);
% end
% if pwm_fuzzy(2)~=0
%     [bit_parada]=comunicacion_serial(a, 6, pwm_fuzzy(2), 1, num_pulsos_vertical);
% end
% if pwm_fuzzy(3)~=0
%     [bit_parada]=comunicacion_serial(a, 7, pwm_fuzzy(3), 1, num_pulsos_horizontal);
% end


% --- Executes on button press in pushbutton17.
function pushbutton17_Callback(hObject, eventdata, handles)
global temperatura a vid umbral z cont_temp
temperatura=a.analogRead(0);
temperatura=temperatura*500/1023
set(handles.text12,'String',temperatura);
cont_temp=cont_temp+1;

 img=snapshot(vid);
 I = rgb2gray(img); 
 area = im2bw(I,umbral);
areas = bwlabel(area);
areas = logical(areas);
areas = bwpropfilt(areas,'perimeter',1);
%centroide de areas
stat = regionprops(areas,'centroid');
% obtenemos la posición del centroide 
center=[stat(1).Centroid(1) stat(1).Centroid(2)]

z(cont_temp,1)=center(1)
z(cont_temp,2)=center(2)
z(cont_temp,3)=temperatura

% --- Executes during object creation, after setting all properties.
function text4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in pushbutton18.
function pushbutton18_Callback(hObject, eventdata, handles)
global z
cla
axes(handles.axes1);
cla
% z=[10 20 30 25 30 29 31 20 38;17 38 20 31 40 29 25 20 19;31 32 30 32 35 31 30 32 36]
%z=[      444.8177 111.4712 26.3930;444.7631 211.5521 23.9492;444.7941 151.4139 24.4379;273.0755 23.2608 23.4604;398.0822 357.4624 27.8592;390.5723 173.1477 27.8592;257.3269 105.4117 29.3255];
      x=z(:,1);
      y=z(:,2);
     zz=z(:,3);
xlin = linspace(min(x),max(x),33);
ylin = linspace(min(y),max(y),33);
[X,Y] = meshgrid(xlin,ylin); 
f = scatteredInterpolant(x,y,zz);
Z = f(X,Y);
mesh(X,Y,Z);
z=Z;
 surf(Z); hold on
 shading interp;
 [c ch] = contour3(Z,20); set(ch, 'edgecolor', 'k')
 [u v] = gradient(Z);
 h = streamslice(-u,-v); % downhill
 set(h, 'color', 'k')
 for i=1:length(h);
 zi = interp2(Z,get(h(i), 'xdata'), get(h(i),'ydata'));
 set(h(i),'zdata', zi);
 end
 view(30,50); axis tight 
 %%
 figure 
  x=z(:,1);
      y=z(:,2);
     zz=z(:,3);
xlin = linspace(min(x),max(x),33);
ylin = linspace(min(y),max(y),33);
[X,Y] = meshgrid(xlin,ylin); 
f = scatteredInterpolant(x,y,zz);
Z = f(X,Y);
mesh(X,Y,Z);
z=Z;
 surf(Z); hold on
 shading interp;
 [c ch] = contour3(Z,20); set(ch, 'edgecolor', 'k')
 [u v] = gradient(Z);
 h = streamslice(-u,-v); % downhill
 set(h, 'color', 'k')
 for i=1:length(h);
 zi = interp2(Z,get(h(i), 'xdata'), get(h(i),'ydata'));
 set(h(i),'zdata', zi);
 end
 view(30,50); axis tight 

% --- Executes during object creation, after setting all properties.
function axes2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes2


% --- Executes on button press in pushbutton19.
