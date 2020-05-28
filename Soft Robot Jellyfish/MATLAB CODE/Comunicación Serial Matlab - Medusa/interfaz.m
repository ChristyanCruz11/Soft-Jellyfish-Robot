function varargout = interfaz(varargin)
% INTERFAZ MATLAB code for interfaz.fig
%      INTERFAZ, by itself, creates a new INTERFAZ or raises the existing
%      singleton*.

% Edit the above text to modify the response to help interfaz

% Last Modified by GUIDE v2.5 18-Jul-2019 19:30:40

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
 axes(handles.axes2);
    im2=imread('upm.png');
    image(im2)
    
    axis off
    
    axes(handles.axes3);
    im2=imread('car.png');
    image(im2)
    
    axis off
    
    axes(handles.axes1);
    im2=imread('inicio.png');
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
global bit_pwm num_pulsos tipo_control

    %axes(handles.axes1);
    %im2=imread('upm.png');
    %image(im2)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
vid=videoinput('winvideo',1,'MJPG_640x480');
set(vid,'FramesPerTrigger',inf)
set(vid,'ReturnedColorspace','rgb')
se = strel('line',14,70);
start(vid);
fondo1 = getsnapshot(vid);
axes(handles.axes1);
imshow(fondo1)
stop(vid);
axis off
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if tipo_control==2
    cadena=bit_pwm*1000000+num_pulsos*100000+10000;
end
if tipo_control==1
   cadena=5910000; 
end

[bit_parada]=comunicacion_serial(cadena);


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% lectura del valor pwm del slider
global pwm_deseado bit_pwm
pwm_deseado = get(handles.slider1,'Value')
if pwm_deseado==0
    bit_pwm=0;
end

if pwm_deseado>0 && pwm_deseado<=50
    bit_pwm=1;
end

if pwm_deseado>50 && pwm_deseado<=100
    bit_pwm=2;
end

if pwm_deseado>100 && pwm_deseado<=150
    bit_pwm=3;
end

if pwm_deseado>150 && pwm_deseado<=200
    bit_pwm=4;
end

if pwm_deseado>200 && pwm_deseado<=250
    bit_pwm=5;
end
% escritura del valor leido en pantalla
set(handles.text4,'String',pwm_deseado);


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
global bit_pwm num_pulsos
cadena=3100001;
[bit_parada]=comunicacion_serial(cadena);

% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
global bit_pwm num_pulsos
cadena=3100010;
[bit_parada]=comunicacion_serial(cadena);

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
global bit_pwm num_pulsos
cadena=3100100;
[bit_parada]=comunicacion_serial(cadena);
% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
global bit_pwm num_pulsos
cadena=3101000;
[bit_parada]=comunicacion_serial(cadena);

% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
global bit_pwm num_pulsos
cadena=3110000;
[bit_parada]=comunicacion_serial(cadena);

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


% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
global num_pulsos

num_pulsos=get(handles.pulsos,'String');
num_pulsos = str2double(num_pulsos);
%set(handles.text5,'String',num_pulsos);

% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton8.
function pushbutton8_Callback(hObject, eventdata, handles)
global bit_pwm num_pulsos tipo_control

if tipo_control==2
cadena=bit_pwm*1000000+num_pulsos*100000+00001;
end
if tipo_control==1
   cadena=5900001; 
end
[bit_parada]=comunicacion_serial(cadena);

% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)
global bit_pwm num_pulsos tipo_control
if tipo_control==2
cadena=bit_pwm*1000000+num_pulsos*100000+00010;
end
if tipo_control==1
   cadena=5900010; 
end

[bit_parada]=comunicacion_serial(cadena);


% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
global bit_pwm num_pulsos tipo_control
if tipo_control==2
    cadena=bit_pwm*1000000+num_pulsos*100000+00100;
end
if tipo_control==1
   cadena=5900100; 
end

[bit_parada]=comunicacion_serial(cadena);


% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
global bit_pwm num_pulsos tipo_control

if tipo_control==2
cadena=bit_pwm*1000000+num_pulsos*100000+01000;
end
if tipo_control==1
   cadena=5901000; 
end
[bit_parada]=comunicacion_serial(cadena);

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
