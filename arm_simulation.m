function varargout = arm_simulation(varargin)
%ARM_SIMULATION MATLAB code file for arm_simulation.fig
%      ARM_SIMULATION, by itself, creates a new ARM_SIMULATION or raises the existing
%      singleton*.
%
%      H = ARM_SIMULATION returns the handle to a new ARM_SIMULATION or the handle to
%      the existing singleton*.
%
%      ARM_SIMULATION('Property','Value',...) creates a new ARM_SIMULATION using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to arm_simulation_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      ARM_SIMULATION('CALLBACK') and ARM_SIMULATION('CALLBACK',hObject,...) call the
%      local function named CALLBACK in ARM_SIMULATION.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help arm_simulation

% Last Modified by GUIDE v2.5 28-Aug-2018 20:42:19

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @arm_simulation_OpeningFcn, ...
                   'gui_OutputFcn',  @arm_simulation_OutputFcn, ...
                   'gui_LayoutFcn',  [], ...
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


global se;
fclose(serial('com14'));
delete(serial('com14'));
clear serial('com14');


% --- Executes just before arm_simulation is made visible.
function arm_simulation_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for arm_simulation
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes arm_simulation wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = arm_simulation_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function theta_1_Callback(hObject, eventdata, handles)
% hObject    handle to theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta_1 as text
%        str2double(get(hObject,'String')) returns contents of theta_1 as a double


% --- Executes during object creation, after setting all properties.
function theta_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.

if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta_2_Callback(hObject, eventdata, handles)
% hObject    handle to theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta_2 as text
%        str2double(get(hObject,'String')) returns contents of theta_2 as a double


% --- Executes during object creation, after setting all properties.
function theta_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta_3_Callback(hObject, eventdata, handles)
% hObject    handle to theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta_3 as text
%        str2double(get(hObject,'String')) returns contents of theta_3 as a double


% --- Executes during object creation, after setting all properties.
function theta_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pos_x_Callback(hObject, eventdata, handles)
% hObject    handle to pos_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pos_x as text
%        str2double(get(hObject,'String')) returns contents of pos_x as a double


% --- Executes during object creation, after setting all properties.
function pos_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pos_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pos_y_Callback(hObject, eventdata, handles)
% hObject    handle to pos_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pos_y as text
%        str2double(get(hObject,'String')) returns contents of pos_y as a double


% --- Executes during object creation, after setting all properties.
function pos_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pos_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pos_z_Callback(hObject, eventdata, handles)
% hObject    handle to pos_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pos_z as text
%        str2double(get(hObject,'String')) returns contents of pos_z as a double


% --- Executes during object creation, after setting all properties.
function pos_z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pos_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in inverse.
function inverse_Callback(hObject, eventdata, handles)
% hObject    handle to inverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

px  =  handles.pos_x.String ;
py  =  handles.pos_y.String ;
pz  =  handles.pos_z.String ;


px  =  str2double(px);
py  =  str2double(py);
pz  =  str2double(pz);

L_1 = 3;
L_2 = 14;
L_3 = 11;

L(1) = Link([0 L_1 0 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 0]);

Robot = SerialLink(L);
Robot.name = '3 DOF Robot';

%T = [ 1 0 0 px;
 %     0 1 0 py;
  %    0 0 1 pz;
   %   0 0 0 1];

%J = Robot.ikine(T,[0 0 0],'mask',[1 1 1 0 0 0])*180/pi;
%J  = Robot.ikcon(T)*180/pi
syms x y z total_z theta1 theta2 theta3
syms h link_1 link_2
syms e %beta 
syms f % gamma
syms g %alpha
syms a h 
syms b 
syms k %i                        % square of a and h solutions %
syms d

x = px;
y = py;
z = pz;
z = z-3;

link_1 = 14 ;
link_2 = 11 ;

b = x^2 + y^2;          %    pythagoras' theorem % b = a^2   
a = sqrt(b);
k = b + z^2;            %    pythagoras' theorem    % i = h^2   
h = sqrt(k);
theta1 = atan2d(y,x);                                          %angle for motor 1%

e = acos ((h^2-link_1^2-link_2^2)/(-2*link_1*link_2))     % law of cosine %
f = asin( link_2*sin(e)/h);                            % law of sine %
g = atan2 (z,a) ;
theta1 = atan2(y,x)*180/pi          ;
theta2 =( g + f ) * 180/pi          ;                                  % angle of motor 2 %
theta3 = 180.0 - e *(180/pi)        ;                                      % angle of motor 3 %
theta3 =   -theta3        
thetamat(1,1) =  theta1 ;
thetamat(1,2) =  theta2 ;
thetamat(1,3) = theta3 ;
thetamat
handles.theta_1.String = num2str(round(thetamat(1)));
handles.theta_2.String = num2str(round(thetamat(2)));
handles.theta_3.String = num2str(round(thetamat(3)));  

th_1 = int16(theta1) ;
th_2 = int16(theta2) ;
th_3 = int16(theta3) ;

Th_1 = num2str(th_1) ;
Th_2 = num2str(th_2) ;
Th_3 = num2str(th_3) ;

s   = ','     ;
s_1 = ','   ;
s_2 = ','   ;
s_3 = ','   ;

m_1 = size(Th_1);
c_1 = m_1(1,2);

m_2 = size(Th_2);
c_2 = m_2(1,2);

for i = 1 : (  5 - c_1 - 1 ) ;
    s_1 = strcat( s_1 , s );
end

for i = 1 : (  5 - c_2 - 1 ) ;
    s_2 = strcat ( s_2 , s );
end
    
s_3 = strcat ( Th_1 , s_1 ) ;
s_4 = strcat ( Th_2 , s_2 ) ;

s_5 = strcat ( s_3 , s_4 ) ;
s_6 = strcat ( s_5 , Th_3 ) ;

global se;

fprintf(se,'%s',s_6);

Robot.plot(thetamat*pi/180);

% --- Executes on button press in forward.
function forward_Callback(hObject, eventdata, handles)
% hObject    handle to forward (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

Th_1 = handles.theta_1.String ;
Th_2 = handles.theta_2.String ;
Th_3 = handles.theta_3.String ;

s = ',';
s_1 = ',';
s_2 = ',';
s_3 = ',';

m_1 = size(Th_1);
c_1 = m_1(1,2);

m_2 = size(Th_2);
c_2 = m_2(1,2);

for i = 1 : (  5 - c_1 - 1 ) ;
    s_1 = strcat( s_1 , s );
end

for i = 1 : (  5 - c_2 - 1 ) ;
    s_2 = strcat ( s_2 , s );
end
    
s_3 = strcat ( Th_1 , s_1 ) ;
s_4 = strcat ( Th_2 , s_2 ) ;

s_5 = strcat ( s_3 , s_4 ) ;
s_6 = strcat ( s_5 , Th_3 ) ;

global se;

fprintf(se,'%s',s_6);

Th_1 = str2double(Th_1)*pi/180;
Th_2 = str2double(Th_2)*pi/180;
Th_3 = str2double(Th_3)*pi/180;

L_1 = 3.0;
L_2 = 14.0;
L_3 = 11.0;

L(1) = Link([0 L_1 0 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 0]);

Robot      = SerialLink(L);
Robot.name = '3 DOF Robot';
Robot.plot([Th_1 Th_2 Th_3]);

forward_result = Robot.fkine([Th_1 Th_2 Th_3])
forward_result = transl(forward_result);
handles.pos_x.String = num2str(round(forward_result(1)));
handles.pos_y.String = num2str(round(forward_result(2)));
handles.pos_z.String = num2str(round(forward_result(3)));


% --- Executes on button press in open.
function open_Callback(hObject, eventdata, handles)
% hObject    handle to open (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global se;
fclose(serial('com3'));
delete(serial('com33'));
clear serial('com3');

se = serial('com3','BaudRate',115200);
fopen(se);
pause(1);
fprintf('Serial communication was started.\n');

% --- Executes on button press in close.
function close_Callback(hObject, eventdata, handles)
% hObject    handle to close (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global se;
fclose(se);
delete(se);
clear se;
fprintf('Serial communication was closed.\n');


% --- Executes on button press in clear.
function clear_Callback(hObject, eventdata, handles)
% hObject    handle to clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

delete(instrfind);
clear all;
clc;
