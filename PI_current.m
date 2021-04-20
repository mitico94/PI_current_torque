function varargout = PI_current(varargin)
% PI_CURRENT MATLAB code for PI_current.fig
%      PI_CURRENT, by itself, creates a new PI_CURRENT or raises the existing
%      singleton*.
%
%      H = PI_CURRENT returns the handle to a new PI_CURRENT or the handle to
%      the existing singleton*.
%
%      PI_CURRENT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PI_CURRENT.M with the given input arguments.
%
%      PI_CURRENT('Property','Value',...) creates a new PI_CURRENT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before PI_current_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to PI_current_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help PI_current

% Last Modified by GUIDE v2.5 29-Jan-2021 15:39:59

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @PI_current_OpeningFcn, ...
                   'gui_OutputFcn',  @PI_current_OutputFcn, ...
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


% --- Executes just before PI_current is made visible.
function PI_current_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to PI_current (see VARARGIN)

% Choose default command line output for PI_current
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes PI_current wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = PI_current_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function current_target_Callback(hObject, eventdata, handles)
% hObject    handle to current_target (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of current_target as text
%        str2double(get(hObject,'String')) returns contents of current_target as a double

handles.v = get(handles.current_target, 'Value');
guidata(hObject,handles);

handles.pop = true;
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function current_target_CreateFcn(hObject, eventdata, handles)
% hObject    handle to current_target (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in start.
function start_Callback(hObject, eventdata, handles)
% hObject    handle to start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cla(handles.current);
cla(handles.torque);
cla();

handles.stop=false;
guidata(hObject,handles);

handles.pop = false;
guidata(hObject,handles);

handles.v = 1;
guidata(hObject,handles);


handles.clockwise=false;
guidata(hObject,handles);

handles.counterclockwise=false;
guidata(hObject,handles);


load('matlab.mat') ;

axes(handles.torque)
plot(Current, Torque);
grid on, hold on

s = serial('COM8', 'BaudRate', 115200, 'Databits', 8,'Parity', "even", 'StopBits', 1)
i=1;
while(1)

    fopen(s);
    
    start = fread(s,1,'char');
    
    if(start == 'b')
        
         data(i,:) = fread(s,3,'uint16')
         ended = fread(s,1,'char');
         
         if(ended == 'e')
            
             axes(handles.current)
             plot(data(:,1),'g');
             grid on, hold on
             axes(handles.current)
             plot(data(:,2),'r');
             
             axes(handles.t_real)
             plot(data(:,3)/1000,'g');
             grid on, hold on
             
             i = i+1;
             
         end
    end
    
    fclose(s);
    
    drawnow()
    handles = guidata(hObject);
    
    if(handles.pop == true)
        v = handles.v;
        fopen(s);
        switch v
            case 1
                fwrite(s,0,'uint16');
            case 2
                fwrite(s,80,'uint16');
            case 3
                fwrite(s,90,'uint16');
            case 4
                fwrite(s,100,'uint16');
            case 5
                fwrite(s,120,'uint16');
            case 6
                fwrite(s,150,'uint16');
            case 7
                fwrite(s,200,'uint16');
            case 8
                fwrite(s,250,'uint16');
            case 9
                fwrite(s,300,'uint16');
            case 10
                fwrite(s,350,'uint16');
            case 11
                fwrite(s,400,'uint16');
            case 12
                fwrite(s,500,'uint16');
            case 13
                fwrite(s,550,'uint16');
            case 14
                fwrite(s,600,'uint16');
            case 15
                fwrite(s,700,'uint16');
            case 16
                fwrite(s,1000,'uint16');
            otherwise
                fwrite(s,0,'uint16');
        end               
        fclose(s);
        handles.pop = false;
    end
    
    if(handles.clockwise==true)
        
        fopen(s)
        fwrite(s,'c','uint16');
        fclose(s)
        handles.clockwise=false
        guidata(hObject,handles);
        
    end
   
    if(handles.counterclockwise==true)
        
        fopen(s)
        fwrite(s,'k','uint16');
        fclose(s)
        handles.counterclockwise=false
        guidata(hObject,handles);
        
   end
    
    if(handles.stop==true)
        fopen(s);
        fwrite(s,'b','uint16');
        fclose(s);
        guidata(hObject,handles);
        break
    end   
end
        

% --- Executes on button press in stop.
function stop_Callback(hObject, eventdata, handles)
% hObject    handle to stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.stop=true;
guidata(hObject,handles);


% --- Executes on button press in clockwise.
function clockwise_Callback(hObject, eventdata, handles)
% hObject    handle to clockwise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


handles.clockwise=true;
guidata(hObject,handles);

% --- Executes on button press in counterclockwise.
function counterclockwise_Callback(hObject, eventdata, handles)
% hObject    handle to counterclockwise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.counterclockwise=true;
guidata(hObject,handles);
