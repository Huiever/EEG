function varargout = activityReco(varargin)
% ACTIVITYRECO MATLAB code for activityReco.fig
%      ACTIVITYRECO, by itself, creates a new ACTIVITYRECO or raises the existing
%      singleton*.
%
%      H = ACTIVITYRECO returns the handle to a new ACTIVITYRECO or the handle to
%      the existing singleton*.
%
%      ACTIVITYRECO('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ACTIVITYRECO.M with the given input arguments.
%
%      ACTIVITYRECO('Property','Value',...) creates a new ACTIVITYRECO or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before activityReco_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to activityReco_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help activityReco

% Last Modified by GUIDE v2.5 04-Jun-2020 15:52:30

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @activityReco_OpeningFcn, ...
                   'gui_OutputFcn',  @activityReco_OutputFcn, ...
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


% --- Executes just before activityReco is made visible.
function activityReco_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to activityReco (see VARARGIN)

global COM;
global rate;
global act a;
global count;
global act_data t;
global p x;
count=1;x=-15;
act=zeros(1,8);
t=0;
p = plot(t,act,'MarkerSize',5);
% axis([x x+20 -200 200]);
% grid(handles.axplotact,'on');
set(handles.axplotact,'XLim',[x x+20],'YLim',[-1000 1000]);
set(handles.axplotact,'XTickLabel',[]);
% legendaxes=legend(handles.axplotact,{'Yaw','Pitch','Roll','Accx','Accy','Accz','GYROx','GYROy','GYROz','Magx','Magy','Magz'},1);
% set(legendaxes,'Location','northeastoutside');
act_data=[]; a=[];
COM='COM5'
rate = 115200;
set(handles.ppcom,'value', 5);
set(handles.ppbandrate,'value',4);
set(handles.pbcloseserial,'Enable','off');
%%设置定时器
% handles.ht=timer;%定义一个定时器，添加到handles结构体中，方便后面使用
% set(handles.ht,'ExecutionMode','FixedRate');%ExecutionMode   执行的模式
% set(handles.ht,'Period',0.2);%周期
% set(handles.ht,'TimerFcn',{@ExecuteTask,handles});%定时器的执行函数
% 
% start(handles.ht);  %启动定时器
% Choose default command line output for activityReco
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes activityReco wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = activityReco_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in rbshowangles.
function rbshowangles_Callback(hObject, eventdata, handles)
% hObject    handle to rbshowangles (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of rbshowangles


% --- Executes on button press in rbshowacc.
function rbshowacc_Callback(hObject, eventdata, handles)
% hObject    handle to rbshowacc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of rbshowacc


% --- Executes on button press in rbshowgyro.
function rbshowgyro_Callback(hObject, eventdata, handles)
% hObject    handle to rbshowgyro (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of rbshowgyro


% --- Executes on button press in rbshowmag.
function rbshowmag_Callback(hObject, eventdata, handles)
% hObject    handle to rbshowmag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of rbshowmag


% --- Executes on selection change in ppcom.
function ppcom_Callback(hObject, eventdata, handles)
% hObject    handle to ppcom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns ppcom contents as cell array
%        contents{get(hObject,'Value')} returns selected item from ppcom
global COM;
val=get(hObject,'value');
switch val
    case 1
        COM='COM1';
        fprintf('ceshi_COM=1\n');
    case 2
        COM='COM2';
    case 3
        COM='COM3';
    case 4
        COM='COM4';
    case 5
        COM='COM5';
    case 6
        COM='COM6';
    case 7
        COM='COM7';
    case 8
        COM='COM8';
    case 9
        COM='COM9';
end
              

% --- Executes during object creation, after setting all properties.
function ppcom_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ppcom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pbopenserial.
function pbopenserial_Callback(hObject, eventdata, handles)
% hObject    handle to pbopenserial (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clc
delete(instrfindall)      % 关闭串口，此句十分重要，下篇再详细解释  

global s;
global rate;
global COM;
global out;
global runingtime;
runingtime=1;
out=1;
format short g
s = serial(COM);       %使用默认设置创建串口s  
set(s,'BaudRate',rate);
% set(s,'DataBits',8);
% set(s,'StopBits',1);
set(s,'InputBufferSize',31);%%%设置输入缓冲区大小为1
% set(handles.pbopenserial,'Enable','off');
% set(handles.pbcloseserial,'Enable','on');

%串口事件回调设置
set(s,'BytesAvailableFcnMode','byte'); %设置中断触发方式  
set(s,'BytesAvailableFcnCount',31); %设置中断触发方式  
s.BytesAvailableFcn ={@ReceiveCallback,handles};       % 定义中断响应函数对象  
% s.BytesAvailableFcn={@EveBytesAvailableFcn,handles};%回调函数的指定
fopen(s);%打开串口

global count;
count=1;
fprintf('Opening finished!\n');

function ReceiveCallback( obj,event,handles)     %创建中断响应函数  
    global s a fid; 
    global count;
    global  act;
    global t x p;   
    fid = fopen('ads_data1.txt','a+');
  fclose('all');
  delete('ads_data1.txt');
  fid = fopen('ads_data1.txt','a+');
  
    str = fread(s); %读取数据
%   hex = compose("%X",str)
    data = zeros(1,8);
    sign_head1 = hex2dec('0D');
    sign_head2 = hex2dec('0A');
    sign_finish1 = hex2dec('0A');
    sign_finish2 = hex2dec('0D');
    a = [];
    a = [a;str];
    j = 1;
    while (~isempty(a))
         if j>size(a,1)
           break;
         end
         if a(j)==sign_head1 && a(j+1) == sign_head2 
            if (j + 31 - 1) > size(a,1) 
                break;
            end
            index_start = j + 2;
            index_finish= index_start + 31 - 2 - 1;
            pack = a(index_start:index_finish);
            if ~isempty(pack) && pack(28) == sign_finish1 && pack(29)== sign_finish2
                for m = 1:8                        
                    n = m * 3 + 1;
                    HEX = bitshift(pack(n),16) + bitshift(pack(n+1),8) + pack(n+2);
                    if bitand(HEX ,hex2dec('800000'))
                        data(m) = (16777216 - HEX) * (-4500000)/8388607;
                    else
                        data(m) = HEX * (4500000)/8388607;                        
                    end
                end
                act = [act;data];

                fprintf(fid,'%4.2f %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f\n',data);
                count=count+1;
                t=[t 0.1*count]; 
                
                if size(act,1)>1000
                    act= act(end-999,end, :); 
                    t= t(end-999,end, : );
                end
                set(handles.edshowdata,'string',num2str(data));

    %                         hold(handles.axplotact,'on');
                axis(handles.axplotact);
                if ~get(handles.rbpause,'Value')
                    if get(handles.rbshowangles,'Value')
    %                                 plot(handles.axplotact,act(1:count,1),'r-')
    %                       -          plot(handles.axplotact,act(1:count,2),'g-')
    %                                 plot(handles.axplotact,act(1:count,3),'m-')
    set(p(1),'Color',[1 0.5 0],'LineWidth',3);                        
    set(p(1),'XData',t,'YData',100);
                    end
                    if get(handles.rbshowacc,'Value')
                            set(p(4),'XData',t,'YData',act(4,:));
                            set(p(5),'XData',t,'YData',act(5,:));
                            set(p(6),'XData',t,'YData',act(6,:));
                    end
                    if  get(handles.rbshowgyro,'Value')
                            set(p(7),'XData',t,'YData',act(7,:));
                            set(p(8),'XData',t,'YData',act(8,:));
                            set(p(9),'XData',t,'YData',act(9,:));
                    end
                    if get(handles.rbshowmag,'Value')
                            set(p(10),'XData',t,'YData',act(10,:));
                            set(p(11),'XData',t,'YData',act(11,:));
                            set(p(12),'XData',t,'YData',act(12,:));
                    end

                     drawnow
                     x=x+0.1;
                     set(handles.axplotact,'ytick',-1000:200:1000);
                     axis(handles.axplotact,[x x+20 -1000 1000]);
    %                          set(handles.axplotact,'xtick',x:x+20);
%                      if size(t,2) >400
%                          t(1)=[];
%                         act(:,1)=[];
%                       end
                a(1:index_finish)=[];
                j = 1;
            else
                j = j + 1;
            end
         else
             j = j + 1;
         end
    end
end
% 
% function ReceiveCallback( obj,event,handles)     %创建中断响应函数  
%    global s a fid;
%    global count;
%    global  act;
%    global act_data;
%    global t x p;
%    str = fread(s,100,'uint8');%读取数据
%    hex=dec2hex(str);
%   IMU_data = [];Motion_data=[];
%   sign_head1=hex2dec('A5');sign_head2 = hex2dec('5A');
%   sign_finish=hex2dec('AA');sign_IMU=hex2dec('A1');sign_Motion=hex2dec('A2');
% 
%   a= [a;str];
%   j=1;
%   while (~isempty(a))
%         if j>size(a,1)
%            break;
%         end
%         if a(j)==sign_head1 && a(j+1) == sign_head2 
%             if (j+a(j+2)+1) > size(a,1) 
%                 break;
%             end
%             index_start = j+2;
%             index_finish= index_start + a(j+2)-1;
%             pack = a(index_start:index_finish);
%             if ~isempty(pack) &&pack(pack(1))== sign_finish
%                   if pack(2) == sign_IMU
%                         IMU_data(1,:) = Get_IMU(pack);
%                         j = index_finish;
%                         continue;
%                   end
%                    if pack(2) ==sign_Motion
%                           Motion_data(1,:) = Get_Motion(pack);
%                           j = index_finish;
%                    end
%                    if ~isempty(IMU_data) && ~isempty(Motion_data)
%                         count=count+1;
%                         act_data = [IMU_data,Motion_data]';
% %                         fprintf(fid,'%8.1f%8.1f%8.1f%8.1f%8.1f%8.1f%8d%8d%8d%8d%8d%8d%8d%8d%8d\n',act_data);
%                         t=[t 0.1*count]; 
%                         act=[act,[act_data(1:3);act_data(7:9)*100/16384;act_data(10:12)*pi/180;act_data(13:15)]];%%绘图数据归一化-200-200
%                         set(handles.edshowdata,'string',num2str(act_data));
%              
% %                         hold(handles.axplotact,'on');
%                         axis(handles.axplotact);
%                         if ~get(handles.rbpause,'Value')
%                             if get(handles.rbshowangles,'Value')
% %                                 plot(handles.axplotact,act(1:count,1),'r-')
% %                                 plot(handles.axplotact,act(1:count,2),'g-')
% %                                 plot(handles.axplotact,act(1:count,3),'m-')
%                                     set(p(1),'XData',t,'YData',act(1,:));
%                                     set(p(2),'XData',t,'YData',act(2,:));
%                                     set(p(3),'XData',t,'YData',act(3,:));
%                             end
%                             if get(handles.rbshowacc,'Value')
%                                     set(p(4),'XData',t,'YData',act(4,:));
%                                     set(p(5),'XData',t,'YData',act(5,:));
%                                     set(p(6),'XData',t,'YData',act(6,:));
%                             end
%                             if  get(handles.rbshowgyro,'Value')
%                                     set(p(7),'XData',t,'YData',act(7,:));
%                                     set(p(8),'XData',t,'YData',act(8,:));
%                                     set(p(9),'XData',t,'YData',act(9,:));
%                             end
%                             if get(handles.rbshowmag,'Value')
%                                     set(p(10),'XData',t,'YData',act(10,:));
%                                     set(p(11),'XData',t,'YData',act(11,:));
%                                     set(p(12),'XData',t,'YData',act(12,:));
%                             end
%                       
%                              drawnow
%                               x=x+0.1;
%                              set(handles.axplotact,'ytick',-200:50:200);
%                              axis(handles.axplotact,[x x+20 -200 200]);
% %                          set(handles.axplotact,'xtick',x:x+20);
%                              if size(t,2) >400
%                                  t(1)=[];
%                                 act(:,1)=[];
%                               end
%                         end 
%                    end
% %                   set(handles.edshowdata,'String',num2str(act));
%                     Motion_data=[];IMU_data=[];
%                     a(1:index_finish)=[];
%                     j=1;
% %                   pause(0.005);
%             end
%          else
%                 j=j+1;
%         end    
%   end  
%   
  
%   %%基于时间的线程
% function ExecuteTask(obj,eventdata,handles)
% global act_data;
% global act;
% global count;
%     set(handles.edshowdata,'string',num2str(act_data)); 
%     set(handles.axplotact,'ytick',-200:50:200);
%     hold(handles.axplotact,'on');
%     if ~get(handles.rbpause,'Value')
%        if get(handles.rbshowangles,'Value')
%            plot(handles.axplotact,act(1:count,1),'r-')
%            plot(handles.axplotact,act(1:count,2),'g-')
%            plot(handles.axplotact,act(1:count,3),'m-')
%        end
%        if get(handles.rbshowacc,'Value')
%            plot(handles.axplotact,act(1:count,4),'r--')
%            plot(handles.axplotact,act(1:count,5),'g--')
%            plot(handles.axplotact,act(1:count,6),'m--')
%        end
%        if  get(handles.rbshowgyro,'Value')
%            plot(handles.axplotact,act(1:count,7),'r:')
%            plot(handles.axplotact,act(1:count,8),'g:')
%            plot(handles.axplotact,act(1:count,9),'m:')
%        end
%        if get(handles.rbshowmag,'Value')
%            plot(handles.axplotact,act(1:count,10),'b')
%            plot(handles.axplotact,act(1:count,11),'m')
%            plot(handles.axplotact,act(1:count,12),'k')
%        end
%     end 
%   
  
  
% --- Executes on selection change in ppbandrate.
function ppbandrate_Callback(hObject, eventdata, handles)
% hObject    handle to ppbandrate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns ppbandrate contents as cell array
%        contents{get(hObject,'Value')} returns selected item from ppbandrate
global rate;
val=get(hObject,'value');
switch val
    case 1
        rate=9600;
    case 2
        rate=19200;
    case 3
        rate=38400;
    case 4
        rate=115200;
end 

% --- Executes during object creation, after setting all properties.
function ppbandrate_CreateFcn(hObject, ~, handles)
% hObject    handle to ppbandrate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pbcloseserial.
function pbcloseserial_Callback(hObject, eventdata, handles)
% hObject    handle to pbcloseserial (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global s;
fclose(s);
delete(s);
set(handles.pbcloseserial,'Enable','on');
set(handles.pbopenserial,'Enable','off');
fprintf('close com');



function edshowdata_Callback(hObject, eventdata, handles)
% hObject    handle to edshowdata (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edshowdata as text
%        str2double(get(hObject,'String')) returns contents of edshowdata as a double


% --- Executes during object creation, after setting all properties.
function edshowdata_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edshowdata (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edact_Callback(hObject, eventdata, handles)
% hObject    handle to edact (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edact as text
%        str2double(get(hObject,'String')) returns contents of edact as a double


% --- Executes during object creation, after setting all properties.
function edact_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edact (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --------------------------------------------------------------------
function mnnew_Callback(hObject, eventdata, handles)
% hObject    handle to mnnew (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pbcleandata.
function pbcleandata_Callback(hObject, eventdata, handles)
% hObject    handle to pbcleandata (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.edshowdata,'string','');


% --------------------------------------------------------------------
function mnopen_Callback(hObject, eventdata, handles)
% hObject    handle to mnopen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pbsavedata.
function pbsavedata_Callback(hObject, eventdata, handles)
% hObject    handle to pbsavedata (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edtextname_Callback(hObject, eventdata, handles)
% hObject    handle to edtextname (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edtextname as text
%        str2double(get(hObject,'String')) returns contents of edtextname as a double
global textname;
textname=get(hObject,'String');


% --- Executes during object creation, after setting all properties.
function edtextname_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edtextname (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function axplotact_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axplotact (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
set(hObject,'XLim',[-15 5],'YLim',[-200 200]);
set(hObject,'YTickLabelMode','manual');
% legend(handles.axplotact,{'Yaw','Pitch','Roll','Accx','Accy','Accz','GYROx','GYROy','GYROz','Magx','Magy','Magz'},1);
% Hint: place code in OpeningFcn to populate axplotact


% --- Executes on button press in cbsave.
function cbsave_Callback(hObject, eventdata, handles)
% hObject    handle to cbsave (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cbsave


% --- Executes on button press in rbpause.
function rbpause_Callback(hObject, eventdata, handles)
% hObject    handle to rbpause (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of rbpause


% --- Executes during object creation, after setting all properties.
function axes7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes7
