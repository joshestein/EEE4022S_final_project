function varargout = interactive(varargin)
% INTERACTIVE MATLAB code for interactive.fig
%      INTERACTIVE, by itself, creates a new INTERACTIVE or raises the existing
%      singleton*.
%
%      H = INTERACTIVE returns the handle to a new INTERACTIVE or the handle to
%      the existing singleton*.
%
%      INTERACTIVE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in INTERACTIVE.M with the given input arguments.
%
%      INTERACTIVE('Property','Value',...) creates a new INTERACTIVE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before interactive_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to interactive_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help interactive

% Last Modified by GUIDE v2.5 25-Sep-2019 08:53:45

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @interactive_OpeningFcn, ...
                   'gui_OutputFcn',  @interactive_OutputFcn, ...
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


% --- Executes just before interactive is made visible.
function interactive_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to interactive (see VARARGIN)

% Choose default command line output for interactive
handles.output = hObject;

if (nargin > 3)
    % display and plot polygons
    imshow(varargin{1}); hold on;
    p = plot(varargin{2});
    handles.polys = varargin{2};
    handles.p = p;
end

% Update handles structure
guidata(hObject, handles);

set(handles.select, 'State', 'on');
select_OnCallback(handles.select, eventdata, handles)

% UIWAIT makes interactive wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = interactive_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
% disp(handles.polys)
global polygons;
varargout{1} = polygons;


% --------------------------------------------------------------------
function done_btn_ClickedCallback(hObject, eventdata, handles)
% hObject    handle to done_btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargout{1} = handles.polys;
close(gcf);


% --------------------------------------------------------------------
function select_OnCallback(hObject, eventdata, handles)
% hObject    handle to select (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% set(gcf,'pointer','crosshair');

% global for use in vargout (sending data out from GUI)
global polygons;

while (ishghandle(hObject) && strcmp(get(hObject, 'State'), 'on'))
    % get x,y cooridnates
    [x, y, button] = ginput(1);
    if ((button == 100)) % 'd' key
        set(handles.select, 'State', 'off')
        select_OnCallback(handles.select, eventdata, handles)
        break;
    elseif(isempty(button)) % 'return' key
        close(gcf);
        break;
    end

    selection = -1;

    % check of selected coordinate is in a polygon
    for i = 1:size(polygons)
        if (inpolygon(x, y, polygons(i).Vertices(:,1), polygons(i).Vertices(:,2)))
            selection = i;
            break;
        end
    end

    polygons = handles.polys;

    if (selection ~= -1)
        if (button == 1) % left mouse click
            % delete selected polygon
            polygons(selection) = [];
        elseif(button == 3) % right mouse click
            % only keep selected polygon
            polygons = polygons(selection);
        end
        handles.polys = polygons;

        % force a refresh with holdon
        set(handles.p, 'Visible', 'off');
        p = plot(polygons);
        handles.p = p;
    end
    % update handles
    guidata(hObject, handles);
end

% --------------------------------------------------------------------
function select_OffCallback(hObject, eventdata, handles)
% hObject    handle to select (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(gcf,'pointer','arrow');


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
delete(hObject);

function figure1_WindowKeyPressFcn(hObject, eventData, handles)


% --- Executes on key press with focus on figure1 and none of its controls.
function figure1_KeyPressFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.FIGURE)
%	Key: name of the key that was pressed, in lower case
%	Character: character interpretation of the key(s) that was pressed
%	Modifier: name(s) of the modifier key(s) (i.e., control, shift) pressed
% handles    structure with handles and user data (see GUIDATA)

if (eventdata.Key == 'return')
    delete(hObject);
elseif (eventdata.Key == 'c')
    set(handles.select, 'State', 'on')
end