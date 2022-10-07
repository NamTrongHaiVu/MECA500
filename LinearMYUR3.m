classdef LinearMYUR3 < handle
    properties
        %> Robot my3
        my3; 
        
        %>
        workspace = [-1 1 -1 1 -0.1 1];   
        
        %> Flag to indicate if gripper is used
        useGripper = false;        
    end
    
    methods%% Class for UR5 robot simulation
function myUR3 = LinearMYUR3(useGripper)
myUR3.GetUR3Robot(); 
myUR3.PlotAndColourRobot();%robot,workspace);
end

%% GetUR3Robot
% Given a name (optional), create and return a UR5 robot my3
function GetUR3Robot(myUR3)
%     if nargin < 1
        % Create a unique name (ms timestamp after 1ms pause)
        pause(0.001);
        name = ['LinearUR_3_',datestr(now,'yyyymmddTHHMMSSFFF')];
%     end

    % Create the UR5 my3 mounted on a linear rail
            L1 = Link('theta',pi,'a',0,'alpha',pi/2,'qlim',[-0.8 0], 'offset',0);
            L2 = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            L3 = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            L4 = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            L5 = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            L6 = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            L7 = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
    
    myUR3.my3 = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name',name);
    
    % Rotate robot to the correct orientation
    myUR3.my3.base = myUR3.my3.base * trotx(pi/2) * troty(pi/2);
end
%% PlotAndColourRobot
% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available 
function PlotAndColourRobot(myUR3)%robot,workspace)
    for linkIndex = 0:myUR3.my3.n
        if myUR3.useGripper && linkIndex == myUR3.my3.n
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['Uur3link_',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
        else
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['Uur3link_',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
        end
        myUR3.my3.faces{linkIndex+1} = faceData;
        myUR3.my3.points{linkIndex+1} = vertexData;
    end

    % Display robot
    myUR3.my3.plot3d(zeros(1,myUR3.my3.n),'noarrow','workspace',myUR3.workspace,'tile1color',[1 1 1]);
    if isempty(findobj(get(gca,'Children'),'Type','Light'))
        camlight
    end  
    myUR3.my3.delay = 0;

    % Try to correctly colour the arm (if colours are in ply file data)
    for linkIndex = 0:myUR3.my3.n
        handles = findobj('Tag', myUR3.my3.name);
        h = get(handles,'UserData');
        try 
            h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                          , plyData{linkIndex+1}.vertex.green ...
                                                          , plyData{linkIndex+1}.vertex.blue]/255;
            h.link(linkIndex+1).Children.FaceColor = 'interp';
        catch ME_1
            disp(ME_1);
            continue;
        end
    end
end        
    end
end