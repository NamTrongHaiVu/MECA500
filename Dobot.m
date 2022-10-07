classdef Dobot < handle
    properties     
        my3; %> Robot my3
        workspace = [-0.6 0.6 -0.6 0.6 -0.2 1.1];   
    end
    
    methods%% Class for Dobot robot simulation
        function dobot = Dobot(toolModelAndTCPFilenames)
            if 0 < nargin
                if length(toolModelAndTCPFilenames) ~= 2
                    error('Please pass a cell with two strings, toolModelFilename and toolCenterPointFilename');
                end
                dobot.toolModelFilename = toolModelAndTCPFilenames{1};
                dobot.toolParametersFilenamure = toolModelAndTCPFilenames{2};
            end
            
            dobot.DobotRobot();
            dobot.PlotAndColourRobot();%robot,workspace);

            drawnow
        end

        %% DobotRobot
        % Given a name (optional), create and return a Dobot robot my3
        function DobotRobot(dobot)
            pause(0.001);
            name = ['Dobot_',datestr(now,'yyyymmddTHHMMSSFFF')];
            %L1 = Link('d',0.008,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            L1 = Link('d',0.15,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            L2 = Link('d',0,'a',0.135,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            L3 = Link('d',0,'a',0.147,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            L4 = Link('d',0,'a',0.0597,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);

            dobot.my3 = SerialLink([L1 L2 L3 L4],'name',name);
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobot(dobot)%robot,workspace)
            for linkIndex = 0:dobot.my3.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['dobot_Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>                
                dobot.my3.faces{linkIndex + 1} = faceData;
                dobot.my3.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            dobot.my3.plot3d(zeros(1,dobot.my3.n),'workspace',dobot.workspace,'tile1color',[1 1 1]);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            dobot.my3.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:dobot.my3.n
                handles = findobj('Tag', dobot.my3.name);
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




