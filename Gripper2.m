classdef Gripper2 < handle
    properties
        robotiq2;
        workspace = [-0.2 0.2 -0.2 0.2 -0.1 0.2];
    end
    methods
        %% Main 
        function gripper2 = Gripper2(tool2)
             if 0 < nargin
                if length(tool) ~=2
                    error('cell');
                end 
                gripper2.toolModelFilename = tool2{1};
                gripper2.toolParametersFilenamure = tool2{2};
            end

            gripper2.DH2();
            gripper2.Meat2();
        end
        %% DH parameters
        function DH2(gripper2)
            L1 = Link('theta',0,'a',0,'alpha',pi,'qlim',[-0.018 0], 'offset',0);

            gripper2.robotiq2 = SerialLink([L1]);
        end 

        %% Body
        function Meat2(gripper2)
            for linkIndex = 0:gripper2.robotiq2.n
                [faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Gripper2_',num2str(linkIndex),'.ply'],'tri');         
                gripper2.robotiq2.faces{linkIndex + 1} = faceData;
                gripper2.robotiq2.points{linkIndex + 1} = vertexData;
            end

            gripper2.robotiq2.plot3d(zeros(1,gripper2.robotiq2.n),'noarrow','workspace',gripper2.workspace,'tile1color',[1 1 1]);

            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end

            gripper2.robotiq2.delay = 0;

        end
    end
end

