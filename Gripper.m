classdef Gripper < handle
    properties     
        robotiq;
        workspace = [-0.2 0.2 -0.2 0.2 -0.1 0.2];
    end
    methods
        %% Main run
        function gripper = Gripper(tool)
            if 0 < nargin
                if length(tool) ~=2
                    error('cell')
                end 
                gripper.toolModelFilename = tool{1};
                gripper.toolParametersFilenamure = tool{2};
            end

            gripper.DH();
            gripper.Meat();

            drawnow
        end
        %% DH
        function DH(gripper)
            pause(0.001);
            L1 = Link('theta',0,'a',0,'alpha',pi,'qlim',[-0.018 0], 'offset',0);

            gripper.robotiq = SerialLink([L1]);
        end

        %% Meat
        function Meat(gripper)
            for linkIndex = 0:gripper.robotiq.n
                [faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Gripper1_',num2str(linkIndex),'.ply'],'tri');         
                gripper.robotiq.faces{linkIndex + 1} = faceData;
                gripper.robotiq.points{linkIndex + 1} = vertexData;
            end

            gripper.robotiq.plot3d(zeros(1,gripper.robotiq.n),'noarrow','workspace',gripper.workspace,'tile1color',[1 1 1]);

            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end

            gripper.robotiq.delay = 0;

        end
    end
end    