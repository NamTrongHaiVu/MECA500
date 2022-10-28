%% PlaceObject
classdef placetube < handle
    properties
        file;
        VertexCount;
        Verts;
        Pose;
        mesh_h;
    end
    methods
        % Function Place Object
        function self = placetube(file) %% Constructor
            self.file = file;
            [f,v,data] = plyread(file,'tri');
            self.VertexCount = size(v,1); %% Number of vertex in ply file
            midPoint = sum(v)/self.VertexCount;
            self.Verts = v - repmat(midPoint,self.VertexCount,1);
            self.Pose = eye(4);
            try
                try
                    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
                catch
                    try
                        vertexColours = [data.face.red, data.face.green, data.face.blue] / 255;
                    catch
                        vertexColours = [0.5,0.5,0.5];
                    end
                end

                    self.mesh_h = trisurf(f,self.Verts(:,1), self.Verts(:,2), self.Verts(:,3),'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');

            catch ME_1
                disp(ME_1);
            end
        end
        % Update pose
        function update(self,pose)
            self.Pose = eye(4); %% Each time move
            translation = transl(pose(1:3));
            rotateX = trotx(pose(4));
            rotateY = troty(pose(5));
            rotateZ = trotz(pose(6));
            self.Pose = self.Pose * translation *rotateX*rotateY*rotateZ;

            updatedPoints = [self.Pose * [self.Verts,ones(self.VertexCount,1)]']';  
            self.mesh_h.Vertices = updatedPoints(:,1:3);
        end
    end
end
