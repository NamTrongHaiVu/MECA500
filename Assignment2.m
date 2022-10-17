clear all; close all; clc;
%% Environment
workspace = [-2 2 -2 2 -0.61 2]; 
environment();

%% Call Robots
r = Meca500;
r2 = myUR3;

%% Object
Obstruction();

%% Run 1
% q1 = [-pi/4,0,0,0,0,0];
% q2 = [pi/4,0,0,pi/3,0,pi/6];
% steps = 2;
% while ~isempty(find(1 < abs(diff(rad2deg(jtraj(q1,q2,steps)))),1))
%     steps = steps + 1;
% end
% qMatrix = jtraj(q1,q2,steps);
% 
% result = true(steps,1);
% for i = 1: steps
%     result(i) = IsCollision(r2.model,qMatrix(i,:),faces,vertex,faceNormals,false);
%     r2.model.animate(qMatrix(i,:));
%     drawnow
% end

%% Run 2
hold on
[f,v,data] = plyread('HalfSizedRedGreenBrick.ply','tri');
brickVertexCount = size(v,1);
midPoint = sum(v)/brickVertexCount;
brickVerts = v - repmat(midPoint,brickVertexCount,1);

brickPose(:,:,1)=transl(-0.12,-0.18,0.03)*trotx(pi);
brickPose(:,:,2)=transl(-0.07,-0.18,0.03)*trotx(pi);
brickPose(:,:,3)=transl(0.01,-0.18,0.03)*trotx(pi);
brickPose(:,:,4)=transl(-0.15,-0.04,0.03)*trotx(pi);
brickPose(:,:,5)=transl(-0.07,-0.04,0.03)*trotx(pi);
brickPose(:,:,6)=transl(0.01,-0.04,0.03)*trotx(pi);
brickPose(:,:,7)=transl(-0.15,0.1,0.03)*trotx(pi);
brickPose(:,:,8)=transl(-0.07,0.1,0.03)*trotx(pi);
brickPose(:,:,9)=transl(0.01,0.1,0.03)*trotx(pi);

vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
brickMesh_h1 = trisurf(f,brickVerts(:,1),brickVerts(:,2), brickVerts(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
updatedPoints = [brickPose(:,:,1) * [brickVerts,ones(brickVertexCount,1)]']';  
brickMesh_h1.Vertices = updatedPoints(:,1:3);

brickMesh_h2 = trisurf(f,brickVerts(:,1),brickVerts(:,2), brickVerts(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
updatedPoints = [brickPose(:,:,2) * [brickVerts,ones(brickVertexCount,1)]']';  
brickMesh_h2.Vertices = updatedPoints(:,1:3);

brickMesh_h3 = trisurf(f,brickVerts(:,1),brickVerts(:,2), brickVerts(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
updatedPoints = [brickPose(:,:,3) * [brickVerts,ones(brickVertexCount,1)]']';  
brickMesh_h3.Vertices = updatedPoints(:,1:3);

brickMesh_h4 = trisurf(f,brickVerts(:,1),brickVerts(:,2), brickVerts(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
updatedPoints = [brickPose(:,:,4) * [brickVerts,ones(brickVertexCount,1)]']';  
brickMesh_h4.Vertices = updatedPoints(:,1:3);

brickMesh_h5 = trisurf(f,brickVerts(:,1),brickVerts(:,2), brickVerts(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
updatedPoints = [brickPose(:,:,5) * [brickVerts,ones(brickVertexCount,1)]']';  
brickMesh_h5.Vertices = updatedPoints(:,1:3);

brickMesh_h6 = trisurf(f,brickVerts(:,1),brickVerts(:,2), brickVerts(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
updatedPoints = [brickPose(:,:,6) * [brickVerts,ones(brickVertexCount,1)]']';  
brickMesh_h6.Vertices = updatedPoints(:,1:3);

brickMesh_h7 = trisurf(f,brickVerts(:,1),brickVerts(:,2), brickVerts(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
updatedPoints = [brickPose(:,:,7) * [brickVerts,ones(brickVertexCount,1)]']';  
brickMesh_h7.Vertices = updatedPoints(:,1:3);

brickMesh_h8 = trisurf(f,brickVerts(:,1),brickVerts(:,2), brickVerts(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
updatedPoints = [brickPose(:,:,8) * [brickVerts,ones(brickVertexCount,1)]']';  
brickMesh_h8.Vertices = updatedPoints(:,1:3);

brickMesh_h9 = trisurf(f,brickVerts(:,1),brickVerts(:,2), brickVerts(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
updatedPoints = [brickPose(:,:,9) * [brickVerts,ones(brickVertexCount,1)]']';  
brickMesh_h9.Vertices = updatedPoints(:,1:3);

%% Place bricks to build wall
Tb(:,:,7) = transl(-0.98, 0.4, 0.115)*trotx(pi)*trotz(pi/2);
Tb(:,:,8) = transl(-0.98, 0.4, 0.145)*trotx(pi)*trotz(pi/2);
Tb(:,:,9) = transl(-0.98, 0.4, 0.175)*trotx(pi)*trotz(pi/2);
Tb(:,:,4) = transl(-0.84, 0.4, 0.115)*trotx(pi)*trotz(pi/2);
Tb(:,:,5) = transl(-0.84, 0.4, 0.145)*trotx(pi)*trotz(pi/2);
Tb(:,:,6) = transl(-0.84, 0.4, 0.175)*trotx(pi)*trotz(pi/2);
Tb(:,:,1) = transl(-0.7, 0.4, 0.115)*trotx(pi)*trotz(pi/2);
Tb(:,:,2) = transl(-0.7, 0.4, 0.145)*trotx(pi)*trotz(pi/2);
Tb(:,:,3) = transl(-0.7, 0.4, 0.175)*trotx(pi)*trotz(pi/2);

%% Run
for m =1:9

qt1 = my3.ikcon(transl(0,0,0.09)*brickPose(:,:,m),qstart);
qmiddle = [0 90 -90 0 -90 0 0]*pi/180;
qt2=my3.ikcon(Tb(:,:,m));
steps = 50;

qtest = jtraj(qstart,qt1,steps); %from start to brick
for i =1:steps
    if qtest(i,1) > 0
        qtest(i,1) =0;
    end
end
qtest2 = jtraj(qt1,qmiddle,steps); %from brick to middle trajactory
for i =1:steps
    if qtest2(i,1) > 0
        qtest2(i,1) =0;
    end
end
qtest3 = jtraj(qmiddle,qt2,steps); %from middle to the build wall
qtest4 = jtraj(qt2,qmiddle,steps);%go back to middle
for i =1:steps
    if qtest4(i,1) > 0
        qtest4(i,1) =0;
    end
end
qtest5 = jtraj(qmiddle,qstart,steps);%back to start


for i =1:steps %from start to brick (loop)
    my3.animate(qtest(i,:));
GT = my3.fkine(getpos(my3));
GR.base = GT*transl(-0.003,0.012,0.05)*troty(270,'deg')*trotx(180,'deg')*trotx(90,'deg')*transl(-0.005,-0.01,-0.013);
GR2.base = GT*transl(0.004,-0.012,0.05)*troty(-90,'deg')*trotx(90,'deg')*transl(-0.005,-0.01,-0.0165);
GR.animate(-pi/8);
GR2.animate(-pi/8);
    drawnow()
end

switch m %9 cases for each brick
          case 1
        brickMesh_h = brickMesh_h1;
          case 2
        brickMesh_h = brickMesh_h2;
          case 3
        brickMesh_h = brickMesh_h3;
          case 4
        brickMesh_h = brickMesh_h4;
          case 5
        brickMesh_h = brickMesh_h5;
          case 6
        brickMesh_h = brickMesh_h6;
          case 7
        brickMesh_h = brickMesh_h7;
          case 8
        brickMesh_h = brickMesh_h8;
          case 9
        brickMesh_h = brickMesh_h9;
end

for i =1:steps %from brick to middle trajactory with brick
    my3.animate(qtest2(i,:));
 GT = my3.fkine(getpos(my3));
GR.base = GT*transl(-0.003,0.012,0.05)*troty(270,'deg')*trotx(180,'deg')*trotx(90,'deg')*transl(-0.005,-0.01,-0.013);
GR2.base = GT*transl(0.004,-0.012,0.05)*troty(-90,'deg')*trotx(90,'deg')*transl(-0.005,-0.01,-0.0165);
GR.animate(-pi/8);
GR2.animate(-pi/8);
    ee1 = my3.fkine(qtest2(i,:))*transl(0,0,0.09); %end of effect
    updatedPoints = [ee1 * [brickVerts,ones(brickVertexCount,1)]']';  
    brickMesh_h.Vertices = updatedPoints(:,1:3);
    drawnow()
end

for i =1:steps %from middle to the build wall with brick
    my3.animate(qtest3(i,:));
GT = my3.fkine(getpos(my3));
GR.base = GT*transl(-0.003,0.012,0.05)*troty(270,'deg')*trotx(180,'deg')*trotx(90,'deg')*transl(-0.005,-0.01,-0.013);
GR2.base = GT*transl(0.004,-0.012,0.05)*troty(-90,'deg')*trotx(90,'deg')*transl(-0.005,-0.01,-0.0165);
GR.animate(-pi/8);
GR2.animate(-pi/8);
    ee = my3.fkine(qtest3(i,:))*transl(0,0,0.09);
    updatedPoints = [ee * [brickVerts,ones(brickVertexCount,1)]']';  
    brickMesh_h.Vertices = updatedPoints(:,1:3);
    drawnow()
end

for i =1:steps %go back steps
    my3.animate(qtest4(i,:));
GT = my3.fkine(getpos(my3));
GR.base = GT*transl(-0.003,0.012,0.05)*troty(270,'deg')*trotx(180,'deg')*trotx(90,'deg')*transl(-0.005,-0.01,-0.013);
GR2.base = GT*transl(0.004,-0.012,0.05)*troty(-90,'deg')*trotx(90,'deg')*transl(-0.005,-0.01,-0.0165);
GR.animate(-pi/8);
GR2.animate(-pi/8);
    drawnow()
end

for i =1:steps
    my3.animate(qtest5(i,:));
GT = my3.fkine(getpos(my3));
GR.base = GT*transl(-0.003,0.012,0.05)*troty(270,'deg')*trotx(180,'deg')*trotx(90,'deg')*transl(-0.005,-0.01,-0.013);
GR2.base = GT*transl(0.004,-0.012,0.05)*troty(-90,'deg')*trotx(90,'deg')*transl(-0.005,-0.01,-0.0165);
GR.animate(-pi/8);
GR2.animate(-pi/8);
    drawnow()
end
end