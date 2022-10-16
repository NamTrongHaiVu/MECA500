function environment()
    %Floor
surf([-3.75,-3.75;3,3],[-3,3.2;-3,3.2],[-0.75,-0.75;-0.75,-0.75],'CData',imread('floor.jpg'),'FaceColor','texturemap');
hold on
PlaceObject('table.ply',[1.7,0,0]);
PlaceObject('wall.ply',[1.5,-3,0.77]);
%PlaceObject('sit.ply',[-2.4,2,-0.7]);

axis equal

% Glasses
hold on
t1 = surf([-1.55,-1.55;-1.55,-1.55],[-0.6,0.6;-0.6,0.6],[-0.1,-0.1;1.5,1.5],'FaceColor', [0.4 0.6 0.7]);
t1.FaceAlpha = 1/2.5;
hold on
t2 = surf([1.3,1.3;1.3,1.3],[-0.6,0.6;-0.6,0.6],[0.28,0.28;1.5,1.5],'FaceColor', [0.4 0.6 0.7]);
t2.FaceAlpha = 1/2.5;
hold on
t3 = surf([0.8,1.3;0.8,1.3],[-0.63,-0.63;-0.63,-0.63],[0,0;1.4,1.4],'FaceColor',[0.4 0.6 0.7]);
t3.FaceAlpha = 1/2.5
hold on
t4 = surf([-1.5,-1;-1.5,-1],[-0.63,-0.63;-0.63,-0.63],[0,0;1.4,1.4],'FaceColor',[0.4 0.6 0.7]);
t4.FaceAlpha = 1/2.5
hold on
t5 = surf([-1,0.8;-1,0.8],[-0.63,-0.63;-0.63,-0.63],[0.3,0.3;1.4,1.4],'FaceColor',[0.4 0.6 0.7]);
t5.FaceAlpha = 1/2.5
hold on
t6 = surf([-1.5,1.3;-1.5,1.3],[0.63,0.63;0.63,0.63],[-0.1,-0.1;1.4,1.4],'FaceColor',[0.4 0.6 0.7]);
t6.FaceAlpha = 1/2.5

% Fence
hold on
f = surf([-1.8,1.9;-1.8,1.9],[1.175,1.175;1.175,1.175],[0.38,0.38;0.47,0.47],'FaceColor',[1 1 0]);
f.FaceAlpha = 1/4

% View
hold on
v1 = surf([-2.9,2.9;-2.9,2.9],[-2.9,-2.9;-2.9,-2.9],[0,0;1.8,1.8],'FaceColor',[0.4 0.6 0.7]);
v1.FaceAlpha = 1/2;
%surf([-3,3;-3,3],[-3,-3;-3,-3],[-0.6,-0.6;2,2],'CData',imread('view2.jpg'),'FaceColor','texturemap');

% Wall
tra = 1/3;
hold on
surf([-3.75,-3.75;-3.75,-3.75],[-3,3.2;-3,3.2],[-0.75,-0.75;2.24,2.24],'CData',imread('wall.jpg'),'FaceColor','texturemap');
hold on
w1 = surf([3,3;3,3],[2,3.2;2,3.2],[1.24,1.24;2.24,2.24],'FaceColor',[0.4 0.6 0.7]);
w1.FaceAlpha = tra;
hold on
w2 = surf([3,3;3,3],[-3,-2;-3,-2],[-0.75,-0.75;2.24,2.24],'FaceColor',[0.4 0.6 0.7]);
w2.FaceAlpha = tra;
hold on
w3 = surf([3,3;3,3],[-2,-1;-2,-1],[-0.75,-0.75;2.24,2.24],'FaceColor',[0.4 0.6 0.7]);
w3.FaceAlpha = tra;
hold on
w4 = surf([3,3;3,3],[-1,0;-1,0],[-0.75,-0.75;2.24,2.24],'FaceColor',[0.4 0.6 0.7]);
w4.FaceAlpha = tra;
hold on
w5 = surf([3,3;3,3],[0,1;0,1],[-0.75,-0.75;2.24,2.24],'FaceColor',[0.4 0.6 0.7]);
w5.FaceAlpha = tra;
hold on
w6 = surf([3,3;3,3],[1,2;1,2],[-0.75,-0.75;2.24,2.24],'FaceColor',[0.4 0.6 0.7]);
w6.FaceAlpha = tra;

hold on
w7 = surf([3,3;3,3],[2,3.2;2,3.2],[-0.75,-0.75;1.24,1.24],'FaceColor',[0.4 0.6 0.7]);
w7.FaceAlpha = tra;


end