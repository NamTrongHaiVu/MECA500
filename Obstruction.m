function Obstruction(vertex,faces,faceNormals)
    centerpnt = [0,0.26,0.1];
    side = 0.27;
    plotOptions.plotFaces = true;
    [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);
end
