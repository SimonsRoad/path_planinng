function rgbcube()
    vertices = [0 0 0;0 0 1;0 1 0;0 1 1;1 0 0;1 0 1;1 1 0;1 1 1];
    faces = [1 5 6 2;1 3 7 5;1 2 4 3;2 4 8 6;3 7 8 4;5 6 8 7];
    colors = vertices;
    patch('Vertices', vertices, 'Faces', faces, ...
        'FaceVertexCData', colors, 'FaceColor', 'interp', ...
        'EdgeAlpha', 0)
    xlabel('red');
    ylabel('green');
    zlabel('blue');

    rotate3d on
