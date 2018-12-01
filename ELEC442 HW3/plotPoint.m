function [] = plotPoint(origin, j, k)
%plotPoint
%   Summary: Plots a single point, along with its j, and k unit vectors,
%   based on specifications 
%   Input: Point origin, j vector, k vector
%   Output: Graph showing point and vectors
    

    plot3(origin(1), origin(2), origin(3),'*');
    xlabel('x');
    ylabel('y');
    zlabel('z');
    quiver3(origin(1), origin(2), origin(3), k(1), k(2), k(3),50,'green'); 
    quiver3(origin(1), origin(2), origin(3), j(1), j(2), j(3),50,'red');
    xlim([-700 700]);
    ylim([-700 700]);
    zlim([-500 500]);
    view([-37.5 -30]);

    

end

