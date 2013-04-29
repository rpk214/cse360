function makeMap()
% Generates the map according to the map given in the assignment in the
% current figure.

points = [  -1.25   -1      ;
            -1.25   1       ;
            0       2.25    ;
            1.25    1       ;
            1.25    -1      ;
            0       -2.25   ;
            0       0       ];

hold on;
axis equal;
plot([points(1,1) points(2,1)],[points(1,2) points(2,2)],'linewidth',3);
plot([points(2,1) points(4,1)],[points(2,2) points(4,2)],'linewidth',3);
plot([points(4,1) points(5,1)],[points(4,2) points(5,2)],'linewidth',3);
plot([points(5,1) points(1,1)],[points(5,2) points(1,2)],'linewidth',3);
plot([points(1,1) points(4,1)],[points(1,2) points(4,2)],'linewidth',3);
plot([points(3,1) points(6,1)],[points(3,2) points(6,2)],'linewidth',3);


t = linspace(0, pi, 100);
r = 1.25;

x = r * cos(t);
y = r * sin(t) + 1;
plot(x,y,'linewidth',3)
plot(x,-y,'linewidth',3)