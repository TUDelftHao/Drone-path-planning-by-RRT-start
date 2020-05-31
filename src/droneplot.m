function [shape] = droneplot(x, y, z)

c =[x,y,z];
w = c + [0, 15, 0];
e = c + [0, -15, 0];
n = c + [15, 0 ,0];
s = c + [-15, 0, 0];
shape = [c; w; e; n; s];


plot3([w(1) e(1)],[w(2) e(2)],[w(3) e(3)],'Color', 'r', 'LineWidth', 3)
hold on
plot3([n(1) s(1)],[n(2) s(2)],[n(3) s(3)],'Color', 'r', 'LineWidth', 3)
hold on
plot3(w(1), w(2), w(3),'o','LineWidth', 2, 'MarkerSize',2, 'MarkerEdgeColor', [1 1 0])
hold on
plot3(e(1), e(2), e(3),'o','LineWidth', 2, 'MarkerSize',2, 'MarkerEdgeColor', [1 1 0])
hold on
plot3(n(1), n(2), n(3),'o','LineWidth', 2, 'MarkerSize',2, 'MarkerEdgeColor', [1 1 0])
hold on
plot3(s(1), s(2), s(3),'o','LineWidth', 2, 'MarkerSize',2, 'MarkerEdgeColor', [1 1 0])
hold on
plot3(c(1), c(2), c(3),'o','LineWidth', 2, 'MarkerSize',5, 'MarkerEdgeColor', [1 1 0], 'MarkerFaceColor', [1 1 0])
hold on
end

