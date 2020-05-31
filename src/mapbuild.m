function [obstacle] = mapbuild(map,q_goal,q_start)
r = 5:8; % radius of obstacle
height = 0.2*map.z : 10: 0.9*map.z;
obstacle.number = 15;

obs_x = linspace(20,(map.x - 20),obstacle.number+2);
obs_y = linspace(20,(map.y - 20),obstacle.number+2);
index_x = randperm(length(obs_x));
index_y = randperm(length(obs_y));

obstacle.x = [];
obstacle.y = [];
obstacle.z = [];
obstacle.radius = [];
obstacle.height = [];

plot3(q_goal.coord(1),q_goal.coord(2),q_goal.coord(3),'o','Color','r','MarkerSize',10,'MarkerFaceColor',[1 0 0], 'MarkerEdgeColor', [1 0 0]);
hold on
plot3(q_start.coord(1),q_start.coord(2),q_start.coord(3),'o','Color','g','MarkerSize',10,'MarkerFaceColor',[0 1 0], 'MarkerEdgeColor', [0 1 0]);
hold on

for i =1:obstacle.number
    l_r = length(r);
    index_r = randperm(l_r);
    l_h = length(height);
    index_h = randperm(l_h);
    
    [X,Y,Z] = cylinder(r(index_r(1)),100);
    Z(2,:) = height(index_h(1));
    surf(X+obs_x(index_x(i+1)), Y+obs_y(index_y(i+1)), Z, 'LineStyle', 'none', 'EdgeColor', 'none', 'FaceColor', 'interp');
    Xi = X+obs_x(index_x(i+1));
    Yi = Y+obs_y(index_y(i+1));
    %colormap('summer')
    colormap(gray)
    hold on
    fill3(Xi(1,:),Yi(1,:),Z(1,:),'w')
    fill3(Xi(2,:),Yi(2,:),Z(2,:),'w')
    
    
    obstacle.x = [obstacle.x; obs_x(index_x(i+1))];
    obstacle.y = [obstacle.y; obs_y(index_y(i+1))];
    obstacle.z = [obstacle.z; height(index_h(1))];
    obstacle.radius = [obstacle.radius; r(index_r(1))];
    obstacle.height = [obstacle.height; height(index_h(1))];
end
end

