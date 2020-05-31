%%
clear; clc;
close all


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% build up the map
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x_max = 200;
y_max = 200;
z_max = 50;
map.x = x_max;
map.y = y_max;
map.z = z_max;
map.origincorner = [0;0;0];
map.endcorner = [map.x; map.y; map.z];


EPS = 20;        
Length = 1;

q_start.coord = [0 0 0];
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = [map.x-10 map.y-10 map.z-25];
q_goal.cost = 0;

nodes(1) = q_start;
figure(1)
set(gcf,'outerposition',get(0,'screensize'));
[obstacle] = mapbuild(map,q_goal,q_start);
hold on
axis([0 map.x 0 map.y 0 map.z])

numNodes = 1000;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%RRTstar
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i = 1:1:numNodes
    q_rand = [rand(1)*x_max rand(1)*y_max rand(1)*z_max];
    plot3(q_rand(1), q_rand(2), q_rand(3), '*', 'Color',  [0 0.4470 0.7410])
    
    % Break if goal node is already reached
    for j = 1:1:length(nodes)
        if nodes(j).coord == q_goal.coord
            break
        end
    end
    
    % Pick the closest node from existing list to branch out from
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist_3d(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx);
    
    % steering function
    q_new.coord = steer3d(q_rand, q_near.coord, val, EPS);
    
    [~, node_in_obs] = collision(q_near.coord, q_rand, map, obstacle, Length);
    
    if collision(q_near.coord, q_rand, map, obstacle, Length) == 0
        line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], [q_near.coord(3), q_new.coord(3)], 'Color', 'b', 'LineWidth', 2);
        drawnow
        %hold on
        q_new.cost = dist_3d(q_new.coord, q_near.coord) + q_near.cost;
    
        % Within a radius of r, find all existing nodes
        q_nearest = [];
        lamda = 200;
        r = lamda*((log(numNodes)/numNodes)^(1/3));
        neighbor_count = 1;
        for j = 1:1:length(nodes)
            if (dist_3d(nodes(j).coord, q_new.coord)) <= r
                q_nearest(neighbor_count).coord = nodes(j).coord;
                q_nearest(neighbor_count).cost = nodes(j).cost;
                neighbor_count = neighbor_count+1;
            end
        end
    
        % Initialize cost to currently known value
        q_min = q_near;
        C_min = q_new.cost;
    
        % Iterate through all nearest neighbors to find alternate lower
        % cost paths
    
        for k = 1:1:length(q_nearest)
            if q_nearest(k).cost + dist_3d(q_nearest(k).coord, q_new.coord) < C_min
                q_min = q_nearest(k);
                C_min = q_nearest(k).cost + dist_3d(q_nearest(k).coord, q_new.coord);
                line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], [q_min.coord(3), q_new.coord(3)], 'Color', 'y');            
                %hold on
            end
        end
    
        % Update parent to least cost-from node
        for j = 1:1:length(nodes)
            if nodes(j).coord == q_min.coord
                q_new.parent = j;
            end
        end
    
        % Append to nodes
        nodes = [nodes q_new];
    end
end

D = [];
for j = 1:1:length(nodes)
    tmpdist = dist_3d(nodes(j).coord, q_goal.coord);
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
[val, idx] = min(D);
q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes q_goal];
bestline.x = [];
bestline.y = [];
bestline.z = [];
while q_end.parent ~= 0
    start = q_end.parent;
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], [q_end.coord(3), nodes(start).coord(3)], 'Color', 'r', 'LineWidth', 2);
    bestline.x = [bestline.x, q_end.coord(1)];
    bestline.y = [bestline.y, q_end.coord(2)];
    bestline.z = [bestline.z, q_end.coord(3)];
    hold on
    q_end = nodes(start);

end
bestline.x = [0,fliplr(bestline.x)];
bestline.y = [0,fliplr(bestline.y)];
bestline.z = [0,fliplr(bestline.z)];
bestline;

save obs.mat obstacle

% sample the trajectory
dt=0.02;
t_seg = 1;
samples = t_seg/dt;

sample_xl = [];
sample_yl = [];
sample_zl = [];

for k = 1 : length(bestline.x)-1
    sampled_x = linspace(bestline.x(k), bestline.x(k+1), samples);
    sampled_y = linspace(bestline.y(k), bestline.y(k+1), samples);
    sampled_z = linspace(bestline.z(k), bestline.z(k+1), samples);
    sample_xl = [sample_xl, sampled_x];
    sample_yl = [sample_yl, sampled_y];
    sample_zl = [sample_zl, sampled_z];
end
sample_path = [sample_xl; sample_yl; sample_zl];

save bestline.mat % save the trajectory data

% simulation of trajactory (realy slow)
% for i = 1:length(sample_xl)
%     c =[sample_xl(i), sample_yl(i), sample_zl(i)];
%     w = c + [0, 10, 0];
%     e = c + [0, -10, 0];
%     n = c + [10, 0 ,0];
%     s = c + [-10, 0, 0];
%     shape = [c; w; e; n; s];
% 
% 
%     h_r1 = plot3([w(1) e(1)],[w(2) e(2)],[w(3) e(3)],'Color', 'r', 'LineWidth', 3);
%     hold on
%     h_r2 = plot3([n(1) s(1)],[n(2) s(2)],[n(3) s(3)],'Color', 'r', 'LineWidth', 3);
%     hold on
%     h_w = plot3(w(1), w(2), w(3),'o','LineWidth', 2, 'MarkerSize',3, 'MarkerEdgeColor', [1 1 0]);
%     hold on
%     h_e = plot3(e(1), e(2), e(3),'o','LineWidth', 2, 'MarkerSize',3, 'MarkerEdgeColor', [1 1 0]);
%     hold on
%     h_n = plot3(n(1), n(2), n(3),'o','LineWidth', 2, 'MarkerSize',3, 'MarkerEdgeColor', [1 1 0]);
%     hold on
%     h_s = plot3(s(1), s(2), s(3),'o','LineWidth', 2, 'MarkerSize',3, 'MarkerEdgeColor', [1 1 0]);
%     hold on
%     h_c = plot3(c(1), c(2), c(3),'o','LineWidth', 2, 'MarkerSize',5, 'MarkerEdgeColor', [1 1 0], 'MarkerFaceColor', [1 1 0]);
%     hold on
%     %droneplot(sample_xl(i), sample_yl(i), sample_zl(i));
%     %h = plot3(sample_xl(i), sample_yl(i), sample_zl(i),'o','Color','g','MarkerSize',5,'MarkerFaceColor',[0 1 0], 'MarkerEdgeColor', [0 1 0]);
%     pause(10e-6)
%     delete(h_r1);
%     delete(h_r2);
%     delete(h_w);
%     delete(h_e);
%     delete(h_s);
%     delete(h_n);
%     delete(h_c);
% end
% axis image;
%% plot simulation in separate figrue;

figure;
set(gcf,'outerposition',get(0,'screensize'));

plot3(q_goal.coord(1),q_goal.coord(2),q_goal.coord(3),'o','Color','r','MarkerSize',10,'MarkerFaceColor',[1 0 0], 'MarkerEdgeColor', [1 0 0]);
hold on
plot3(q_start.coord(1),q_start.coord(2),q_start.coord(3),'o','Color','g','MarkerSize',10,'MarkerFaceColor',[0 1 0], 'MarkerEdgeColor', [0 1 0])
hold on

for i =1:obstacle.number   
    [X,Y,Z] = cylinder(obstacle.radius(i),100);
    Z(2,:) = obstacle.height(i);
    surf(X+obstacle.x(i), Y+obstacle.y(i), Z, 'LineStyle', 'none', 'EdgeColor', 'none', 'FaceColor', 'interp');
    Xi = X+obstacle.x(i);
    Yi = Y+obstacle.y(i);
    colormap(gray)
    hold on
    fill3(Xi(1,:),Yi(1,:),Z(1,:),'w')
    fill3(Xi(2,:),Yi(2,:),Z(2,:),'w')
    
    movievector_1(i) = getframe;
end


line(sample_xl, sample_yl, sample_zl, 'Color', 'y', 'LineWidth', 2)
hold on


for i = 1:length(sample_xl)
    c =[sample_xl(i), sample_yl(i), sample_zl(i)];
    w = c + [0, 10, 0];
    e = c + [0, -10, 0];
    n = c + [10, 0 ,0];
    s = c + [-10, 0, 0];
    shape = [c; w; e; n; s];


    h_r1 = plot3([w(1) e(1)],[w(2) e(2)],[w(3) e(3)],'Color', 'r', 'LineWidth', 3);
    hold on
    h_r2 = plot3([n(1) s(1)],[n(2) s(2)],[n(3) s(3)],'Color', 'r', 'LineWidth', 3);
    hold on
    h_w = plot3(w(1), w(2), w(3),'o','LineWidth', 2, 'MarkerSize',3, 'MarkerEdgeColor', [1 1 0]);
    hold on
    h_e = plot3(e(1), e(2), e(3),'o','LineWidth', 2, 'MarkerSize',3, 'MarkerEdgeColor', [1 1 0]);
    hold on
    h_n = plot3(n(1), n(2), n(3),'o','LineWidth', 2, 'MarkerSize',3, 'MarkerEdgeColor', [1 1 0]);
    hold on
    h_s = plot3(s(1), s(2), s(3),'o','LineWidth', 2, 'MarkerSize',3, 'MarkerEdgeColor', [1 1 0]);
    hold on
    h_c = plot3(c(1), c(2), c(3),'o','LineWidth', 2, 'MarkerSize',5, 'MarkerEdgeColor', [1 1 0], 'MarkerFaceColor', [1 1 0]);
    hold on
    
    pause(10e-3)
    movievector_2(i) = getframe;
    delete(h_r1);
    delete(h_r2);
    delete(h_w);
    delete(h_e);
    delete(h_s);
    delete(h_n);
    delete(h_c);
    
end

axis image;

movievector = [movievector_1, movievector_2];
clear myWriter
delete RRT.avi
myWriter = VideoWriter('RRT');
myWriter.FrameRate = 20;
open(myWriter)
writeVideo(myWriter,movievector)
close(myWriter);

% %% trajectory tracking
% 
% %load bestline.mat
% 
% figure;
% % path_data = extractfield(nodes, 'coord');
% % path_data = reshape(path_data', 3,length(path_data)/3);
% d_x = diff(bestline.x);
% d_y = diff(bestline.y);
% d_z = diff(bestline.z);
% d_cood = [d_x;d_y;d_z];
% L = 0;
% dL = [];
% for i = 1: length(d_x)
%     d = sqrt(d_x(i).^2 + d_y(i).^2 + d_z(i).^2);
%     dL = [dL; d]; % length between each two points
%     L = L+d;    % total length of trajectory
% end
% 
% 
% Xl = [bestline.x', bestline.y', bestline.z'];
% 
% 
% % simulation paraments set up
% stime=t_seg*length(dL);
% loop=stime/dt;
% 
% % flocking paraments set up
% d=3;
% n=1;
% 
% % init state
% s=zeros(12,n);
% s(1:3,:)=unifrnd(-0,0,[3,n]); %[0;0;0]
% s(4:6,:)=unifrnd(-0,0,[3,n]); %[0;0;0]
% s(7,:)=unifrnd(-0.0*pi,0.0*pi,[1,n]);
% s(8,:)=unifrnd(-0.0*pi,0.0*pi,[1,n]);
% s(9,:)=unifrnd(-0.0*pi,0.0*pi,[1,n]);
% x=s(1);y=s(2);z=s(3);
% vx=s(4);vy=s(5);vz=s(6);
% phi=s(7);theta=s(8);psi=s(9);
% vphi=s(10);vtheta=s(11);vpsi=s(12);
% 
% % public virtual leadr init
% xl=Xl(1,:)';
% v=[0;0;0];
% 
% 
% %parameters for quadrotor
% para.g=9.8;
% para.m=1.2;
% para.Iy=0.05;
% para.Ix=0.05;
% para.Iz=0.1;
% para.b=10^-4; % kF
% para.l=1; % length
% para.d=10^-6; % kM
% para.Jr=0.01;
% para.k1=0.01;
% para.k2=0.01;
% para.k3=0.01;
% para.k4=0.1;
% para.k5=0.1;
% para.k6=0.1;
% para.omegaMax=430;
% 
% % history capture
% xyHis=zeros(d,n+1,loop+1);
% xyHis(:,:,1)=[xl s(1:3)];
% 
% %simulation start
% hwait=waitbar(0,'simulating');
% 
% sp=1;
% omegaHis=zeros(4,loop);
% 
% acceleration = [];
% 
% for i = 1:length(d_cood)
%     a = 2.*(d_cood(:,i) - v*t_seg)./(t_seg^2); 
%     v = v + a.*t_seg;
%     acceleration = [acceleration, a];
% end
% 
% 
% vl = [0;0;0];
% for t=1:loop
%     
%     %leader information generator
%     for j = 0:length(acceleration)-1
%         if samples*(j)<t<=samples*(j+1)
%             al = acceleration(:,j+1)-vl;
%         end
%     end
%         
%     vl = vl+dt*al;   
%     xl = sample_path(:,t);
%     %xl=xl+dt*vl;
%     
%     % get motor speeds from the controller
%     omega=quadrotor_controller(s,xl,vl,0,para,10,0.01);
%     
%     %record the speeds
%     omegaHis(:,t)=omega;
%     
%     %send speeds of four motors to quadrotor and get its state
%     s=quadrotor_kinematics(s,omega,para,dt);
%     
%     %recodrd the position of quadrotor at time t/loop*stime
%     xyHis(:,:,t+1)=[xl s(1:3)];
%     
%     waitbar(t/loop,hwait,'simulating...');
% 
% end
% 
% close(hwait);
% %show the animation of the flight process
% figure;
% plotHis3(xyHis,dt,-1,200)
% axis equal
% grid on
% 
