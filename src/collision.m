function [collision_flag,node_in_obs] = collision(node, parent, map, obstacle, r)
% r = length of quadrtor;

collision_flag = 0; % no collision

for i=1:3
   if (node(i)>map.endcorner(i))||(node(i)<map.origincorner(i))
       collision_flag = 1;
   end
end

node_in_obs = [];

if collision_flag == 0 
    for sigma = 0:.1:1
    p = sigma*node(1:3) + (1-sigma)*parent(1:3);
      % check each obstacle
      for i=1:obstacle.number
          if p(3) <= obstacle.z(i) 
             if (norm([p(1);p(2)]-[obstacle.x(i); obstacle.y(i)])<=1*obstacle.radius(i)+2*r)             
                collision_flag = 1;
                node_in_obs = [node_in_obs; node];
                break;
             end
          else
              collision_flag = 0;
          end
      end
    end
end


end