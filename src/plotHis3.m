function plotHis3(xHis,dt,tailLength,speed,tt)
if nargin==2
    tailLength=-1;
    speed=-1;
    tt=-1;
elseif nargin==3
    speed=-1;
    tt=-1;
elseif nargin==4
    tt=-1;
end
[~,~,loop]=size(xHis);
ts=dt*(loop-1);
if tailLength>loop || tailLength<0
    tailLength=loop;
end
if speed>loop || speed<0
    speed=loop;
end
xyHis=[];
skip=round(loop/speed);
if skip>1
    for k=1:skip:loop
        if k==1
            xyHis=xHis(:,:,1);
        else
            xyHis(:,:,end+1)=xHis(:,:,k);
        end
    end
    if k~=loop
        xyHis(:,:,end+1)=xHis(:,:,end);
    end
else
    xyHis=xHis;
end
[~,n,loop]=size(xyHis);
amin=min(min(min(xyHis(1,:,:))));
amax=max(max(max(xyHis(1,:,:))));
bmin=min(min(min(xyHis(2,:,:))));
bmax=max(max(max(xyHis(2,:,:))));
cmin=min(min(min(xyHis(3,:,:))));
cmax=max(max(max(xyHis(3,:,:))));
aw=amax-amin;
if aw==0
    aw=1;
end
bw=bmax-bmin;
if bw==0
    bw=1;
end
amin=amin-0.05*aw;
amax=amax+0.05*aw;
bmin=bmin-0.05*bw;
bmax=bmax+0.05*bw;
xyz=reshape(xyHis,3*n,loop);
hwait=waitbar(0,'simulating');
for k=1:loop
    plot3(xyHis(1,:,1),xyHis(2,:,1),xyHis(3,:,1),'kx')
    hold on
    plot3(xyHis(1,1,k),xyHis(2,1,k),xyHis(3,1,k),'rp')
    plot3(xyHis(1,2:end,k),xyHis(2,2:end,k),xyHis(3,2:end,k),'bo', 'MarkerFaceColor','b','MarkerSize',5)
    
    tail=tailLength;
    if k<=tailLength || tailLength<0
        plot3(xyz(1,1:k),xyz(2,1:k),xyz(3,1:k),'m--')
    else
        plot3(xyz(1,k-tail:k),xyz(2,k-tail:k),xyz(3,k-tail:k),'m--')
    end
    for i=2:n
        if k<=tailLength || tailLength<0
            plot3(xyz(3*i-2,1:k),xyz(3*i-1,1:k),xyz(3*i,1:k),'c-')
        else
            plot3(xyz(3*i-2,k-tail:k),xyz(3*i-1,k-tail:k),xyz(3*i,k-tail:k),'c-')
        end
    end
    hold off
    title(['time=',num2str(k/loop*ts)]);
    
    axis([amin amax bmin bmax cmin cmax]);
    if tt>0
        if k/loop*ts>=tt
            break
        end
    end

      
    
%     axis equal
    %pause(0.1)
    waitbar(k/loop,hwait,'simulating');
end
close(hwait);
xlabel('x')
ylabel('y')
zlabel('z')
