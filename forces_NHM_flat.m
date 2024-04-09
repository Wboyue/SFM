function [ F1,F2 ] = forces_NHM(X)

global n_groups theta
global N 
global r 
global m v0Mu v0Md
global v0  
global tau pp
global A 
global B 
global Aw
% global Bw
global k1 
global k2 
global e_act tho
global e_ind 
global e_seq 
% global e_n 
global num_walls lemad
global map_walls nvector 


% X decomposition
position=zeros(N,3);%当前位置
vel=zeros(N,3);     %瞬时速度矢量
% ev=zeros(N,3);      %瞬时速度的方向矢量
for i=1:N
   position(i,:)=[X(6*i-5) X(6*i-4) X(6*i-3)];    %将X中的行人位置信息复制到position变量下（表示行人当前的位置）
   vel(i,:)=[X(6*i-2) X(6*i-1) X(6*i)];
%    ev(i,:)=vel(i,:)/norm(vel(i,:));
end

%Determination of the current waypoints
%%%%%%这一段的目的是更新期望速度的方向，设定最终的目标点是（2.5,1000）以及（2.5,-1000）
for i=1:n_groups(1)
         e_act{1}(i,:)=e_seq{1}(:,e_ind{1}(i));
         e(i,:)=(e_act{1}(i,:)'-position(i,:)')/norm(e_act{1}(i,:)'-position(i,:)');

end
for n=2:length(n_groups)
    for i=1:n_groups(n)
            e_act{n}(i,:)=e_seq{n}(:,e_ind{n}(i));
            e(sum(n_groups(1:n-1))+i,:)=(e_act{n}(i,:)'-position(sum(n_groups(1:n-1))+i,:)')/norm(e_act{n}(i,:)'-position(sum(n_groups(1:n-1))+i,:)');
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fi0=zeros(N,3); % velocity force

% Interindividual forces f_ij^p中的3项
fij1=zeros(N,3);% repulsive排斥项
fij2=zeros(N,3);% compression
fij3=zeros(N,3);% friction

% Obstacles   f_iw^w中的3项
fiw1=zeros(N,3);% repulsive
fiw2=zeros(N,3);% compression
fiw3=zeros(N,3);% friction

fig=zeros(N,3);%重力驱动力
%fivs=zeros(N,3);%下降速度心理力
% Groups?凝聚力？?
%fgroup=zeros(N,2);
% e=([1 0]'*ones(1,N))';

for k=1:length(n_groups)%遍历所有的小组
    for i=sum(n_groups(1:k))-n_groups(k)+1:sum(n_groups(1:k))%遍历第k小组中的每一个人
        videl=v0(i)*e(i,:);
       % e=videl./norm(videl);
        count=0;
        vaverage=[0 0 0];
        for j=1:N
            %A ~= B 返回一个逻辑数组，当数组 A 和 B 不相等时，其对应设置上的元素设为逻辑值 1 (true)；否则设为逻辑值 0 (false)。
            if i~=j     %i不等于j时，进行下面的计算
                rij=r(i)+r(j);
                dij=norm(position(i,:)-position(j,:));%position是行人的当前位置
                nij=(position(i,:)'-position(j,:)')/dij;
              
%                 if (norm(vel(i,:)))==0
%                 costheta=dot(-nij,e(i,:))./(norm(e(i,:))*norm(-nij));
%                 else
%                 costheta=dot(-nij,vel(i,:))./(norm(vel(i,:))*norm(-nij));
%                 end
                 fij1(i,:)=fij1(i,:)'+A*exp((rij-dij)/B)*nij; 
             %   fij1(i,:)=fij1(i,:)'+A*exp((rij-dij)/B)*nij*(lemad+(1-lemad)*((1+costheta)/2));
             %   %这条代码导致程序似乎停滞了，为什么？

                if dij<rij%如果行人ij之间的距离小于两人的半径和（现实中不允许发生的）,此时出现fij2，fij3
                    fij2(i,:)=fij2(i,:)'+k1*(rij-dij)*nij;
%                     tij=[-nij(2) nij(1)]';
                  %  tij=cross(nij',nvector);
                   tij=cross(nvector,nij');
%                     dvij=(vel(j,:)'-vel(i,:)')'*tij;%点积
                    vd=vel(j,:)-vel(i,:);
                    dvij=dot(vd,tij);
                    fij3(i,:)=fij3(i,:)+k2*(rij-dij)*dvij*tij;
                end 

                if  dij<=2  %对于上行行人，如果j在i2米范围内，则j对i的期望速度有影响
                    vaverage=vaverage+vel(j,:);
                    count=count+1;
                end 
%                 if k==2 && dij<=4  %对于下行行人，如果j在i4米范围内，则j对i的期望速度有影响
%                     vaverage=vaverage+vel(j,:);
%                     count=count+1;
%                 end 
            end
        end
        
        if count~=0

        vaverage=vaverage./count;
%         if k==1 %更新上行行人的期望速度
%             videl=(1-0.4*tho)*(1-sin(theta))*((1+cos(theta))/2)*videl+0.4*tho*vaverage ;
%         end
%         if k==2%跟新下行行人的期望速度
%             videl=(1-0.4*tho)*(1+sin(theta))*((1+cos(theta))/2)*videl+0.4*tho*vaverage ;
%         end
        if k==1 %更新上行行人的期望速度
            videl=(1-pp)*v0Mu*e(i,:)+pp*vaverage ;
        end
        if k==2%跟新下行行人的期望速度
            videl=(1-pp)*v0Md*e(i,:)+pp*vaverage ;
        end
%         if k==1 %更新上行行人的期望速度
%             videl=(1-0.4*tho)*(1-sin(theta))*videl+0.4*tho*vaverage ;
%         end
%         if k==2%跟新下行行人的期望速度
%             videl=(1-0.4*tho)*(1+sin(theta))*videl+0.4*tho*vaverage ;
%         end
        end

        ee=videl./norm(videl);%ee是目前的期望速度方向
        d=3;
        pd=2.5;
        for j=1:N
            %A ~= B 返回一个逻辑数组，当数组 A 和 B 不相等时，其对应设置上的元素设为逻辑值 1 (true)；否则设为逻辑值 0 (false)。
            
            if i~=j     %i不等于j时，进行下面的计算
                pij=position(i,:)-position(j,:);
                xx=norm(pij)*sqrt(1-((dot(pij,ee)./(norm(pij)*norm(ee)))).^2);
                if xx==r(j)
                    pd=sqrt(norm(pij).^2-xx.^2);
                end
                if xx<r(j)                  %如果j会与i的期望速度相交                 
                    pd=norm(pij)*(dot(pij,ee)./(norm(pij)*norm(ee)))-sqrt((r(j)).^2-xx.^2);%pd即为行人i到与行人j相交点的距离
                end
                if pd<=1.25 && pd<d && k==1    %对于上行行人，触须长度为1米，当行人j在i的观察范围内
                    d=pd;               %更新最近的交点的距离
                    count=j;            %记录下相交最近的行人编号
                end 
                if pd<=2 && pd<d && k==2    %对于下行行人，触须长度为2米，当行人j在i的观察范围内
                    d=pd;               %更新最近的交点的距离
                    count=j;            %记录下相交最近的行人编号
                end 
            end
            
        end
        if d~=3  %d不等于3说明有行人在1.25米范围内与期望速度方向相交
            if (k==1 && count>(N/2)) %上行行人与异向行人的情况

                if ee(1)~=0
                if ee(2)/ee(1)>0 
                if position(count,2)>=((ee(2)/ee(1))*position(count,1)+position(i,2)-(ee(2)/ee(1))*position(i,1))
                    %如果行人在（上行行人）靠左边或中间，则上行行人向右转期望速度,
                    ee(1:2)=ee(1:2)*[cos(-pi/6) sin(-pi/6);-sin(-pi/6) cos(-pi/6)];%矢量右转30度
                else %上行行人向左转期望速度
                    ee(1:2)=ee(1:2)*[cos(pi/6) sin(pi/6);-sin(pi/6) cos(pi/6)];%矢量左转30度
                end     
                end
                if ee(2)/ee(1)<0 
                if position(count,2)>((ee(2)/ee(1))*position(count,1)+position(i,2)-(ee(2)/ee(1))*position(i,1))
                    %如果行人在（上行行人）靠右边，则上行行人向左转期望速度,
                     ee(1:2)=ee(1:2)*[cos(pi/6) sin(pi/6);-sin(pi/6) cos(pi/6)];%矢量左转30度
                else 
                     ee(1:2)=ee(1:2)*[cos(-pi/6) sin(-pi/6);-sin(-pi/6) cos(-pi/6)];%矢量右转30度
                end   
                end
                end

                if ee(1)==0
                    if position(i,1)>=position(count,1)
                       ee(1:2)=ee(1:2)*[cos(-pi/6) sin(-pi/6);-sin(-pi/6) cos(-pi/6)];%矢量右转30度  
                    else
                        ee(1:2)=ee(1:2)*[cos(pi/6) sin(pi/6);-sin(pi/6) cos(pi/6)];%矢量左转30度
                    end
                end


            elseif ((k==2) && count<=(N/2)) %下行行人与异向行人的情况

                if ee(1)~=0
                    if ee(2)/ee(1)>0
                    if position(count,2)>((ee(2)/ee(1))*position(count,1)+position(i,2)-(ee(2)/ee(1))*position(i,1))
                    %如果行人在（下行行人）靠右侧，则下行行人向左转期望速度,
                    ee(1:2)=ee(1:2)*[cos(pi/6) sin(pi/6);-sin(pi/6) cos(pi/6)];%矢量左转30度
                    else %下行行人向右转期望速度
                    ee(1:2)=ee(1:2)*[cos(-pi/6) sin(-pi/6);-sin(-pi/6) cos(-pi/6)];%矢量右转30度
                    end
                    end

                    if ee(2)/ee(1)<0
                    if position(count,2)>=((ee(2)/ee(1))*position(count,1)+position(i,2)-(ee(2)/ee(1))*position(i,1))
                    %如果行人在（下行行人）靠右侧，则下行行人向左转期望速度,
                    ee(1:2)=ee(1:2)*[cos(-pi/6) sin(-pi/6);-sin(-pi/6) cos(-pi/6)];%矢量右转30度
                    else %下行行人向右转期望速度
                    ee(1:2)=ee(1:2)*[cos(pi/6) sin(pi/6);-sin(pi/6) cos(pi/6)];%矢量左转30度
                    end
                    end

                end

                  if ee(1)==0
                    if position(i,1)>position(count,1)
                       ee(1:2)=ee(1:2)*[cos(pi/6) sin(pi/6);-sin(pi/6) cos(pi/6)];%矢量左转30度
                    else
                       ee(1:2)=ee(1:2)*[cos(-pi/6) sin(-pi/6);-sin(-pi/6) cos(-pi/6)];%矢量右转30度
                    end
                  end

            else %行人同向
                if d<=0.75 && k==1 %上行行人遇到同向行人

                   if ee(1)~=0
                     if ee(2)/ee(1)>0 
                     if position(count,2)>=((ee(2)/ee(1))*position(count,1)+position(i,2)-(ee(2)/ee(1))*position(i,1))
                    %如果行人在（上行行人）靠左边或中间，则上行行人向右转期望速度,
                    ee(1:2)=ee(1:2)*[cos(-pi/6) sin(-pi/6);-sin(-pi/6) cos(-pi/6)];%矢量右转30度
                      else %上行行人向左转期望速度
                    ee(1:2)=ee(1:2)*[cos(pi/6) sin(pi/6);-sin(pi/6) cos(pi/6)];%矢量左转30度
                     end     
                     end

                     if ee(2)/ee(1)<0 
                         if position(count,2)>((ee(2)/ee(1))*position(count,1)+position(i,2)-(ee(2)/ee(1))*position(i,1))
                    %如果行人在（上行行人）靠右边，则上行行人向左转期望速度,
                     ee(1:2)=ee(1:2)*[cos(pi/6) sin(pi/6);-sin(pi/6) cos(pi/6)];%矢量左转30度
                         else 
                     ee(1:2)=ee(1:2)*[cos(-pi/6) sin(-pi/6);-sin(-pi/6) cos(-pi/6)];%矢量右转30度
                          end   
                     end
                   end

                if ee(1)==0
                    if position(i,1)>=position(count,1)
                       ee(1:2)=ee(1:2)*[cos(-pi/6) sin(-pi/6);-sin(-pi/6) cos(-pi/6)];%矢量右转30度  
                    else
                        ee(1:2)=ee(1:2)*[cos(pi/6) sin(pi/6);-sin(pi/6) cos(pi/6)];%矢量左转30度
                    end
                end

                end

                if d<=0.75 && k==2 %下行行人遇到同向行人
                if ee(1)~=0
                    if ee(2)/ee(1)>0
                    if position(count,2)>((ee(2)/ee(1))*position(count,1)+position(i,2)-(ee(2)/ee(1))*position(i,1))
                    %如果行人在（下行行人）靠右侧，则下行行人向左转期望速度,
                    ee(1:2)=ee(1:2)*[cos(pi/6) sin(pi/6);-sin(pi/6) cos(pi/6)];%矢量左转30度
                    else %下行行人向右转期望速度
                    ee(1:2)=ee(1:2)*[cos(-pi/6) sin(-pi/6);-sin(-pi/6) cos(-pi/6)];%矢量右转30度
                    end
                    end

                    if ee(2)/ee(1)<0
                    if position(count,2)>=((ee(2)/ee(1))*position(count,1)+position(i,2)-(ee(2)/ee(1))*position(i,1))
                    %如果行人在（下行行人）靠右侧，则下行行人向左转期望速度,
                    ee(1:2)=ee(1:2)*[cos(-pi/6) sin(-pi/6);-sin(-pi/6) cos(-pi/6)];%矢量右转30度
                    else %下行行人向右转期望速度
                    ee(1:2)=ee(1:2)*[cos(pi/6) sin(pi/6);-sin(pi/6) cos(pi/6)];%矢量左转30度
                    end
                    end

                end

                  if ee(1)==0
                    if position(i,1)>position(count,1)
                       ee(1:2)=ee(1:2)*[cos(pi/6) sin(pi/6);-sin(pi/6) cos(pi/6)];%矢量左转30度
                    else
                       ee(1:2)=ee(1:2)*[cos(-pi/6) sin(-pi/6);-sin(-pi/6) cos(-pi/6)];%矢量右转30度
                    end
                  end
     
                end

            end
        end
        videl=norm(videl)*ee;

        %为每个行人i，更新fi0，ang，fij1、2、3，fiw1、2、3
        fi0(i,:)=m(i)*(videl'-vel(i,:)')/tau; 


        % Walls forces       
        for w=1:num_walls
            xp=position(i,1);
            yp=position(i,2);
            rp=[xp yp]';
            ra=max([map_walls(2*w-1,1) map_walls(2*w,1)]',[map_walls(2*w-1,2) map_walls(2*w,2)]'); 
            rb=min([map_walls(2*w-1,1) map_walls(2*w,1)]',[map_walls(2*w-1,2) map_walls(2*w,2)]');
            xa=ra(1);
            ya=ra(2);
            xb=rb(1);
            yb=rb(2);
            % a point on AB can be parametrized as s(t)=ra+t(tb-ta), t in [0,1]
            % distance from s to p is phi(t)=||s(t)-p||
            % d(phi^2) gives the t which minimizes the distance from p to the
            % line in which AB lives. Since t in [0,1], t_star=min(max(0,t),1);
            % and the distance from p to AB is ||s(t_star)-p||
            t=((xp-xa)*(xb-xa)+(yp-ya)*(yb-ya))/(((xb-xa)^2+(yb-ya)^2));
            t_star=min(max(0,t),1);
            rh=ra+t_star*(rb-ra);
            diw=norm(rp-rh);%行人i距离墙w的距离
            niw=(rp-rh)/norm(rp-rh);
            tiw=[-niw(2) niw(1)]';
            fiw1(i,1:2)=fiw1(i,1:2)'+Aw*exp((r(i)-diw)/B)*niw;
            if diw<r(i)
                fiw2(i,1:2)=fiw2(i,1:2)'+k1*(r(i)-diw)*niw;
                fiw3(i,1:2)=fiw3(i,1:2)'-k2*(r(i)-diw)*(vel(i,1:2)*tiw)*tiw;%这行代码和论文中的相比似乎有误
%                  fiw3(i,1:2)=fiw3(i,1:2)'-k2*(r(i)-diw)*(-vel(i,1:2)*tiw)*tiw;
            end
        end

%         重力驱动力
%         下降速度心理力
     
%         vz=abs(vel(i,3));
%         vxy=sqrt(vel(i,1).^2+vel(i,2).^2);
%         if vxy~=0
%         the=atan(vz/vxy);
%         end
%         if vxy==0
%             the=0;
%         end
%         if the==0
%             fig(i,:)=[0 0 0];
%             fivs(i,:)=[0 0 0];
%         end
% 
%         if the~=0 && k==1
%             fig(i,:)=-9.8*m(i)*sin(the).*ev(i,:);
%            fivs(i,:)=60*9.8*exp(0.5*(norm(v0(i)*e(i,:)-vel(i,:))));
% 
%         end
% 
%         if the~=0 && k==2
            fig(i,:)=9.8*sin(theta).*[0 -1 -tan(theta)];
           % fivs(i,1:2)=(10.02+0.15*exp(6.58*((vel(i,1:2)-v0(i)*e(i,1:2)))));
% 
%         end

    end
end

% Force due to the desire to move as v0
F1=fi0;

% Other forces
% F2=fij1+fij2+fij3+fiw1+fiw2+fiw3;
F2=fij1+fij2+fij3+fiw1+fig;

end

