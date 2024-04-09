function [ FT ] = forces_NHM(X)

global n_groups theta
global N 
global r 
global m v0Mu v0Md
global v0  
global tau 
global A 
global B le  lplatf
global Aw
% global Bw
global k1 
global k2 
global e_act tho
global e_ind 
global e_seq1 e_seq2 e_seq3
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
end

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

for n=1:length(n_groups)%n=1意味着是第一组的行人，n=2是第二组的行人
for i=sum(n_groups(1:n))-n_groups(n)+1:sum(n_groups(1:n))

if     position(i,2)<le                                        %如果第i个行人的y坐标小于le，则用第一段算法
%Determination of the current waypoints
%%%%%%这一段的目的是更新期望速度的方向，设定最终的目标点是（2.5,1000）以及（2.5,-1000）

    e_act{n}(i,:)=e_seq1{n};
    e(i,:)=(e_act{n}(i,:)'-position(i,:)')/norm(e_act{n}(i,:)'-position(i,:)');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if n==1
v0(i)=v0Mu;
elseif n==2
 v0(i)=v0Md;
end
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
              

                 fij1(i,:)=fij1(i,:)'+A*exp((rij-dij)/B)*nij; 
    

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

            end
        end
        
        if count~=0

        vaverage=vaverage./count;

        if n==1 %更新上行行人的期望速度
            videl=0.8*v0Mu*e(i,:)+0.2*vaverage ;
        end
        if n==2%跟新下行行人的期望速度
            videl=0.8*v0Md*e(i,:)+0.2*vaverage ;
        end

        end

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

            fig(i,:)=9.8*sin(theta).*[0 -1 -tan(theta)];

            FT(i,:)=fi0(i,:)+fij1(i,:)+fij2(i,:)+fij3(i,:)+fiw1(i,:)+fig(i,:);
            FT(i,3)=FT(i,2)*tan(theta);

end
if    position(i,2)>=le &&  position(i,2)<=le+lplatf            %行人坐标大于le小于le+lpla，用水平社会力模型算法
%Determination of the current waypoints
%%%%%%这一段的目的是更新期望速度的方向，设定最终的目标点是（2.5,1000）以及（2.5,-1000）
e_act{n}(i,:)=e_seq2{n};
e(i,:)=(e_act{n}(i,:)'-position(i,:)')/norm(e_act{n}(i,:)'-position(i,:)');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    v0(i,:)=1.34;
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
              

                 fij1(i,:)=fij1(i,:)'+A*exp((rij-dij)/B)*nij; 
    

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
        if n==1 %更新上行行人的期望速度
            videl=0.8*v0Mu*e(i,:)+0.2*vaverage ;
        end
        if n==2%跟新下行行人的期望速度
            videl=0.8*v0Md*e(i,:)+0.2*vaverage ;
        end
%         if k==1 %更新上行行人的期望速度
%             videl=(1-0.4*tho)*(1-sin(theta))*videl+0.4*tho*vaverage ;
%         end
%         if k==2%跟新下行行人的期望速度
%             videl=(1-0.4*tho)*(1+sin(theta))*videl+0.4*tho*vaverage ;
%         end
        end

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

            fig(i,:)=[0 0 0];
 FT(i,:)=fi0(i,:)+fij1(i,:)+fij2(i,:)+fij3(i,:)+fiw1(i,:)+fig(i,:);
 FT(i,3)=0;
end
if      position(i,2)>le+lplatf         %行人y坐标大于le+lpla，用阶梯路段算法
%Determination of the current waypoints
%%%%%%这一段的目的是更新期望速度的方向，设定最终的目标点是（2.5,1000）以及（2.5,-1000）
e_act{n}(i,:)=e_seq3{n};
e(i,:)=(e_act{n}(i,:)'-position(i,:)')/norm(e_act{n}(i,:)'-position(i,:)');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if n==1
v0(i)=v0Mu;
elseif n==2
 v0(i)=v0Md;
end
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
              

                 fij1(i,:)=fij1(i,:)'+A*exp((rij-dij)/B)*nij; 
    

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
        if n==1 %更新上行行人的期望速度
            videl=0.8*v0Mu*e(i,:)+0.2*vaverage ;
        end
        if n==2%跟新下行行人的期望速度
            videl=0.8*v0Md*e(i,:)+0.2*vaverage ;
        end
%         if k==1 %更新上行行人的期望速度
%             videl=(1-0.4*tho)*(1-sin(theta))*videl+0.4*tho*vaverage ;
%         end
%         if k==2%跟新下行行人的期望速度
%             videl=(1-0.4*tho)*(1+sin(theta))*videl+0.4*tho*vaverage ;
%         end
        end

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

            fig(i,:)=9.8*sin(theta).*[0 -1 -tan(theta)];
            FT(i,:)=fi0(i,:)+fij1(i,:)+fij2(i,:)+fij3(i,:)+fiw1(i,:)+fig(i,:);
            FT(i,3)=FT(i,2)*tan(theta);
end
end

end
