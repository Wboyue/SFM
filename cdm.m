clear all
close all

global n_groups  
global N 
global r 
global m 
global  v0Mu v0Md
global v 

global tau 
global A 
global B le wide
% global C 
% global D
global Aw


global k1 
global k2 
% global p 
global e_act 
 
global e_seq1  e_seq2 e_seq3  lplatf
global e_n 
global ps 
global num_walls 
global map_walls 

global am hig
global theta lemad
global nvector

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%% path definition %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%k=1;
le=10;%斜坡路段的长度
wide=5;                                 %路段的宽度
lplatf=10;%平台的长度
theat=30;%角度
theta=theat*pi/180;%参数theta表示阶梯的倾斜角度（通常在8.5度到29.5°之间）
hig=le*tan(theta);                           %路段的高度
nvector=[0 -tan(theta) 1];
nvector=nvector/norm(nvector);               %斜面法向量
% the=theta;
%%若是斜面，则le，wide表示向水平面的投影长、宽
s{1}=[0 0; 0 2*le+lplatf]';
s{2}=[wide 0; wide 2*le+lplatf]';
s{3}=[0 0; wide 0]';
s{4}=[0 2*le+lplatf; wide 2*le+lplatf]';
n_s=length(s); 

map_walls=[];
for i=1:n_s
   map_walls=[map_walls;s{i}];
end
        
[dnum_walls, ~] = size(map_walls);

%number of walls
num_walls = dnum_walls/2;         

%%%%%%%%%%%%%%%%%%% Simulation and model parameters %%%%%%%%%%%%%%%%%%%%%%%
% Simulation time 
TF=40;
%TF=80;
t_fine=1/30;                % timing accuracy in the movie (framerate=30)，
                            % 每秒更新30次图片,即每秒内包含了30次计算

% SFM Parameters
tau=0.5; %松弛时间
A=2000;
B=0.08;%0.8
Aw=2000;
% Bw=0.08;
% beta2=3;

%这是Helbing的参数，没有考虑人体的可压缩范围
k1=1.2*10^5;
k2=2.4*10^5;
%Lakoba带来了修正
% k1=4.4*10^4;
% k2=6*10^5;
% HSFM Parameters
% kd=500;  %式9中出现
% ko=1;
% k1g=200; % forward group cohesion force strength
% k2g=200; % sideward group cohesion force strength
% d_o=0.5; % sideward maximum distance from the center of mass
% d_f=1;   % forward maximum distance from the center of mass 
lemad=0.2;
% individual characteristics
% Radius
rm=0.33; % minimum radius人的肩宽一般37cm
rM=0.4; % maximum radius
% Mass 质量
mm=45; % minimum mass
mM=75; % maximum mass

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%% Path definition %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% waypoints sequence
e_n=cell(0);    %C = cell(n) 返回由空矩阵构成的 n×n 元胞数组。


% e_seq{i} contains the points through which the members of group i have to
% pass

% e_seq{1}=[3 10; 2.5 1000]';
% e_seq{2}=[2 10; 2.5 -1000]';
e_seq1{1}=[wide/2 1000 1000*tan(theta)]';%此处仅定义行人的终点目标
e_seq1{2}=[wide/2 -1000 -1000*tan(theta)]';

e_seq2{1}=[wide/2 1000 hig]';%此处仅定义行人的终点目标
e_seq2{2}=[wide/2 -1000 hig]';

e_seq3{1}=[wide/2 1000 1000*2*hig\(2*le+lplatf)]';%此处仅定义行人的终点目标
e_seq3{2}=[wide/2 -1000 -1000*2*hig\(2*le+lplatf)]';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%% Initial conditions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% density=[0.1:0.1:1];
 kk=1;
 for cc=0:0.02:1
     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%行人数量定义%%%%%%%21%%%%%%%%%%%%%%%%%%%%%%%%%%
tho1=cc;                                %定义为第一段台阶的行人流密度
tho2=cc;                                %定义为平台的行人密度
tho3=cc;                                %定义为第二段台阶的行人密度

up1=round(tho1*wide*le/2);              %第一段台阶的上行行人数
up2=round(tho2*wide*lplatf/2);              %第一段平台的上行行人数
up3=round(tho3*wide*le/2);              %第二段台阶的上行行人数


up=up1+up2+up3;        %上行行人数=下行行人数

down=up;
n_groups = [up down];%两群人，每群人里有6个人。
% Total number of individuals
N=sum(n_groups);


                           
for i=1:length(n_groups)                % random radii and masses 
    % random radii行人的随机半径
    % rand(n_groups(i),1))产生6*1的随机数矩阵
    % 如果 A 是矩阵，则 sort(A) 会将 A 的列视为向量并对每列进行排序
    % x = j:k 创建一个包含元素 [j,j+1,j+2,...,j+m] 的单位间距向量 x，其中 m = fix(k-j)。如果 j 和 k 都是整数，则简化为 [j,j+1,...,k]。
    r(sum(n_groups(1:i))-n_groups(i)+1:sum(n_groups(1:i)))=sort(rm+(rM-rm)*rand(n_groups(i),1)); 
    
    % random masses行人的随机质量
    m(sum(n_groups(1:i))-n_groups(i)+1:sum(n_groups(1:i)))=sort((mm)+(mM-mm)*rand(n_groups(i),1)); 
    
    cumulative(i)=sum(n_groups(1:i));
end


% Desired speed%期望速度的初始化要考虑位于阶梯还是平台
v0Mu=-0.125*log(le)-0.014*theat+1.619; % minimum velocity
v0Md=-0.131*log(le)-0.017*theat+1.715;
% v0mu=-0.137*log(le)-0.015*theat+1.527;
% v0md=-0.145*log(le)-0.021*theat+1.593;
vavg=[];
vuavg=[];
vdavg=[];

% Individual characteristics
%随机化期望速率 
   
v=0*ones(N,3);                          % initial speed [vx,vy，vz]
             
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%初始化行人的随机位置%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% "am" represents the amplitude of the starting zone. Fix the starting 
% points at least at "am" meters from the walls
% am=2; 
am=wide/2;   
bm1=le/2-0.5;
bm2=lplatf/2-0.5;
%下面两行是初始化行人的随机位置在通道外
s{1} = [wide/2 le/2]; 
s{2} = [wide/2 le+lplatf/2];
s{3} = [wide/2 le+lplatf+le/2];  
%下面两行初始化行人的随机位置在通道内
% s{1} = [wide/2 le/2]; 
% s{2} = [wide/2 le/2];  
% System initialization
p=zeros(N,3);
k=1;    

% random initial positions
X1=[];

if N~=0 %只有在N!=0的情况下计算出X
while k<=N
    i=k;
    if (i<=up1) || (i>=(N/2+1) && i<=up1+N/2) %初始化第一段阶梯的行人位置
    pos=[s{1}(1)-am+2*am*rand(1,1)  s{1}(2)-bm1+2*bm1*rand(1,1) 0];
    pos(3)=pos(2)*tan(theta);
    % minimum distance between pedestrians
    for l=1:i-1
        d(l)=norm(pos-p(l,1:3))<=r(i)+r(l);
    end
    % minimum distance from walls
    for l=i:i+1
        xp=pos(1);
        yp=pos(2);
        rp=[xp yp]';
        ra=map_walls(2*(l+1-i)-1:2*(l+1-i),1); 
        rb=map_walls(2*(l+1-i)-1:2*(l+1-i),2);
        xa=ra(1);
        ya=ra(2);
        xb=rb(1);
        yb=rb(2);
        t=((xp-xa)*(xb-xa)+(yp-ya)*(yb-ya))/(((xb-xa)^2+(yb-ya)^2));
        t_star=min(max(0,t),1);
        rh=ra+t_star*(rb-ra);
        d(l)=norm(rp-rh)<=r(i);
    end
        if sum(d)==0
        p(i,:)=[pos(1) pos(2) pos(3)]; 
        X1=[X1 pos(1) pos(2) pos(3) v(i,1) v(i,2) v(i,3)];
         k=k+1;
        end
    end

     if ((i>=up1+1) &&(i<=up1+up2)) || ((i>=up1+1+N/2) &&(i<=up1+up2+N/2)) %初始化第二段平台上的行人位置
    pos=[s{2}(1)-am+2*am*rand(1,1)  s{2}(2)-bm2+2*bm2*rand(1,1) 0];
    pos(3)=hig;
    % minimum distance between pedestrians
    for l=1:i
        d(l)=norm(pos-p(l,1:3))<=r(i)+r(l);
    end
    % minimum distance from walls
    for l=i+1:i+2
        xp=pos(1);
        yp=pos(2);
        rp=[xp yp]';
        ra=map_walls(2*(l-i)-1:2*(l-i),1); 
        rb=map_walls(2*(l-i)-1:2*(l-i),2);
        xa=ra(1);
        ya=ra(2);
        xb=rb(1);
        yb=rb(2);
        t=((xp-xa)*(xb-xa)+(yp-ya)*(yb-ya))/(((xb-xa)^2+(yb-ya)^2));
        t_star=min(max(0,t),1);
        rh=ra+t_star*(rb-ra);
        d(l)=norm(rp-rh)<=r(i);
    end
    if sum(d)==0
        p(i,:)=[pos(1) pos(2) pos(3)]; 
       % X1=[X1 pos(1) pos(2) th(i) norm(v(i,:)) 0 omg];
%         X1=[X1 pos(1) pos(2) th(i) v(i,1) v(i,2) omg];
        X1=[X1 pos(1) pos(2) pos(3) v(i,1) v(i,2) v(i,3)];
         k=k+1;
    end


    end

    if ((i>=up1+up2+1) &&(i<=N/2)) || ((i>=up1+up2+1+N/2) &&(i<=N)) %初始化第三段平台上的行人位置
    pos=[s{3}(1)-am+2*am*rand(1,1)  s{3}(2)-bm1+2*bm1*rand(1,1) 0];
    pos(3)=pos(2)*tan(theta)-tan(theta)*(le+lplatf)+hig;
    % minimum distance between pedestrians
    for l=1:i
        d(l)=norm(pos-p(l,1:3))<=r(i)+r(l);
    end
    % minimum distance from walls
    for l=i+1:i+2
        xp=pos(1);
        yp=pos(2);
        rp=[xp yp]';
        ra=map_walls(2*(l-i)-1:2*(l-i),1); 
        rb=map_walls(2*(l-i)-1:2*(l-i),2);
        xa=ra(1);
        ya=ra(2);
        xb=rb(1);
        yb=rb(2);
        t=((xp-xa)*(xb-xa)+(yp-ya)*(yb-ya))/(((xb-xa)^2+(yb-ya)^2));
        t_star=min(max(0,t),1);
        rh=ra+t_star*(rb-ra);
        d(l)=norm(rp-rh)<=r(i);
    end
    if sum(d)==0
        p(i,:)=[pos(1) pos(2) pos(3)]; 
       % X1=[X1 pos(1) pos(2) th(i) norm(v(i,:)) 0 omg];
%         X1=[X1 pos(1) pos(2) th(i) v(i,1) v(i,2) omg];
        X1=[X1 pos(1) pos(2) pos(3) v(i,1) v(i,2) v(i,3)];
        k=k+1;
    end
   

    end

end  

%%%%%%%%%%%%%%%%%%%%%%%%%%e_act表示目标位置%%%%%%%%%%%%%%%%%%%%%%%%%
ps=p(:,1:3);  %仅包含行人位置坐标向量
e_act{1}=ps(1:n_groups(1),:);%第一组中的行人位置坐标

for i=2:length(n_groups)   %L = length(X) 返回 X 中最大数组维度的长度。对于向量，长度仅仅是元素数量。对于具有更多维度的数据，长度为 max(size(X))。空数组的长度为零。
     % current waypoint
    e_act{i}=ps(sum(n_groups(1:i-1))+1:sum(n_groups(1:i)),:);%第i组中的行人位置坐标
end

Xf=[];
Xf(1,:)=X1;
%%%%%%%%%%%%%%%%%%%%%%%%%%% System simulation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tic            
% tspan=0:t_fine:TF;
jup=N;
jdown=up;
for i=1:750
tspan=0:0.02:0.04;
%ode45 是一个通用型 ODE 求解器，[t,y] = ode45(odefun,tspan,y0)（其中 tspan = [t0 tf]）求微分方程组 y′=f(t,y) 从 t0 到 tf 的积分，初始条件为 y0。解数组 y 中的每一行都与列向量 t 中返回的值相对应。
[t,X]=ode45(@system_model_NHM,tspan,X1');
Xf(i+1,:)=X(3,:)%存下当前结果
X1=X(3,:);%把当前结果存入X1当作初值


    %运动方向向上的行人
    for j=1:up
            if X1(1,6*j-4)>=2*le+lplatf-1
                %j表示了当前需要重置的行人编号
                X1(1,6*j-4)=0.5;
                X1(1,6*j-5)=(2*rand-1)*(wide/2-1)+wide/2;
                while abs(X1(1,6*j-5)-X1(1,6*jup-5))<=r(j)+r(jup)     %当当前重置行人的位置与上一次重置行人的位置间距离小于两人的半径和时，再重置一次
                       X1(1,6*j-5)=(2*rand-1)*(wide/2-1)+wide/2;
                end
       
                jup=j;                       %记录下需要重置位置的行人编号
            end

            if X1(1,6*j-4)>=le && X1(1,6*j-4)<=le+lplatf
                 X1(1,6*j-3)=hig;
            end
            if X1(1,6*j-4)<le   
                 X1(1,6*j-3)=X1(1,6*j-4)*tan(theta); 
            end
            if   X1(1,6*j-4)>le+lplatf     
             X1(1,6*j-3)=X1(1,6*j-4)*tan(theta)-tan(theta)*(le+lplatf)+hig;
            end

    end
    for j=up+1:N
             %运动方向向下的行人
            if X1(1,6*j-4)<=1
                X1(1,6*j-4)=2*le+lplatf-0.5;
                X1(1,6*j-5)=(2*rand-1)*(wide/2-1)+wide/2;
                while abs(X1(1,6*j-5)-X1(1,6*jdown-5))<=r(j)+r(jdown)     %当当前重置行人的位置与上一次重置行人的位置间距离小于两人的半径和时，再重置一次
                       X1(1,6*j-5)=(2*rand-1)*(wide/2-1)+wide/2;
                end

                jdown=j;  
            end

            if X1(1,6*j-4)>=le && X1(1,6*j-4)<=le+lplatf
                 X1(1,6*j-3)=hig;
            end

            if X1(1,6*j-4)<le   
                 X1(1,6*j-3)=X1(1,6*j-4)*tan(theta); 
            end
            if   X1(1,6*j-4)>le+lplatf     
             X1(1,6*j-3)=X1(1,6*j-4)*tan(theta)-tan(theta)*(le+lplatf)+hig;
            end


    end
end
toc  
% % 
    Xfinal{kk}=Xf;%k代表了不同密度下的仿真结果
    kk=kk+1;
end


if N==0
    vavg0=0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
%%%%%%%%%%%%%%%%%%%%%%%%%% avi视频文件输出 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%mplay;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
%%%%%%%%%%%%%%%%%%%%%% 输出所有密度下的视频文件  %%%%%%%%%%%%%%%%%%%%%%%%%%%
%media;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%平均速度%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
vup=0; 
for i=650:750
    for j=1:up
        vup=vup+norm(Xf(i,6*j-2:6*j));
    end
end
vuavg=[vuavg (1/(101))*(1/up)*vup];

vdown=0; 
for i=650:750
    for j=up+1:N
        vdown=vdown+norm(Xf(i,6*j-2:6*j));
    end
end
vdavg= [vdavg (1/(101))*(1/down)*vdown];
end
vvavg=(vdavg+vuavg)/2;
% save vuavg vdavg Xfinal
%save('test.mat','Xfinal','vuavg','vdavg','N','n_groups','map_walls','hig');
% maxXrow=0;%maxXrow包含行人全都进入通道的时间信息
% for py=2:4:4*up
%     a=find(X(:,py)>0,1);
%     if a>maxXrow
%         maxXrow=a;
%     end
% end
% for py=4*up+2:4:4*N
%     a=find(X(:,py)<le,1);
%     if a>maxXrow
%         maxXrow=a;
%     end
% end
% 
% minXrow=0;%minXrow包含第一个行人离开通道的时间信息
% for py=2:4:4*up
%     a=find(X(:,py)>le,1);
%     if a>minXrow
%         minXrow=a;
%     end
% end
% for py=4*up+2:4:4*N
%     a=find(X(:,py)<0,1);
%     if a>minXrow
%         minXrow=a;
%     end
% end
% for i=maxXrow:minXrow-1
%     for vy=3:4:4*N
%         va=va+sqrt((X(i,vy)).^2+(X(i,vy+1)).^2);
%     end
% end
% vavg=[vavg (1/(minXrow-maxXrow))*(1/N)*va];
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
% %%%%%%%%%%%%%%%%%%%%%%%%%%% 输出速度-密度曲线图 %%%%%%%%%%%%%%%%%%%%%%%%%
%vavg(1)=[];

% title('速度-密度曲线图')
% xlabel('density')
% ylabel('volosity')
% axis([0 2 0 1]);
% plot(density,vavg);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% END %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

