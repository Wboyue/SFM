function [ dX ] = system_model_NHM(t,X)
%X表示行人当前（即此次求解微分方程时）[位置x，位置y，vx，vy]
global N  m  n_groups theta 

% Positions and velocities
position=zeros(N,3);
vel=zeros(N,3);
for i=1:N
   position(i,:)=[X(6*i-5) X(6*i-4) X(6*i-3)];    %将X中的行人位置信息复制到position变量下（表示行人当前的位置）
   vel(i,:)=[X(6*i-2) X(6*i-1) X(6*i)];
end

% Acting forces

[FT]=forces_NHM(X);


dX=zeros(6*N,1);


for k=1:length(n_groups)
    for i=sum(n_groups(1:k))-n_groups(k)+1:sum(n_groups(1:k))


%         p_i=(ci{k}-position(i,:)');% 参看1e）部分的定义
        %X表示行人当前（即此次求解微分方程时）[位置x，位置y，位置z，vx，vy，vz]
       dX(6*i-5)=X(6*i-2);                  %dx=vx
       dX(6*i-4)=X(6*i-1);                    %dy=vy
       dX(6*i-3)=X(6*i);                    %dz=vz
        dX(6*i-2)=1/m(i)*FT(i,1);           %ax=(1/m)*fx
        dX(6*i-1)=1/m(i)*FT(i,2);             %ay=(1/m)*fy
        dX(6*i)=1/m(i)*FT(i,3);             %ay=(1/m)*fy
%        dX(6*i)=(1/m(i))*FT(i,2)*tan(theta);             %ay=(1/m)*fy
    end
end

end

