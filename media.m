global wide le lplatf hig
% movie speed, 1=30 fps
%step=10;
step=10;

color=[];
cir=0:0.01:2*pi;
if N==n_groups(1) % if there are no groups random colors
    color=[rand(N,1) rand(N,1) rand(N,1)];
else %if there are groups a color for each group
    for i=1:length(n_groups)
        %color=[color; ones(n_groups(i),1)*rand ones(n_groups(i),1)*rand ones(n_groups(i),1)*rand];
        color=[color; ones(n_groups(i),1)*(2-i) ones(n_groups(i),1)*0 ones(n_groups(i),1)*(i-1)];%假设两组，第一组红色，第二组蓝色
    end
end

%%% MOVIE
scrsz = get(0,'ScreenSize');
vid=VideoWriter('simulation,');%v = VideoWriter(filename) 创建一个 VideoWriter 对象以将视频数据写入采用 Motion JPEG 压缩技术的 AVI 文件。
open(vid);
h=figure('Name','斑图','Color','w','Position',[1 1 scrsz(3) scrsz(4)]);

%for tt=1:step:length(tspan) %step=10，则共取120帧，而每秒30帧，所以共4秒，而对应实际是40秒，所以播放速度加快了10倍
for tt=1:751
    
    % Plot of the walls
     subplot(1,2,1);
        plot3([0 0],[0 le],[0 hig],'k','LineWidth',2 );
        hold on  
        plot3([wide wide],[0 le],[0 hig],'k','LineWidth',2 );
        hold on 
        plot3([0 0],[le le+lplatf],[hig hig],'k','LineWidth',2 );
        hold on 
        plot3([wide wide],[le le+lplatf],[hig hig],'k','LineWidth',2 );
        hold on 
        plot3([0 0],[le+lplatf 2*le+lplatf],[hig 2*hig],'k','LineWidth',2 );
        hold on 
        plot3([wide wide],[le+lplatf 2*le+lplatf],[hig 2*hig],'k','LineWidth',2 );
        hold on   
        plot3([0 wide],[le le],[hig hig],'k','LineWidth',1 );
        hold on   
        plot3([0 wide],[le+lplatf le+lplatf],[hig hig],'k','LineWidth',1 );
        hold on
        plot3([0 0],[le le],[0 hig],'k','LineWidth',1 );
        hold on
        plot3([0 0],[le+lplatf le+lplatf],[0 hig],'k','LineWidth',1 );
        hold on
        plot3([wide wide],[le le],[0 hig],'--','LineWidth',1 );
        hold on
        plot3([wide wide],[le+lplatf le+lplatf],[0 hig],'--','LineWidth',1 );
        hold on
    % Plot of the pedestrians represented as circles
    for i=1:N
        plot3(r(i)*cos(cir)+Xf(tt,6*i-5),r(i)*sin(cir)+Xf(tt,6*i-4),Xf(tt,6*i-3)+zeros(1,length(cir)),'Color',color(i,:),'LineWidth',2) % plot cerchi
        hold on
%         plot(r(i)*cos(X(tt,6*i-3))+X(tt,6*i-5),r(i)*sin(X(tt,6*i-3))+X(tt,6*i-4),'ok','MarkerFaceColor',[0,0,0])
%         hold on
    end
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    grid on
    hold off
    axis equal%沿每个坐标轴使用相同的数据单位长度。
    axis on  %显示坐标区的线条和背景。
    
 subplot(1,2,2);
    % Plot of the walls
    for i=1:num_walls
        plot(map_walls(2*i-1,:),map_walls(2*i,:),'k','LineWidth',2);
        hold on   %hold on 保留当前坐标区中的绘图，从而使新添加到坐标区中的绘图不会删除现有绘图。新绘图基于坐标区的 ColorOrder 和 LineStyleOrder 属性使用后续的颜色和线型。MATLAB® 将调整坐标区的范围、刻度线和刻度标签以显示完整范围的数据。如果不存在坐标区，hold 命令会创建坐标区。
    end
    
    % Plot of the pedestrians represented as circles
    for i=1:N
        plot(r(i)*cos(cir)+Xf(tt,6*i-5),r(i)*sin(cir)+Xf(tt,6*i-4),'Color',color(i,:),'LineWidth',2) % plot cerchi
        hold on
 %        plot(r(i)*cos(X(tt,6*i-3))+X(tt,6*i-5),r(i)*sin(X(tt,6*i-3))+X(tt,6*i-4),'ok','MarkerFaceColor',[0,0,0])
 %        hold on
    end

    hold off  %将保留状态设置为 off，从而使新添加到坐标区中的绘图清除现有绘图并重置所有的坐标区属性。添加到坐标区的下一个绘图基于坐标区的 ColorOrder 和 LineStyleOrder 属性使用第一个颜色和线型。此选项为默认行为。
    axis equal%沿每个坐标轴使用相同的数据单位长度。
    axis on  %显示坐标区的线条和背景。

   
    M=getframe(h);    %捕获坐标区或图窗作为影片帧
    writeVideo(vid,M);%writeVideo(v,img) 将数据从数组写入与 v 相关联的视频文件。必须先调用 open(v)，然后再调用 writeVideo。
end
close(vid)


