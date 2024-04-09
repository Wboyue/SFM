density=[0:0.02:1];
up=round(density*30*5/2);
N=2*up;
vuavg=[];
vdavg=[];
vup=0; 
for k=1:50
   
for i=650:750
    for j=1:up(k+1)
        vup=vup+norm(abc{k}(i,6*j-2:6*j));
    end
end
vuavg=[vuavg (1/(101))*(1/up)*vup];

vdown=0; 
for i=650:750
    for j=up(k+1)+1:N(k+1)
        vdown=vdown+norm(abc{k}(i,6*j-2:6*j));
    end
end
vdavg= [vdavg (1/(101))*(1/down)*vdown];

end


title('速度-密度曲线图')
xlabel('density')
ylabel('volosity')
axis([0 1 0 1.5]);
plot(density,vavg);