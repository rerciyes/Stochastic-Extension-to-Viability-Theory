%Stability Analysis of Deterministic Case of Car on the Hill with continuous control
tic
tspan= 0:0.1:30;
%%{
hold all
bool=false;
m=1;
eq_points=[9.5,6.69,3.91];
d=1.3;
for i=0:0.1:12
    n=1;
    for j=-6:0.1:6
        bool=false;
        [t,y] = ode45(@vdp_Car,tspan,[i;j]);
        A=(0>y(:,1)); B=(y(:,1)>12);
        if A+B==0
            bool=true;
            if(eq_points(1)-d<=y(:,1))&(y(:,1)<=eq_points(1)+d)
                plot(i,j,'*',"Color",'g');
            elseif (eq_points(2)-d<=y(:,1))&(y(:,1)<=eq_points(2)+d)
                plot(i,j,'*',"Color",'g');
            elseif (eq_points(3)-d<=y(:,1))&(y(:,1)<=eq_points(3)+d)
                plot(i,j,'*',"Color",'g');
            else
                plot(i,j,'*',"Color",'b');
            end
        else
            plot(i,j,'*',"Color",'r');
        end
        table(m,n)=bool;
        n=n+1;
    end
    m=m+1;
end
%table
plot(13,0,'*',"Color",'r')
hold off

s_dot=-6:0.1:6;
for i=1:length(table(1,:))
    a=find(table(:,i)==1,1,"first");
    s_dot1(i)=s_dot(a);
    b=find(table(:,i)==1,1,"last");
    s_dot2(i)=s_dot(b);
end

s=0:0.1:12;
figure
hold all
plot(s,s_dot1)
plot(s,s_dot2)
patch([s fliplr(s)], [s_dot1 fliplr(s_dot2)], 'g')
hold off
%%}
%[t,y] = ode45(@vdp_Car,tspan,[2.35;-4]);
toc

%%
function dydt = vdp_Car(t,y)
%v1 = unifrnd(-0.4,0.4);v2 = unifrnd(-0.07,0.07);
%v3 = unifrnd(-0.2,0.2);
u=0;
y1=y(2);
%{
%control
if abs(12-y(1))<y(2)&&y(2)>0&&(0<=y(1))<=12
    %set u
    if y(2)+y(1)<=9
        u=-1;
    elseif  y(2)+y(1)>9 
        u=-2;
    end
elseif abs(0-y(1))<abs(y(2))&&y(2)<0&&(0<=y(1))<=12
    %set u
    if y(2)+y(1)<=3
        u=2;
    elseif  y(2)+y(1)>3 
        u=1;
    end    
end
%}

%continuous control
%our critical points:=eq_points=[9.5,6.69,3.91];
d=0.3;
if 3.91-d<=y(1)&&3.91+d>=y(1)
   u=-sin((pi/d)*(y(1)-3.91));
elseif 6.69-d<=y(1)&&6.69+d>=y(1)
   u=-sin((pi/d)*(y(1)-6.69));
elseif 9.5-d<=y(1)&&9.5+d>=y(1)
   u=-sin((pi/d)*(y(1)-9.5));
end

y2 = -9.81*sin(0.55*sin(1.2*y(1))-0.6*sin(1.1*y(1)))-0.7*y(2)+u;
dydt=[y1;y2];
end