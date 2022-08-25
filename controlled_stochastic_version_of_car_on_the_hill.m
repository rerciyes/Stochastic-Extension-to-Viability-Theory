%stochastic car on the hill with control parameter
tspan= 0:1:30;
number_of_penalty=0;number_of_trajectories=9;
m=1;
for i=0:1:12
    n=1;
    for j=-6:1:6
        number_of_penalty=0;
        for k=1:number_of_trajectories
            [t,y] = ode15s(@vdp_Car,tspan,[i;j]);
            A=(0>y(:,1)); B=(y(:,1)>12);
            if A+B==0
                %reward
            else
                %penalty
                number_of_penalty=number_of_penalty+1;
            end
        end
        table(m,n)=1-(number_of_penalty/number_of_trajectories);
        n=n+1;
    end
    m=m+1;
end

x=0:1:12;y=-6:1:6;
z = table(x+1,y+7);
surf(x,y,z)

%%
function dydt = vdp_Car(t,y)
%v1 = unifrnd(-0.4,0.4);v2 = unifrnd(-0.07,0.07);
v3 = unifrnd(-0.1,0.1);u=0;
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

y2 = -9.81*sin(0.55*sin(1.2*y(1))-0.6*sin(1.1*y(1)))-0.7*y(2)+v3+u;
dydt=[y1;y2];
end