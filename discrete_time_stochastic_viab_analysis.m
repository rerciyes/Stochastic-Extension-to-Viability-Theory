%Basic Example-I, Discrete Time Stochastic Viability Case
clear all;
%we start with # of 9 Trajectories(= Finite Time Observation, Evolution)
%t=40, finite time
trajectories=zeros(9,40);
number_of_penalty=0;number_of_trajectories=9;
for j=1:number_of_trajectories
    %initial_settings
    x(1)=0;reward=true;
    trajectories(j,1)=x(1);
    u=1;p=0.01;
    P=[p p 1-2*p];
    W=[1 -1 0];
    
    for i=2:40
    
    %assign the control
    if u+x(i-1)>0
        u=-1;
    elseif u+x(i-1)<0
        u=1;
    end
    
    %uncertainty
    w=randsample(W,1,true,P);
    
    %The evolution of a scalar x(t), discrete-time dynamics
    x(i)=x(i-1)+u+w;
    trajectories(j,i)=x(i);
    
    %check viable or not (penalized or reward)
    if x(i)>=-1 && x(i)<=1
        %reward
    else
        %penalty
        reward=false;
    end
    
    end
    
    if reward==false
       number_of_penalty=number_of_penalty+1;
    end
    
end

t=1:1:40;
for m=1:number_of_trajectories
   plot(t,trajectories(m,:))
   hold on
end

%Viability probability value function:= V(0=x)
V=number_of_penalty/number_of_trajectories