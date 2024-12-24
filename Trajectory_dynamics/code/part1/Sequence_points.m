


function [q, q_dot, q_dot_dot] = trap_v(init, final,time_i, time_f)
   
     t_f = time_f-time_i; 
    distance = norm(final - init); 
   
    a_max = 4*distance/t_f^2+2;    % !!!make sure acceleration equation solvable
    
    % calculate t_c, where velocity got flat
   
    t_c = t_f / 2 - 0.5 * sqrt((t_f^2 * a_max - 4 * distance) / a_max);
    
    num_steps = round(t_f *1000);
    q = zeros(1, num_steps);
    q_dot = zeros(1, num_steps);
    q_dot_dot = zeros(1, num_steps);
    
    time = linspace(0, t_f, num_steps);
 
% applying trapezoidal velocity rule
    for idx = 1:num_steps
        t = time(idx); 
        if 0 <= t && t <= t_c
            q(idx) = 0.5 * a_max * t^2;
            q_dot(idx) = a_max * t;
            q_dot_dot(idx) = a_max;
        elseif t_c < t && t <= t_f - t_c
            q(idx) = 0.5 * a_max * t_c^2 + a_max * t_c * (t - t_c);
            q_dot(idx) = a_max * t_c; % constant velocity
            q_dot_dot(idx) = 0; % no acceleration
        elseif t_f - t_c < t && t <= t_f
            q(idx) = distance - 0.5 * a_max * (t_f - t)^2;
            q_dot(idx) = a_max*t_c-a_max*(t-t_f+t_c);
            q_dot_dot(idx) = -a_max;
        end
    end
end




function [pd,pd_dot, pd_dot_dot] =  seq_point(init, points,time,anticipation)

pd=zeros(3,4000);
pd_dot=zeros(3,4000);
pd_dot_dot=zeros(3,4000);

anticipation=anticipation*1000;
length=size(points,1);


% iterate through each via points
for j=0:1:length-1

    % deal with start and end time
    t_start=round(time(j+1,1));
    t_end=round(time(j+1,2));
    t_range=round((t_end-t_start)/0.001);
    
    ant=0;
   
    %set up anticipation
    index1=t_range*(j)-ant+1;
    index2=(j+1)*t_range-ant;


    % a direction vector point to where to go
    increment=(points(j+1,:)-init)/norm(points(j+1,:)-init);
    
    %update the velocity, position, and acceleration
    [q,q_dot,q_dot_dot]=trap_v(init,points(j+1,:),t_start,t_end);
    
    pd(:,index1:index2)=pd(:,index1:index2)+(q'*increment)';
    pd(:,index1+ant:index2)= pd(:,index1-ant:index2)+init';
    pd_dot(:,index1:index2)=pd_dot(:,index1:index2)+(q_dot'*increment)';
    pd_dot_dot(:,index1:index2)= pd_dot_dot(:,index1:index2)+(q_dot_dot'*increment)';
        
    %init become the new position reached
    init=points(j+1,:);
    %increment anticipation time
    ant=ant+anticipation;
    
end

end


points=[[0 -0.8 0.5];[0.5,-0.6 0.5];[0.8 0.0 0.5]; [0.8 0.0 0.0]];
time= [[0 0.6];[0.6 2.0]; [2.0 3.4]; [3.4 4.0]];
t=0:0.001:4;
t=t';
Tc=0.001;


[pd,pd_dot, pd_dot_dot]=seq_point([0 -0.8 0], points,time,0.2);

init=[0 -0.8 0];
pd=pd';
pd_dot=pd_dot';
pd_dot_dot=pd_dot_dot';

pd=[[0 -0.8 0];pd];
pd_dot=[[0,0,0];pd_dot];
pd_dot_dot=[[0,0,0];pd_dot_dot];


figure(1)
subplot(3,1,1); 
plot(t, pd(:,1));
xlabel('Time (s)');
ylabel('X');

subplot(3,1,2); 
plot(t, pd(:,2));
xlabel('Time (s)');
ylabel('Y');

subplot(3,1,3); 
plot(t, pd(:,3));
xlabel('Time (s)');
ylabel('Z ');

figure(2)
subplot(3,1,1); 
plot(t, pd_dot(:,1));
xlabel('Time (s)');
ylabel('X_dot');

subplot(3,1,2); 
plot(t, pd_dot(:,2));
xlabel('Time (s)');
ylabel('Y_dot');

subplot(3,1,3); 
plot(t, pd_dot(:,3));
xlabel('Time (s)');
ylabel('Z_dot ');


figure(3)
subplot(3,1,1); 
plot(t, pd_dot_dot(:,1));
xlabel('Time (s)');
ylabel('X_dot_dot');

subplot(3,1,2); 
plot(t, pd_dot_dot(:,2));
xlabel('Time (s)');
ylabel('Y_dot_dot');

subplot(3,1,3); 
plot(t, pd_dot_dot(:,3));
xlabel('Time (s)');
ylabel('Z_dot_dot ');


save('generated_traj.mat', 'pd', 'pd_dot', 'pd_dot_dot','t',"Tc",'init');

