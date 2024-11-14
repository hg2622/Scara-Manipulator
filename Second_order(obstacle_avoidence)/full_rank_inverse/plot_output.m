
init;


figure(1)
subplot(4,1,1); 
plot(t, pd(:,1)-tracking(:,1));
xlabel('Time (s)');
ylabel('X Error');

subplot(4,1,2); 
plot(t, pd(:,2)-tracking(:,2));
xlabel('Time (s)');
ylabel('Y Error');

subplot(4,1,3); 
plot(t, pd(:,3)-tracking(:,3));
xlabel('Time (s)');
ylabel('Z Error');

subplot(4,1,4); 
plot(t, theta_d(:,1)-tracking_theta(:,1));
xlabel('Time (s)');
ylabel('\theta Error');



temp = squeeze(q(:,1,:));
figure(2)
subplot(4,1,1); 
plot(t, temp(1,:));
xlabel('Time (s)');
ylabel('q_1');

subplot(4,1,2); 
plot(t, temp(2,:));
xlabel('Time (s)');
ylabel('q_2');

subplot(4,1,3); 
plot(t, temp(3,:));
xlabel('Time (s)');
ylabel('q_3');

subplot(4,1,4); 
plot(t, temp(4,:));
xlabel('Time (s)');
ylabel('q_4');