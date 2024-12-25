

tracking=out.tracking;


figure(1)
subplot(3,1,1); 
plot(t, pd(:,1)-tracking(:,1));
xlabel('Time (s)');
ylabel('X Error');

subplot(3,1,2); 
plot(t, pd(:,2)-tracking(:,2));
xlabel('Time (s)');
ylabel('Y Error');

subplot(3,1,3); 
plot(t, pd(:,3)-tracking(:,3));
xlabel('Time (s)');
ylabel('Z Error');




