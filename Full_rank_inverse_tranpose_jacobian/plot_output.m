%plot tracking

% plot pd and theta_d error
figure(2)

subplot(4,1,1); plot(t, pd(:,1)-tracking(:,1));
subplot(4,1,2); plot(t, pd(:,2)-tracking(:,2));
subplot(4,1,3); plot(t, pd(:,3)-tracking(:,3));
subplot(4,1,4); plot(t, theta_d(:,1)-tracking_theta(:,1));

% plot q 1-4 values
temp=squeeze(q(:,1,:))';  % temp is q changed to a different shape for output plot
figure(3)
subplot(4,1,1); plot(t,temp(:,1));
subplot(4,1,2); plot(t, temp(:,2));
subplot(4,1,3); plot(t, temp(:,3));
subplot(4,1,4); plot(t, temp(:,4));

