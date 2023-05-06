% Calculate the Loss
loss = sum(sum(uHistory)) * Ts;

time = 0:Ts:Duration;
yreftot = QuadrotorReferenceTrajectory(time)';

xref = yreftot(:,1);
yref = yreftot(:,2);
zref = yreftot(:,3);

x = xHistory(:,1);
y = xHistory(:,2);
z = xHistory(:,3);

xdif = abs(x - xref);
ydif = abs(y - yref);
zdif = abs(z - zref);

loss = loss + (sum(xdif.^2) + sum(ydif.^2) + sum(zdif.^2)) * Ts;
disp(['The loss is: ', num2str(loss)])

