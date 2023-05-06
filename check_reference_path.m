t = 0:0.1:20;
global Duration
Duration = 20;
xdesired = QuadrotorReferenceTrajectory(t);
x = xdesired(1,:);
y = xdesired(2,:);
z = xdesired(3,:);

plot3(x,y,z)