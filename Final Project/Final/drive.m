function [] = drive(velocity, angularVelocity, velocityPublisher)
velmsg = rosmessage(velocityPublisher);
velmsg.Linear.X = velocity;
velmsg.Angular.Z = angularVelocity;
send(velocityPublisher,velmsg);
end