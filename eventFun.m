function [position,isterminal,direction] = eventFun(t,z,l1,l2,l3,l4,l5)

q1m = z(1);
q2m = z(2);
q3m = z(3);
q4m = z(4);
q5m = z(5);

height = l1.*cos(q1m)+l2.*cos(q2m)-l4.*cos(q4m)-l5.*cos(q5m);
position = height;

if t<0.3
isterminal = 0;
else
isterminal = 1;
end

direction = 0;

end