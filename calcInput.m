## This function calculates the control input after the ode is solved

function [retval retval2] = calcInput (t,x,p,polyp,polyv,polya)
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
retval = zeros(size(x,1),size(polyp,1));

m1 = p.m1;  I1 = p.I1;
m2 = p.m2;  I2 = p.I2;
m3 = p.m3;  I3 = p.I3;
m4 = p.m4;  I4 = p.I4;
m5 = p.m5;  I5 = p.I5;
l1 = p.l1;  c1 = p.c1;
l2 = p.l2;  c2 = p.c2;
l3 = p.l3;  c3 = p.c3;
l4 = p.l4;  c4 = p.c4;
l5 = p.l5;  c5 = p.c5;
g = p.g;

%% controller design
wn = 1e5;
kp = wn^2;
kd = 2*wn;

for i = 1:size(x,1)
  
  q1 =x(i,1);   dq1 = x(i,6);
  q2 =x(i,2);   dq2 = x(i,7);
  q3 =x(i,3);   dq3 = x(i,8);
  q4 =x(i,4);   dq4 = x(i,9);
  q5 =x(i,5);   dq5 = x(i,10);

  ya = [q2 - q1; q3 - q2; q4 - q3 + pi; q5 - q4];
  dya = [dq2 - dq1; dq3 - dq2; dq4 - dq3; dq5 - q4];
  Jy = [-1  1  0  0  0;
         0 -1  1  0  0;
         0  0 -1  1  0;
         0  0  0 -1  1];

  tau = t(i); %%%%%%%%%%%%%%
  tFinal = 0.5412;
  if t(i)>= tFinal
      tau = tFinal;
  end

  for i2 = 1:4
      yd(i2,:) = polyval(polyp(i2,:),tau);
      dyd(i2,:) = polyval(polyv(i2,:),tau);
      ddyd(i2,:) = polyval(polya(i2,:),tau);
  end

  % yd = polyval(polyp,tau);
  % dyd = polyval(polyv,tau);
  % ddyd = polyval(polya,tau);

  %%%%%%%%%%%%%%%%%%%%
  y2 = ya-yd;
  dy2 = dya-dyd;

  % these are only the bottom half of f(x) and g(x), namely fq and gq
  M = [l1*m5*sin(q1)*(c5*sin(q5) - l1*sin(q1) - l2*sin(q2) + l4*sin(q4)) - m1*(c1*cos(q1) - l1*cos(q1))^2 - m1*(c1*sin(q1) - l1*sin(q1))^2 - l1*m3*sin(q1)*(l1*sin(q1) - c3*sin(q3) + l2*sin(q2) + l3*sin(q3)) - I1 - l1*m2*cos(q1)*(l1*cos(q1) - c2*cos(q2) + l2*cos(q2)) - l1*m4*cos(q1)*(l1*cos(q1) - c4*cos(q4) + l2*cos(q2)) - l1*m2*sin(q1)*(l1*sin(q1) - c2*sin(q2) + l2*sin(q2)) - l1*m4*sin(q1)*(l1*sin(q1) - c4*sin(q4) + l2*sin(q2)) - l1*m3*cos(q1)*(l1*cos(q1) - c3*cos(q3) + l2*cos(q2) + l3*cos(q3)) + l1*m5*cos(q1)*(c5*cos(q5) - l1*cos(q1) - l2*cos(q2) + l4*cos(q4)), m2*(c2*cos(q2) - l2*cos(q2))*(l1*cos(q1) - c2*cos(q2) + l2*cos(q2)) - I2 + m2*(c2*sin(q2) - l2*sin(q2))*(l1*sin(q1) - c2*sin(q2) + l2*sin(q2)) - l2*m3*sin(q2)*(l1*sin(q1) - c3*sin(q3) + l2*sin(q2) + l3*sin(q3)) + l2*m5*sin(q2)*(c5*sin(q5) - l1*sin(q1) - l2*sin(q2) + l4*sin(q4)) - l2*m4*cos(q2)*(l1*cos(q1) - c4*cos(q4) + l2*cos(q2)) - l2*m4*sin(q2)*(l1*sin(q1) - c4*sin(q4) + l2*sin(q2)) - l2*m3*cos(q2)*(l1*cos(q1) - c3*cos(q3) + l2*cos(q2) + l3*cos(q3)) + l2*m5*cos(q2)*(c5*cos(q5) - l1*cos(q1) - l2*cos(q2) + l4*cos(q4)), m3*(c3*cos(q3) - l3*cos(q3))*(l1*cos(q1) - c3*cos(q3) + l2*cos(q2) + l3*cos(q3)) - I3 + m3*(c3*sin(q3) - l3*sin(q3))*(l1*sin(q1) - c3*sin(q3) + l2*sin(q2) + l3*sin(q3)), c4*m4*cos(q4)*(l1*cos(q1) - c4*cos(q4) + l2*cos(q2)) - l4*m5*sin(q4)*(c5*sin(q5) - l1*sin(q1) - l2*sin(q2) + l4*sin(q4)) - I4 + c4*m4*sin(q4)*(l1*sin(q1) - c4*sin(q4) + l2*sin(q2)) - l4*m5*cos(q4)*(c5*cos(q5) - l1*cos(q1) - l2*cos(q2) + l4*cos(q4)), - I5 - c5*m5*sin(q5)*(c5*sin(q5) - l1*sin(q1) - l2*sin(q2) + l4*sin(q4)) - c5*m5*cos(q5)*(c5*cos(q5) - l1*cos(q1) - l2*cos(q2) + l4*cos(q4));
                                                                                                                                                                                      l1*m5*cos(q1)*(c5*cos(q5) - l2*cos(q2) + l4*cos(q4)) - l1*m3*cos(q1)*(l2*cos(q2) - c3*cos(q3) + l3*cos(q3)) - l1*m3*sin(q1)*(l2*sin(q2) - c3*sin(q3) + l3*sin(q3)) + l1*m5*sin(q1)*(c5*sin(q5) - l2*sin(q2) + l4*sin(q4)) + l1*m2*cos(q1)*(c2*cos(q2) - l2*cos(q2)) + l1*m4*cos(q1)*(c4*cos(q4) - l2*cos(q2)) + l1*m2*sin(q1)*(c2*sin(q2) - l2*sin(q2)) + l1*m4*sin(q1)*(c4*sin(q4) - l2*sin(q2)),                                                                                                                                                         l2*m5*cos(q2)*(c5*cos(q5) - l2*cos(q2) + l4*cos(q4)) - m2*(c2*cos(q2) - l2*cos(q2))^2 - m2*(c2*sin(q2) - l2*sin(q2))^2 - l2*m3*cos(q2)*(l2*cos(q2) - c3*cos(q3) + l3*cos(q3)) - I2 - l2*m3*sin(q2)*(l2*sin(q2) - c3*sin(q3) + l3*sin(q3)) + l2*m5*sin(q2)*(c5*sin(q5) - l2*sin(q2) + l4*sin(q4)) + l2*m4*cos(q2)*(c4*cos(q4) - l2*cos(q2)) + l2*m4*sin(q2)*(c4*sin(q4) - l2*sin(q2)),                           m3*(c3*cos(q3) - l3*cos(q3))*(l2*cos(q2) - c3*cos(q3) + l3*cos(q3)) - I3 + m3*(c3*sin(q3) - l3*sin(q3))*(l2*sin(q2) - c3*sin(q3) + l3*sin(q3)),                                                   - I4 - l4*m5*cos(q4)*(c5*cos(q5) - l2*cos(q2) + l4*cos(q4)) - l4*m5*sin(q4)*(c5*sin(q5) - l2*sin(q2) + l4*sin(q4)) - c4*m4*cos(q4)*(c4*cos(q4) - l2*cos(q2)) - c4*m4*sin(q4)*(c4*sin(q4) - l2*sin(q2)),                           - I5 - c5*m5*cos(q5)*(c5*cos(q5) - l2*cos(q2) + l4*cos(q4)) - c5*m5*sin(q5)*(c5*sin(q5) - l2*sin(q2) + l4*sin(q4));
                                                                                                                                                                                                                                                                                                                                                            l1*m3*cos(q1)*(c3*cos(q3) - l3*cos(q3)) + l1*m5*cos(q1)*(c5*cos(q5) + l4*cos(q4)) + l1*m3*sin(q1)*(c3*sin(q3) - l3*sin(q3)) + l1*m5*sin(q1)*(c5*sin(q5) + l4*sin(q4)) + c4*l1*m4*cos(q1)*cos(q4) + c4*l1*m4*sin(q1)*sin(q4),                                                                                                                                                                                                                                                                                                                  l2*m3*cos(q2)*(c3*cos(q3) - l3*cos(q3)) + l2*m5*cos(q2)*(c5*cos(q5) + l4*cos(q4)) + l2*m3*sin(q2)*(c3*sin(q3) - l3*sin(q3)) + l2*m5*sin(q2)*(c5*sin(q5) + l4*sin(q4)) + c4*l2*m4*cos(q2)*cos(q4) + c4*l2*m4*sin(q2)*sin(q4),                                                                                                   - I3 - m3*(c3*cos(q3) - l3*cos(q3))^2 - m3*(c3*sin(q3) - l3*sin(q3))^2,                                                                                                                         - I4 - c4^2*m4*cos(q4)^2 - c4^2*m4*sin(q4)^2 - l4*m5*cos(q4)*(c5*cos(q5) + l4*cos(q4)) - l4*m5*sin(q4)*(c5*sin(q5) + l4*sin(q4)),                                                     - I5 - c5*m5*cos(q5)*(c5*cos(q5) + l4*cos(q4)) - c5*m5*sin(q5)*(c5*sin(q5) + l4*sin(q4));
                                                                                                                                                                                                                                                                                                                                                                                                                                                l1*m5*cos(q1)*(c5*cos(q5) + l4*cos(q4)) + l1*m5*sin(q1)*(c5*sin(q5) + l4*sin(q4)) + c4*l1*m4*cos(q1)*cos(q4) + c4*l1*m4*sin(q1)*sin(q4),                                                                                                                                                                                                                                                                                                                                                                                                      l2*m5*cos(q2)*(c5*cos(q5) + l4*cos(q4)) + l2*m5*sin(q2)*(c5*sin(q5) + l4*sin(q4)) + c4*l2*m4*cos(q2)*cos(q4) + c4*l2*m4*sin(q2)*sin(q4),                                                                                                                                                                        0,                                                                                                                         - I4 - c4^2*m4*cos(q4)^2 - c4^2*m4*sin(q4)^2 - l4*m5*cos(q4)*(c5*cos(q5) + l4*cos(q4)) - l4*m5*sin(q4)*(c5*sin(q5) + l4*sin(q4)),                                                     - I5 - c5*m5*cos(q5)*(c5*cos(q5) + l4*cos(q4)) - c5*m5*sin(q5)*(c5*sin(q5) + l4*sin(q4));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    c5*l1*m5*cos(q1)*cos(q5) + c5*l1*m5*sin(q1)*sin(q5),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          c5*l2*m5*cos(q2)*cos(q5) + c5*l2*m5*sin(q2)*sin(q5),                                                                                                                                                                        0,                                                                                                                                                                                                    - c5*l4*m5*cos(q4)*cos(q5) - c5*l4*m5*sin(q4)*sin(q5),                                                                                                 - I5 - c5^2*m5*cos(q5)^2 - c5^2*m5*sin(q5)^2];


  Ff = [g*m5*(c5*sin(q5) - l1*sin(q1) - l2*sin(q2) + l4*sin(q4)) - g*m3*(l1*sin(q1) - c3*sin(q3) + l2*sin(q2) + l3*sin(q3)) - g*m2*(l1*sin(q1) - c2*sin(q2) + l2*sin(q2)) - g*m4*(l1*sin(q1) - c4*sin(q4) + l2*sin(q2)) - m2*(- l1*cos(q1)*dq1^2 + (c2*cos(q2) - l2*cos(q2))*dq2^2)*(l1*sin(q1) - c2*sin(q2) + l2*sin(q2)) + m2*(- l1*sin(q1)*dq1^2 + (c2*sin(q2) - l2*sin(q2))*dq2^2)*(l1*cos(q1) - c2*cos(q2) + l2*cos(q2)) - m4*(l1*cos(q1) - c4*cos(q4) + l2*cos(q2))*(l1*sin(q1)*dq1^2 + l2*sin(q2)*dq2^2 - c4*sin(q4)*dq4^2) + m4*(l1*sin(q1) - c4*sin(q4) + l2*sin(q2))*(l1*cos(q1)*dq1^2 + l2*cos(q2)*dq2^2 - c4*cos(q4)*dq4^2) - m3*(l1*sin(q1)*dq1^2 + l2*sin(q2)*dq2^2 + (l3*sin(q3) - c3*sin(q3))*dq3^2)*(l1*cos(q1) - c3*cos(q3) + l2*cos(q2) + l3*cos(q3)) + m3*(l1*cos(q1)*dq1^2 + l2*cos(q2)*dq2^2 + (l3*cos(q3) - c3*cos(q3))*dq3^2)*(l1*sin(q1) - c3*sin(q3) + l2*sin(q2) + l3*sin(q3)) + m5*(c5*sin(q5) - l1*sin(q1) - l2*sin(q2) + l4*sin(q4))*(- l1*cos(q1)*dq1^2 - l2*cos(q2)*dq2^2 + l4*cos(q4)*dq4^2 + c5*cos(q5)*dq5^2) - m5*(- l1*sin(q1)*dq1^2 - l2*sin(q2)*dq2^2 + l4*sin(q4)*dq4^2 + c5*sin(q5)*dq5^2)*(c5*cos(q5) - l1*cos(q1) - l2*cos(q2) + l4*cos(q4)) + g*m1*(c1*sin(q1) - l1*sin(q1));...
        m3*(l2*sin(q2) - c3*sin(q3) + l3*sin(q3))*(l1*cos(q1)*dq1^2 + l2*cos(q2)*dq2^2 + (l3*cos(q3) - c3*cos(q3))*dq3^2) - m2*(c2*cos(q2) - l2*cos(q2))*(- l1*sin(q1)*dq1^2 + (c2*sin(q2) - l2*sin(q2))*dq2^2) - m4*(c4*sin(q4) - l2*sin(q2))*(l1*cos(q1)*dq1^2 + l2*cos(q2)*dq2^2 - c4*cos(q4)*dq4^2) + m4*(c4*cos(q4) - l2*cos(q2))*(l1*sin(q1)*dq1^2 + l2*sin(q2)*dq2^2 - c4*sin(q4)*dq4^2) + m5*(c5*sin(q5) - l2*sin(q2) + l4*sin(q4))*(- l1*cos(q1)*dq1^2 - l2*cos(q2)*dq2^2 + l4*cos(q4)*dq4^2 + c5*cos(q5)*dq5^2) - m3*(l1*sin(q1)*dq1^2 + l2*sin(q2)*dq2^2 + (l3*sin(q3) - c3*sin(q3))*dq3^2)*(l2*cos(q2) - c3*cos(q3) + l3*cos(q3)) - m5*(c5*cos(q5) - l2*cos(q2) + l4*cos(q4))*(- l1*sin(q1)*dq1^2 - l2*sin(q2)*dq2^2 + l4*sin(q4)*dq4^2 + c5*sin(q5)*dq5^2) - g*m3*(l2*sin(q2) - c3*sin(q3) + l3*sin(q3)) + g*m5*(c5*sin(q5) - l2*sin(q2) + l4*sin(q4)) + g*m2*(c2*sin(q2) - l2*sin(q2)) + g*m4*(c4*sin(q4) - l2*sin(q2)) + m2*(- l1*cos(q1)*dq1^2 + (c2*cos(q2) - l2*cos(q2))*dq2^2)*(c2*sin(q2) - l2*sin(q2));...
        m5*(c5*sin(q5) + l4*sin(q4))*(- l1*cos(q1)*dq1^2 - l2*cos(q2)*dq2^2 + l4*cos(q4)*dq4^2 + c5*cos(q5)*dq5^2) - m3*(c3*sin(q3) - l3*sin(q3))*(l1*cos(q1)*dq1^2 + l2*cos(q2)*dq2^2 + (l3*cos(q3) - c3*cos(q3))*dq3^2) + m3*(c3*cos(q3) - l3*cos(q3))*(l1*sin(q1)*dq1^2 + l2*sin(q2)*dq2^2 + (l3*sin(q3) - c3*sin(q3))*dq3^2) - m5*(c5*cos(q5) + l4*cos(q4))*(- l1*sin(q1)*dq1^2 - l2*sin(q2)*dq2^2 + l4*sin(q4)*dq4^2 + c5*sin(q5)*dq5^2) + g*m3*(c3*sin(q3) - l3*sin(q3)) + g*m5*(c5*sin(q5) + l4*sin(q4)) + c4*g*m4*sin(q4) - c4*m4*sin(q4)*(l1*cos(q1)*dq1^2 + l2*cos(q2)*dq2^2 - c4*cos(q4)*dq4^2) + c4*m4*cos(q4)*(l1*sin(q1)*dq1^2 + l2*sin(q2)*dq2^2 - c4*sin(q4)*dq4^2);
        m5*(c5*sin(q5) + l4*sin(q4))*(- l1*cos(q1)*dq1^2 - l2*cos(q2)*dq2^2 + l4*cos(q4)*dq4^2 + c5*cos(q5)*dq5^2) - m5*(c5*cos(q5) + l4*cos(q4))*(- l1*sin(q1)*dq1^2 - l2*sin(q2)*dq2^2 + l4*sin(q4)*dq4^2 + c5*sin(q5)*dq5^2) + g*m5*(c5*sin(q5) + l4*sin(q4)) + c4*g*m4*sin(q4) - c4*m4*sin(q4)*(l1*cos(q1)*dq1^2 + l2*cos(q2)*dq2^2 - c4*cos(q4)*dq4^2) + c4*m4*cos(q4)*(l1*sin(q1)*dq1^2 + l2*sin(q2)*dq2^2 - c4*sin(q4)*dq4^2);
        c5*g*m5*sin(q5) - c5*m5*cos(q5)*(- l1*sin(q1)*dq1^2 - l2*sin(q2)*dq2^2 + l4*sin(q4)*dq4^2 + c5*sin(q5)*dq5^2) + c5*m5*sin(q5)*(- l1*cos(q1)*dq1^2 - l2*cos(q2)*dq2^2 + l4*cos(q4)*dq4^2 + c5*cos(q5)*dq5^2)];

  Fg = -eye(5);

  fq = M\Ff;

  gq = M\Fg;

  dJy = Jy;
  Lf = dJy*fq;
  A = dJy*gq;
  %     con = cond(A)

  % mu = ddyd -(2*e*dy2 + e^2*y2);
  mu = ddyd -(kd*dy2 + kp*y2);

  % [ddq1, ddq2] = Lf + A*u
  u = A \ (-Lf + mu);
  fx = [x(i,6:10)'; fq];
  gx = [zeros(5,5); gq];
  dx = fx + gx*u;
  retval(i,:) = u(2:5)';
  retval2(i,:) = dx';
    
end

% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

endfunction
