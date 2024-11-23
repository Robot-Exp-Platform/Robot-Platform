DH = [0     0.333   0        0;
      0     0       0        -pi/2;
      0     0.316   0        pi/2;
      0     0       0.0825   pi/2;
      0     0.384   -0.0825  -pi/2;
      0     0       0        pi/2;
      0     0       0.088    pi/2;
      -0.7854   0.107    0   0]; % add hand

q = [0.3056, -0.6165, 0.9617, -2.1034, 0.2473, 2.0862, 1.2662];

for i=1:7
    DH(i,1) = q(i);
end
M = diag([1,1,1,1])

for i=1:7

    R=[cos(DH(i,1))              -sin(DH(i,1))               0;
       sin(DH(i,1))*cos(DH(i,4)) cos(DH(i,1))*cos(DH(i,4))  -sin(DH(i,4));
       sin(DH(i,1))*sin(DH(i,4)) cos(DH(i,1))*sin(DH(i,4))   cos(DH(i,4))];

    T=[DH(i,3);
        -DH(i,2)*sin(DH(i,4));
        DH(i,2)*cos(DH(i,4))];

    Currect_M =[R T; zeros(1,3) 1]

    M=M*Currect_M;
    M
end