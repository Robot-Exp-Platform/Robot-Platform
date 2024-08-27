    njoint = 7;
    horizon = 4;
    qd = eye(njoint);
    Q1 = [];
    for i=1:horizon
        Q1((i-1)*njoint+1:i*njoint,(i-1)*njoint+1:i*njoint)=qd;
        if i==horizon
            Q1((i-1)*njoint+1:i*njoint,(i-1)*njoint+1:i*njoint)=qd;
        end
    end

    % The velocity
    Vdiff = eye(horizon*njoint)-diag(ones(1,(horizon-1)*njoint),njoint);
    Q2 = Vdiff(1:(horizon-1)*njoint,:)'*Vdiff(1:(horizon-1)*njoint,:);

    % The acceleration
    Adiff = Vdiff-diag(ones(1,(horizon-1)*njoint),njoint)+diag(ones(1,(horizon-2)*njoint),njoint*2);
    Q3 = Adiff(1:(horizon-2)*njoint,:)'*Adiff(1:(horizon-2)*njoint,:);

    % The weight
    c = [0.8,10,20];
    Qref = Q1*c(1)+Q2*c(2);
    Qabs = Q3*c(3);
    H = Qref+Qabs;
    
    disp(H)