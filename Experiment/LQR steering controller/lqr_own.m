function K  = lqr_own(A,B,Q,R)
    
    [p1,p2] = size(Q);
    p = [-A' -Q; -B*inv(R)*B' A];

    [v,d] = eig(p);
    d = diag(d);
    
    [m,n] = sort(real(d));
    v = v(:,n);

    phis = v(:,1:p1);
    phis1 = phis(1:p1,:);
    phis2 = phis(p1+1:end,:);

    K = real(inv(R)*B'*phis1*inv(phis2));
end