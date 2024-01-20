function [Q_X, R_U] = my_hessian(Q, Q_N, R, N)
    
    Q_X = [];
    for kk = 1:N-1               
        Q_X = blkdiag(Q_X, Q);
    end
    Q_X = blkdiag(Q_X, Q_N);
    
    R_U = [];
    for kk = 1:N                
        R_U = blkdiag(R_U, R);
    end
    
end