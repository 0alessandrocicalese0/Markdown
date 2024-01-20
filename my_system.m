function [A_x0, B_z] = my_system(A, B, N)
    n = size(A, 2);
    m = size(B, 2);

    A_x0 = [];
    for kk = 1:N
        A_x0 = [ A_x0; 
                A^kk];
    end
    
    B_z = zeros(n*N, m*N);
    for kk = 1:N
        for hh = 1:kk
            B_z((kk-1)*n +1 : (kk)*n , (hh-1)*(m)+1:hh*m) = A^(kk-hh)*B;
        end
    end
    
end