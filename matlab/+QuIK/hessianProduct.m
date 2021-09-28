function A = hessianProduct(lt, J, dQ)
    % HESSIANPRODUCT Computes the product of H * dQ. , where H is the
    % geometric hessian of the robot. This function never computes H
    % explicitely, and instead just reduces the computed values through 
    % multiplication as they are computed. The result is added to A.
    %
    % A = hessianProduct(lt, J, dQ)
    
    DOF = int32(size(lt, 1));
    
    % Break out some variables
    is_rev = ~lt;
    
    % Build jacobian gradient    
    A = zeros(6, DOF);
    
    % Loop
    if all( is_rev )
        % Special code for revolute joint robots only (most common, can
        % skip branched coding which is slightly faster)
        
        for k = 1:DOF

            % Off diagonal terms first
            for i = 1:k-1
                % A(4:6, k) += jwi x jwk * dQi
                A(4:6, k) = A(4:6, k) + cross( J(4:6, i), J(4:6, k) ) * dQ(i);
                
                % A(1:3, k) += jwi x jvk*dQi
                cp = cross( J(4:6, i), J(1:3, k) );
                A(1:3, k) = A(1:3, k) + cp * dQ(i);

                % Symmetry
                A(1:3, i) = A(1:3, i) + dQ(k) * cp;
            end
            
            % For diagonal entries, can skip the omega term since jwk x jwk
            % = 0
            % A(4:6, k) += jwi x jwk * dQi
            % A(4:6, k) = A(4:6, k) + cross( J(4:6, k), J(4:6, k) ) * dQ(k);

            % A(1:3, k) += jwi x jvk*dQi
            A(1:3, k) = A(1:3, k) + cross( J(4:6, k), J(1:3, k) ) * dQ(k);
        end
        
    else
        % General case
        
        for k = 1:DOF

            % Off diagonal terms first
            for i = 1:k-1
               if is_rev(i)
                   if is_rev(k)
                        % A(4:6, k) += jwi x jwk * dQi
                        A(4:6, k) = A(4:6, k) + cross( J(4:6, i), J(4:6, k) ) * dQ(i);
                   end

                   % A(1:3, k) += jwi x jvk*dQi
                   cp = cross( J(4:6, i), J(1:3, k) );
                   A(1:3, k) = A(1:3, k) + cp * dQ(i);

                   % Symmetry
                   A(1:3, i) = A(1:3, i) + cp * dQ(k);
               end
            end
            
            % Special code for diagonal entries
            % Don't do symmetry, and can skip diagonal entries
            % So, its just this
            if is_rev(k)
                A(1:3, k) = A(1:3, k) + cross( J(4:6, k), J(1:3, k) ) * dQ(k);
            end
        end
        
    end
    
end