function koopman_matrix = identify_matrix(X_lift, Y_lift, threshold)
% identifies a single matrix in the bilinear model either for the generator
% or the koopman operator
% truncated SVD
[U_svd, S_svd, V_svd] = svd(X_lift, 'econ');
r = max(find(diag(S_svd)>threshold));
disp(['truncating SVD at r = ', num2str(r)])

U_trunc = U_svd(:, 1:r);
S_trunc = S_svd(1:r, 1:r);
V_trunc = V_svd(:, 1:r);
koopman_matrix = Y_lift*V_trunc*inv(S_trunc)*U_trunc';
end

