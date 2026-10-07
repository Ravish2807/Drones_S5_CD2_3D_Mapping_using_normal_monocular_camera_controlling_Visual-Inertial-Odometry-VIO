function w = vee(S)
%VEE  Inverse of HAT: extracts the 3-vector from a skew-symmetric matrix.
%   INPUT : S  [3x3]  skew-symmetric matrix
%   OUTPUT: w  [3x1]  vector with hat(w) = S
w = [S(3,2); S(1,3); S(2,1)];
end
