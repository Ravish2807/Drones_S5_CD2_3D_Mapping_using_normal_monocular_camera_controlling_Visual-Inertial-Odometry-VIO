function S = hat(w)
%HAT  Skew-symmetric matrix [w]x such that hat(w)*b = cross(w,b).
%   INPUT : w  [3x1]  any 3-vector
%   OUTPUT: S  [3x3]  skew-symmetric matrix, S' = -S
S = [  0    -w(3)   w(2);
      w(3)    0    -w(1);
     -w(2)   w(1)    0  ];
end
