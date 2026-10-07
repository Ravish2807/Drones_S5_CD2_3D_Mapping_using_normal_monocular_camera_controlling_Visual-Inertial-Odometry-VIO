function x = pack_state(p, v, R, w)
%PACK_STATE  Inverse of UNPACK_STATE: x = [p; v; vec(R); w]  [18x1].
x = [p(:); v(:); R(:); w(:)];
end
