function C = myCostFunction(X, H, f)   
    w = reshape(X', [], 1) - f;
    C = w'*H*w;
end