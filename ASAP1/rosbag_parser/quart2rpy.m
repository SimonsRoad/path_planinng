function rpy = quart2rpy(quart)
    rpy = zeros(3,1);
    X = quart(4);
    Y = quart(1);
    Z = quart(2);
    W = quart(3);
    rpy(1) = atan2((2*X*Y + Z*W), 1-2*(Y*Y + Z*Z));
    rpy(2) = asin(2*(X*Z-W*Y));
    rpy(3) = atan2(2*(X*W + Y*Z), 1-2*(Z*Z + W*W));
end