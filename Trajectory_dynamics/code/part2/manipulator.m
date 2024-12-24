function output=manipulator(B,u,n)
  
    difference= u- n;

    output= inv(B)*difference;
 

end
