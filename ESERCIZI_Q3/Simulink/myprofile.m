function profile=myprofile(t,a,b)

profile= ones(size(t));
profile(t<a)= 0.5*(1+cos(pi*(1+t(t<a)/a)));
profile( t>(max(t)-b) )  =  ...
     0.5*(1 + cos( pi*(    (t(t>(max(t)-b))- (max(t)-b))  /b)   ))  ;
end