function [y,t,k1,k2,k3,k4]=wRK4(f,y0,a,b,N)
h=(b-a)/N
y(1)=y0;t(1)=a;
for i=1:N
    t(i+1)=a+i*h;
    
    k1(i)=feval(f,t(i),y(i));
    k2(i)=feval(f,t(i)+h/2,y(i)+(h/2)*k1(i));
    k3(i)=feval(f,t(i)+h/2,y(i)+(h/2)*k2(i));
    k4(i)=feval(f,t(i)+h,y(i)+h*k3(i));
    
    y(i+1)=y(i)+(h/6)*(k1(i)+2*k2(i)+2*k3(i)+k4(i));

    %disp([i k1 k2 k3 k4 y(i+1) ]);
    
end
%plot(t,y)


    
