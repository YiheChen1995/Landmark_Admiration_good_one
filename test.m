fun = @(x)100*(x(2)-x(1)^2)^2 + (1-x(1))^2;
lb  = [0,0.2];
ub  = [0.5,0.8];
A   = [1,1;1,2];
b   = [2;3];
x0 = [1/4,1/4];

Aeq = [];
beq = [];
spam = 3;

nonlcon = @circlecon;
x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,@(x)nonlcon(x,spam))

function [c,ceq] = circlecon(x,spam)
  td2 = spam*2;
  c = (x(1)-1/3)^2 + (x(2)-1/3)^2 - (1/3)^2;
  c = [c;c;c;c];
  ceq = [];
end