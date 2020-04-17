DOG_LEG_A1=0.0;
DOG_LEG_A2=55.0;
DOG_LEG_A3=60.0;
DOG_LEG_ALPHA1=pi / 2;
DOG_LEG_ALPHA2=0.0;
DOG_LEG_ALPHA3=0.0;
DOG_LEG_D1=0.0;
DOG_LEG_D2=27.0;
DOG_LEG_D3=0.0;
d2 = DOG_LEG_D2;
a2 = DOG_LEG_A2;
a3 = DOG_LEG_A3;

LL(1) = Link([0 0 0 pi/2 0]);
LL(2) = Link([0 27 55 0 0]);
LL(3)=Link([0 0 60 0 0]);
E = 40;
H = 20;
MAX_X = 85;
otherleg = SerialLink(LL);
alpha = (0:0.1:2*pi);
x = MAX_X - H.*sin(alpha);
for j=1:63
x(j) = MAX_X - H.*sin(alpha(j));
if x(j)>MAX_X
    x(j) = MAX_X;
end
end

z = -E .* cos(alpha) - 25;
y = -27 + (E-10) * cos(alpha);

px = x;
py = y;
pz = z;

rxy = px .* px + py .* py;
rxyz = px .* px + py .* py + pz .* pz;
c3 = (rxyz - a2 .* a2 - a3 .* a3 - d2 .* d2) / (2 .* a2 .* a3);
s3 = sqrt(1 - c3 .* c3);
theta1 = atan2(py, px) - atan2(-d2, sqrt(rxy - d2 .* d2));
theta3 = atan2(s3, c3);
s1 = sin(theta1);
c1 = cos(theta1);
s3 = sin(theta3);
c3 = cos(theta3);
theta2 = atan2(pz .* (a3 + a2 .* c3) + a2 .* s3 .* (px .* c1 + py .* s1), a3 .* (px .* c1 + py .* s1) + a2 .* c3 .* (px .* c1 + py .* s1) - pz .* a2 .* s3);
theta2 = theta2-theta3;

%theta2 = theta2 - (25/180)*pi;
plot3(x,y,z);
for i=1:63
    %poit = otherleg.fkine([theta1(i) theta2(i) theta3(i)]);
    otherleg.plot([theta1(i) theta2(i) theta3(i)]);
end

