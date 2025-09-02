clear
syms Xe Ye The ai bi Xi Yi

C = cos(The); S = sin(The); 
A = [C, -S; S C]; ui = [ai;bi]; Pi=[Xi;Yi]; re=[Xe;Ye]; X=[re;The];
Li = re+A*ui-Pi;
li = sqrt(Li.'*Li);
Hi = simplify(jacobian(li, X));
pretty(Hi.')

ltxli = latex(li);
ltxHi = latex(Hi.');
str10 = {'Xe' 'Ye' 'The' 'ai' 'bi' 'Xi' 'Yi'};
str11 = {'X_\text e' 'Y_\text e' '\theta_\text e' 'a_i' 'b_i' 'X_i' 'Y_i'};
ltxHi = strrep(ltxHi, '\mathrm', '');
ltxli = strrep(ltxli, '\mathrm', '');

for is=1:length(str10)
ltxHi = strrep(ltxHi, str10{is}, str11{is});
ltxli = strrep(ltxli, str10{is}, str11{is});
end

str20 = {'\X_i', '\\',...
    '\cos\left({\theta_\text e}\right)', ...
    '\sin\left({\theta_\text e}\right)'};
str21 = {'X_i' '\\{\tiny } \\' 'C' 'S'};
ltxHi = strrep(ltxHi, '\mathrm', '');

for is=1:length(str20)
ltxHi = strrep(ltxHi, str20{is}, str21{is});
ltxli = strrep(ltxli, str20{is}, str21{is});
end

ltxHi = ['\boldsymbol J_i = ' ltxHi '^T']

ltxli = ['\ell_i = ' ltxli ]

%%
h=3; a=.1; b=.15; 
PP_ = [0 0; 0 h; h h; h 0];
AB_ = [-a, b; a, b; a,-b; -a,-b];
X_ = [1.5; 1.5; 0];

np = size(PP_,1);
JJ = zeros(np,3);
for ip = 1:np
JJ(ip,1:3) = double(subs(Hi,[Xi Yi ai bi Xe Ye The], [PP_(ip,:) AB_(ip,:) X_.']));
end

JJ, rJ = rank(JJ) 
