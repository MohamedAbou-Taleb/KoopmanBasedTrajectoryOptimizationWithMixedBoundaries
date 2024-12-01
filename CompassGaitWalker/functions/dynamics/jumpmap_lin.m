function linearized_jump_map = jumpmap_lin(in1,in2)
%JUMPMAP_LIN
%    LINEARIZED_JUMP_MAP = JUMPMAP_LIN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    13-Sep-2024 17:58:06

a = in2(:,1);
b = in2(:,2);
m = in2(:,3);
m_h = in2(:,4);
theta_swing = in1(1,:);
theta_stance = in1(2,:);
t2 = a+b;
t3 = a.^2;
t4 = a.^3;
t5 = b.^2;
t6 = b.^3;
t7 = theta_swing.*2.0;
t8 = theta_stance.*2.0;
t9 = -theta_stance;
t12 = a.*b.*m.*2.0;
t13 = a.*b.*m_h.*4.0;
t10 = -t8;
t11 = m.*t5;
t14 = m.*t3.*3.0;
t15 = m_h.*t3.*2.0;
t16 = m_h.*t5.*2.0;
t17 = t9+theta_swing;
t18 = cos(t17);
t19 = t7+t10;
t20 = cos(t19);
t21 = m.*t3.*t20;
t22 = t11.*t20;
t24 = a.*b.*m.*t20.*-2.0;
t25 = -t21;
t26 = -t22;
t27 = t11+t12+t13+t14+t15+t16+t24+t25+t26;
t28 = 1.0./t27;
linearized_jump_map = reshape([a.*m.*t2.*t18.*t28.*-2.0,a.*b.*m.*t28.*-2.0,(t28.*(a.*t22.*2.0+b.*t21.*4.0-m.*t4.*2.0-m_h.*t4+m_h.*t6+a.*m_h.*t5-b.*m_h.*t3+m.*t4.*t20.*2.0+m_h.*t4.*t20+m_h.*t6.*t20+a.*m_h.*t5.*t20.*3.0+b.*m_h.*t3.*t20.*3.0))./b,t2.*t18.*t28.*(a.*m+a.*m_h+b.*m_h).*2.0],[2,2]);
end