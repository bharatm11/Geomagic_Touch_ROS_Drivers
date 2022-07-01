% TODO: pole placement
kp = 10;    % propotional gain
kd = 5;     % differential gain
t = 1e-3;   % 離散化における周期 (サンプリング周期)
omega = 30;  % 疑似微分に用いるLPFのカットオフ角周波数
tau = 1.0 / omega;
Hd = tf([kd 0], [tau 1])
Hp = tf([kp], [1])
H = Hd + Hp % PD制御器
Hc = c2d(H, t, 'tustin') % tustin変換 (双一次z変換) により離散化
