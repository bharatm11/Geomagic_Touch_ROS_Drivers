kp = 500
kd = 50
t = 1e-3
tau = 1e-2
Hd = tf([kd 0], [tau 1])
Hp = tf([kp], [1])
H = Hd + Hp
Hc = c2d(H, t, 'tustin')
