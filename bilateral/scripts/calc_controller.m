kp = 10
kd = 5
t = 1e-3
tau = 1.5e-1
Hd = tf([kd 0], [tau 1])
Hp = tf([kp], [1])
H = Hd + Hp
Hc = c2d(H, t, 'tustin')
