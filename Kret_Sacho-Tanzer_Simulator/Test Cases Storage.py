#Test Cases
import EOMs_main as EOMs
import numpy as np
import Parameters_Test as p

case_num = int(input("Enter the test case number: "))

#1. Zero Initial Conditions
if case_num == 1:
    title = "Zero Initial Conditions"
    x_0 = np.array([0, #p_n
                    0, #p_e
                    0, #p_d
                    0,  #u
                    0, #v
                    0, #w
                    0, #phi
                    0, #theta
                    0, #psi
                    0, #p
                    0, #q
                    0 #r
                    ])

    U = np.array([0, #f_x
                    0, #f_y
                    0, #f_z
                    0, #l
                    0, #m
                    0 #n
                    ])


myPlane = EOMs.rigid_body(p.mass, p.J_xx, p.J_yy, p.J_zz, p.J_xz, p.S, p.b, p.c, p.S_prop, p.rho, p.k_motor, p.k_T_p, p.k_Omega, p.e, gravity=True)
t, x = myPlane.simulate(x_0, U, 0, 10, dt=0.1)
EOMs.pr.plot_results(t, x, title=title)