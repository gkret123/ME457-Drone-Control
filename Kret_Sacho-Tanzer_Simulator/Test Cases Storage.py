#Test Cases
import EOMs_main as EOMs
import numpy as np

case_num = int(input("Enter the test case number: "))

#1. Zero Initial Conditions
if case_num == 1:
    IC_x_0 = np.array([0, #p_n
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

    IC_U = np.array([0, #f_x
                    0, #f_y
                    0, #f_z
                    0, #l
                    0, #m
                    0 #n
                    ])

t_zero_IC, x_zero_IC = EOMs.myPlane.simulate(IC_x_0, IC_U, 0, 10, dt=0.1)
EOMs.pr.plot_results(t_zero_IC, x_zero_IC, "Zero IC's")