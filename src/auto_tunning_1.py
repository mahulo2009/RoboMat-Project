#!/usr/bin/env python

import rospy

from robomat_project.srv import *



def hover(p):

	print("------------------------------------")
	print("Try: ", p)

	try:
		rospy.wait_for_service('test_srv')
		service = rospy.ServiceProxy('test_srv',PidTunning)
		result = service(1,p[0],p[1],0,10)

		print("Result: ", result)

	except Exception as e:
		print(e)

	

	return result

def cost_fun(result):

	RT = result.rise_time
	OS = result.percent_overshoot
	ST = result.settling_time

	if RT==-1:
		RT= 20000.0
	RT=RT/20000.0

	if OS==-1:
		OS = 100
	OS=OS/100.0

	if ST==-1:
		ST = 20000.0
	ST=ST/20000.0

	cost =  OS * 6 + RT * 5 + ST * 1

	print("OS {}, RT = {} ST = {} Total {}".format(OS * 6, RT * 5,ST * 1,cost))

	print("------------------------------------")

	return cost


def twiddle(tol=0.05):
    p = [0.1, 0.1]
    dp = [0.10, 0.10]
    soln = hover(p)
    best_cost = cost_fun(soln)

    ### TO DO: Complete the Twiddle ALgorithm
    it = 0
    while sum(dp) > tol:

        print("---------------------------------------------------------------------------------------------------")
        print("Iteration {}, best error = {} sum dp = {}".format(it, best_cost,sum(dp)))




        for i in range(len(p)):
            p[i] += dp[i]
            # Can't have negative gains
            if p[i] < 0:
                p[i] = 0
            soln = hover(p)
            cost = cost_fun(soln)

            if cost < best_cost:
                best_cost = cost
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                # Can't have negative gains
                if p[i] < 0:
                    p[i] = 0
                soln = hover(p)
                cost = cost_fun(soln)

                if cost < best_cost:
                    best_cost = cost
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    # Can't have negative gains
                    if p[i] < 0:
                        p[i] = 0
                    dp[i] *= 0.9
        it += 1
    return p



	
if __name__ == "__main__":

	print("Ros auto-tunning tool")

	rospy.init_node("auto_tunning")

	p = twiddle()

	print ("Final p:" ,p)


#[0.10980999999999999, 0.48697955417886274]
#[0.1486809188705028, 0.8729722824460904]

