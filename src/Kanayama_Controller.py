#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3Stamped
import pandas as pd


class NNode:
    def __init__(self):
        self.path = []
        rospy.Subscriber('/mobile_system_control/ego_vehicle', Float32MultiArray, self.callback, queue_size=10)
        self.cur_x = 0
        self.cur_y = 0 
        self.cur_theta = 0 
        self.cur_vel = 0
        self.cur_steer = 0
        self.pub_msg = rospy.Publisher('/mobile_system_control/control_msg', Vector3Stamped, queue_size=10)
    
    def callback(self,msg):
        self.cur_x = msg.data[0]
        self.cur_y = msg.data[1]
        self.cur_theta = msg.data[2]
        self.cur_vel = msg.data[3]
        self.cur_steer = msg.data[4]
        self.cur_pos = [self.cur_x, self.cur_y, self.cur_theta, self.cur_vel, self.cur_steer]
				
				
        # Implement your Kanayama controller logic here
        # This is a placeholder; replace it with your actual controller
        kx = 4
        ky = 0.3
        kw = 0.6  

        v_ref = 20*1000/3600
        w_ref = 0.05
        max_vel=20*1000/3600
    		
        x_ref = []
        y_ref = []
        theta_ref = 0
        
        csv_file_path = 'yangpath3.csv'
        df = pd.read_csv(csv_file_path)
        matrix = df.to_numpy()

        for i in range(len(matrix)):
            x_point = matrix[i][0]
            y_point = matrix[i][1]
            x_ref.append(x_point)
            y_ref.append(y_point)

        min_distance = float('inf') #가장 가까운 골 포인트 찾는 로직
        nearest_path = None
        index = 0
        
        for i in range(len(x_ref)):
            distance = ((self.cur_x - x_ref[i]) ** 2 + (self.cur_y - y_ref[i]) ** 2) ** 0.5
						
            if distance < min_distance:
                if (i == len(x_ref) - 1):
                    i = 0
                    index = i
                    nearest_path = [x_ref[index+1], y_ref[index+1]]
                    theta_ref = math.atan2(y_ref[index+1] - y_ref[index], x_ref[index+1] - x_ref[index])
                    break
                min_distance = distance
                index = i
                theta_ref = math.atan2(y_ref[index+1] - y_ref[index], x_ref[index+1] - x_ref[index])
                
                nearest_path = [x_ref[index+1], y_ref[index+1]]
																

        x_ref = nearest_path[0]
        y_ref = nearest_path[1]
        
        print(x_ref, y_ref)
        print("theta reference:" + str(theta_ref*180/3.14))
        #카나야마 컨트롤 메인
        x_e = (x_ref - self.cur_x) * math.cos(self.cur_theta) + (y_ref - self.cur_y) * math.sin(self.cur_theta)
        y_e = -(x_ref - self.cur_x) * math.sin(self.cur_theta) + (y_ref - self.cur_y) * math.cos(self.cur_theta)
        theta_e = theta_ref - self.cur_theta  

        kana_vel = v_ref * math.cos(theta_e) + kx * x_e
        omega = w_ref + v_ref * (ky * y_e + kw * math.sin(theta_e))
        print("통나무vel:" + str(v_ref * math.cos(theta_e))+ ", " + str(kx *x_e))
        print("통나무: " + str(w_ref) +", "  + str(v_ref * ky * y_e)+ ", "+str(kw * math.sin(theta_e)))
				# omega : ky 항이 가장 비중이 큼, 3가지 항의 크기를 유사하게 조절 
			  
        throttle = 0
        brake = 0
        kana = Vector3Stamped()
        kana.vector.x = kana_vel
        kana.vector.y = omega
        kana.vector.z = 0
        
        throttle = 1
        brake = 0	
      
        steer_fake = (math.atan2(1.023*omega, self.cur_vel)*180/math.pi)
        print("steer fake: "+ str(steer_fake))
        steer = -min(20, max(steer_fake, -20)) /20        #steer 범위 제한
        
        cmsg = Vector3Stamped()
        cmsg.header.frame_id = 'group8'
        cmsg.vector.x = throttle
        cmsg.vector.y = steer
        cmsg.vector.z = brake
        self.pub_msg.publish(cmsg)
        print(cmsg)        

    def main(self):   
        rospy.spin()

if __name__ == '__main__' :
    
    rospy.init_node('msc', anonymous=False)
    node = NNode()
    print('실행전')
    node.main()
    print('실행후')