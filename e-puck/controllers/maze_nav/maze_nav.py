from controller import Robot
import numpy as np
import cv2


robot= Robot()



timestep= int(robot.getBasicTimeStep())


camera=  robot.getDevice('Camera')
camera.enable(timestep)


left_motor= robot.getDevice("left wheel motor")
left_motor.setPostition(float("inf"))
left_motor.setVelocity(0.0)



right_motor= robot.getDevice("right wheek motor")
right_motor.setPostition(float('inf'))
right_motor.setVelocity(0.0)


print('Camera resolution:', camera.getWidth ,'x' , camera.getHeight)


while timestep!=-1:
    image= camera.getImage()
    width= camera.getWidth()
    height=camera.getHeight()

    img= np.frombuffer(image, np.uint8).reshape((height,width,4))

    img_bgr= cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)


    cv2.imshow("Robot Camera Feed", cv2.resize(img_bgr, 300,300))

    cv2.waitKey(1)


    left_motor.setVelocity(2.0)
    right_motor.setVelocity(2.0)

cv2.destroyAllWindows()    
    

