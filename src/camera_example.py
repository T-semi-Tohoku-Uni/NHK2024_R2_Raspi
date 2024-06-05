from NHK2024_Camera_Library import MainProcess
from hardware_module import Field
import cv2

if __name__ == "__main__":
    mainprocess = MainProcess(
        field=Field.BLUE,
        save_image_dir="cameratest",
        show=True
    )
    mainprocess.thread_start()
    
    while True:
        try:
            # if mainprocess.detector.show:   
            frame, id = mainprocess.q_out.get()
            cv2.imshow(f'{id}', frame)
            key = cv2.waitKey(1)
            if key == ord("q"):
                break
            items,x,y,z,is_obtainable = mainprocess.update_ball_camera_out()
            # print(f"\n{items=}, {x=}, {y=}, {z=}, {is_obtainable=}")
            # _, _= mainprocess.q_out.get()
            continue
            
            # items,x,y,z,is_obtainable = mainprocess.update_ball_camera_out()
            # x,y,z = mainprocess.update_silo_camera_out()
            # forward, right, left, x, theta = mainprocess.update_line_camera_out()
            # print(f"\n{items=}, {x=}, {y=}, {z=}, {is_obtainable=}")
            # print(f"\n{x=}, {y=}, {z=}")
            # print(f"\n{forward=}, {right=}, {left=}, {x=}, {theta=}")
            
        except KeyboardInterrupt:
            break
        
    mainprocess.finish()
    cv2.destroyAllWindows()
    