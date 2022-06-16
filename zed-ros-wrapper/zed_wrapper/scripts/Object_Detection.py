#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS Düğümü Oluşturma
"""
from std_msgs.msg import Int8
from std_msgs.msg import Int32
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import math


class Kamera():
    def __init__(self):
        rospy.init_node("kamera_dugumu")
        self.bridge = CvBridge()
        rospy.Subscriber("zed2i/zed_node/rgb_raw/image_raw_color",Image,self.nesne)
        
        rospy.spin()
    def nesne(self, mesaj):
        frame = self.bridge.imgmsg_to_cv2(mesaj, "bgr8")
        frame = cv2.resize(frame, (600, 360))
        pub = rospy.Publisher("prediction",Int8,queue_size=1)
        pub1 = rospy.Publisher("start_x",Int32,queue_size=1)
        pub2 = rospy.Publisher("start_y",Int32,queue_size=1)
        pub3 = rospy.Publisher("end_x",Int32,queue_size=1)
        pub4 = rospy.Publisher("end_y",Int32,queue_size=1)
        frame_width = frame.shape[1]

        frame_height = frame.shape[0]

        frame_blob = cv2.dnn.blobFromImage(frame, 1 / 255, (416, 416), swapRB=True, crop=False)

        labels = ["Dur", "Durak", "Girisi_olmayan", "İleri_veya_saga", "İleri_veya_sola", "Kirmizi_isik", "Park",
                  "Parka_yasak", "Saga_don", "Saga_donulmez", "Sola_don", "Sola_donulmez", "Yesil_isik"]

        colors = ["0,0,255", "0,0,255", "255,0,0", "255,255,0", "0,255,0"]
        colors = [np.array(color.split(",")).astype("int") for color in colors]
        colors = np.array(colors)
        colors = np.tile(colors, (18, 1))

        model = cv2.dnn.readNetFromDarknet("/home/ali/Downloads/yolov3-tiny_custom.cfg",
                                           "/home/ali/Downloads/yolov3-tiny_custom_last.weights")

        layers = model.getLayerNames()
        output_layer = [layers[layer[0] - 1] for layer in model.getUnconnectedOutLayers()]

        model.setInput(frame_blob)

        detection_layers = model.forward(output_layer)

        ############## NON-MAXIMUM SUPPRESSION - OPERATION 1 ###################

        ids_list = []
        boxes_list = []
        confidences_list = []

        ############################ END OF OPERATION 1 ########################

        for detection_layer in detection_layers:
            for object_detection in detection_layer:

                scores = object_detection[5:]
                predicted_id = np.argmax(scores)
                confidence = scores[predicted_id]

                if confidence > 0.20:
                    label = labels[predicted_id]
                    bounding_box = object_detection[0:4] * np.array(
                        [frame_width, frame_height, frame_width, frame_height])
                    (box_center_x, box_center_y, box_width, box_height) = bounding_box.astype("int")

                    start_x = int(box_center_x - (box_width / 2))
                    start_y = int(box_center_y - (box_height / 2))

                    ############## NON-MAXIMUM SUPPRESSION - OPERATION 2 ###################

                    ids_list.append(predicted_id)
                    confidences_list.append(float(confidence))
                    boxes_list.append([start_x, start_y, int(box_width), int(box_height)])

            ############################ END OF OPERATION 2 ########################

            ############## NON-MAXIMUM SUPPRESSION - OPERATION 3 ###################

        max_ids = cv2.dnn.NMSBoxes(boxes_list, confidences_list, 0.5, 0.4)

        for max_id in max_ids:

            max_class_id = max_id[0]
            box = boxes_list[max_class_id]

            start_x = box[0]
            start_y = box[1]
            box_width = box[2]
            box_height = box[3]

            predicted_id = ids_list[max_class_id]
            label = labels[predicted_id]
            confidence = confidences_list[max_class_id]

            ############################ END OF OPERATION 3 ########################

            end_x = start_x + box_width
            end_y = start_y + box_height

            box_color = colors[predicted_id]
            box_color = [int(each) for each in box_color]
            # roi = frame[start_y:end_y, start_x:end_x]
            
            if predicted_id == 9 or predicted_id == 11:
                
                start_x1 = start_x + (end_x - start_x) / 2
                start_x1 = int(start_x1)
                start_y1 = start_y + (end_y - start_y) / 2
                start_y1 = int(start_y1)
                roi = frame[start_y1:end_y, start_x1:end_x]
                data = cv2.medianBlur(roi, 5)
                roi = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY)
                _, roi = cv2.threshold(roi, thresh=40, maxval=255, type=cv2.THRESH_BINARY_INV)
                

                mask = cv2.inRange(roi, 240, 255)
                roi = cv2.dilate(mask, (3, 3), iterations=3)

                contour, _ = cv2.findContours(roi, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                for i in range(len(contour)):
                    cv2.drawContours(roi, contour, i, (0, 0, 255), 4)

                if 0 == len(contour):
                    predicted_id = 9
                    box_color = colors[predicted_id]
                    box_color = [int(each) for each in box_color]
                    label = labels[predicted_id]
                    print("sola_donulmez")
                    cv2.putText(frame,"Saga Don",(290,260),cv2.FONT_HERSHEY_DUPLEX,1,(0,255,255),2,cv2.LINE_4)


                else:
                    predicted_id = 11
                    box_color = colors[predicted_id]
                    box_color = [int(each) for each in box_color]
                    label = labels[predicted_id]
                    print("saga_donulmez")
                    cv2.putText(frame,"Sola Don",(290,260),cv2.FONT_HERSHEY_DUPLEX,1,(0,255,255),2,cv2.LINE_4)

            label = "{}: {:.2f}%".format(label, confidence * 100)
            # print("predicted object {}".format(label))
            
            cv2.rectangle(frame, (start_x - 1, start_y), (end_x + 1, start_y - 30), box_color, -1)
            cv2.putText(frame, label, (start_x, start_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        pub.publish(predicted_id)
        try:
            pub1.publish(start_x)
            pub2.publish(start_y)
            pub3.publish(end_x)
            pub4.publish(end_y)
            cv2.imshow("roi", roi)
        except:
            pass
        cv2.imshow("Detector", frame)
        
        cv2.waitKey(1)
    
	
    # %%


Kamera()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    

