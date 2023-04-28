;; Auto-generated. Do not edit!


(when (boundp 'esw::MotorCommand)
  (if (not (find-package "ESW"))
    (make-package "ESW"))
  (shadow 'MotorCommand (find-package "ESW")))
(unless (find-package "ESW::MOTORCOMMAND")
  (make-package "ESW::MOTORCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude MotorCommand.msg.html


(defclass esw::MotorCommand
  :super ros::object
  :slots (_motor1_speed _motor2_speed ))

(defmethod esw::MotorCommand
  (:init
   (&key
    ((:motor1_speed __motor1_speed) 0.0)
    ((:motor2_speed __motor2_speed) 0.0)
    )
   (send-super :init)
   (setq _motor1_speed (float __motor1_speed))
   (setq _motor2_speed (float __motor2_speed))
   self)
  (:motor1_speed
   (&optional __motor1_speed)
   (if __motor1_speed (setq _motor1_speed __motor1_speed)) _motor1_speed)
  (:motor2_speed
   (&optional __motor2_speed)
   (if __motor2_speed (setq _motor2_speed __motor2_speed)) _motor2_speed)
  (:serialization-length
   ()
   (+
    ;; float32 _motor1_speed
    4
    ;; float32 _motor2_speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _motor1_speed
       (sys::poke _motor1_speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _motor2_speed
       (sys::poke _motor2_speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _motor1_speed
     (setq _motor1_speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _motor2_speed
     (setq _motor2_speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get esw::MotorCommand :md5sum-) "44cbd85de4c6dd4929217d06c0fc9617")
(setf (get esw::MotorCommand :datatype-) "esw/MotorCommand")
(setf (get esw::MotorCommand :definition-)
      "float32 motor1_speed
float32 motor2_speed
")



(provide :esw/MotorCommand "44cbd85de4c6dd4929217d06c0fc9617")


