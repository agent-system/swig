;;;
;;;
/* interface file */
%module webotslib

%insert(lisphead) %{
(setq webotslib (load-foreign (format nil "~A/lib/controller/libController.so" (unix:getenv "WEBOTS_HOME"))))

(defmacro defcenum (type &rest args)
   `(defcstruct ,type (x :integer))
   )
(defmacro defanonenum (&rest enums)
   "Converts anonymous enums to defconstants."
 (let ((index 0)
       (ret))
    (setq ret
      (mapcar
      #'(lambda (x)
	 (cond
	  ((listp x) (setq index (second x)) (list 'defconstant (car x) index))
	  (t (list 'defconstant x (incf index)))))
      enums))
  `(progn ,@ret)))

%}

%include "webots/types.h"
%include "webots/robot.h"
%include "webots/nodes.h"
%include "webots/accelerometer.h"
%include "webots/brake.h"
%include "webots/camera.h"
%include "webots/compass.h"
%include "webots/connector.h"
%include "webots/console.h"
%include "webots/device.h"
%include "webots/differential_wheels.h"
%include "webots/display.h"
%include "webots/distance_sensor.h"
%include "webots/emitter.h"
%include "webots/gps.h"
%include "webots/gyro.h"
%include "webots/inertial_unit.h"
%include "webots/joystick.h"
%include "webots/keyboard.h"
%include "webots/led.h"
%include "webots/lidar.h"
%include "webots/lidar_point.h"
%include "webots/light_sensor.h"
%include "webots/microphone.h"
%include "webots/motor.h"
%include "webots/mouse.h"
%include "webots/mouse_state.h"
%include "webots/pen.h"
%include "webots/position_sensor.h"
%include "webots/radar.h"
%include "webots/radar_target.h"
%include "webots/radio.h"
%include "webots/receiver.h"
  // %include "webots/remote_control.h"
%include "webots/robot_window.h"
%include "webots/robot_wwi.h"
%include "webots/skin.h"
%include "webots/speaker.h"
%include "webots/supervisor.h"
%include "webots/touch_sensor.h"
%include "webots/utils/motion.h"
%include "webots/utils/system.h"

%insert(swiglisp) %{

(defun cout-float-vector (adr len &optional (r (instantiate float-vector len)))
  (dotimes (i len) (setf (elt r i)
                         #+:x86_64
                         (sys:peek (+ adr (* 8 i)) :double)
                         #-:x86_64
                         (sys:peek (+ adr (* 4 i)) :double)
                         ))
  r)
;;
;;(defun nao-gyro nil (cout-float-vector (wb_gyro_get_values gyro) 2))
(defun fstring-to-string (fstr)
  (let* ((len (length fstr)) (str (make-string len)))
    (dotimes (i len) (setf (elt str i) (elt fstr i)))
    str))
;;;
(defun cout-string (fstr size)
  (sys:poke
   (+ 2 (* 4 size)) ;; length + 2;; bgra8
   (+ (sys::address fstr) 8) ;; address
   :long)
  fstr)

(defun wb_camera_image_get_alfa (im camera_width m n)
 (elt im (+ 3 (* 4 (+ (* n camera_width) m)))))
(defun wb_camera_image_get_byte (im camera_width m n &optional (offset 0))
 (if (= offset 3)
   (wb_camera_image_get_grey im camera_width m n)
     (elt im (+ offset (* 4 (+ (* n camera_width) m))))))
(defun wb_camera_image_get_red (im camera_width m n)
 (wb_camera_image_get_byte im camera_width m n 2))
(defun wb_camera_image_get_green (im camera_width m n)
 (wb_camera_image_get_byte im camera_width m n 1))
(defun wb_camera_image_get_blue (im camera_width m n)
 (wb_camera_image_get_byte im camera_width m n 0))

(defun wb_camera_image_get_grey (im camera_width m n)
 (/ (+
     (wb_camera_image_get_red im camera_width m n)
     (wb_camera_image_get_green im camera_width m n)
     (wb_camera_image_get_blue im camera_width m n))
  3))

(defun wb_camera_image_get_yellow (im camera_width m n)
 (/ (+
     (wb_camera_image_get_red im camera_width m n)
     (wb_camera_image_get_green im camera_width m n))
  2))

(defun webots-camera-fstring (camera)
  (let* ((w (wb_camera_get_width camera))
         (h (wb_camera_get_height camera)))
    (cout-string (wb_camera_get_image camera) (* 4 w h))))

(defun webots-camera-image (camera
                            &optional str)
  (let* ((w (wb_camera_get_width camera))
         (h (wb_camera_get_height camera))
         (fs (cout-string (wb_camera_get_image camera) (* 4 w h))))
    (unless str (setq str (make-string (* 3 w h))))
    (do* ((y 0 (+ 1 y)) (wy (* y w) (* y w)))
         ((>= y h))
         (do* ((x 0 (+ 1 x))
               (p3 (* 3 (+ x wy)) (* 3 (+ x wy)))
               (p4 (* 4 (+ x wy)) (* 4 (+ x wy))))
              ((>= x w))
              (setf (elt str p3) (elt fs (+ 2 p4))
                    (elt str (+ 1 p3)) (elt fs (+ 1 p4))
                    (elt str (+ 2 p3)) (elt fs p4))))
    #|
    (do ((i 0 (+ 1 i)) (p3 0 (+ 3 p3)) (p4 0 (+ 4 p4)))
    ((>= i wh))
    (setf (elt str p3) (elt fs (+ 2 p4))
    (elt str (+ 1 p3)) (elt fs (+ 1 p4))
    (elt str (+ 2 p3)) (elt fs p4)))
    (do* ((y 0 (+ 1 y)) (wy (* y w) (* y w)))
	 ((>= y h))
	 (do* ((x 0 (+ 1 x))
	       (p3 (* 3 (+ x wy)) (* 3 (+ x wy)))
	       (p4 (* 4 (+ x wy)) (* 4 (+ x wy))))
	      ((>= x w))
	      (setf (elt fs p3) (elt fs (+ 2 p4))
		    (elt fs (+ 1 p3)) (elt fs (+ 1 p4))
		    (elt fs (+ 2 p3)) (elt fs p4))))
    (do ((i 0 (+ 1 i)) (p3 0 (+ 3 p3)) (p4 0 (+ 4 p4)))
	((>= i wh))
	(setf (elt str p3) (elt fs (+ 2 p4))
	      (elt str (+ 1 p3)) (elt fs (+ 1 p4))
	      (elt str (+ 2 p3)) (elt fs p4)))
    (do* ((y 0 (+ 1 y)) (wy (* y w) (* y w)))
	 ((>= y h))
	 (do* ((x 0 (+ 1 x))
	       (p3 (* 3 (+ x wy)) (* 3 (+ x wy)))
	       (p4 (* 4 (+ x wy)) (* 4 (+ x wy))))
	      ((>= x w))
	      (setf (elt str p3) (elt fs (+ 2 p4))
		    (elt str (+ 1 p3)) (elt fs (+ 1 p4))
		    (elt str (+ 2 p3)) (elt fs p4))))
    (do* ((y 0 (+ 1 y)) (wy (* y w) (* y w)))
	 ((>= y h))
	 (do* ((x 0 (+ 1 x))
	       (p3 (* 3 (+ x wy)) (* 3 (+ x wy)))
	       (p4 (* 4 (+ x wy)) (* 4 (+ x wy))))
	      ((>= x w))
	      (setf (elt str p3) (elt fs (+ 2 p4)))
	      (setf (elt str (+ 1 p3)) (elt fs (+ 1 p4)))
	      (setf (elt str (+ 2 p3)) (elt fs p4))))
    (do* ((y 0 (+ 1 y)) (wy (* y w) (* y w)))
	 ((>= y h))
	 (do* ((x 0 (+ 1 x)) (wxy (+ x wy) (+ x wy)))
	      ((>= x w))
	      (setf (elt str (* 3 wxy))
		    (elt fs (+ 2 (* 4 wxy))))
	      (setf (elt str (+ 1 (* 3 wxy)))
		    (elt fs (+ 1 (* 4 wxy))))
	      (setf (elt str (+ 2 (* 3 wxy)))
		    (elt fs (+ 0 (* 4 wxy))))))
    (dotimes (i wh)
      (setf (elt str ptr30) (elt fs ptr42)
    (elt str ptr31) (elt fs ptr41)
    (elt str ptr32) (elt fs ptr40))
      (incf ptr30 3) (incf ptr31 3) (incf ptr32 3)
      (incf ptr40 4) (incf ptr41 4) (incf ptr42 4))
    |#
    str)
  )

(defun print_camera_image (camera &optional (offset 0) (SCALED 4)
                                  &aux width height image grey)
  (setq width (wb_camera_get_width camera))
  (setq height (wb_camera_get_height camera))
  (setq image (webots-camera-fstring camera))
  (format t "image length=~s~%" (length image))
  (format t "image length=~d~%" (length image))
  (format t "original resolution: ~d x ~d, scaled to ~d x ~d~%"
          width height (/ width SCALED) (/ height SCALED))

  (setq line (make-array (/ width SCALED)))
  (do ((y 0 (+ y SCALED)))
      ((>= y height))
      (format t "~d line:" y)
      (do ((x 0 (+ x SCALED)) (count 0 (1+ count)))
          ((>= x width) )
          (setq grey (wb_camera_image_get_byte image width x y offset))
          (format t " ~x" grey)
          (setf (elt line count) grey))
      (format t "~%")
      )
  (format t "end of y-do ~%")
  )
;;;
;;;
(defvar
  *node-type-symbol-list*
  (mapcan
   #'(lambda (n)
       (if (boundp n)
           (list (cons (symbol-value n) n))
         nil))
   '(
     WB_NODE_NO_NODE
     WB_NODE_APPEARANCE
     WB_NODE_BACKGROUND
     WB_NODE_BOX
     WB_NODE_COLOR
     WB_NODE_CONE
     WB_NODE_COORDINATE
     WB_NODE_CYLINDER
     WB_NODE_DIRECTIONAL_LIGHT
     WB_NODE_ELEVATION_GRID
     WB_NODE_EXTRUSION
     WB_NODE_FOG
     WB_NODE_GROUP
     WB_NODE_IMAGE_TEXTURE
     WB_NODE_INDEXED_FACE_SET
     WB_NODE_INDEXED_LINE_SET
     WB_NODE_MATERIAL
     WB_NODE_POINT_LIGHT
     WB_NODE_SHAPE
     WB_NODE_SPHERE
     WB_NODE_SPOT_LIGHT
     WB_NODE_SWITCH
     WB_NODE_TEXTURE_COORDINATE
     WB_NODE_TEXTURE_TRANSFORM
     WB_NODE_TRANSFORM
     WB_NODE_VIEWPOINT
     WB_NODE_WORLD_INFO
     WB_NODE_CAPSULE
     WB_NODE_PLANE
     WB_NODE_ROBOT
     WB_NODE_SUPERVISOR
     WB_NODE_DIFFERENTIAL_WHEELS
     WB_NODE_BALL_JOINT
     WB_NODE_BALL_JOINT_PARAMETERS
     WB_NODE_CAMERA_ZOOM
     WB_NODE_CHARGER
     WB_NODE_CONTACT_PROPERTIES
     WB_NODE_DAMPING
     WB_NODE_HINGE_JOINT
     WB_NODE_HINGE_JOINT_PARAMETERS
     WB_NODE_HINGE_2_JOINT_PARAMETERS
     WB_NODE_HINGE_2_JOINT
     WB_NODE_JOINT_PARAMETERS
     WB_NODE_PHYSICS
     WB_NODE_SLIDER_JOINT
     WB_NODE_SOLID
     WB_NODE_SOLID_REFERENCE
     WB_NODE_ACCELEROMETER
     WB_NODE_CAMERA
     WB_NODE_COMPASS
     WB_NODE_CONNECTOR
     WB_NODE_DISPLAY
     WB_NODE_DISTANCE_SENSOR
     WB_NODE_EMITTER
     WB_NODE_GPS
     WB_NODE_GYRO
     WB_NODE_INERTIAL_UNIT
     WB_NODE_LED
     WB_NODE_LIGHT_SENSOR
     WB_NODE_LINEAR_MOTOR
     WB_NODE_MICROPHONE
     WB_NODE_PEN
     WB_NODE_POSITION_SENSOR
     WB_NODE_PROPELLER
     WB_NODE_RADIO
     WB_NODE_ROTATIONAL_MOTOR
     WB_NODE_RECEIVER
     WB_NODE_SERVO
     WB_NODE_SPEAKER
     WB_NODE_TOUCH_SENSOR
     ))
  )

(defun webots-node-type-symbol (num)
  (cdr (assoc num *node-type-symbol-list*)))

(defun webots-device-names nil
  (mapcar #'(lambda (s) (get s :device-name)) (webots-device-symbols)))

(defun init-webots-device-symbols nil
  (let* ((n_devices (wb_robot_get_number_of_devices))
         (robot-type (wb_robot_get_type))
         dev2 name dev-sym
         (dev 0) type-sym type dev-symbols type-symbols
         (robot-name (string-upcase (wb_robot_get_name)))
         (robot (intern robot-name)))
    (when (= WB_NODE_DIFFERENTIAL_WHEELS robot-type)
      (setq type-sym (webots-node-type-symbol robot-type))
      (setf (get type-sym :value) (symbol-value type-sym))
      (format t "--- Differetial wheels name=~A, type-sym=~A~%" robot type-sym)
      (dolist (n '("-BOTH" "-LEFT" "-RIGHT"))
        (setq name (concatenate string robot-name n))
        (setq dev-sym (intern name))
        (setf (get dev-sym :device) dev)
        (setf (get type-sym :device-names)
              (push name (get type-sym :device-names)))
        (setf (get type-sym :device-symbols)
              (push dev-sym
                    (get type-sym :device-symbols)))
        (incf dev)
        )
      (setf (get type-sym :device-names)
            (reverse (get type-sym :device-names)))
      (setf (get type-sym :device-symbols) 
            (reverse (get type-sym :device-symbols)))
      )
    (dotimes (i n_devices)
      (setq dev (wb_robot_get_device_by_index i))
      ;;(setq name (format nil "~A" (wb_device_get_name dev)))
      (setq name (fstring-to-string (wb_device_get_name dev)))
      (setq dev2 (wb_robot_get_device name))
      (setq type (wb_device_get_type dev))
      (setq type-sym (webots-node-type-symbol type))
      (setq dev-sym (intern (string-upcase name)))
      (setf (get dev-sym :device-name) name)
      (setf (get dev-sym :device) dev)
      (setf (get dev-sym :device-type) type)
      (setf (get dev-sym :device-type-symbol) type-sym)
      (format t "dev-sym=~A(~A) type=~A(~A) dev=~A(~A in ~A)~%"
              dev-sym name type type-sym dev i n_devices)
      (setf (get type-sym :device-names)
            (cons name (get type-sym :device-names)))
      (setf (get type-sym :device-symbols)
            (cons dev-sym (get type-sym :device-symbols)))
      (setf (get type-sym :value) (symbol-value type-sym))
      (push dev-sym dev-symbols)
      (pushnew type-sym type-symbols)
      )
    (dolist (typ type-symbols)
      (setf (get typ :device-names) (reverse (get typ :device-names)))
      (setf (get typ :device-symbols) (reverse (get typ :device-symbols)))
      (setf (get robot typ) (get typ :device-symbols)))
    (setf (get robot :device-types) type-symbols)
    (setf (get robot :device-symbols) (reverse dev-symbols))
    
    (setq *webots-camera-symbols*
          (get 'wb_node_camera :device-symbols))
    (setq *webots-all-motor-symbols*
          (append
           (get 'WB_NODE_DIFFERENTIAL_WHEELS :device-symbols)
           (get 'WB_NODE_ROTATIONAL_MOTOR :device-symbols)
           (get 'WB_NODE_LINEAR_MOTOR :device-symbols)
           (get 'WB_NODE_SERVO :device-symbols)))
    ))

(defun webots-camera-symbols nil *webots-camera-symbols*)
(defun webots-all-motor-symbols nil *webots-all-motor-symbols*)

(defun webots-device-enables (time_step)
  (when (get 'WB_NODE_DIFFERENTIAL_WHEELS :device-symbols)
    (wb_differential_wheels_enable_encoders)
    )
  (dolist (s (get 'WB_NODE_ROTATIONAL_MOTOR :device-symbols))
    (wb_motor_enable_position (get s :device) time_step)
    (wb_motor_set_position (get s :device) 0.0)
    )
  (dolist (s (get 'WB_NODE_LINEAR_MOTOR :device-symbols))
    (wb_motor_enable_position (get s :device) time_step)
    ;;(wb_motor_set_position (get s :device) 0.0))
    )
  (dolist (s (get 'WB_NODE_SERVO :device-symbols))
    (wb_servo_enable_position (get s :device) time_step))
  ;;
  (dolist (s (get 'WB_NODE_CAMERA :device-symbols))
    (wb_camera_enable (get s :device) time_step))
  (dolist (s (get 'WB_NODE_ACCELEROMETER :device-symbols))
    (wb_accelerometer_enable (get s :device) time_step))
  (dolist (s (get 'WB_NODE_GYRO :device-symbols))
    (wb_gyro_enable (get s :device) time_step))
  (dolist (s (get 'WB_NODE_GPS :device-symbols))
    (wb_gps_enable (get s :device) time_step))
  (dolist (s (get 'WB_NODE_TOUCH_SENSOR :device-symbols))
    (wb_touch_sensor_enable (get s :device) time_step))
  (dolist (s (get 'WB_NODE_DISTANCE_SENSOR :device-symbols))
    (wb_distance_sensor_enable (get s :device) time_step))
  (dolist (s (get 'WB_NODE_COMPASS :device-symbols))
    (wb_compass_enable (get s :device) time_step))
  (dolist (s (get 'WB_NODE_INERTIAL_UNIT :device-symbols))
    (wb_inertial_unit_enable (get s :device) time_step))
  )
;;;
(defun wb_differential_wheels_set_wheel_speed (d v)
  (cond
   ((= d 0)
    (wb_differential_wheels_set_speed  v v))
   ((= d 1)
    (wb_differential_wheels_set_speed
     v (wb_differential_wheels_get_right_speed)))
   ((= d 2)
    (wb_differential_wheels_set_speed
     (wb_differential_wheels_get_left_speed) v))))

(defun wb_differential_wheels_get_wheel_speed (d)
  (cond
   ((= d 0)
    (/ (+ (wb_differential_wheels_get_left_speed)
          (wb_differential_wheels_get_right_speed)) 2.0)
    )
   ((= d 1) (wb_differential_wheels_get_left_speed))
   ((= d 2) (wb_differential_wheels_get_right_speed))))

(defun init-webots-define-motor-functions nil
  (mapcar
   #'(lambda (s)
       (let* ((d (get s :device))
              (max (wb_differential_wheels_get_max_speed))
              (min (- max))
              form)
         (setf (get s :min) min)
         (setf (get s :max) max)
         (setf (get s :dtheta) (wb_differential_wheels_get_speed_unit))
         (setq form
               `(defun ,s (&optional v)
                  (if v (wb_differential_wheels_set_wheel_speed ,d v)
                    (wb_differential_wheels_get_wheel_speed ,d))
                  ))
         (eval form)
         (setf (get s :def) form)
         s))
   (get 'wb_node_differential_wheels :device-symbols))
  (mapcar
   #'(lambda (s)
       (let* ((d (get s :device))
              (min (wb_motor_get_min_position d))
              (max (wb_motor_get_max_position d))
              (zero-range (zerop (- max min)))
              form)
         (setf (get s :min) (if zero-range -135.0 (rad2deg min)))
         (setf (get s :max) (if zero-range 135.0 (rad2deg max) ))
         (setf (get s :dtheta) (* (- (get s :max) (get s :min)) 0.1))
         (setq form
               `(defun ,s (&optional v)
                  (if v (wb_motor_set_position ,d (deg2rad v)))
                  (rad2deg (wb_motor_get_position ,d))
                  ))
         (setf (get s :def) form)
         (eval form)
         s))
   (get 'wb_node_rotational_motor :device-symbols))
  (mapcar
   #'(lambda (s)
       (let* ((d (get s :device))
              (min (wb_motor_get_min_position d))
              (max (wb_motor_get_max_position d))
              (zero-range (zerop (- max min)))
              form)
         (setf (get s :min) (if zero-range 0.0 min))
         (setf (get s :max) (if zero-range 1.0 max) )
         (setf (get s :dtheta) (* (- (get s :max) (get s :min)) 0.1))
         (setq form
               `(defun ,s (&optional v)
                  (if v (wb_motor_set_position ,d v))
                  (wb_motor_get_position ,d)
                  ))
         (setf (get s :def) form)
         (eval form)
         s))
   (get 'wb_node_linear_motor :device-symbols))
  (mapcar
   #'(lambda (s)
       (let* ((d (get s :device))
              (min (wb_servo_get_min_position d))
              (max (wb_servo_get_max_position d))
              (zero-range (zerop (- max min)))
              form)
         (setf (get s :min) (if zero-range -135.0 (rad2deg min)))
         (setf (get s :max) (if zero-range 135.0 (rad2deg max) ))
         (setf (get s :dtheta) (* (- (get s :max) (get s :min)) 0.1))
         (setq form
               `(defun ,s (&optional v)
                  (if v (wb_servo_set_position ,d (deg2rad v)))
                  (rad2deg (wb_servo_get_position ,d))
                  ))
         (setf (get s :def) form)
         (eval form)
         s))
   (get 'wb_node_servo :device-symbols))
  )

(defun update-to-webots (&optional (av (send *robot* :angle-vector)))
  (let ((servo-names
         (append (get 'wb_node_servo :device-symbols)
                 (get 'wb_node_rotational_motor :device-symbols))))
    (dotimes (i (length servo-names))
      (funcall (elt servo-names i) (elt av i))))
  )
(defun update-to-eus-robot (&optional (av (send *robot* :angle-vector)))
  (let ((servo-names
         (append (get 'wb_node_servo :device-symbols)
                 (get 'wb_node_rotational_motor :device-symbols))))
    (send *robot* :angle-vector
          (coerce servo-names float-vector)))
  )

;;;
(defun robot-accelerometer (&optional (cnt 1)
                                      (acc (instantiate float-vector 3)))
  (dotimes (i cnt)
    (v+ acc
        (cout-float-vector
         (wb_accelerometer_get_values (get 'accelerometer :device)) 3)
        acc)
    )
  (scale (/ 1.0 cnt) acc))
(defun robot-gyro nil (cout-float-vector (wb_gyro_get_values (get 'gyro :device)) 2))

(defun terminate nil
  (wb_robot_cleanup))

(defun simulation_step nil
  (if (= (wb_robot_step time_step) -1)
      (terminate)))

;;;

(defun webot-jvl-images (&optional (x 0) (y 30) 
                                   &aux (width 160) (height 120)
                                   remove-list)
  (when
      (get 'wb_node_camera :device-symbols)
    (dolist (s (get 'wb_node_camera :device-symbols))
      (setq width (wb_camera_get_width (get s :device)))
      (setq height (wb_camera_get_height (get s :device)))
      (format t "jvl-images s=~S w=~A,h=~A~%" s width height)
      (cond
       ((= height 1) ;; remove sick lms, it should not be camera!>cyberbotics
        (push s remove-list))
       (t (setf (get s :jvl-image)
                (instance jvl-image :init :width width :height height))
          (send (get s :jvl-image) :color-image24)
          (setf (get s :image-canvas)
                (create-image-canvas (get s :jvl-image) :title (get s :device-name)))
          (send (get s :image-canvas) :move x y)
          (setf (get s :image-width) width)
          (setf (get s :image-height) height)
          (setf (get s :x) x (get s :y) y)
          (setq y (+ y height 30)))))
    (setf (get 'wb_node_camera :device-symbols)
          (set-difference (get 'wb_node_camera :device-symbols)
                          remove-list)))
  (format t "jvl-images =~A~%" (get 'wb_node_camera :device-symbols))
  )

(defun webot-edge-images ()
  (dolist (s (get 'wb_node_camera :device-symbols))
    (setf (get s :edge-converter)
          (instance edge-converter :init :image (get s :jvl-image)))
    ;;(send *edge* :set-param ":function convSobel")
    ;;(send *edge* :set-param ":function convSmooth")
    ;;(send *edge* :set-param ":function convXY")
    ;;(send *edge* :set-param ":function convMagXY")
    (send (get s :edge-converter) :set-param ":function convCanny")
    (send (get s :edge-converter) :set-param ":gauss-kernel-sigma 1.0")
    (send (get s :edge-converter) :set-param ":gaussderiv-kernel-sigma 1.0")
    (setf (get s :edge-canvas)
          (create-image-canvas (send (get s :edge-converter) :get-outimage)
                               :title (format nil "~A Edge" (get s :device-name))))

    (send (get s :edge-canvas) :move
          (+ (get s :x) (get s :image-width) 5)
          (get s :y))
    )
  )

(defun webot-hsicolor-images (&optional (x 0) (y 330))
  (dolist (s (get 'wb_node_camera :device-symbols))
    (setf (get s :hsicolor-converter)
          (instance hsicolor-converter :init :image (get s :jvl-image)))
    (setf (get s :hsicolor-canvas)
          (create-image-canvas (send (get s :hsicolor-converter) :get-outimage)
                               :title (format nil "~A Color" (get s :device-name))))
    (send (get s :hsicolor-canvas) :move
          (+ (get s :x) (get s :image-width) (get s :image-width) 10)
          (get s :y))

    (setf (get s :hsicolor-panel)
          (instance X::color-converter-panel :create
                    (get s :hsicolor-converter)
                    :filename "ball-color-params.l"
                    :title (format nil "~A Color Panel" (get s :device-name))))
    (send (get s :hsicolor-panel) :move x y)
    (send (get s :hsicolor-panel) :refresh)
    (setq y (+ y 200))
    )
  )

(defun webot-mepzoom-images (&optional (x 420) (y 330))
  (dolist (s (get 'wb_node_camera :device-symbols))
    (setf (get s :mepzoom-converter)
          (instance mepzoom-converter :init :image (get s :jvl-image)
                    :image-type IMAGE_RGB
                    :ref-width 32 :ref-height 32
                    :ser-width 120 :ser-height 100 :step 1))
    (send (get s :mepzoom-converter) :ser-x 80)
    (send (get s :mepzoom-converter) :ser-y 60)
    (setf (get s :mepzoom-refview)
               (create-image-canvas (send (get s :mepzoom-converter) :get-refimage)
                                    :title (format nil "~A Mep" (get s :device-name))))
    (send (get s :mepzoom-refview) :move x y)
    (setq y (+ y 200))
    )
  )

(defun webot-init-images nil
  (webot-jvl-images 0 30)
  (webot-edge-images)
  (webot-hsicolor-images)
  (webot-mepzoom-images)
  )

(defun webot-image-capture nil
  (dolist (s (get 'wb_node_camera :device-symbols))
    (unless (get s :capture-off)
      (send (get s :jvl-image) :set-buffer
            (webots-camera-image
             (get s :device)
             (make-foreign-string (send (get s :jvl-image) :get-buffer)
                                  (* 3 (get s :image-width) (get s :image-height)))))
      (send (get s :image-canvas) :putimage (get s :jvl-image) :flush t)))
  )
;;;
;;;
;;;
(defun webot-proc-edge nil
  (dolist (s (get 'wb_node_camera :device-symbols))
    (unless (get s :edge-off)
      (send (get s :edge-converter) :convert)
      (send (get s :edge-canvas) :putimage 
            (send (get s :edge-converter) :get-outimage) :flush t)
      )
    ))

(defun webot-proc-color nil
  (dolist (s (get 'wb_node_camera :device-symbols))
    (unless (get s :hsicolor-off)
      (send (get s :hsicolor-converter) :convert)
      (send (get s :hsicolor-converter) :get-result 0)
      (send (get s :hsicolor-canvas)
            :putimage (send (get s :hsicolor-converter) :get-outimage) :flush t)
      (when (> (send (get s :hsicolor-converter) :area) 0)
        (send (get s :hsicolor-canvas)
              :draw-cross (send (get s :hsicolor-converter) :center-xy) 5 x::*red*)
        (send (get s :hsicolor-canvas)
              :draw-cross (send (get s :hsicolor-converter) :xy1) 5 x::*yellow*)
        (send (get s :hsicolor-canvas)
              :draw-cross (send (get s :hsicolor-converter) :xy2) 5 x::*white*)
        (send (get s :hsicolor-canvas) :flush))
      )
    )
  )
(defun webot-proc-mepzoom nil
  (dolist (s (get 'wb_node_camera :device-symbols))
    (unless (get s :mepzoom-off)
      (proc-mepzoom-one
       (get s :mepzoom-converter)
       (get s :image-canvas)
       (get s :mepzoom-refview)
       'track))
    )
  )

(defun proc-mepzoom-one (mep view refview mep-state)
  (send mep :convert)
  (send view :putimage (send mep :get-inimage) :flush t)
  (send refview :putimage (send mep :get-refimage) :flush t)
  ;;
  (send mep :draw-ser-rectangle view)
  (send mep :draw-mv-rectangle view)
  (send mep :draw-ref-rectangle refview) 

  (send view :string 0 50 (format nil "~5,3f" (send mep :distmin)))
  (send view :string 0 75 (format nil "~5,3f [sec]" (send mep :conv-time)))
  (send view :draw-cross (send mep :mv))

  (send view :flush)
  (send refview :flush)
  ;;
  (when (eq mep-state 'track)
    (send mep :ser-x (send mep :mv-x))
    (send mep :ser-y (send mep :mv-y)))
  
  (when (and (eq mep-state 'track) (> (send mep :distmin) 0.15))
    (send mep :ser-x 80)
    (send mep :ser-y 60)
    (send mep :ser-width 100)
    (send mep :ser-height 70)
    (setq mep-state 'search)
    )
  (when (and (eq mep-state 'search) (< (send mep :distmin) 0.15))
    (format t "mep-state=search pos=(~s ~s)~%"
            (send mep :mv-x) (send mep :mv-y))
    (send mep :ser-x (send mep :mv-x))
    (send mep :ser-y (send mep :mv-y))
    (send mep :ser-width 80)
    (send mep :ser-height 70)
    (setq mep-state 'track)
    )
  )

(defun webot-proc-mouse
  (&aux ev et)
  (while (setq ev (x::next-event))
    (dolist (s (get 'wb_node_camera :device-symbols))
      (cond
       ((eq (x::event-window ev) (get s :image-canvas))
        (when (eq (x::event-type ev) :buttonrelease)
          (format t "mouse button=~s in ~A pos=~s~%"
                  (x::event-button ev) s (x::event-pos ev))
          (let* ((mp (x::event-pos ev))
                 (x (elt mp 0)) (y (elt mp 1)))
            (cond
             ((= 3 (x::event-button ev))
              (setq *attention-camera* s)
              (setq *lookat-run* :mep)
              )
             ((>= 2 (x::event-button ev))
              (send (get s :image-canvas) :buttonpress ev)
              (send (get s :mepzoom-converter) :update-zoom-ref x y)
              (send (get s :mepzoom-converter)  :ser-x x)
              (send (get s :mepzoom-converter)  :ser-y y)
              )))))
       ((eq (x::event-window ev) (get s :hsicolor-canvas))
        (when (eq (x::event-type ev) :buttonrelease)
          (format t " mouse button=~s in ~A pos=~s~%"
                  (x::event-button ev) s (x::event-pos ev))
          (setq *attention-camera* s)
          (setq *lookat-run* :color))
        )))
    (setq et (x::event-type ev))
    (if (not (eq et :noexpose))
        (send (x::event-window ev) et ev)))
  )

(defun webot-image-proc-one nil
  (webot-image-capture)
  (webot-proc-mouse)
  (webot-proc-edge)
  (webot-proc-color)
  (webot-proc-mepzoom)
  )
%}


