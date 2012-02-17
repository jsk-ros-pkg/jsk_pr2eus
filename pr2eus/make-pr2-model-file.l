(defun make-pr2-model-file (&key (output-directory (ros::rospack-find "pr2eus")))
  (let ((robot "pr2") s
        fname_urdf fname_collada fname_yaml fname_lisp
	valid-cameras camera-paths openni-frames)
    (unless (setq s (ros::get-param "/robot_description"))
      (ros::ros-error "could not load model file from /robot_description~%")
      (return-from make-pr2-model-file))

    ;; variable setup
    (setq fname_urdf (format nil "/tmp/~a_~d.urdf" robot (unix::getpid)))
    (setq fname_collada (format nil "/tmp/~a_~d.dae" robot (unix::getpid)))
    (setq fname_yaml (ros::resolve-ros-path (format nil "package://euscollada/~A.yaml" robot)))
    (setq fname_lisp (format nil "/tmp/~a_~d.l" robot (unix::getpid)))

    ;; urdf -> collada -> euslisp
    (unless (probe-file fname_lisp)
      (with-open-file
       (f fname_urdf :direction :output)
       (format f s))

      (unix::system (format nil "rosrun collada_urdf_jsk_patch urdf_to_collada ~A ~A" fname_urdf fname_collada))
      (unix::system (format nil "rosrun euscollada collada2eus ~A ~A ~A" fname_collada fname_yaml fname_lisp))

      (warning-message 2 "load model file from parameter server /robot_description to ~A~%" fname_lisp)
      )

    ;; camera setup
    (with-open-file
     (f fname_lisp :direction :output :if-exists :append)
     (format f ";;~%")
     (format f ";; additional robot model description from camera_info~%")
     (format f ";;   based on ~A~%" (lisp-implementation-version))
     (format f ";;        and irteus ~A~%" (car ros::roseus-svnversion))
     (format f ";;~%")
     (format f "~A~%" (nconc (list 'defun 'make-camera-from-ros-camera-info-aux) (cddddr #'make-camera-from-ros-camera-info-aux)))
     (format f ";;~%")
     (format f "(defun pr2 () (setq *pr2* (instance pr2-sensor-robot :init)))~%")
     (format f "~%")
     (setq camera-paths
           (list "narrow_stereo/left"
                 "narrow_stereo/right"
                 "wide_stereo/left"
                 "wide_stereo/right"
                 "l_forearm_cam"
                 "r_forearm_cam"
                 "prosilica"
		 "camera/rgb"
		 "camera/depth"
		 ))
     (setq openni-frames (list "openni_camera" "openni_rgb_frame" "openni_depth_frame" "openni_rgb_optical_frame" "openni_depth_optical_frame"))
     (dolist (camera-path camera-paths)
       (let* ((camera-info (format nil "~A/camera_info" camera-path))
	      (camera-name (substitute #\- #\/ (string-downcase camera-path)))
	      (var (intern (string-upcase camera-name)))
	      (frame-id)
	      (i 0))
         (ros::subscribe camera-info sensor_msgs::CameraInfo
                         #'(lambda (msg)
                             (set var msg)
                             var))

         (ros::rate 10)
         (while (and (ros::ok) (not (boundp var)) (< (incf i) 50))
           (ros::spin-once)
           (ros::sleep))
         (ros::unsubscribe camera-info)
	 (if (boundp var)
	     (progn
	       (setq var (eval var))
	       (setq frame-id (send var :header :frame_id))
	       (if (eq (elt frame-id 0) #\/) (setq frame-id (subseq frame-id 1)))
	       (warning-message 2 "received ~A ~A ~A~%" camera-info var frame-id)
	       (push (list (cons :camera-path camera-path) (cons :camera-name camera-name) (cons :camera-info camera-info) (cons :frame-id frame-id)) valid-cameras))
	   (ros::ros-error "could not receive ~A ~A" camera-info var))
	   ) ;; let
        ) ;; dolist
     (format f "(defclass ~A-sensor-robot~%" robot)
     (format f "  :super pr2-robot~%")
     (format f "  :slots (")
     (dolist (valid-camera valid-cameras)
       (let* ((camera-name (cdr (assoc :camera-name valid-camera))))
	 (format f "~A " camera-name)))
     (dolist (openni-frame openni-frames)
       (format f "~A " openni-frame))
     (format f "))~%")
     (format f "~%")
     (format f "(defmethod ~A-sensor-robot~%" robot)
     (format f "  (:init (&rest args)~%")
     (format f "   (send-super* :init args)~%")
     ;; pr2 specific openni frames
     (format f "   ;; openni frame definition, this data is taken from jsk_pr2_startup kinect_head_launch ~%")
     (format f "#|~%")
     (dolist (name (list "head_to_kinect_rgb" "kinect_base_link" "kinect_base_link1" "kinect_base_link2" "kinect_base_link2"))
       (let ((command (format nil "roslaunch --args ~A jsk_pr2_startup kinect_head.launch" name)))
	 (format f "$ ~A~%" command)
	 (format f "~A~%" (read-line (piped-fork command )))))
     (format f "|#~%")
     (format f "   (setq openni_camera (make-cascoords :coords (send (make-coords :pos (float-vector 22 40 150) :rpy (float-vector 0 -0.02 0)) :transform (send head_plate_frame :worldcoords) :world) :parent head_plate_frame :name :openni_camera))~%")
     (format f "   (setq openni_depth_frame (make-cascoords :coords (send (make-coords :pos (float-vector 0 -20 0)) :transform (send openni_camera :worldcoords) :world) :parent openni_camera :name :openni_depth_frame))~%")
     (format f "   (setq openni_rgb_frame (make-cascoords :coords (send (make-coords :pos (float-vector 0 -40 0)) :transform (send openni_camera :worldcoords) :world) :parent openni_camera :name :openni_rgb_frame))~%")
     (dolist (frame (list "depth" "rgb"))
       (format f "   (setq openni_~A_optical_frame (make-cascoords :coords (send (make-coords :rpy (float-vector -pi/2 0 -pi/2)) :transform (send openni_~A_frame :worldcoords) :world) :parent openni_~A_frame :name :openni_~A_optical_frame))~%" frame frame frame frame))
     (format f "~%")
     (format f "   ;; define cameras~%")
     (dolist (valid-camera valid-cameras)
       (let* ((camera-path (cdr (assoc :camera-path valid-camera)))
	      (camera-name (cdr (assoc :camera-name valid-camera)))
	      (frame-id (cdr (assoc :frame-id valid-camera)))
	      (var (eval (intern (string-upcase camera-name)))))
	 (format f "   ;; ~A ~A~%" var (send var :P))
	 (format f "   (setq ~A (make-camera-from-ros-camera-info-aux ~A ~A ~A ~A :name :~A))~%~%" camera-name (send var :width) (send var :height) (send var :p) frame-id camera-path)))
     (format f "   self)~%")
     (format f "~%")
     ;; accessor to camera
     (dolist (valid-camera valid-cameras)
       (let* ((camera-name (cdr (assoc :camera-name valid-camera))))
	 (format f "  (:~A (&rest args) (forward-message-to ~A args))~%" camera-name camera-name)))
     (dolist (openni-frame openni-frames)
       (format f "  (:~A (&rest args) (forward-message-to ~A args))~%" openni-frame openni-frame))
     ;; :cameras
     (format f "~%")
     (format f "  (:cameras (&rest args)~%")
     (format f "   (forward-message-to-all (list")
     (dolist (valid-camera valid-cameras)
       (let ((camera-name (cdr (assoc :camera-name valid-camera))))
	 (format f " (send self :~A)" camera-name)))
     (format f ") args))~%")
     ;; :camera
     (format f "~%")
     (format f "  (:camera (&rest args)~%")
     (format f "    (cond ((integerp (car args))~%")
     (format f "           (forward-message-to (elt (send self :cameras) (car args)) (cdr args)))~%")
     (format f "          ((stringp (car args)) ;; this is obsoleted~%")
     (format f "           (forward-message-to (send self (intern (string-upcase (substitute #\\\- #\\\/ (car args))) \"KEYWORD\")) (cdr args)))~%")
     (format f "          ((keywordp (car args))~%")
     (format f "           (send* self args))~%")
     (format f "          (t~%")
     (format f "           (forward-message-to (car (send self :cameras)) args))))~%")
     (format f "  ) ;; defmethod ~A-robot~%~%~%" robot)
     ) ;; with-open-file
    (warning-message 1 "copy model file from ~A to ~A/~A.l~%" fname_lisp output-directory robot)
    (unix::system (format nil "mv ~A ~A/~A.l" fname_lisp output-directory robot))
    ))

;;(ros::roseus "make-pr2-modle-file")
;;(make-pr2-model-file)

