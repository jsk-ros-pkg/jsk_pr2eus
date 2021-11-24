;; re-define until https://github.com/jsk-ros-pkg/jsk_roseus/pull/629 is merged
(defun ros::list-param ()
  "Get the list of all the parameters in the server"
  (let ((s (piped-fork "rosparam list")) tmp ret)
    (while (setq tmp (read s nil))
      (push (format nil "~A" tmp) ret))
    ret))

(defun get-pr2-names ()
  ;; get list of rosparam, if /robot_description found, returns (list "")
  (reverse (mapcan #'(lambda (x) (if (substringp "robot_description" x)
                                     (if (> (length x) (+ (length "robot_description") 1)) (list (subseq x 1 (- (length x) (length "/robot_description")))) (list ""))))
                   (ros::list-param))))

(defun make-pr2-robot-model (robot robot-description)
  (let (fname_urdf fname_collada fname_yaml fname_lisp)
    ;; variable setup
    (setq fname_urdf (format nil "/tmp/~a_~d.urdf" robot (unix::getpid)))
    (setq fname_collada (format nil "/tmp/~a_~d.dae" robot (unix::getpid)))
    (setq fname_yaml (ros::resolve-ros-path (format nil "package://euscollada/~A.yaml" robot)))
    (setq fname_lisp (format nil "/tmp/~a_~d.l" robot (unix::getpid)))

    ;; urdf -> collada -> euslisp
    (unless (probe-file fname_lisp)
      (with-open-file
       (f fname_urdf :direction :output)
       (format f robot-description))

      (unix::system (format nil "rosrun collada_urdf urdf_to_collada ~A ~A" fname_urdf fname_collada))
      (unix::system (format nil "rosrun euscollada collada2eus ~A ~A ~A" fname_collada fname_yaml fname_lisp))

      (warning-message 2 "load model file from parameter server /robot_description to ~A~%" fname_lisp)
      )
    ))

(defun make-pr2-sensor-model (robot robot-camera-map)
  (let (fname_list robot-names all-valid-cameras)
    (setq robot-names (mapcar #'car robot-camera-map))
    (setq all-valid-cameras (remove-duplicates (mapcan #'cdr (copy-object robot-camera-map)) :test #'string= :key #'(lambda (x) (cdr (assoc :camera-name x)))))
    (setq fname_lisp (format nil "/tmp/~a_~d.l" robot (unix::getpid)))
    (with-open-file
     (f fname_lisp :direction :output :if-exists :append)
     (format f ";;~%")
     (format f ";; additional robot model description from camera_info~%")
     (format f ";;   based on ~A~%" (lisp-implementation-version))
     (format f ";;        and irteus ~A~%" (car ros::roseus-repo-version))
     (format f ";;~%")
     (format f ";; make-camera-from-ros-camera-info-aux is defined in roseus since 1.0.1, but to use this code in jskeus, we re-define here~%")
     (format f "~A~%" (nconc (list 'defun 'make-camera-from-ros-camera-info-aux) (cddddr #'make-camera-from-ros-camera-info-aux)))
     (format f ";;~%")
     (format f "(defun pr2 (&optional (name :~A)) (setq *pr2* (instance pr2-sensor-robot :init name)))~%" (if (string= (car robot-names) "") "pr2" (car robot-names)))
     (format f "~%")
     ;;
     (format f "(defclass ~A-sensor-robot~%" robot)
     (format f "  :super pr2-robot~%")
     (format f "  :slots (name ")
     (dolist (valid-camera all-valid-cameras)
       (let* ((camera-name (cdr (assoc :camera-name valid-camera))))
         (format f "~A " camera-name)))
     (format f "))~%")
     (format f "~%")
     (format f "(defmethod ~A-sensor-robot~%" robot)
     (format f "  (:init (&optional (n :~A) &rest args)~%" (if (string= (car robot-names) "") "pr2" (car robot-names)))
     (format f "   (send-super* :init args)~%")
     (format f "   (setq name n)~%")
     ;; pr2 specific kinect_head frames
     (format f "   ;; kinect_head frame definition, this data is taken from jsk_pr2_startup kinect_head_launch ~%")
     (format f "#|~%")
     (format f " get frame coordinates data from pr1012:/etc/ros/distro/urdf/robot.xml~%")
     (format f "|#~%")
     (format f "   (case name~%")
     (dolist (robot-name robot-names)
       (setq valid-cameras (cdr (assoc robot-name robot-camera-map :test #'string=)))
       (format f "     ;; define cameras for ~A~%" robot-name)
       (format f "     (:~A~%" (if (string= robot-name "") "pr2" robot-name))
       (dolist (valid-camera valid-cameras)
         (let* ((camera-path (cdr (assoc :camera-path valid-camera)))
                (camera-name (cdr (assoc :camera-name valid-camera)))
                (frame-id (cdr (assoc :frame-id valid-camera)))
                (var (eval (intern (string-upcase (format nil "~A_~A" robot-name camera-name))))))
           (format f "      ;; ~A ~A~%" (send (class var) :name) (send var :P))
           (format f "      (setq ~A (make-camera-from-ros-camera-info-aux ~A ~A ~A ~A_lk :name :~A))~%~%" camera-name (send var :width) (send var :height) (send var :p) frame-id camera-path)))
       ;; :cameras
       (format f "      (setq cameras (list ")
       (dolist (valid-camera valid-cameras)
         (let ((camera-name (cdr (assoc :camera-name valid-camera))))
           (format f " (send self :~A)" camera-name)))
       (format f "))~%")
       (format f "      ) ;; case ~A~%" robot-name)) ;; ease
     (format f "   )~%")
     (format f "   self)~%")
     (format f "~%")
     ;; accessor to camera
     (dolist (valid-camera all-valid-cameras)
       (let* ((camera-name (cdr (assoc :camera-name valid-camera))))
	 (format f "  (:~A (&rest args) (forward-message-to ~A args))~%" camera-name camera-name)))
     (format f "  ) ;; defmethod ~A-robot~%~%~%" robot)
     ) ;; with-open-file
    ))

(defun make-pr2-model-file (&key (output-directory (ros::rospack-find "pr2eus")))
  (let ((robot "pr2") robot-description robot-names
        robot-camera-map camera-paths)

    ;; get robot naems from robot_description
    (setq robot-names (get-pr2-names))
    (ros::ros-info "found ~A robots, ~A" (length robot-names) robot-names)

    (unless (setq robot-description (ros::get-param (format nil "/~A/robot_description" (car robot-names))))
      (ros::ros-error "could not load model file from /robot_description~%")
      (return-from make-pr2-model-file))

    (make-pr2-robot-model robot robot-description)

    (setq camera-paths
          (list "narrow_stereo/left"
                "narrow_stereo/right"
                "wide_stereo/left"
                "wide_stereo/right"
                "l_forearm_cam"
                "r_forearm_cam"
                "prosilica"
                "kinect_head/rgb"
                "kinect_head/depth"
                ))
    (dolist (robot-name robot-names)
      (let (valid-cameras)
        (dolist (camera-path camera-paths)
          (let* ((camera-info (format nil "/~A/~A/camera_info" robot-name camera-path))
                 (camera-name (substitute #\- #\/ (string-downcase camera-path)))
                 (var (intern (string-upcase (format nil "~A_~A" robot-name camera-name))))
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
                  (warning-message 2 "received ~A ~A ~A~%" camera-info (send var :P) frame-id)
                  (push (list (cons :camera-path camera-path) (cons :camera-name camera-name) (cons :camera-info camera-info) (cons :frame-id frame-id)) valid-cameras))
              (ros::ros-error "could not receive ~A ~A" camera-info var))
            )) ;; dolist (let *
        (push (cons robot-name valid-cameras) robot-camera-map)
        ))
    (setq robot-camera-map (reverse robot-camera-map))
    (make-pr2-sensor-model robot robot-camera-map)


    (warning-message 2 ";;~%")
    (warning-message 2 ";; copy model file from ~A to ~A/~A.l~%" fname_lisp output-directory robot)
    (warning-message 2 ";;~%")
    (unix::system (format nil "mv ~A ~A/~A.l" fname_lisp output-directory robot))
    (warning-message 2 ";;~%")
    (warning-message 2 ";; successfully generated ~A/~A robot file, now exiting program...~%" output-directory robot)
    (warning-message 2 ";;~%")
    ))

;;(ros::roseus "make-pr2-modle-file")
;;(make-pr2-model-file)

