(defun make-pr2-model-file ()
  (let ((robot "pr2") s
        fname_urdf fname_fix_urdf fname_collada fname_yaml fname_lisp)
    (unless (setq s (ros::get-param "/robot_description"))
      (ros::ros-error "could not load model file from /robot_description~%")
      (return-from make-pr2-model-file))

    ;; variable setup
    (setq fname_urdf (format nil "/tmp/~a_~d.urdf" robot (unix::getpid)))
    (setq fname_fix_urdf (format nil "/tmp/~a_~d_fix.urdf" robot (unix::getpid)))
    (setq fname_collada (format nil "/tmp/~a_~d.dae" robot (unix::getpid)))
    (setq fname_yaml (ros::resolve-ros-path (format nil "package://euscollada/~A.yaml" robot)))
    (setq fname_lisp (format nil "/tmp/~a_~d.l" robot (unix::getpid)))

    ;; urdf -> collada -> euslisp
    (unless (probe-file fname_lisp)
      (with-open-file
       (f fname_urdf :direction :output)
       (format f s))

      (unix::system (format nil "sed s/\.dae/\.stl/ < ~A > ~A" fname_urdf fname_fix_urdf))
      (unix::system (format nil "rosrun collada_urdf_jsk_patch urdf_to_collada ~A ~A" fname_fix_urdf fname_collada))
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
     (format f ";;~%")
     (format f "(defmethod ~A-robot~%" robot)
     (setq camera-vars
           (list "narrow_stereo/left"
                 "narrow_stereo/right"
                 "wide_stereo/left"
                 "wide_stereo/right"
                 "l_forearm_cam"
                 "r_forearm_cam"
                 "prosilica"))
     (dolist (camera-var camera-vars)
       (let ((var (intern (substitute #\- #\/ (string-upcase camera-var))))
             (camera-info (format nil "~A/camera_info" camera-var))
             (camera-name (substitute #\- #\/ (string-downcase camera-var))))
         (ros::subscribe camera-info sensor_msgs::CameraInfo
                         #'(lambda (msg)
                             (set var msg)
                             var))
         (ros::rate 10)
         (while (and (ros::ok) (not (boundp var)))
           (ros::spin-once)
           (ros::sleep))
         (ros::unsubscribe camera-info)
         (setq var (eval var))
         (warning-message 2 "received ~A ~A ~A~%" camera-info var (send var :header :frame_id))
         (format f "  (:~A (&rest args)~%" camera-name)
         (format f "   (let ((c (get self :~A-model)))~%" camera-name)
         (format f "     (unless c~%")
         (format f "       (setq c (make-camera-from-ros-camera-info-aux ~A ~A ~A ~A))~%" (send var :width) (send var :height) (send var :p) (send var :header :frame_id))
         (format f "       (send c :name \"~A\")~%" camera-var)
         (format f "       (setf (get self :~A-model) c))~%" camera-name)
         (format f "     (forward-message-to c args)))~%")
         ) ;; let
        ) ;; dolist
     (format f "  (:cameras (&rest args)~%")
     (format f "   (forward-message-to-all (list")
     (dolist (camera-var camera-vars)
       (format f " (send self :~A)" (substitute #\- #\/ camera-var)))
     (format f ") args))~%")
     (format f "  (:camera (&rest args)~%")
     (format f "    (cond ((integerp (car args))~%")
     (format f "           (forward-message-to (elt (send self :cameras) (car args)) (cdr args)))~%")
     (format f "          ((stringp (car args))~%")
     (format f "           (forward-message-to (send self (intern (string-upcase (substitute #\\\- #\\\/ (car args))) \"KEYWORD\")) (cdr args)))~%")
     (format f "          (t~%")
     (format f "           (forward-message-to (car (send self :cameras)) args))))~%")
     (format f "  ) ;; defmethod ~A-robot~%" robot)
     ) ;; with-open-file
    (warning-message 1 "copy model file from ~A to ~A.l~%" fname_lisp robot)
    (unix::system (format nil "mv ~A ~A/~A.l" fname_lisp (ros::rospack-find "pr2eus") robot))
    ))

(ros::roseus "make-pr2-modle-file")
(make-pr2-model-file)


