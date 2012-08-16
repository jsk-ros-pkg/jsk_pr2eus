(ros::load-ros-manifest "arm_navigation_msgs")

(defclass collision-object-publisher
  :super propertied-object
  :slots (object-list topicname))

(defmethod collision-object-publisher
  (:init (&key (topic-name "collision_object"))
   (unless (ros::ok)
     (ros::roseus "publish_collision_eusobj"))
   (setq topicname topic-name)
   (ros::advertise topicname arm_navigation_msgs::CollisionObject)
   (setq object-list (make-hash-table)))
  (:add-object
   (obj &key (frame_id "base_link"))
   (let ((msg (gethash obj object-list)))
     (when msg (return-from :add-object)))
   (let* ((id (symbol-string (gensym "COLOBJ")))
          (colobj (instance arm_navigation_msgs::CollisionObject :init :id id)))

     (send colobj :operation :operation arm_navigation_msgs::CollisionObjectOperation::*ADD*)

     (send colobj :header :frame_id frame_id)
     (send colobj :header :stamp (ros::time-now))

     (setq geom (instance arm_navigation_msgs::Shape :init))

     (cond
      ((and (derivedp obj body) (eq (car (send obj :body-type)) :cylinder))
       (send geom :type arm_navigation_msgs::Shape::*CYLINDER*)
       (send geom :dimensions (float-vector
                               (/ (radius-of-cylinder obj) 1000.0)
                               (/ (height-of-cylinder obj) 1000.0))))
      ((and (derivedp obj body) (eq (car (send obj :body-type)) :cube))
       (send geom :type arm_navigation_msgs::Shape::*BOX*)
       (send geom :dimensions (float-vector
                               (/ (elt (send a :body-type) 1) 1000.0)
                               (/ (elt (send a :body-type) 2) 1000.0)
                               (/ (elt (send a :body-type) 3) 1000.0))))
      ((and (derivedp obj body) (eq (car (send obj :body-type)) :gdome))
       (send geom :type arm_navigation_msgs::Shape::*SPHERE*)
       (send geom :dimensions (float-vector
                               (/ (radius-of-sphere obj) 1000.0))))
      (t
       (warn ";; not supported object type~%")
       (return-from :add-object)
       ))

     (setq pose (ros::coords->tf-pose (send obj :worldcoords)))
     (send colobj :shapes (list geom))
     (send colobj :poses (list pose))
     (setf (gethash obj object-list) colobj)
     (ros::publish topicname colobj)
     obj))
  (:clear-all ()
   (dolist (obj (send object-list :list-keys))
     (send self :delete-object obj))
   (setq object-list (make-hash-table))
   t)
  (:delete-object
   (obj)
   (let ((msg (gethash obj object-list)))
     (unless msg
       (return-from :delete-object))
     (send msg :operation :operation arm_navigation_msgs::CollisionObjectOperation::*REMOVE*)
     (remhash obj object-list)
     (ros::publish topicname msg)
     obj))
  )
