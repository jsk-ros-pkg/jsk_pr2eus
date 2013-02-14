(ros::load-ros-manifest "arm_navigation_msgs")

(defclass collision-object-publisher
  :super propertied-object
  :slots (object-list topicname))

(defmethod collision-object-publisher
  (:init (&key (topic-name "collision_object"))
   (unless (ros::ok)
     (ros::roseus "publish_collision_eusobj"))
   (setq topicname topic-name)
   (ros::advertise topicname arm_navigation_msgs::CollisionObject 100)
   (setq object-list (make-hash-table)))
  (:set-planning-environment
   (&optional ap-env)
   (when ap-env
     (setf (get self :arm-planning-environment) ap-env)
     ;;(setq (ap-env . robot) robot)
     )
   (get self :arm-planning-environment))
  (:planning-environment
   (&rest args)
   (let ((env (get self :arm-planning-environment)))
     (when env
       (send* env args))))
  (:add-object
   (obj &key (frame_id "base_link"))
   (let ((msg (gethash obj object-list)))
     (when msg (return-from :add-object)))
   (let* ((id (symbol-string (gensym "COLOBJ")))
          (colobj (instance arm_navigation_msgs::CollisionObject :init :id id))
          geom-lst pose-lst)

     (send colobj :operation :operation
           arm_navigation_msgs::CollisionObjectOperation::*ADD*)

     (send colobj :header :frame_id frame_id)
     (send colobj :header :stamp (ros::time-now))

     (cond
      ((and (derivedp obj body) (eq (car (send obj :body-type)) :cylinder))
       (let (geom pose)
         (setq geom (instance arm_navigation_msgs::Shape :init))
         (send geom :type arm_navigation_msgs::Shape::*CYLINDER*)
         (send geom :dimensions (float-vector
                                 (/ (radius-of-cylinder obj) 1000.0)
                                 (/ (height-of-cylinder obj) 1000.0)))
         (setq pose (ros::coords->tf-pose (send obj :worldcoords)))
         (push pose pose-lst)
         (push geom geom-lst)))
      ((and (derivedp obj body) (eq (car (send obj :body-type)) :cube))
       (let (geom pose)
         (setq geom (instance arm_navigation_msgs::Shape :init))
         (send geom :type arm_navigation_msgs::Shape::*BOX*)
         (send geom :dimensions (float-vector
                                 (/ (elt (send obj :body-type) 1) 1000.0)
                                 (/ (elt (send obj :body-type) 2) 1000.0)
                                 (/ (elt (send obj :body-type) 3) 1000.0)))
         (setq pose (ros::coords->tf-pose (send obj :worldcoords)))
         (push pose pose-lst)
         (push geom geom-lst)))
      ((and (derivedp obj body) (eq (car (send obj :body-type)) :gdome))
       (let (geom pose)
         (setq geom (instance arm_navigation_msgs::Shape :init))
         (send geom :type arm_navigation_msgs::Shape::*SPHERE*)
         (send geom :dimensions (float-vector
                                 (/ (radius-of-sphere obj) 1000.0)))
         (setq pose (ros::coords->tf-pose (send obj :worldcoords)))
         (push pose pose-lst)
         (push geom geom-lst)))
      ((listp obj)
       (dolist (p vox-center-3dp-lst)
         (let ((geom (instance arm_navigation_msgs::Shape :init))
               pose)
           (send geom :type arm_navigation_msgs::Shape::*BOX*)
           (send geom :dimensions (float-vector
                                   meter-grid-size
                                   meter-grid-size
                                   meter-grid-size))
           (setq pose (ros::coords->tf-pose
                       (make-coords
                        :pos (float-vector (elt p 0) (elt p 1) (elt p 2)))))
           (push pose pose-lst)
           (push geom geom-lst))))
      ((find-method obj :faces)
       (let ((org-cds (send obj :copy-worldcoords)))
         (send obj :reset-coords)
         (send obj :worldcoords)
         (let ((fs (body-to-faces obj))
               (geom (instance arm_navigation_msgs::Shape :init))
               pose idx-lst vertices)
           (send geom :type arm_navigation_msgs::Shape::*MESH*)
           (setq pose (ros::coords->tf-pose (send obj :worldcoords)))
           (dolist (f (send fs :faces))
             (let* ((vs (send f :vertices))
                    (v0 (car vs))
                    (v1 (cadr vs))
                    (v2 (caddr vs))
                    (p0
                     (instance geometry_msgs::Point :init
                               :x (/ (elt v0 0) 1000.0)
                               :y (/ (elt v0 1) 1000.0)
                               :z (/ (elt v0 2) 1000.0)))
                    (p1
                     (instance geometry_msgs::Point :init
                               :x (/ (elt v1 0) 1000.0)
                               :y (/ (elt v1 1) 1000.0)
                               :z (/ (elt v1 2) 1000.0)))
                    (p2
                     (instance geometry_msgs::Point :init
                               :x (/ (elt v2 0) 1000.0)
                               :y (/ (elt v2 1) 1000.0)
                               :z (/ (elt v2 2) 1000.0))))
               (push p0 vertices)
               (push p1 vertices)
               (push p2 vertices)))
           (setq idx-lst (instantiate integer-vector (length vertices)))
           (dotimes (i (length vertices))
             (setf (elt idx-lst i) i))
           (send geom :triangles idx-lst)
           (send geom :vertices (reverse vertices))
           (push pose pose-lst)
           (push geom geom-lst))
         (send obj :transform org-cds)
         (send obj :worldcoords)
         ))
      ((derivedp obj pointcloud)
       ;; making voxel grid ...
       )
      ((find-method obj :vertices)
       ;; making bounding box
       ;; (send obj :vertices)
       )
      (t
       (warn ";; not supported object type~%")
       (return-from :add-object)
       ))

     (send colobj :shapes geom-lst)
     (send colobj :poses pose-lst)
     (setf (gethash obj object-list) colobj)
     (ros::publish topicname colobj)
     obj))
  (:clear-all ()
   (dolist (obj (send object-list :list-keys))
     (send self :delete-object obj))
   (setq object-list (make-hash-table))
   t)
  (:wipe-all ()
   (let ((scene (send self :planning-environment :get-planning-scene)))
     (when scene
       (dolist (msg (send (send scene :planning_scene) :collision_objects))
         (send msg :header :stamp (ros::time-now))
         (send msg :operation :operation arm_navigation_msgs::CollisionObjectOperation::*REMOVE*)
         (ros::publish topicname msg))
       (setq object-list (make-hash-table))
       )))
  (:delete-object (obj)
   (let ((msg (gethash obj object-list)))
     (unless msg
       (return-from :delete-object))
     (send msg :header :stamp (ros::time-now))
     (send msg :operation :operation arm_navigation_msgs::CollisionObjectOperation::*REMOVE*)
     (remhash obj object-list)
     (ros::publish topicname msg)
     obj))
  )

(defun make-collision-map (vox-center-3dp-lst &key (stamp (ros::time-now))
                                          (frame "/base_footprint") (grid-size 30))
  (let* ((cmap (instance arm_navigation_msgs::CollisionMap :init))
         (hd (instance std_msgs::header :init))
         lst (meter-grid-size (* (/ grid-size 2) 0.001)))
    ;;
    (send hd :frame_id frame)
    (send hd :stamp stamp)
    (send cmap :header hd)
    ;;
    (dolist (p vox-center-3dp-lst)
      (let ((bx
             (instance arm_navigation_msgs::OrientedBoundingBox :init)))
        (send bx :center :x (* 0.001 (elt p 0)))
        (send bx :center :y (* 0.001 (elt p 1)))
        (send bx :center :z (* 0.001 (elt p 2)))
        (send bx :extents :x meter-grid-size)
        (send bx :extents :y meter-grid-size)
        (send bx :extents :z meter-grid-size)
        (push bx lst)))
    (send cmap :boxes (nreverse lst))
    cmap))
