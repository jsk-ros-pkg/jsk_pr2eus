#!/usr/bin/env roseus
;; pr2-tabletop-object-grasp.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(defvar *grasp-target-bounding-box-topic* "/bounding_box_marker/selected_box")
(defvar *grasp-status-topic* "/tabletop_object_grasp_status")
(defvar *base-frame-id* "/base_footprint")
(defvar *camera-frame-id* "/head_mount_kinect_rgb_optical_frame")
(defvar *click-point-topic* "/kinect_head/rgb/throttled/image_rect_color/screenpoint")
(defvar *ray-marker-topic* "/ray_marker_array")
(defvar *image-marker-topic* "/image_marker")
(defvar *screenpoint-output-topic* "/pointcloud_screenpoint_nodelet/output_point")

(ros::load-ros-manifest "geometry_msgs")
(ros::load-ros-manifest "image_view2")
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "jsk_rviz_plugins")

(ros::roseus "pr2_tabletop_object_grasp_node")
(ros::rate 10)

(require :pr2-interface "package://pr2eus/pr2-interface.l")


(defun pr2-pregrasp-pose ()
  (send *pr2* :reset-manip-pose)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
  (send *ri* :wait-interpolation))

(defun publish-status (msg-type fmt)
  (let ((color
         (case msg-type
          ('error (instance std_msgs::ColorRGBA :init
                            :r 1.0 :g 0.0 :b 0.0 :a 1.0))
          ('warn (instance std_msgs::ColorRGBA :init
                           :r 1.0 :g 1.0 :b 0.0 :a 1.0))
          (t (instance std_msgs::ColorRGBA :init
                       :r (/ 157.0 255) :g (/ 204.0 255) :b (/ 224.0 255) :a 1.0)))))
    (unless (ros::get-topic-publisher *grasp-status-topic*)
      (ros::advertise *grasp-status-topic* jsk_rviz_plugins::OverlayText 5)
      (unix:sleep 1))
    (ros::publish *grasp-status-topic*
                  (instance jsk_rviz_plugins::OverlayText :init
                            :width 1920 :height 400
                            :left 0 :top 0
                            :line_width 10 :text_size 50
                            :fg_color color
                            :text fmt))))

(defun publish-info (&rest args)
  (let ((s (format nil (car args) (cdr args))))
    (ros::ros-info s)
    (publish-status 'info s)))
(defun publish-warn (&rest args)
  (let ((s (format nil (car args) (cdr args))))
    (ros::ros-warn s)
    (publish-status 'warn s)))
(defun publish-error (&rest args)
  (let ((s (format nil (car args) (cdr args))))
    (ros::ros-error s)
    (publish-status 'error s)))

(defun vector3-> (applier v)
  (funcall applier
         (* (send v :x) 1000.0)
         (* (send v :y) 1000.0)
         (* (send v :z) 1000.0)))

(defun bounding-box->cube (msg)
  (let ((cds (ros::tf-pose->coords (send msg :pose)))
        (d (vector3-> 'list (send msg :dimensions)))
        (base->parent (send *tfl* :lookup-transform
                            *base-frame-id* (send msg :header :frame_id)
                            (ros::time 0))))
    (send *tfb* :send-transform
          (send (send base->parent :copy-worldcoords)
                :transform cds)
          *base-frame-id* "target_object" (send msg :header :stamp))
    (send (apply #'make-cube d)
          :transform
          (send (send base->parent :copy-worldcoords)
                :transform cds))))

;; Mainly copied from https://github.com/jsk-ros-pkg/jsk_recognition/blob/master/jsk_pcl_ros/euslisp/pointcloud_screenpoint.l
(defun show-marker (frame p1)
  "publish markers
@param frame frame id
@param p1 position of clicked point"
  (let* ((header (instance std_msgs::header :init
                           :stamp (ros::time-now) :frame_id frame))
         (p0 (float-vector 0 0 0))
         (sp (make-sphere 30 :pos p1))
         ;; Marker::SPHERE
         (sp-msg (sphere->marker-msg sp header
                                     :color (float-vector 1 0 1) :alpha 1.0))
         ;; Marker::LINE
         (li-msg (line->marker-msg (list p0 p1) header :scale 15
                                   :color (float-vector 0 1 1) :alpha 0.5))
         ;; MarkerArray msg
         (msg (instance visualization_msgs::MarkerArray :init))
         )
    (send sp-msg :ns "test_sphere")
    (send li-msg :ns "test_line")
    (send sp-msg :lifetime (ros::time 5))
    (send li-msg :lifetime (ros::time 5))
    (send msg :markers (list sp-msg li-msg))
    (ros::publish *ray-marker-topic* msg)
    ))

(defun grasp-target-callback (msg)
  (let ((camera-to-base (send *tfl* :lookup-transform *camera-frame-id* *base-frame-id* (ros::time 0)))
        (coords (make-coords :pos (send (bounding-box->cube msg) :worldpos))))
    (send coords :transform camera-to-base :world)
    (show-marker *camera-frame-id* (send coords :pos))
    (grasp (send (bounding-box->cube msg) :worldpos))))

(defun grasp-target-2d-callback (msg)
  (let ((base-to-camera (send *tfl* :lookup-transform *base-frame-id* *camera-frame-id* (ros::time 0)))
        (coords (make-coords :pos (ros::tf-point->pos (send msg :point)))))
    (show-marker *camera-frame-id* (ros::tf-point->pos (send msg :point)))
    (send coords :transform base-to-camera :world)
    (grasp (send coords :pos))))

(defun click-point-callback (msg)
  (let ((mrk (instance image_view2::ImageMarker2 :init))
        (x (send (send msg :point) :x))
        (y (send (send msg :point) :y)))
    (send mrk :type image_view2::ImageMarker2::*CIRCLE*)
    (send mrk :id 0)
    (send mrk :position (instance geometry_msgs::Point :init :x x :y y))
    (send mrk :outline_color (instance std_msgs::ColorRGBA :init :r 0.0 :g 1.0 :b 0.0 :a 1.0))
    (send mrk :lifetime (ros::Time 5))
    (send mrk :scale 20.0)
    (ros::publish *image-marker-topic* mrk)))

(defun grasp (pos)
  (if *grasping-object-p*
    (progn
      (publish-warn "cancel grasping...")
      (send *ri* :stop-motion)
      (send *ri* :stop-grasp *arm* :wait t)
      (setq *grasping-object-p* nil))
    (setq *grasping-object-p* t))
  (unless
      (send *pr2* *arm* :inverse-kinematics
            (make-coords :pos (v+ pos
                                  (float-vector -120 0 0))
                         :rpy #f(0 0 0))
            :rotation-axis t)
    (publish-error "out of reach...")
    (return-from grasp nil))
  (publish-info "openning gripper...")

  (setq *grasp-state* "reaching")
  (publish-grasp-state)
  (publish-info "pre grasp pose...")
  (send *ri* :stop-grasp *arm* :wait t)
  (send *pr2* :look-at-hand *arm*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 6000)
  (send *ri* :wait-interpolation)

  (setq *grasp-state* "grasp-approaching")
  (publish-grasp-state)
  (publish-info "reaching...")
  (send *pr2* *arm* :move-end-pos #f(160 0 0) :world)
  (send *pr2* :look-at-hand *arm*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (setq *grasp-state* "grasp-closing")
  (publish-grasp-state)
  (publish-info "grasping...")
  (let ((grasp-result (send *ri* :start-grasp *arm*)))
    (send *ri* :wait-interpolation)
    (unless (< 5.0 grasp-result)
      (publish-error "failed to grasp ~A" grasp-result)
      (send *ri* :stop-grasp *arm* :wait t)
      (publish-info "back to pre grasp...")
      (pr2-pregrasp-pose)
      (return-from grasp nil)))

  (setq *grasp-state* "picking")
  (publish-grasp-state)
  (publish-info "picking up...")
  (send *pr2* *arm* :move-end-pos #f(0 0 150) :world)
  (send *pr2* :look-at-hand *arm*)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  (publish-info "grasp succeeded!")

  (setq *grasp-state* "placing")
  (publish-grasp-state)
  (publish-info "placing ...")
  (send *pr2* *arm* :move-end-pos #f(0 0 -150) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (setq *grasp-state* "grasp-opening")
  (publish-grasp-state)
  (send *ri* :stop-grasp *arm* :wait t)
  (publish-info "return object...")

  (setq *grasp-state* "leaving")
  (publish-grasp-state)
  (send *pr2* *arm* :move-end-pos #f(-160 0 0) :world)
  (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  (publish-info "pre grasp pose")

  (setq *grasp-state* "returning")
  (publish-grasp-state)
  (pr2-pregrasp-pose)
  (ros::ros-info "click tabletop object in Rviz or in image_view2!")
  (publish-status 'info (format nil "click tabletop object~%in Rviz or in image_view2!"))
  )


(defun wait-for-grasp-target ()
  (ros::subscribe *grasp-target-bounding-box-topic*
                  jsk_recognition_msgs::BoundingBox #'grasp-target-callback)
  )

(defun wait-for-grasp-target-2d ()
  (ros::advertise *image-marker-topic* image_view2::ImageMarker2 1)
  (ros::advertise *ray-marker-topic* visualization_msgs::MarkerArray 10)
  (ros::subscribe *screenpoint-output-topic*
                  geometry_msgs::PointStamped #'grasp-target-2d-callback)
  (ros::subscribe *click-point-topic*
                  geometry_msgs::PointStamped #'click-point-callback)
  )

(defun publish-grasp-state ()
  (ros::publish "/grasp_state" (instance std_msgs::String :init :data *grasp-state*)))

(defun demo ()
  (setq *grasping-object-p* nil)
  (setq *arm* :rarm)
  (setq *tfl* (instance ros::transform-listener :init))
  (setq *tfb* (instance ros::transform-broadcaster :init))
  (ros::advertise "/grasp_state" std_msgs::String 1)
  (setq *grasp-state* "waiting")
  (pr2-init)
  (pr2-pregrasp-pose)
  (wait-for-grasp-target)
  (wait-for-grasp-target-2d)
  (ros::ros-info "click tabletop object in Rviz or in image_view2!")
  (publish-status 'info (format nil "click tabletop object~%in Rviz or in image_view2!"))
  (ros::spin))
(demo)
